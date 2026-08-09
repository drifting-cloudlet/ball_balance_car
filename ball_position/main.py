"""MaixCAM2 / MaixCAM-Pro steel-ball YOLO position, UART, and JPG collector.

The detector path is kept identical to the original application.  Dataset
frames are copied immediately after the optional lens correction and before
any axis, detection box, text, or touchscreen control is drawn.
"""

import atexit
import gc
import math
import os
import queue
import threading

from maix import app, camera, display, err, image, nn, sys, time, touchscreen

# 逐帧打印会明显降低检测帧率；需要分析各阶段耗时时再打开。
DEBUG_LOG = False
BUILD_ID = "YOLO11_CENTER_ZERO_UART0_MODELPATHFIX_XINV_ZERO464_CAL5_20260731"


def print_debug(message):
    if DEBUG_LOG:
        print(message)


# MaixVision 临时运行只上传 main.py，因此板端必需逻辑保持在本文件内。
UART_DEVICE = "/dev/ttyS0"
UART_BAUD_RATE = 115200
UART_WIRE_MIN_DMM = -1250
UART_WIRE_MAX_DMM = 1250
UART0_LAST_OPEN_ERROR = ""


def xor_checksum(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return checksum


def serialize_ball_frame(seq, x_dmm, valid, confidence):
    if valid == 0:
        x_dmm = 0
        confidence = 0
    body = "B,{},{},{},{}".format(seq, x_dmm, valid, confidence)
    return "${}*{:02X}\r\n".format(body, xor_checksum(body))


class BallUart0Sender:
    def __init__(self, serial_device):
        self.serial_device = serial_device
        self.sequence = 0
        self.last_error = ""

    def send(self, x_cm, confidence):
        valid = x_cm is not None and math.isfinite(float(x_cm))
        x_dmm = int(round(float(x_cm) * 100.0)) if valid else 0
        if not UART_WIRE_MIN_DMM <= x_dmm <= UART_WIRE_MAX_DMM:
            valid = False
            x_dmm = 0
        confidence_percent = (
            max(0, min(100, int(round(float(confidence) * 100.0))))
            if valid
            else 0
        )
        frame = serialize_ball_frame(
            self.sequence, x_dmm, int(valid), confidence_percent
        )
        try:
            payload = frame.encode("ascii")
            written = self.serial_device.write(payload)
            if isinstance(written, int) and written != len(payload):
                raise RuntimeError(
                    "UART short write: {}/{}".format(written, len(payload))
                )
        except Exception as exc:
            self.last_error = str(exc).replace("\r", " ").replace("\n", " ")
            return False
        self.sequence = (self.sequence + 1) & 0xFF
        self.last_error = ""
        return True

    def close(self):
        if hasattr(self.serial_device, "close"):
            self.serial_device.close()


def open_uart0_sender():
    global UART0_LAST_OPEN_ERROR

    try:
        from maix import uart

        serial_device = uart.UART(UART_DEVICE, UART_BAUD_RATE)
    except Exception as exc:
        UART0_LAST_OPEN_ERROR = str(exc).replace("\r", " ").replace("\n", " ")
        print("UART0 open failed: {}".format(exc))
        return None
    UART0_LAST_OPEN_ERROR = ""
    print("UART0 opened: {} @ {}".format(UART_DEVICE, UART_BAUD_RATE))
    return BallUart0Sender(serial_device)


def project_onto_axis(point, axis_start, axis_end):
    px, py = point
    x0, y0 = axis_start
    x1, y1 = axis_end
    dx = x1 - x0
    dy = y1 - y0
    length_squared = dx * dx + dy * dy
    if length_squared <= 0:
        raise ValueError("calibration endpoints must be different")

    relative_x = px - x0
    relative_y = py - y0
    ratio = (relative_x * dx + relative_y * dy) / float(length_squared)
    projected_x = x0 + ratio * dx
    projected_y = y0 + ratio * dy
    distance = math.hypot(px - projected_x, py - projected_y)
    return ratio, distance


def axis_point(ratio, axis_start, axis_end):
    x0, y0 = axis_start
    x1, y1 = axis_end
    return (
        x0 + ratio * (x1 - x0),
        y0 + ratio * (y1 - y0),
    )


def position_from_axis_ratio(
    ratio, start_position_cm, end_position_cm, zero_ratio=0.5
):
    if not 0.0 < zero_ratio < 1.0:
        raise ValueError("zero ratio must be inside the calibration axis")
    if start_position_cm >= 0.0 or end_position_cm <= 0.0:
        raise ValueError("physical calibration positions must straddle zero")
    if ratio <= zero_ratio:
        return start_position_cm * (1.0 - ratio / zero_ratio)
    return end_position_cm * (ratio - zero_ratio) / (1.0 - zero_ratio)


def calibrate_position_cm(measured_cm):
    """Convert a geometric reading to the measured physical coordinate."""
    points = POSITION_CALIBRATION_POINTS_CM
    if measured_cm <= points[0][0]:
        left, right = points[0], points[1]
    elif measured_cm >= points[-1][0]:
        left, right = points[-2], points[-1]
    else:
        for index in range(1, len(points)):
            if measured_cm <= points[index][0]:
                left, right = points[index - 1], points[index]
                break

    measured_span = right[0] - left[0]
    ratio = (measured_cm - left[0]) / measured_span
    return left[1] + ratio * (right[1] - left[1])


def position_from_pixel(
    point,
    axis_start,
    axis_end,
    start_position_cm,
    end_position_cm,
    zero_ratio=0.5,
):
    ratio, distance = project_onto_axis(point, axis_start, axis_end)
    position_cm = position_from_axis_ratio(
        ratio, start_position_cm, end_position_cm, zero_ratio
    )
    return position_cm, ratio, distance


class AdaptiveAlphaBetaFilter:
    def __init__(
        self,
        alpha_slow=0.20,
        alpha_fast=0.95,
        beta_slow=0.01,
        beta_fast=0.12,
        fast_error_px=10.0,
        reset_ms=250,
    ):
        self.alpha_slow = alpha_slow
        self.alpha_fast = alpha_fast
        self.beta_slow = beta_slow
        self.beta_fast = beta_fast
        self.fast_error_px = fast_error_px
        self.reset_ms = reset_ms
        self.reset()

    def reset(self):
        self.x = None
        self.y = None
        self.vx = 0.0
        self.vy = 0.0
        self.last_ms = None

    def update(self, measured_x, measured_y, now_ms):
        if self.x is None:
            self.x = float(measured_x)
            self.y = float(measured_y)
            self.last_ms = now_ms
            return self.x, self.y

        dt = (now_ms - self.last_ms) * 0.001
        dt = max(0.005, min(0.100, dt))

        predicted_x = self.x + self.vx * dt
        predicted_y = self.y + self.vy * dt
        error_x = measured_x - predicted_x
        error_y = measured_y - predicted_y
        error = math.hypot(error_x, error_y)

        motion = min(1.0, error / self.fast_error_px) ** 2
        alpha = self.alpha_slow + (self.alpha_fast - self.alpha_slow) * motion
        beta = self.beta_slow + (self.beta_fast - self.beta_slow) * motion

        self.x = predicted_x + alpha * error_x
        self.y = predicted_y + alpha * error_y
        self.vx += beta * error_x / dt
        self.vy += beta * error_y / dt
        self.last_ms = now_ms
        return self.x, self.y

    def mark_missing(self, now_ms):
        if self.last_ms is not None and now_ms - self.last_ms >= self.reset_ms:
            self.reset()


def validate_calibration(
    axis_start,
    axis_end,
    start_position_cm,
    end_position_cm,
    width,
    height,
    zero_ratio=0.5,
):
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    for name, point in (("start", axis_start), ("end", axis_end)):
        if not (0 <= point[0] < width and 0 <= point[1] < height):
            raise ValueError("{} calibration point is outside the image".format(name))
    position_from_pixel(
        axis_start,
        axis_start,
        axis_end,
        start_position_cm,
        end_position_cm,
        zero_ratio,
    )


# 位置传感器更重视“采集到结果”的延迟，而不是最高吞吐帧率。
# 低延迟模式下 dual_buff=False，每次检测结果都对应本次传入的图像。
# 只有在测试最高检测帧率时，才建议把该开关改为 False。
LOW_LATENCY_MODE = True

# 可选网络输出保留原来的开关；通常只开启一种。
USE_RTSP = False
USE_JPEG = False

# 可选镜头畸变校正。数据集会保存校正后的、送入 YOLO 的同一幅干净图像。
LENS_CORR_ENABLE = False
LENS_CORR_STRENGTH = 0.6

# 钢珠位置三点标定：左右端自动铺满画面，零点按装机尺面单独校准。
AXIS_START_PX = (40, 112)
AXIS_END_PX = (280, 112)
AXIS_START_CM = -12.5
AXIS_END_CM = 12.5
AXIS_ZERO_RATIO = 0.464
POSITION_CALIBRATION_POINTS_CM = (
    (-9.36, -10.0),
    (-4.65, -5.0),
    (0.0, 0.0),
    (4.17, 5.0),
    (8.25, 10.0),
)
DETECTION_CONFIDENCE = 0.50

# MaixCAM-Pro 和 MaixCAM2 使用不同的模型文件。
MAIXCAM_MODEL_PATH = (
    "models/steel_ball_yolo11_maixcam_pro_320x320/model.mud"
)
MAIXCAM2_MODEL_PATH = (
    "/root/models/steel_ball_yolo11n_640x160_clean1077/"
    "steel_ball_yolo11n_640x160_clean1077.mud"
)
MAIXCAM2_LEGACY_MODEL_PATH = (
    "/root/models/steel_ball_yolo11_640x160_maixcam2/"
    "steel_ball_yolo11n_640x160_clean1077.mud"
)

# JPG 数据集配置。队列保持很小，磁盘忙时丢采集帧，不阻塞 YOLO。
CAPTURE_ROOT = "/root/steel_ball_dataset"
CAPTURE_TARGET_FPS = 30
CAPTURE_JPEG_QUALITY = 95
CAPTURE_QUEUE_SIZE = 3
CAPTURE_BUTTON_WIDTH = 124
CAPTURE_BUTTON_HEIGHT = 38
CAPTURE_TOUCH_DEBOUNCE_MS = 240
CAPTURE_EXIT_SYNC_SECONDS = 8.0


def normalize_device_name(device_name):
    """Normalize board names across firmware spelling differences."""
    return "".join(
        char for char in str(device_name).strip().lower() if char.isalnum()
    )


def model_path_for_device(device_name):
    """Return the checked-in YOLO11 model matching a Maix device name."""
    # Firmware releases have used hyphens and underscores inconsistently.
    normalized = normalize_device_name(device_name)
    if normalized == "maixcam2":
        return MAIXCAM2_MODEL_PATH
    if normalized in ("maixcam", "maixcampro"):
        return MAIXCAM_MODEL_PATH
    raise ValueError("unsupported device: {}".format(device_name))


def resolve_project_file(relative_path):
    """Find a project asset even when Python was started from another cwd."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    requested_paths = [relative_path]
    if relative_path == MAIXCAM2_MODEL_PATH:
        requested_paths.append(MAIXCAM2_LEGACY_MODEL_PATH)

    candidates = []
    for requested_path in requested_paths:
        candidates.extend(
            [requested_path, os.path.join(script_dir, requested_path)]
        )
    get_app_path = getattr(app, "get_app_path", None)
    if get_app_path is not None:
        try:
            app_path = get_app_path()
            if app_path:
                for requested_path in requested_paths:
                    candidates.append(os.path.join(app_path, requested_path))
        except Exception:
            pass
    for path in dict.fromkeys(candidates):
        if os.path.isfile(path):
            return path
    raise RuntimeError("model not found; checked: " + ", ".join(candidates))


def next_numbered_directory(root):
    """Atomically create and return the next six-digit dataset directory."""
    os.makedirs(root, exist_ok=True)
    largest = 0
    for name in os.listdir(root):
        path = os.path.join(root, name)
        if name.isdigit() and os.path.isdir(path):
            largest = max(largest, int(name))

    number = largest + 1
    while True:
        candidate = os.path.join(root, "{:06d}".format(number))
        try:
            os.mkdir(candidate)
            return number, candidate
        except FileExistsError:
            # Another process may have created the directory after our scan.
            number += 1


class JpegCaptureWorker:
    """Non-blocking producer plus one background JPG writer.

    States are idle -> capturing -> stopping -> done.  STOP only disables the
    producer; the state remains ``stopping`` until every queued image has been
    written.  This prevents a completed batch from being reported too early.
    """

    def __init__(self):
        self._queue = queue.Queue(maxsize=CAPTURE_QUEUE_SIZE)
        self._lock = threading.Lock()
        self._drained = threading.Event()
        self._drained.set()
        self._shutdown = threading.Event()
        self._closed = False
        self._inflight = 0

        self.phase = "idle"
        self.accepting = False
        self.session_number = 0
        self.session_dir = ""
        self.enqueued = 0
        self.saved = 0
        self.dropped = 0
        self.failed = 0
        self.saved_fps = 0.0
        self.last_error = ""
        self._last_submit_ms = -1000
        self._fps_window_started_ms = 0
        self._fps_window_saved = 0

        self._thread = threading.Thread(
            target=self._writer_loop,
            name="jpg-writer",
            daemon=True,
        )
        self._thread.start()

    def _pending_locked(self):
        # Unlike queue.empty(), this remains non-zero while the writer has
        # already dequeued an image but has not finished Image.save().
        return max(0, self.enqueued - self.saved - self.failed)

    def start_session(self):
        """Create the next directory and start accepting clean frames."""
        with self._lock:
            if self.phase in ("capturing", "stopping"):
                return False
            if self._pending_locked():
                self.phase = "stopping"
                return False

        try:
            number, directory = next_numbered_directory(CAPTURE_ROOT)
        except Exception as exc:
            with self._lock:
                self.accepting = False
                self.phase = "error"
                self.last_error = "create folder: {}".format(exc)
            print("[capture] " + self.last_error)
            return False

        now_ms = time.ticks_ms()
        with self._lock:
            self.session_number = number
            self.session_dir = directory
            self.enqueued = 0
            self.saved = 0
            self.dropped = 0
            self.failed = 0
            self.saved_fps = 0.0
            self.last_error = ""
            self._last_submit_ms = now_ms - max(
                1, 1000 // CAPTURE_TARGET_FPS
            )
            self._fps_window_started_ms = now_ms
            self._fps_window_saved = 0
            self.accepting = True
            self.phase = "capturing"
            self._drained.set()
        print("[capture] START #{:06d}: {}".format(number, directory))
        return True

    def request_stop(self):
        """Stop accepting new frames; queued frames continue to disk."""
        with self._lock:
            if self.phase != "capturing":
                return False
            self.accepting = False
            if self._pending_locked():
                self.phase = "stopping"
                self._drained.clear()
            else:
                self.phase = "done" if not self.last_error else "error"
                self._drained.set()
            number = self.session_number
            enqueued = self.enqueued
        print(
            "[capture] STOP #{:06d}: synchronizing {} queued images".format(
                number, enqueued
            )
        )
        return True

    def submit_if_due(self, frame, now_ms):
        """Queue at most 30 clean frame copies per second without waiting."""
        interval_ms = max(1, 1000 // CAPTURE_TARGET_FPS)
        with self._lock:
            if not self.accepting or self.phase != "capturing":
                return False
            if now_ms - self._last_submit_ms < interval_ms:
                return False
            # Advance the sampling clock even on a drop.  This avoids bursts
            # that could steal time from the detector after storage stalls.
            self._last_submit_ms = now_ms
            if self._queue.full():
                self.dropped += 1
                return False

        try:
            clean_copy = frame.copy()
        except Exception as exc:
            with self._lock:
                self.accepting = False
                self.failed += 1
                self.last_error = "copy image: {}".format(exc)
                if self._pending_locked():
                    self.phase = "stopping"
                    self._drained.clear()
                else:
                    self.phase = "error"
                    self._drained.set()
            print("[capture] " + self.last_error)
            return False

        with self._lock:
            # STOP may have been pressed while the image was being copied.
            if not self.accepting or self.phase != "capturing":
                del clean_copy
                return False
            index = self.enqueued + 1
            path = os.path.join(
                self.session_dir, "frame_{:06d}.jpg".format(index)
            )
            try:
                self._queue.put_nowait((path, clean_copy))
            except queue.Full:
                self.dropped += 1
                del clean_copy
                return False
            self.enqueued = index
            self._drained.clear()
        return True

    def snapshot(self):
        """Return a consistent, UI-friendly copy of the current state."""
        with self._lock:
            return {
                "phase": self.phase,
                "session_number": self.session_number,
                "session_dir": self.session_dir,
                "enqueued": self.enqueued,
                "saved": self.saved,
                "dropped": self.dropped,
                "failed": self.failed,
                "saved_fps": self.saved_fps,
                "queued": self._pending_locked(),
                "last_error": self.last_error,
            }

    def synchronize(self, timeout_seconds=CAPTURE_EXIT_SYNC_SECONDS):
        """Wait a bounded time for the current STOP batch to reach disk."""
        self.request_stop()
        return self._drained.wait(timeout_seconds)

    def close(self):
        """Best-effort final flush; safe when called by finally and atexit."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
        completed = self.synchronize(CAPTURE_EXIT_SYNC_SECONDS)
        if not completed:
            print("[capture] exit sync timed out; writer will finish if possible")
        self._shutdown.set()
        self._thread.join(timeout=2.0)

    def _finish_item(self, success, error_message, now_ms):
        with self._lock:
            self._inflight -= 1
            if success:
                self.saved += 1
                self._fps_window_saved += 1
                period_ms = now_ms - self._fps_window_started_ms
                if period_ms >= 1000:
                    self.saved_fps = (
                        self._fps_window_saved * 1000.0 / period_ms
                    )
                    self._fps_window_saved = 0
                    self._fps_window_started_ms = now_ms
            else:
                self.failed += 1
                self.last_error = error_message
                self.accepting = False

            pending = self._pending_locked()
            if not self.accepting and pending:
                self.phase = "stopping"
            if not self.accepting and pending == 0:
                self.phase = "error" if self.last_error else "done"
                self._drained.set()

    def _writer_loop(self):
        while not self._shutdown.is_set() or not self._queue.empty():
            try:
                path, clean_frame = self._queue.get(timeout=0.10)
            except queue.Empty:
                continue

            with self._lock:
                self._inflight += 1
            success = False
            error_message = ""
            try:
                result = clean_frame.save(path, quality=CAPTURE_JPEG_QUALITY)
                err.check_raise(result, "JPG save failed")
                success = True
            except Exception as exc:
                error_message = "save {}: {}".format(path, exc)
                print("[capture] " + error_message)
            finally:
                del clean_frame
                self._finish_item(success, error_message, time.ticks_ms())
                self._queue.task_done()
                if (self.saved + self.failed) % 100 == 0:
                    gc.collect()


def map_touch_to_frame(raw_x, raw_y, disp, frame):
    """Map display coordinates through Display.show()'s FIT_CONTAIN mode."""
    display_width = max(1, disp.width())
    display_height = max(1, disp.height())
    frame_width = max(1, frame.width())
    frame_height = max(1, frame.height())
    scale = min(
        display_width / float(frame_width),
        display_height / float(frame_height),
    )
    shown_width = frame_width * scale
    shown_height = frame_height * scale
    offset_x = (display_width - shown_width) * 0.5
    offset_y = (display_height - shown_height) * 0.5
    if (
        raw_x < offset_x
        or raw_x >= offset_x + shown_width
        or raw_y < offset_y
        or raw_y >= offset_y + shown_height
    ):
        return None
    x = int((raw_x - offset_x) / scale)
    y = int((raw_y - offset_y) / scale)
    return (
        max(0, min(frame_width - 1, x)),
        max(0, min(frame_height - 1, y)),
    )


def capture_button_rect(frame):
    width = min(CAPTURE_BUTTON_WIDTH, frame.width())
    height = min(CAPTURE_BUTTON_HEIGHT, frame.height())
    return frame.width() - width, frame.height() - height, width, height


def point_in_rect(x, y, rect):
    return (
        rect[0] <= x < rect[0] + rect[2]
        and rect[1] <= y < rect[1] + rect[3]
    )


def handle_capture_touch(
    touch,
    disp,
    frame,
    touch_state,
    capture_worker,
):
    if touch is None:
        return
    if touch.available(0):
        raw_x, raw_y, pressed = touch.read()
        if pressed and not touch_state["pressed"]:
            now_ms = time.ticks_ms()
            mapped = map_touch_to_frame(raw_x, raw_y, disp, frame)
            if (
                mapped is not None
                and now_ms - touch_state["last_action_ms"]
                >= CAPTURE_TOUCH_DEBOUNCE_MS
                and point_in_rect(
                    mapped[0], mapped[1], capture_button_rect(frame)
                )
            ):
                touch_state["last_action_ms"] = now_ms
                phase = capture_worker.snapshot()["phase"]
                if phase == "capturing":
                    capture_worker.request_stop()
                elif phase != "stopping":
                    capture_worker.start_session()
        touch_state["pressed"] = bool(pressed)


COLOR_AXIS = image.Color.from_rgb(0, 220, 255)
COLOR_ZERO = image.Color.from_rgb(255, 220, 0)
COLOR_PANEL = image.Color.from_rgb(0, 0, 0)
COLOR_BLUE = image.Color.from_rgb(30, 100, 235)
COLOR_ORANGE = image.Color.from_rgb(245, 145, 20)
COLOR_DARK_RED = image.Color.from_rgb(205, 40, 50)


def draw_calibration_axis(frame):
    frame.draw_line(
        AXIS_START_PX[0],
        AXIS_START_PX[1],
        AXIS_END_PX[0],
        AXIS_END_PX[1],
        COLOR_AXIS,
        2,
    )
    frame.draw_cross(AXIS_START_PX[0], AXIS_START_PX[1], COLOR_AXIS, 7, 2)
    frame.draw_cross(AXIS_END_PX[0], AXIS_END_PX[1], COLOR_AXIS, 7, 2)
    zero_x, zero_y = axis_point(
        AXIS_ZERO_RATIO, AXIS_START_PX, AXIS_END_PX
    )
    frame.draw_cross(int(zero_x), int(zero_y), COLOR_ZERO, 9, 2)


def draw_capture_button(frame, rect, label, color):
    frame.draw_rect(rect[0], rect[1], rect[2], rect[3], image.COLOR_BLACK, -1)
    frame.draw_rect(
        rect[0] + 2,
        rect[1] + 2,
        rect[2] - 4,
        rect[3] - 4,
        color,
        -1,
    )
    label_width = image.string_size(label, scale=0.78, thickness=1).width()
    text_x = rect[0] + max(3, (rect[2] - label_width) // 2)
    frame.draw_string(
        text_x,
        rect[1] + 10,
        label,
        color=image.COLOR_WHITE,
        scale=0.78,
        thickness=1,
    )


def draw_capture_ui(frame, capture_worker):
    state = capture_worker.snapshot()
    rect = capture_button_rect(frame)
    panel_width = max(0, frame.width() - rect[2])
    panel_y = rect[1]

    if state["phase"] == "capturing":
        label = "STOP JPG"
        button_color = COLOR_DARK_RED
        line1 = "JPG #{:06d} S:{}/{} {:.1f}fps".format(
            state["session_number"],
            state["saved"],
            state["enqueued"],
            state["saved_fps"],
        )
        line2 = "Q:{} DROP:{} FAIL:{}".format(
            state["queued"], state["dropped"], state["failed"]
        )
        frame.draw_rect(1, 1, frame.width() - 2, frame.height() - 2,
                        COLOR_DARK_RED, 2)
    elif state["phase"] == "stopping":
        label = "SAVING..."
        button_color = COLOR_ORANGE
        line1 = "SYNC #{:06d} S:{}/{}".format(
            state["session_number"], state["saved"], state["enqueued"]
        )
        line2 = "Q:{}  PLEASE WAIT".format(state["queued"])
    elif state["phase"] == "done":
        label = "START JPG"
        button_color = COLOR_BLUE
        line1 = "DONE #{:06d} JPG:{}".format(
            state["session_number"], state["saved"]
        )
        line2 = "DROP:{} FAIL:{}".format(state["dropped"], state["failed"])
    elif state["phase"] == "error":
        label = "RETRY JPG"
        button_color = COLOR_DARK_RED
        line1 = "JPG ERROR #{:06d}".format(state["session_number"])
        line2 = state["last_error"][-38:]
    else:
        label = "START JPG"
        button_color = COLOR_BLUE
        line1 = "JPG READY  {}FPS Q{}".format(
            CAPTURE_TARGET_FPS, CAPTURE_JPEG_QUALITY
        )
        line2 = CAPTURE_ROOT

    if panel_width > 0:
        frame.draw_rect(0, panel_y, panel_width, rect[3], COLOR_PANEL, -1)
        frame.draw_string(
            5, panel_y + 3, line1, image.COLOR_WHITE, 0.64, 1
        )
        frame.draw_string(
            5, panel_y + 20, line2, image.COLOR_YELLOW, 0.58, 1
        )
    draw_capture_button(frame, rect, label, button_color)


def draw_uart0_status(frame, status, error_message=""):
    """Draw UART0 health below the position banner on the local preview."""
    color = {
        "OK": image.COLOR_GREEN,
        "ERR": image.COLOR_RED,
        "OFF": COLOR_ORANGE,
    }.get(status, COLOR_ORANGE)
    label = "UART0 {}".format(status)
    short_error = error_message.replace("\r", " ").replace("\n", " ")[:32]
    label_width = image.string_size(label, scale=0.68, thickness=1).width()
    error_width = (
        image.string_size(short_error, scale=0.52, thickness=1).width()
        if short_error
        else 0
    )
    panel_width = min(frame.width(), max(label_width, error_width) + 10)
    panel_height = 36 if short_error else 20
    panel_y = 38
    frame.draw_rect(0, panel_y, panel_width, panel_height, COLOR_PANEL, -1)
    frame.draw_string(
        5,
        panel_y + 3,
        label,
        color,
        0.68,
        1,
    )
    if short_error:
        frame.draw_string(
            5,
            panel_y + 20,
            short_error,
            color,
            0.52,
            1,
        )


def main():
    global AXIS_START_PX, AXIS_END_PX

    device_name = sys.device_name()
    selected_model = model_path_for_device(device_name)
    print("build: {}".format(BUILD_ID))
    print("device: {} requested model: {}".format(device_name, selected_model))
    model = resolve_project_file(selected_model)
    print("resolved model: {}".format(model))

    detector = nn.YOLO11(model=model, dual_buff=not LOW_LATENCY_MODE)

    # 保留原程序行为：默认标定轴横跨当前模型输入的中央。
    AXIS_START_PX = (5, detector.input_height() // 2)
    AXIS_END_PX = (
        detector.input_width() - 5,
        detector.input_height() // 2,
    )

    validate_calibration(
        AXIS_START_PX,
        AXIS_END_PX,
        AXIS_START_CM,
        AXIS_END_CM,
        detector.input_width(),
        detector.input_height(),
        AXIS_ZERO_RATIO,
    )

    cam = camera.Camera(
        detector.input_width(),
        detector.input_height(),
        detector.input_format(),
    )
    disp = display.Display()
    position_filter = AdaptiveAlphaBetaFilter()
    capture_worker = JpegCaptureWorker()
    atexit.register(capture_worker.close)

    is_maixcam2 = normalize_device_name(device_name) == "maixcam2"
    uart_sender = open_uart0_sender() if is_maixcam2 else None
    uart0_status = "OK" if uart_sender is not None else "OFF"
    uart0_error = UART0_LAST_OPEN_ERROR if is_maixcam2 else ""

    touch = None
    try:
        touch = touchscreen.TouchScreen()
        touch.clear()
    except Exception as exc:
        print("[capture] touchscreen unavailable: {}".format(exc))

    # Keep stream references alive for the whole program.
    rtsp_server = None
    jpeg_server = None
    stream_channels = []

    if USE_RTSP:
        from maix import rtsp

        rtsp_channel = cam.add_channel(320, 180, image.Format.FMT_YVU420SP)
        stream_channels.append(rtsp_channel)
        rtsp_server = rtsp.Rtsp()
        rtsp_server.bind_camera(rtsp_channel)
        rtsp_server.start()
        print(rtsp_server.get_url())

    if USE_JPEG:
        from maix import http

        jpeg_server = http.JpegStreamer()
        jpeg_server.start()

    print(detector.input_width(), detector.input_height())
    print("capture root: {} ({} JPG/s target)".format(
        CAPTURE_ROOT, CAPTURE_TARGET_FPS
    ))

    touch_state = {"pressed": False, "last_action_ms": -1000}
    last_ms = time.ticks_ms()
    loop_ms = 1000

    try:
        while not app.need_exit():
            loop_ms = time.ticks_ms() - last_ms
            print_debug("loop cost " + str(loop_ms) + "ms")
            last_ms = time.ticks_ms()

            frame = cam.read()
            stage_ms = time.ticks_ms()

            if LENS_CORR_ENABLE:
                frame = frame.lens_corr(strength=LENS_CORR_STRENGTH)
            print_debug(
                "lens_corr cost " + str(time.ticks_ms() - stage_ms) + "ms"
            )

            # This is the exact clean YOLO input.  Queue its deep copy before
            # detector/UI drawing; the worker writes it while YOLO runs.
            capture_worker.submit_if_due(frame, time.ticks_ms())

            stage_ms = time.ticks_ms()
            objects = detector.detect(
                frame, conf_th=DETECTION_CONFIDENCE, iou_th=0.45
            )
            print_debug(
                "detect cost " + str(time.ticks_ms() - stage_ms) + "ms"
            )

            stage_ms = time.ticks_ms()
            now_ms = time.ticks_ms()
            draw_calibration_axis(frame)

            # 模型只有 steel_ball 一个类别。一帧多个框时只选最高置信度框。
            ball = max(objects, key=lambda obj: obj.score) if objects else None
            uart_position_cm = None
            uart_confidence = 0.0
            if ball is not None:
                raw_x = ball.x + ball.w * 0.5
                raw_y = ball.y + ball.h * 0.5
                filtered_x, filtered_y = position_filter.update(
                    raw_x, raw_y, now_ms
                )
                position_cm, axis_ratio, _axis_distance = position_from_pixel(
                    (filtered_x, filtered_y),
                    AXIS_START_PX,
                    AXIS_END_PX,
                    AXIS_START_CM,
                    AXIS_END_CM,
                    AXIS_ZERO_RATIO,
                )
                # Keep the reported coordinate inside [-12.5, +12.5].
                axis_ratio = max(0.0, min(1.0, axis_ratio))
                position_cm = calibrate_position_cm(
                    position_from_axis_ratio(
                        axis_ratio,
                        AXIS_START_CM,
                        AXIS_END_CM,
                        AXIS_ZERO_RATIO,
                    )
                )
                projected_x, projected_y = axis_point(
                    axis_ratio, AXIS_START_PX, AXIS_END_PX
                )

                frame.draw_rect(
                    ball.x,
                    ball.y,
                    ball.w,
                    ball.h,
                    color=image.COLOR_GREEN,
                    thickness=2,
                )

                if DEBUG_LOG:
                    frame.draw_cross(
                        int(filtered_x),
                        int(filtered_y),
                        image.COLOR_GREEN,
                        12,
                        2,
                    )
                    frame.draw_cross(
                        int(projected_x),
                        int(projected_y),
                        COLOR_ZERO,
                        7,
                        2,
                    )
                    message = (
                        f"{detector.labels[ball.class_id]}: {ball.score:.2f} "
                        f"px=({filtered_x:.1f},{filtered_y:.1f})"
                    )
                    frame.draw_string(
                        ball.x, ball.y, message, color=image.COLOR_RED
                    )
                    frame.draw_rect(
                        0, 0, detector.input_width(), 35, COLOR_PANEL, -1
                    )
                frame.draw_string(
                    8,
                    5,
                    f"BALL {position_cm:+.2f} cm",
                    color=image.COLOR_WHITE,
                    scale=1.4,
                    thickness=2,
                )
                uart_position_cm = position_cm
                uart_confidence = ball.score
            else:
                position_filter.mark_missing(now_ms)
                frame.draw_rect(
                    0, 0, detector.input_width(), 35, COLOR_PANEL, -1
                )
                frame.draw_string(
                    8,
                    5,
                    "BALL LOST",
                    color=image.COLOR_RED,
                    scale=1.4,
                    thickness=2,
                )

            # 每个处理帧只发送一次；丢检时发送 valid=0，不能沿用旧坐标。
            if uart_sender is not None:
                uart_send_ok = uart_sender.send(
                    uart_position_cm, uart_confidence
                )
                uart0_status = "OK" if uart_send_ok else "ERR"
                uart0_error = "" if uart_send_ok else uart_sender.last_error

            fps_text = "FPS:" + str(0 if loop_ms == 0 else 1000 // loop_ms)
            frame.draw_string(
                frame.width()
                - image.string_size(
                    fps_text, scale=1.4, thickness=2
                ).width(),
                5,
                fps_text,
                color=image.COLOR_GREEN,
                scale=1.4,
                thickness=2,
            )
            print_debug(
                "draw and get position cost "
                + str(time.ticks_ms() - stage_ms)
                + "ms"
            )

            # Preserve the original HTTP JPEG output: it receives YOLO
            # annotations but not the local touchscreen collection controls.
            if USE_JPEG:
                jpeg_server.write(frame)

            handle_capture_touch(
                touch,
                disp,
                frame,
                touch_state,
                capture_worker,
            )
            draw_capture_ui(frame, capture_worker)
            if is_maixcam2:
                draw_uart0_status(frame, uart0_status, uart0_error)
            disp.show(frame)
    finally:
        # STOP is synchronous here: queued clean frames get a bounded flush.
        capture_worker.close()
        if uart_sender is not None:
            try:
                uart_sender.close()
            except Exception as exc:
                print("UART0 close warning: {}".format(exc))
        if touch is not None:
            touch.close()
        cam.close()
        disp.close()
        gc.collect()


if __name__ == "__main__":
    main()
