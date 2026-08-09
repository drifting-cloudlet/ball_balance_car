"""Hardware-independent geometry and filtering for steel-ball tracking."""

import math

POSITION_CALIBRATION_POINTS_CM = (
    (-9.36, -10.0),
    (-4.65, -5.0),
    (0.0, 0.0),
    (4.17, 5.0),
    (8.25, 10.0),
)


def project_onto_axis(point, axis_start, axis_end):
    """Return normalized position and perpendicular distance to an image axis."""
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
    """Return the image-space point at *ratio* along an axis."""
    x0, y0 = axis_start
    x1, y1 = axis_end
    return (
        x0 + ratio * (x1 - x0),
        y0 + ratio * (y1 - y0),
    )


def position_from_axis_ratio(
    ratio, start_position_cm, end_position_cm, zero_ratio=0.5
):
    """Convert an axis ratio using an explicit physical zero point."""
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
    """Project an image point onto the beam and convert it to centimeters."""
    ratio, distance = project_onto_axis(point, axis_start, axis_end)
    position_cm = position_from_axis_ratio(
        ratio, start_position_cm, end_position_cm, zero_ratio
    )
    return position_cm, ratio, distance


class AdaptiveAlphaBetaFilter:
    """Velocity-aware position filter with adaptive measurement gain."""

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
    """Validate calibration against the detector image dimensions."""
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
