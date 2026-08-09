"""MaixCAM2 steel-ball position transport over UART0."""

import math


UART_DEVICE = "/dev/ttyS0"
UART_BAUD_RATE = 115200
UART_WIRE_MIN_DMM = -1250
UART_WIRE_MAX_DMM = 1250
UART0_LAST_OPEN_ERROR = ""


def xor_checksum(body):
    """Return the 8-bit XOR checksum used by the controller protocol."""
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return checksum


def serialize_ball_frame(seq, x_dmm, valid, confidence):
    """Serialize one ASCII steel-ball measurement frame."""
    if valid == 0:
        x_dmm = 0
        confidence = 0
    body = "B,{},{},{},{}".format(seq, x_dmm, valid, confidence)
    return "${}*{:02X}\r\n".format(body, xor_checksum(body))


class BallUart0Sender:
    """Encode measurements and write one complete frame to UART0."""

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
    """Open the MaixCAM2 system UART0 device without changing pinmux."""
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
