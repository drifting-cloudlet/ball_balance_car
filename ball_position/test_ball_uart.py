"""Host-side regression tests for the MaixCAM2 UART0 transport."""

import inspect
import sys
import types
import unittest
from unittest import mock

import ball_uart
import ball_position


class FakeSerial:
    def __init__(self):
        self.payloads = []
        self.closed = False

    def write(self, payload):
        self.payloads.append(payload)
        return len(payload)

    def close(self):
        self.closed = True


class ShortWriteSerial:
    def write(self, payload):
        return len(payload) - 1


class BallUartTests(unittest.TestCase):
    def test_reference_frame_is_exact(self):
        self.assertEqual(ball_uart.xor_checksum("B,42,-503,1,93"), 0x64)
        self.assertEqual(
            ball_uart.serialize_ball_frame(42, -503, 1, 93),
            "$B,42,-503,1,93*64\r\n",
        )

    def test_valid_and_lost_frames_match_reference_protocol(self):
        serial = FakeSerial()
        sender = ball_uart.BallUart0Sender(serial)
        sender.sequence = 42

        self.assertTrue(sender.send(-5.03, 0.93))
        self.assertEqual(serial.payloads[-1], b"$B,42,-503,1,93*64\r\n")

        self.assertTrue(sender.send(None, 0.0))
        self.assertEqual(
            serial.payloads[-1],
            ball_uart.serialize_ball_frame(43, 0, 0, 0).encode("ascii"),
        )

    def test_sequence_wraps_only_after_successful_write(self):
        sender = ball_uart.BallUart0Sender(FakeSerial())
        sender.sequence = 255
        self.assertTrue(sender.send(0.0, 0.8))
        self.assertEqual(sender.sequence, 0)

        sender = ball_uart.BallUart0Sender(ShortWriteSerial())
        self.assertFalse(sender.send(0.0, 0.8))
        self.assertEqual(sender.sequence, 0)

    def test_out_of_wire_range_becomes_invalid(self):
        serial = FakeSerial()
        sender = ball_uart.BallUart0Sender(serial)

        self.assertTrue(sender.send(12.51, 0.9))
        self.assertEqual(
            serial.payloads[-1],
            ball_uart.serialize_ball_frame(0, 0, 0, 0).encode("ascii"),
        )

    def test_per_frame_send_path_has_no_print(self):
        self.assertNotIn("print(", inspect.getsource(ball_uart.BallUart0Sender.send))

    def test_uart0_opens_without_pinmap(self):
        serial = FakeSerial()
        fake_maix = types.ModuleType("maix")
        fake_maix.uart = types.SimpleNamespace(
            UART=lambda device, baud: (
                serial
                if (device, baud) == ("/dev/ttyS0", 115200)
                else None
            )
        )

        with mock.patch.dict(sys.modules, {"maix": fake_maix}):
            sender = ball_uart.open_uart0_sender()

        self.assertIsNotNone(sender)
        self.assertEqual(ball_uart.UART_DEVICE, "/dev/ttyS0")
        self.assertEqual(ball_uart.UART_BAUD_RATE, 115200)
        self.assertIs(sender.serial_device, serial)
        sender.close()
        self.assertTrue(serial.closed)

    def test_uart0_open_failure_is_reported(self):
        fake_maix = types.ModuleType("maix")
        fake_maix.uart = types.SimpleNamespace(
            UART=mock.Mock(side_effect=RuntimeError("port busy"))
        )

        with mock.patch.dict(sys.modules, {"maix": fake_maix}):
            sender = ball_uart.open_uart0_sender()

        self.assertIsNone(sender)
        self.assertEqual(ball_uart.UART0_LAST_OPEN_ERROR, "port busy")


class BallPositionTests(unittest.TestCase):
    def test_five_point_physical_calibration(self):
        calibration_pairs = (
            (-9.36, -10.0),
            (-4.65, -5.0),
            (0.0, 0.0),
            (4.17, 5.0),
            (8.25, 10.0),
        )
        for measured_cm, actual_cm in calibration_pairs:
            self.assertAlmostEqual(
                ball_position.calibrate_position_cm(measured_cm), actual_cm
            )

    def test_explicit_zero_ratio_preserves_both_endpoints(self):
        axis_start = (5.0, 80.0)
        axis_end = (635.0, 80.0)
        zero_ratio = 0.464
        zero_x = axis_start[0] + zero_ratio * (axis_end[0] - axis_start[0])

        cases = (
            (axis_start, -12.5),
            ((zero_x, 80.0), 0.0),
            (axis_end, 12.5),
        )
        for point, expected in cases:
            position_cm, _, _ = ball_position.position_from_pixel(
                point,
                axis_start,
                axis_end,
                -12.5,
                12.5,
                zero_ratio,
            )
            self.assertAlmostEqual(position_cm, expected)

        self.assertAlmostEqual(zero_x, 297.32)

    def test_zero_ratio_must_be_inside_axis(self):
        with self.assertRaises(ValueError):
            ball_position.position_from_axis_ratio(0.5, -12.5, 12.5, 1.0)


if __name__ == "__main__":
    unittest.main()
