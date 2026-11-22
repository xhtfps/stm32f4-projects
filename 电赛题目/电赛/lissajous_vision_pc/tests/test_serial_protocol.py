import unittest

from lissajous_vision.serial_protocol import (
    AppliedFrame,
    ErrorFrame,
    FrameParser,
    SetCommand,
    StartFrame,
    encode_done,
    encode_failed,
    encode_locked,
    encode_set,
    encode_run,
    encode_stop,
)


class SerialProtocolTests(unittest.TestCase):
    def test_run_frame_selects_one_of_three_target_shapes(self) -> None:
        self.assertEqual(encode_run(1), b"$RUN,1\r\n")
        self.assertEqual(encode_run(3), b"$RUN,3\r\n")
        with self.assertRaises(ValueError):
            encode_run(4)

    def test_encodes_all_pc_frames(self):
        command = SetCommand(7, 9, 1_000_000, 37_000, 471, 2)
        self.assertEqual(
            b"$SET,7,9,1000000,37000,471,2\r\n",
            encode_set(command),
        )
        self.assertEqual(b"$LOCKED,7,12300000\r\n", encode_locked(7, 12_300_000))
        self.assertEqual(b"$DONE,8,12300000\r\n", encode_done(8, 12_300_000))
        self.assertEqual(b"$FAILED,9,4\r\n", encode_failed(9, 4))
        self.assertEqual(b"$STOP,10\r\n", encode_stop(10))

    def test_fragmented_and_joined_frames(self):
        parser = FrameParser()
        self.assertEqual([], parser.feed(b"debug$APPLIED,7"))
        self.assertEqual(
            [AppliedFrame(sequence=7, probe_id=9), StartFrame(target_shape=2)],
            parser.feed(b",9\r\n$START,2\r\n"),
        )
        self.assertEqual([], parser.feed(b""))

    def test_error_frame(self):
        parser = FrameParser()
        self.assertEqual(
            [ErrorFrame(sequence=4, probe_id=12, error_code=3)],
            parser.feed(b"$ERROR,4,12,3\n"),
        )

    def test_new_dollar_recovers_from_truncated_frame(self):
        parser = FrameParser()
        frames = parser.feed(b"$APPLIED,1$APPLIED,2,3\r\n")
        self.assertEqual([AppliedFrame(sequence=2, probe_id=3)], frames)

    def test_rejects_bad_counts_non_decimal_and_ranges(self):
        parser = FrameParser()
        data = (
            b"$START,0\r\n"
            b"$START,4\r\n"
            b"$START,+1\r\n"
            b"$APPLIED,1\r\n"
            b"$APPLIED,65536,1\r\n"
            b"$ERROR,1,2,256\r\n"
            b"$UNKNOWN,1\r\n"
        )
        self.assertEqual([], parser.feed(data))
        self.assertEqual(7, parser.error_count)

    def test_overlong_frame_is_discarded_and_parser_recovers(self):
        parser = FrameParser(max_line_bytes=24)
        frames = parser.feed(b"$" + b"9" * 40 + b"\r\n$START,3\r\n")
        self.assertEqual([StartFrame(target_shape=3)], frames)
        self.assertEqual(1, parser.error_count)

    def test_outbound_ranges_are_checked(self):
        invalid = (
            SetCommand(0, 0, 999, 0, 100, 1),
            SetCommand(0, 0, 200_000_001, 0, 100, 1),
            SetCommand(0, 0, 1_000_000, 360_000, 100, 1),
            SetCommand(0, 0, 1_000_000, 0, 1001, 1),
            SetCommand(0, 0, 1_000_000, 0, 100, 0),
        )
        for command in invalid:
            with self.subTest(command=command), self.assertRaises(ValueError):
                encode_set(command)
        with self.assertRaises(ValueError):
            encode_locked(1, 999_999)
        with self.assertRaises(ValueError):
            encode_done(1, 100_000_001)


if __name__ == "__main__":
    unittest.main()
