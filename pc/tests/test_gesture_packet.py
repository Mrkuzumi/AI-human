"""Tests for the GST1 board gesture-signal codec and the bridge-side glue."""

import struct
import sys
import unittest
import zlib
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))

from gesture_packet import (  # noqa: E402
    EMPTY_PACKET_BYTES,
    FLAG_METRIC_VALID,
    GESTURE_IDS,
    HEADER_BYTES,
    MAGIC,
    MAX_RECORDS,
    RECORD_BYTES,
    VERSION,
    GestureFrame,
    GestureRecord,
    gesture_name,
    pack_frame,
    unpack_frame,
)

from motion_bridge import board_gesture_edges, board_gesture_summary  # noqa: E402


def record(gesture_id=0, active=0, progress=0.0, metric=0.0, metric_valid=True):
    return GestureRecord(
        id=gesture_id,
        name=gesture_name(gesture_id),
        active=active,
        progress=progress,
        metric=metric,
        metric_valid=metric_valid,
    )


def frame(records=(), sequence=1, width=1920, height=1080):
    return GestureFrame(
        sequence=sequence,
        capture_time_ns=1_000,
        emit_time_ns=2_000,
        frame_width=width,
        frame_height=height,
        source_id=0,
        records=tuple(records),
    )


class GesturePacketTests(unittest.TestCase):
    def test_single_record_round_trip_and_size(self):
        source = frame((record(active=1, progress=1.0, metric=0.0),))
        data = pack_frame(source)
        # 48-byte header + one 12-byte record + CRC32.
        self.assertEqual(len(data), HEADER_BYTES + RECORD_BYTES + 4)
        self.assertEqual(len(data), 64)
        self.assertEqual(unpack_frame(data), source)

    def test_empty_frame_round_trip(self):
        source = frame(())
        data = pack_frame(source)
        self.assertEqual(len(data), EMPTY_PACKET_BYTES)
        self.assertEqual(unpack_frame(data), source)
        self.assertEqual(unpack_frame(data).records, ())

    def test_max_records_is_accepted_and_rejected_beyond(self):
        records = tuple(record(gesture_id=index) for index in range(MAX_RECORDS))
        data = pack_frame(frame(records))
        self.assertEqual(len(unpack_frame(data).records), MAX_RECORDS)
        with self.assertRaises(ValueError):
            pack_frame(frame(tuple(record(gesture_id=index) for index in range(MAX_RECORDS + 1))))

    def test_active_ids_and_metric_flag(self):
        source = frame((record(gesture_id=0, active=1), record(gesture_id=1, active=0)))
        parsed = unpack_frame(pack_frame(source))
        self.assertEqual(parsed.active_ids(), (0,))
        self.assertTrue(all(item.metric_valid for item in parsed.records))

        without_metric = frame((record(metric_valid=False),))
        parsed = unpack_frame(pack_frame(without_metric))
        self.assertFalse(parsed.records[0].metric_valid)

    def test_unknown_gesture_id_keeps_a_readable_name(self):
        parsed = unpack_frame(pack_frame(frame((record(gesture_id=999),))))
        self.assertEqual(parsed.records[0].name, "Gesture999")
        self.assertEqual(
            GESTURE_IDS,
            {0: "touch_head", 1: "touch_face", 2: "hug"},
        )
        self.assertEqual(gesture_name(0), "touch_head")
        self.assertEqual(gesture_name(1), "touch_face")
        self.assertEqual(gesture_name(2), "hug")
        self.assertEqual(gesture_name(999), "Gesture999")

    def test_rejects_wrong_magic(self):
        data = bytearray(pack_frame(frame(())))
        data[0:4] = b"XXXX"
        with self.assertRaises(ValueError):
            unpack_frame(bytes(data))

    def test_rejects_wrong_version(self):
        data = bytearray(pack_frame(frame(())))
        data[4] = VERSION + 1
        with self.assertRaises(ValueError):
            unpack_frame(bytes(data))
        self.assertEqual(MAGIC, b"GST1")

    def test_rejects_header_length_mismatch(self):
        data = bytearray(pack_frame(frame(())))
        struct.pack_into("<H", data, 6, HEADER_BYTES + 4)
        with self.assertRaises(ValueError):
            unpack_frame(bytes(data))

    def test_rejects_packet_length_mismatch(self):
        data = pack_frame(frame((record(),)))
        with self.assertRaises(ValueError):
            unpack_frame(data[:-1])

    def test_rejects_bad_crc(self):
        data = bytearray(pack_frame(frame((record(),))))
        data[-1] ^= 0xFF
        with self.assertRaises(ValueError):
            unpack_frame(bytes(data))

    def test_rejects_unknown_header_flags(self):
        data = bytearray(pack_frame(frame(())))
        data[5] = 0x80
        with self.assertRaises(ValueError):
            unpack_frame(bytes(data))

    def test_rejects_unknown_record_flags(self):
        data = bytearray(pack_frame(frame((record(metric_valid=False),))))
        data[HEADER_BYTES + 3] = 0x80
        # Repair the CRC so the flag check is what rejects the packet.
        struct.pack_into("<I", data, len(data) - 4, zlib.crc32(bytes(data[:-4])) & 0xFFFFFFFF)
        with self.assertRaises(ValueError):
            unpack_frame(bytes(data))

    def test_rejects_non_boolean_active(self):
        data = bytearray(pack_frame(frame((record(active=1),))))
        # Record layout: id u16, active u8, flags u8, progress f32, metric f32.
        data[HEADER_BYTES + 2] = 2
        struct.pack_into("<I", data, len(data) - 4, zlib.crc32(bytes(data[:-4])) & 0xFFFFFFFF)
        with self.assertRaises(ValueError):
            unpack_frame(bytes(data))

    def test_rejects_record_count_beyond_protocol(self):
        data = bytearray(pack_frame(frame(())))
        struct.pack_into("<H", data, 38, MAX_RECORDS + 1)
        struct.pack_into("<I", data, len(data) - 4, zlib.crc32(bytes(data[:-4])) & 0xFFFFFFFF)
        with self.assertRaises(ValueError):
            unpack_frame(bytes(data))

    def test_short_packet_is_rejected(self):
        with self.assertRaises(ValueError):
            unpack_frame(b"GST1")


class BoardGestureEdgeTests(unittest.TestCase):
    def test_rising_edge_is_reported_once(self):
        last = {}
        first = board_gesture_edges(frame((record(active=1),)), last)
        self.assertEqual([(name, active) for name, active, _ in first], [("touch_head", 1)])
        self.assertEqual(board_gesture_edges(frame((record(active=1),)), last), [])

    def test_falling_edge_is_reported(self):
        last = {}
        board_gesture_edges(frame((record(active=1),)), last)
        edges = board_gesture_edges(frame((record(active=0),)), last)
        self.assertEqual([(name, active) for name, active, _ in edges], [("touch_head", 0)])

    def test_stale_stream_releases_high_gestures(self):
        last = {}
        board_gesture_edges(frame((record(active=1),)), last)
        edges = board_gesture_edges(None, last)
        self.assertEqual([(name, active) for name, active, _ in edges], [("touch_head", 0)])
        self.assertEqual(board_gesture_edges(None, last), [])

    def test_stale_stream_releases_every_gesture(self):
        last = {}
        board_gesture_edges(
            frame((record(gesture_id=0, active=1), record(gesture_id=2, active=1))), last
        )
        edges = board_gesture_edges(None, last)
        self.assertEqual(
            sorted((name, active) for name, active, _ in edges),
            [("hug", 0), ("touch_head", 0)],
        )

    def test_multiple_gestures_tracked_independently(self):
        last = {}
        source = frame(
            (
                record(gesture_id=0, active=1),
                record(gesture_id=1, active=0),
                record(gesture_id=2, active=0),
            )
        )
        edges = board_gesture_edges(source, last)
        self.assertEqual([(name, active) for name, active, _ in edges], [("touch_head", 1)])

        source = frame(
            (
                record(gesture_id=0, active=0),
                record(gesture_id=1, active=1),
                record(gesture_id=2, active=1),
            )
        )
        edges = board_gesture_edges(source, last)
        self.assertEqual(
            sorted((name, active) for name, active, _ in edges),
            [("hug", 1), ("touch_face", 1), ("touch_head", 0)],
        )

    def test_inactive_first_frame_reports_no_edge(self):
        self.assertEqual(board_gesture_edges(frame((record(active=0),)), {}), [])

    def test_summary_lists_high_gestures(self):
        self.assertEqual(board_gesture_summary({}), "off")
        self.assertEqual(board_gesture_summary({"touch_head": 0}), "off")
        self.assertEqual(board_gesture_summary({"touch_head": 1}), "touch_head")
        self.assertEqual(
            board_gesture_summary({"touch_head": 1, "hug": 1}), "hug,touch_head"
        )

    def test_edge_carries_the_board_metrics(self):
        last = {}
        edges = board_gesture_edges(
            frame((record(active=1, progress=1.0, metric=0.25),)), last
        )
        self.assertEqual(len(edges), 1)
        _, active, carried = edges[0]
        self.assertEqual(active, 1)
        self.assertIsNotNone(carried)
        self.assertAlmostEqual(carried.progress, 1.0, places=5)
        self.assertAlmostEqual(carried.metric, 0.25, places=5)


if __name__ == "__main__":
    unittest.main()
