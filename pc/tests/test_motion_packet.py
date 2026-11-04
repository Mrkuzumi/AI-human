import struct
import sys
import unittest
import zlib
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))

from motion_packet import (  # noqa: E402
    EMPTY_PACKET_BYTES,
    FACE_COUNT,
    FULL_PACKET_BYTES,
    POSE_COUNT,
    Detection,
    MotionFrame,
    pack_frame,
    unpack_frame,
)


def detection(count):
    points = tuple((index / max(count - 1, 1), 0.5, 0.9) for index in range(count))
    return Detection(0.95, (0.1, 0.2, 0.3, 0.4), points)


class MotionPacketTests(unittest.TestCase):
    def test_full_packet_round_trip_and_mtu_size(self):
        source = MotionFrame(
            sequence=123,
            capture_time_ns=1_000_000,
            emit_time_ns=2_500_000,
            body=detection(POSE_COUNT),
            face=detection(FACE_COUNT),
        )

        packet = pack_frame(source)
        restored = unpack_frame(packet)

        self.assertEqual(len(packet), FULL_PACKET_BYTES)
        self.assertEqual(FULL_PACKET_BYTES, 904)
        self.assertLess(FULL_PACKET_BYTES + 28, 1500)
        self.assertEqual(restored.sequence, source.sequence)
        self.assertEqual(restored.track_id, 1)
        self.assertEqual(len(restored.body.keypoints), POSE_COUNT)
        self.assertEqual(len(restored.face.keypoints), FACE_COUNT)
        self.assertAlmostEqual(restored.face.keypoints[-1][0], 1.0)

    def test_empty_packet_has_only_header_and_crc(self):
        packet = pack_frame(MotionFrame(0, 10, 20))
        self.assertEqual(len(packet), EMPTY_PACKET_BYTES)
        self.assertEqual(EMPTY_PACKET_BYTES, 48)
        self.assertIsNone(unpack_frame(packet).body)
        self.assertIsNone(unpack_frame(packet).face)

    def test_track_id_round_trip(self):
        source = MotionFrame(4, 10, 20, track_id=2, body=detection(POSE_COUNT))
        self.assertEqual(unpack_frame(pack_frame(source)).track_id, 2)

    def test_legacy_v1_packet_is_primary(self):
        payload = bytearray(struct.pack(
            "<4sBBHIIQQHHBBBB", b"DMV1", 1, 4, 40, 44, 3, 10, 20, 1920, 1080, 0, 0, 0, 0
        ))
        payload.extend(struct.pack("<I", zlib.crc32(payload) & 0xFFFFFFFF))
        self.assertEqual(unpack_frame(bytes(payload)).track_id, 1)

    def test_corruption_is_rejected(self):
        packet = bytearray(pack_frame(MotionFrame(1, 10, 20, body=detection(POSE_COUNT))))
        packet[50] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC32"):
            unpack_frame(bytes(packet))

    def test_wrong_keypoint_count_is_rejected_before_send(self):
        bad_body = detection(POSE_COUNT - 1)
        with self.assertRaisesRegex(ValueError, "17 keypoints"):
            pack_frame(MotionFrame(1, 10, 20, body=bad_body))

    def test_non_normalized_value_is_rejected(self):
        bad = Detection(0.9, (0.0, 0.0, 1.1, 0.5), detection(POSE_COUNT).keypoints)
        with self.assertRaisesRegex(ValueError, r"in \[0, 1\]"):
            pack_frame(MotionFrame(1, 10, 20, body=bad))

    def test_zero_track_id_is_rejected_before_send(self):
        with self.assertRaisesRegex(ValueError, "track_id"):
            pack_frame(MotionFrame(1, 10, 20, track_id=0))


if __name__ == "__main__":
    unittest.main()
