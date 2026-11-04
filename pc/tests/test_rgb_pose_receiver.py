import struct
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reference"))

from rgb_pose_receiver import (  # noqa: E402
    HEADER_BYTES,
    JpegFrameReassembler,
    palm_normal,
    world_landmarks_to_h36m,
)


def _packet(sequence, frame, part, count, capture_ns=123, source=0):
    chunk = (len(frame) + count - 1) // count
    payload = frame[part * chunk : (part + 1) * chunk]
    return struct.pack(
        "<4sBBHIIHHQ", b"DMI1", 1, source, HEADER_BYTES, sequence, len(frame), part, count, capture_ns
    ) + payload


class JpegFrameReassemblerTests(unittest.TestCase):
    def test_reassembles_out_of_order_fragments(self):
        jpeg = b"\xff\xd8" + b"pixels" * 500 + b"\xff\xd9"
        receiver = JpegFrameReassembler()
        self.assertIsNone(receiver.push(_packet(7, jpeg, 1, 3), now=1.0))
        self.assertIsNone(receiver.push(_packet(7, jpeg, 0, 3), now=1.1))
        sequence, capture_ns, decoded = receiver.push(_packet(7, jpeg, 2, 3), now=1.2)
        self.assertEqual((sequence, capture_ns, decoded), (7, 123, jpeg))

    def test_rejects_inconsistent_frame_metadata(self):
        jpeg = b"\xff\xd8abcdef"
        receiver = JpegFrameReassembler()
        self.assertIsNone(receiver.push(_packet(1, jpeg, 0, 2), now=1.0))
        changed = bytearray(_packet(1, jpeg, 1, 2))
        struct.pack_into("<I", changed, 12, len(jpeg) + 1)
        self.assertIsNone(receiver.push(bytes(changed), now=1.1))


class LandmarkMappingTests(unittest.TestCase):
    def test_maps_mediapipe_arms_to_h36m_slots(self):
        points = [(float(index), index + 0.1, index + 0.2) for index in range(33)]
        pose = world_landmarks_to_h36m(points)
        self.assertEqual(len(pose), 17)
        self.assertEqual(pose[11], points[11])
        self.assertEqual(pose[12], points[13])
        self.assertEqual(pose[13], points[15])
        self.assertEqual(pose[14], points[12])
        self.assertEqual(pose[15], points[14])
        self.assertEqual(pose[16], points[16])

    def test_preserves_visibility_for_body_turn_confidence(self):
        points = [(float(index), index + 0.1, index + 0.2, index / 100.0) for index in range(33)]
        pose = world_landmarks_to_h36m(points)
        self.assertEqual(pose[11], points[11])
        self.assertEqual(pose[14], points[12])
        self.assertEqual(pose[0][3], min(points[23][3], points[24][3]))

    def test_computes_unit_palm_normal(self):
        points = [(0.0, 0.0, 0.0)] * 21
        points[5] = (1.0, 0.0, 0.0)
        points[17] = (0.0, 2.0, 0.0)
        self.assertEqual(palm_normal(points), (0.0, 0.0, 1.0))

    def test_returns_zero_for_collapsed_palm(self):
        self.assertEqual(palm_normal([(0.0, 0.0, 0.0)] * 21), (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
