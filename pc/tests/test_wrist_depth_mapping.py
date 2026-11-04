"""Conservative RGB enhancements: real-rig chirality, FK and stale-data rejection."""
import json
import math
import unittest
from pathlib import Path

from test_finger_bridge import _body, _hand
from test_motion_bridge import _standing_pose, _pose3d, _arm_projection_error
from motion_bridge import (
    BodySolver, FingerSolver, quaternion_multiply as mul, quaternion_x as qx,
    quaternion_y as qy, quaternion_z as qz, quaternion_rotate as rotate,
    rgb_observation_matches, matched_arm_depth_pose,
)
from motion_packet import MotionFrame, Detection


def rig_hand(side):
    data = json.loads((Path(__file__).resolve().parents[1] / "docs/avatar_bindings_20261022.json").read_text())
    bones = {b["humanoid"]: b for b in data["bones"]}
    wrist = bones[side + "Hand"]["modelPosition"]
    points = [(0., 0., 0.)]
    for finger in ("Thumb", "Index", "Middle", "Ring", "Little"):
        chain = []
        for segment in ("Proximal", "Intermediate", "Distal"):
            p = bones[side + finger + segment]["modelPosition"]
            chain.append(tuple(p[a] - wrist[a] for a in "xyz"))
        chain.append(tuple(2 * chain[2][i] - chain[1][i] for i in range(3)))
        points.extend(chain)
    return points


def world_hand(side, rotation, mirror=True):
    points = [rotate(rotation, p) for p in rig_hand(side)]
    return {"world_landmarks": [((-1 if mirror else 1) * x, -y, -z) for x, y, z in points]}


class WristDepthMappingTests(unittest.TestCase):
    def test_actual_rig_thumb_edge_for_both_hands_and_both_mirror_modes(self):
        # Test every hand against measured index/little bones, not arbitrary normals.
        for mirror in (True, False):
            solver = FingerSolver(mirror=mirror, hand_orientation_only=True)
            for side in ("Left", "Right"):
                for roll in (-170, -90, 0, 90, 170):
                    for depth in (-65, 0, 65):
                        base = mul(qz(roll), qy(depth))
                        for twist in (-175, -90, 0, 90, 175):
                            target = mul(base, qx(twist))
                            angle = solver._wrist_twist(world_hand(side, target, mirror), base)
                            self.assertIsNotNone(angle)
                            error = (angle - twist + 180) % 360 - 180
                            self.assertLess(abs(error), .002, (mirror, side, roll, depth, twist, angle))

    def test_raised_arm_old_camera_up_reference_was_not_parent_relative(self):
        from rgb_pose_receiver import palm_normal
        base = qz(180)
        hand = world_hand("Left", base)
        hand["palm_normal"] = palm_normal(hand["world_landmarks"])
        solver = FingerSolver(hand_orientation_only=True)
        self.assertAlmostEqual(solver._wrist_twist(hand, base), 0, places=3)
        self.assertGreater(abs(solver._hand_twist("Left", hand)), 170)

    def test_orientation_only_preserves_every_finger_and_wrist_axis(self):
        base_solver = FingerSolver(smoothing=1)
        solver = FingerSolver(smoothing=1, hand_orientation_only=True)
        body, hand = _body(), _hand(.78, closed=True)
        body_bones = BodySolver(smoothing=1).solve(body)
        baseline = dict(base_solver.solve((hand,), body))
        parent = (0., 0., 0., 1.)
        rotations = dict(body_bones)
        for name in ("Hips", "Spine", "Chest", "UpperChest", "LeftShoulder", "LeftUpperArm", "LeftLowerArm"):
            parent = mul(parent, rotations[name])
        base = mul(parent, baseline["LeftHand"])
        rgb = world_hand("Left", mul(base, qx(115)))
        rgb.update(handedness="Right", score=.99, landmarks=tuple((x, y, 0.) for x, y, _ in hand.keypoints))
        result = dict(solver.solve((hand,), body, (rgb,), 1, body_bones=body_bones))
        self.assertTrue(solver.diagnostics["Left"]["wrist3d"]["valid"])
        self.assertAlmostEqual(solver.diagnostics["output_twists"]["Left"], 115, places=3)
        for name in baseline:
            if name not in ("LeftHand", "RightHand"):
                self.assertEqual(result[name], baseline[name], name)
        self.assertLess(math.dist(rotate(result["LeftHand"], (1, 0, 0)),
                                  rotate(baseline["LeftHand"], (1, 0, 0))), 1e-10)

    def test_unmatched_rgb_hand_never_falls_back_to_handedness_label(self):
        solver = FingerSolver(hand_orientation_only=True)
        rgb = dict(handedness="Left", score=.99, landmarks=((.1, .1, 0.),) * 21)
        self.assertEqual(solver._select_hand3d((rgb,), {"Left": _hand(.78)}, strict=True), {})
        # Crossing wrists with ambiguous identity must also be rejected.
        rgb["landmarks"] = ((.78, .70, 0.),) * 21
        self.assertEqual(solver._select_hand3d((rgb,), {"Left": _hand(.78), "Right": _hand(.78)}, strict=True), {})

    def test_no_rgb_enhancement_has_the_same_baseline_output(self):
        body = _standing_pose()
        a, b = BodySolver(), BodySolver(arm_depth_only=True)
        fa, fb = FingerSolver(), FingerSolver(hand_orientation_only=True)
        for _ in range(25):
            ba, bb = a.solve(body), b.solve(body)
            self.assertEqual(ba, bb)
            self.assertEqual(fa.solve((_hand(.78),), body), fb.solve((_hand(.78),), body, body_bones=bb))

    def test_depth_only_keeps_npu_projection_and_unrelated_bones(self):
        body = _standing_pose()
        baseline = dict(BodySolver(smoothing=1).solve(body))
        for forward in (-.5, .5):
            points = list(_pose3d(0))
            for shoulder, elbow, wrist in ((11, 12, 13), (14, 15, 16)):
                s = points[shoulder]
                points[elbow] = (s[0] + .1, s[1] + .15, s[2] - forward, .99)
                points[wrist] = (s[0] + .15, s[1] + .3, s[2] + forward, .99)
            solver = BodySolver(smoothing=1, arm_depth_only=True)
            result = dict(solver.solve(body, points))
            for name in baseline:
                if name not in ("LeftUpperArm", "LeftLowerArm", "RightUpperArm", "RightLowerArm"):
                    self.assertEqual(baseline[name], result[name], name)
            for side in ("Left", "Right"):
                self.assertLess(_arm_projection_error(result, body.keypoints, side), 1e-6)
                upper = rotate(result[side + "UpperArm"], (-1 if side == "Left" else 1, 0, 0))
                lower = rotate(mul(result[side + "UpperArm"], result[side + "LowerArm"]),
                               (-1 if side == "Left" else 1, 0, 0))
                self.assertGreater(upper[2] * forward, 0)
                self.assertLess(lower[2] * forward, 0)

    def test_capture_time_source_and_person_association(self):
        body = _standing_pose()
        frame = MotionFrame(999, 1_000_000_000, 1_000_000_000, body=body)
        obs = dict(sequence=1, capture_ns=900_000_000, source_id=frame.source_id, source_host="192.168.126.20")
        self.assertTrue(rgb_observation_matches(obs, frame, "192.168.126.20", 150))
        self.assertFalse(rgb_observation_matches(obs, frame, "192.168.126.21", 150))
        self.assertFalse(rgb_observation_matches(dict(obs, capture_ns=800_000_000), frame, "192.168.126.20", 150))
        self.assertFalse(rgb_observation_matches(dict(obs, source_id=frame.source_id + 1), frame, "192.168.126.20", 150))
        image = [(0., 0., 0., .99)] * 17
        for a, b in zip((5, 7, 9, 6, 8, 10), (11, 12, 13, 14, 15, 16)):
            image[b] = (*body.keypoints[a][:2], 0., .99)
        obs.update(points=_pose3d(0), image_points=image)
        self.assertIsNotNone(matched_arm_depth_pose(body, obs, 1920, 1080, .25))
        obs["image_points"] = [(p[0] + .3, *p[1:]) for p in image]
        self.assertIsNone(matched_arm_depth_pose(body, obs, 1920, 1080, .25))

    def test_one_occluded_arm_holds_then_falls_back_independently(self):
        body, pose = _standing_pose(), list(_pose3d(0))
        pose[12] = (*pose[12][:2], pose[11][2] - .3, .99)
        pose[15] = (*pose[15][:2], pose[14][2] + .3, .99)
        solver = BodySolver(smoothing=1, arm_depth_only=True)
        solver.solve(body, pose)
        before = solver.arm_depths
        self.assertGreater(abs(before[0] - 10), 10)
        pose[12] = (*pose[12][:3], 0.)
        for _ in range(12):
            solver.solve(body, pose)
            self.assertEqual(solver.arm_depths, before)
        solver.solve(body, pose)
        self.assertEqual(solver.arm_depths, (10., before[1]))
        solver.solve(body, None)
        self.assertEqual(solver.arm_depths, (10., before[1]))
        pose[12] = (float('nan'), *pose[12][1:])
        self.assertNotIn('LeftUpperArm', solver._camera_arm_depths(pose))


if __name__ == "__main__":
    unittest.main()
