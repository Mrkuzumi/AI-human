"""Extend the real Unity rig checks with conservative RGB wrist/depth cases."""
import json
from pathlib import Path

from build_avatar_mapping_cases import main as build_baseline
from test_wrist_depth_mapping import world_hand
from test_finger_bridge import _hand
from test_motion_bridge import _standing_pose, _pose3d
from motion_packet import Detection
from motion_bridge import BodySolver, FingerSolver, quaternion_multiply as mul, quaternion_x as qx, quaternion_rotate as rotate


def main():
    build_baseline()
    output = Path(__file__).resolve().parents[1] / "unity/DigitalManAvatar/Assets/Editor/AvatarMappingCases.json"
    data = json.loads(output.read_text())
    for depth in (-.35, 0, .35):
        for twist in (-170, -90, 0, 90, 170):
            body = _standing_pose()
            pose = list(_pose3d(0))
            for shoulder, elbow, wrist in ((11, 12, 13), (14, 15, 16)):
                x, y, z, _ = pose[shoulder]
                pose[elbow] = x, y + .15, z - depth, .99
                pose[wrist] = x, y + .3, z + depth, .99
            bones = BodySolver(smoothing=1, arm_depth_only=True).solve(body, pose)
            rotations = dict(bones)
            hands, hands3d, edges, segments = [], [], [], []
            for side, wrist in (("Left", 9), ("Right", 10)):
                h = _hand(body.keypoints[wrist][0])
                offset = body.keypoints[wrist][1] - h.keypoints[0][1]
                h = Detection(h.score, h.box, tuple((x, y + offset, c) for x, y, c in h.keypoints))
                hands.append(h)
                local_hand = dict(FingerSolver(smoothing=1).solve((h,), body))[side + "Hand"]
                parent = (0., 0., 0., 1.)
                for name in ("Hips", "Spine", "Chest", "UpperChest", side + "Shoulder", side + "UpperArm", side + "LowerArm"):
                    parent = mul(parent, rotations[name])
                target = mul(mul(parent, local_hand), qx(twist))
                rgb = world_hand(side, target)
                rgb.update(handedness=side, score=.99, landmarks=tuple((x, y, 0.) for x, y, _ in h.keypoints))
                hands3d.append(rgb)
                edges.append(dict(side=side, direction=dict(zip("xyz", rotate(target, (0, 0, 1))))))
                for start, end in (("UpperArm", "LowerArm"), ("LowerArm", "Hand")):
                    segments.append(dict(start=side + start, end=side + end,
                                         imageDirection=dict(x=-1 if side == "Left" else 1, y=0)))
            fingers = FingerSolver(smoothing=1, hand_orientation_only=True).solve(hands, body, hands3d, 1, body_bones=bones)
            data["poses"].append(dict(bones=[dict(name=n, rotation=dict(zip("xyzw", q))) for n, q in bones + fingers],
                                      segments=segments, handEdges=edges))
    output.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    print("Generated", len(data["poses"]), "real-rig cases, including 30 index/little edge checks")


if __name__ == "__main__":
    main()
