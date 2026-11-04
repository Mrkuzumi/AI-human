"""Exercise turning through the real imported rig, keeping the established checks."""
import json
from pathlib import Path

from build_wrist_depth_cases import main as build_previous
from test_body_turn_mapping import board_pose, solver
from test_motion_bridge import _pose3d
from test_finger_bridge import _hand
from test_wrist_depth_mapping import world_hand
from motion_packet import Detection
from motion_bridge import FingerSolver, quaternion_multiply as mul, quaternion_x as qx, quaternion_rotate as rotate


def main():
    build_previous()
    output = Path(__file__).resolve().parents[1] / "unity/DigitalManAvatar/Assets/Editor/AvatarMappingCases.json"
    data = json.loads(output.read_text())
    for yaw in (0, 45, 80, 100, 135, 179, -135, -80):
        for arm in ("down", "up"):
            pose = _pose3d(yaw, left_arm=arm)
            body = board_pose(pose)
            bones = solver().solve(body, pose, turn_pose3d=pose)
            rotations = dict(bones)
            hands, rgb_hands, edges, segments = [], [], [], []
            for side, indices in (("Left", (5, 7, 9)), ("Right", (6, 8, 10))):
                h = _hand(body.keypoints[indices[2]][0])
                offset = body.keypoints[indices[2]][1] - h.keypoints[0][1]
                h = Detection(h.score, h.box, tuple((x, y+offset, c) for x, y, c in h.keypoints))
                hands.append(h)
                local_hand = dict(FingerSolver(smoothing=1).solve((h,), body))[side+"Hand"]
                parent = (0, 0, 0, 1)
                for name in ("Hips", "Spine", "Chest", "UpperChest", side+"Shoulder", side+"UpperArm", side+"LowerArm"):
                    parent = mul(parent, rotations[name])
                target = mul(mul(parent, local_hand), qx(80))
                rgb = world_hand(side, target)
                rgb.update(handedness=side, score=.99, landmarks=[(x, y, 0) for x, y, c in h.keypoints])
                rgb_hands.append(rgb)
                edges.append(dict(side=side, direction=dict(zip("xyz", rotate(target, (0, 0, 1))))))
                for a, b, start, end in ((indices[0], indices[1], "UpperArm", "LowerArm"),
                                          (indices[1], indices[2], "LowerArm", "Hand")):
                    first, second = body.keypoints[a], body.keypoints[b]
                    segments.append(dict(start=side+start, end=side+end,
                                         imageDirection=dict(x=first[0]-second[0], y=first[1]-second[1])))
            fingers = FingerSolver(smoothing=1, hand_orientation_only=True).solve(
                hands, body, rgb_hands, 1, body_bones=bones)
            data["poses"].append(dict(bones=[dict(name=n, rotation=dict(zip("xyzw", q))) for n, q in bones+fingers],
                                      segments=segments, handEdges=edges))
    output.write_text(json.dumps(data, indent=2)+"\n", encoding="utf-8")
    print("Generated", len(data["poses"]), "real-rig cases with torso turns and wrist edges")


if __name__ == "__main__":
    main()
