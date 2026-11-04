"""Regenerate the Unity FK fixtures from the current Python solver (no board needed)."""
import json
import math
from pathlib import Path

from test_motion_bridge import _standing_pose
from motion_bridge import BodySolver
from motion_packet import Detection


def main():
    poses = []
    for depth in (0, 10, 60):
        for upper, lower in ((-155, 80), (-100, -80), (-30, -155), (0, 0),
                             (65, 155), (110, -80), (160, 80)):
            body = _standing_pose()
            points = list(body.keypoints)
            points[5], points[6] = (.62, .25, .99), (.38, .35, .99)
            segments = []
            for side, shoulder, elbow, wrist, sign in (("Left", 5, 7, 9, -1), ("Right", 6, 8, 10, 1)):
                for a, b, angle, start, end in (
                    (shoulder, elbow, upper, "UpperArm", "LowerArm"),
                    (elbow, wrist, upper + lower, "LowerArm", "Hand"),
                ):
                    radians = math.radians(angle)
                    x, y = sign * math.cos(radians), sign * math.sin(radians)
                    points[b] = (points[a][0] - .12 * x, points[a][1] - .12 * y, .99)
                    segments.append({"start": side + start, "end": side + end,
                                     "imageDirection": {"x": x, "y": y}})
            bones = BodySolver(smoothing=1, arm_forward=depth).solve(
                Detection(body.score, body.box, tuple(points)))
            poses.append({"bones": [{"name": name, "rotation": dict(zip("xyzw", rotation))}
                                     for name, rotation in bones], "segments": segments})
    output = Path(__file__).resolve().parents[1] / "unity/DigitalManAvatar/Assets/Editor/AvatarMappingCases.json"
    output.write_text(json.dumps({"poses": poses}, indent=2) + "\n", encoding="utf-8")
    print(f"Generated {len(poses)} poses: {output}")


if __name__ == "__main__":
    main()
