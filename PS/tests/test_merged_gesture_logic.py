"""Compile and execute the actual merged C++ helpers without board hardware."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which("g++"), "requires Linux host g++")
class MergedGestureLogicTests(unittest.TestCase):
    def test_pose_wrist_validation_and_held_gesture_rearming(self):
        source = (ROOT / "src" / "sdicamera+yolov8-pose+yolov8-face+yolov8-finger+hdmi.cpp").read_text(encoding="utf-8")
        helpers = source[source.index("std::vector<finger_udp::Detection> selectPoseWristHands("):source.index("int main(")]
        harness = r'''
#include <cassert>
#include <cmath>
#include <limits>
#include <optional>
#include "motion_udp_sender.hpp"
#include "finger_udp_sender.hpp"
'''+helpers+r'''
int main() {
    assert(selectPoseWristHands(std::nullopt).empty());
    motion_udp::Detection body;
    body.keypoints.assign(17, {0.0f, 0.0f, 0.0f});
    body.keypoints[9] = {0.2f, 0.3f, 0.9f};
    body.keypoints[10] = {0.8f, 0.3f, 0.8f};
    auto hands = selectPoseWristHands(body);
    assert(hands.size() == 2 && hands[0].keypoints.size() == 21);
    assert(hands[0].keypoints[0] == body.keypoints[9]);
    assert(hands[1].keypoints[20] == body.keypoints[10]);
    body.keypoints[10][2] = 0.29f;
    assert(selectPoseWristHands(body).size() == 1);
    body.keypoints[9][0] = std::numeric_limits<float>::quiet_NaN();
    assert(selectPoseWristHands(body).empty());
    body.keypoints.resize(10);
    assert(selectPoseWristHands(body).empty());
    for (double on_len : {1.6, 2.5, 3.0}) {
        GesturePulse state;
        assert(!gesturePulse(state, false, 10.0, on_len, 0.4));
        assert(gesturePulse(state, true, 10.0, on_len, 0.4));
        assert(gesturePulse(state, true, 10.0 + on_len - 0.01, on_len, 0.4));
        assert(!gesturePulse(state, true, 10.0 + on_len + 0.01, on_len, 0.4));
        assert(!gesturePulse(state, true, 10.0 + on_len + 0.39, on_len, 0.4));
        assert(gesturePulse(state, true, 10.0 + on_len + 0.42, on_len, 0.4));
        assert(!gesturePulse(state, false, 20.0, on_len, 0.4));
        assert(!state.on && state.toggle_at == 0.0);
        assert(gesturePulse(state, true, 20.01, on_len, 0.4));
    }
}
'''
        with tempfile.TemporaryDirectory(prefix="merged-gesture-") as directory:
            path = Path(directory)
            cpp = path / "check.cpp"
            cpp.write_text(harness, encoding="utf-8")
            binary = path / "check"
            subprocess.run(["g++", "-std=c++17", "-O2", "-I", str(ROOT / "include"), str(cpp), "-o", str(binary), "-lz"], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
