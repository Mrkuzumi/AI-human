"""End-to-end smoke check for the head-pat signal bit.

Feeds synthetic DMV1 + FNG1 packets into the real motion_bridge.py process and
decodes the OSC traffic that the bridge sends toward Unity.  This is the only
check that proves the wiring inside run_bridge(), not just the detector logic.

Run directly:  python gesture_signal_smoke.py
Exit code 0 means the signal bit went high while the hand rested on the head
and back low after it left.
"""

from __future__ import annotations

import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REFERENCE = HERE.parent / "reference"
LOG_PATH = HERE / "gesture_smoke_bridge.log"
sys.path.insert(0, str(REFERENCE))

from finger_packet import FingerFrame, pack_finger_frame  # noqa: E402
from motion_packet import Detection, MotionFrame, pack_frame  # noqa: E402

DMV_PORT = 39640
FNG_PORT = 39642
VMC_PORT = 39589
FPS = 20.0
# Extra bridge flags, e.g. "--quiet" or "--gesture-verbose".
EXTRA_ARGS = sys.argv[1:]
DWELL_REPORT_AT = 2.2
RELEASE_REPORT_AT = 3.6

NOSE = (0.50, 0.40)
INSIDE_WRIST = (0.50, 0.35, 0.90)
INSIDE_PALM = (0.50, 0.32, 0.90)
FAR_WRIST = (0.50, 0.72, 0.90)
FAR_PALM = (0.50, 0.70, 0.90)


def motion(sequence: int) -> bytes:
    points = [(0.50, 0.50, 0.05)] * 17
    points[0] = (NOSE[0], NOSE[1], 0.90)
    points[3] = (0.55, 0.40, 0.90)
    points[4] = (0.45, 0.40, 0.90)
    # Vary the arms so follow mode produces changing bone rotations. Interaction
    # mode must ignore all of this and keep sending one fixed pose.
    import math

    wobble = 0.08 * math.sin(sequence * 0.3)
    points[5] = (0.40 + wobble, 0.45, 0.90)
    points[6] = (0.60 - wobble, 0.45, 0.90)
    points[7] = (0.35 + wobble, 0.60, 0.90)
    points[8] = (0.65 - wobble, 0.60, 0.90)
    points[9] = (0.30 + wobble, 0.75, 0.90)
    points[10] = (0.70 - wobble, 0.75, 0.90)
    points[11] = (0.45, 0.70, 0.90)
    points[12] = (0.55, 0.70, 0.90)
    points[13] = (0.45, 0.85, 0.90)
    points[14] = (0.55, 0.85, 0.90)
    points[15] = (0.45, 0.95, 0.90)
    points[16] = (0.55, 0.95, 0.90)
    face_points = [(0.50, 0.50, 0.05)] * 51
    face_points[13] = (NOSE[0], NOSE[1], 0.90)
    return pack_frame(
        MotionFrame(
            sequence=sequence,
            capture_time_ns=0,
            emit_time_ns=0,
            body=Detection(0.95, (0.30, 0.20, 0.40, 0.50), tuple(points)),
            face=Detection(0.95, (0.40, 0.30, 0.20, 0.20), tuple(face_points)),
        )
    )


def finger(sequence: int, inside: bool) -> bytes:
    wrist, palm = (INSIDE_WRIST, INSIDE_PALM) if inside else (FAR_WRIST, FAR_PALM)
    points = [(0.50, 0.50, 0.05)] * 21
    points[0] = wrist
    points[9] = palm
    return pack_finger_frame(
        FingerFrame(
            sequence=sequence,
            capture_time_ns=0,
            emit_time_ns=0,
            frame_width=1920,
            frame_height=1080,
            source_id=0,
            hands=(Detection(0.95, (0.40, 0.30, 0.20, 0.20), tuple(points)),),
        )
    )


def read_string(data: bytes, offset: int):
    end = data.index(b"\0", offset)
    value = data[offset:end].decode("utf-8")
    offset = end + 1
    offset += (-offset) % 4
    return value, offset


def decode_osc(data: bytes):
    address, offset = read_string(data, 0)
    tags, offset = read_string(data, offset)
    values = []
    for tag in tags[1:]:
        if tag == "s":
            value, offset = read_string(data, offset)
        elif tag == "i":
            value = struct.unpack_from(">i", data, offset)[0]
            offset += 4
        elif tag == "f":
            value = struct.unpack_from(">f", data, offset)[0]
            offset += 4
        else:
            raise ValueError("unsupported OSC tag: {}".format(tag))
        values.append(value)
    return address, values


def main() -> int:
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", VMC_PORT))
    listener.settimeout(0.05)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    with LOG_PATH.open("w", encoding="utf-8") as sink:
        child = subprocess.Popen(
            [
                sys.executable, str(REFERENCE / "motion_bridge.py"),
                "--bind", "127.0.0.1", "--port", str(DMV_PORT),
                "--finger-port", str(FNG_PORT),
                "--vmc-host", "127.0.0.1", "--vmc-port", str(VMC_PORT),
                "--gesture", "--gesture-dwell", "1.0",
                "--gesture-log", str(LOG_PATH.with_name("gesture_smoke.jsonl")),
            ] + EXTRA_ARGS,
            stdout=sink,
            stderr=subprocess.STDOUT,
            cwd=str(REFERENCE),
        )
        hits = []
        bone_values = {}
        modes = []
        try:
            time.sleep(1.0)  # let the bridge bind its sockets
            started = time.monotonic()
            sequence = 0
            while True:
                elapsed = time.monotonic() - started
                if elapsed > RELEASE_REPORT_AT:
                    break
                inside = elapsed < DWELL_REPORT_AT
                sender.sendto(motion(sequence), ("127.0.0.1", DMV_PORT))
                sender.sendto(finger(sequence, inside), ("127.0.0.1", FNG_PORT))
                sequence += 1
                drain = time.monotonic() + 1.0 / FPS
                while time.monotonic() < drain:
                    try:
                        data, _ = listener.recvfrom(4096)
                    except socket.timeout:
                        continue
                    address, values = decode_osc(data)
                    if address == "/Avatar/Gesture":
                        hits.append(tuple(values))
                        print("  OSC {} t={:.2f}s -> {}".format(address, elapsed, values))
                    elif address == "/Avatar/Mode":
                        modes.append(tuple(values))
                    elif address == "/VMC/Ext/Bone/Pos" and len(values) >= 8:
                        # Per-bone distinct rotations: a held pose gives exactly
                        # one value per bone, a following avatar gives many.
                        bone_values.setdefault(values[0], set()).add(
                            tuple(round(float(v), 4) for v in values[1:])
                        )
        finally:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
            listener.close()

    log = LOG_PATH.read_text(encoding="utf-8", errors="replace")
    print("\n--- bridge [GESTURE] lines ---")
    for line in log.splitlines():
        if "[GESTURE]" in line:
            print("  " + line)

    high = ("touch_head", 1) in hits
    low = ("touch_head", 0) in hits

    moving = sorted(name for name, seen in bone_values.items() if len(seen) > 1)
    print("\nbones received: {}".format(len(bone_values)))
    print("bones that changed: {} {}".format(len(moving), moving if moving else ""))
    print("/Avatar/Mode messages: {} {}".format(len(modes), sorted(set(modes))))
    print("signal went high: {}   signal went low: {}".format(high, low))
    crashed = "Traceback" in log
    print("bridge traceback: {}".format(crashed))
    if not high or not low or crashed:
        print("RESULT: FAIL")
        return 1
    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
