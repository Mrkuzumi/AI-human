"""UDP validation and the real bridge's camera-independent mode control."""

import json
import socket
import struct
import subprocess
import sys
import time
import unittest
from pathlib import Path

REFERENCE = Path(__file__).resolve().parents[1] / "reference"
sys.path.insert(0, str(REFERENCE))
from mode_control import ModeControlReceiver
from motion_bridge import VmcSender
from motion_packet import Detection, MotionFrame, pack_frame
from gesture_packet import GestureFrame, GestureRecord, pack_frame as pack_gesture


def request(seq=1, mode="interact", **extra):
    value = dict(v=1, kind="set", session="a" * 32, seq=seq, mode=mode)
    value.update(extra)
    return json.dumps(value).encode("ascii")


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def decode_osc(packet):
    def string(offset):
        end = packet.index(b"\0", offset)
        return packet[offset:end].decode("utf-8"), (end + 4) & ~3
    address, offset = string(0)
    tags, offset = string(offset)
    values = []
    for tag in tags[1:]:
        if tag == "s":
            value, offset = string(offset)
        else:
            value = struct.unpack_from(">i" if tag == "i" else ">f", packet, offset)[0]
            offset += 4
        values.append(value)
    return address, values


class ModeControlTests(unittest.TestCase):
    def setUp(self):
        self.receiver = ModeControlReceiver("127.0.0.1", 0)
        self.addCleanup(self.receiver.close)
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client.settimeout(0.5)
        self.addCleanup(self.client.close)
        self.destination = self.receiver.socket.getsockname()
        self.applied = []

    def apply(self, mode):
        self.applied.append(mode)
        return mode

    def send(self, packet):
        self.client.sendto(packet, self.destination)
        # sendto enqueues loopback data before poll; no background threads needed.
        self.receiver.poll(self.apply)

    def test_ack_contains_applied_mode_and_request_identity(self):
        self.send(request())
        ack = json.loads(self.client.recv(256))
        self.assertEqual(ack, dict(v=1, kind="ack", session="a" * 32, seq=1, mode="interact"))
        self.assertEqual(self.applied, ["interact"])

    def test_reordered_or_conflicting_requests_do_not_change_mode(self):
        self.send(request(seq=2))
        self.send(request(seq=1, mode="follow"))
        self.send(request(seq=2, mode="follow"))
        self.assertEqual(self.applied, ["interact"])
        self.send(request(seq=2))
        self.assertEqual(self.applied, ["interact"])

    def test_invalid_packets_are_ignored(self):
        invalid = [b"broken", b"[]", b"null", b"\xff", b"x" * 300,
                   request(mode="toggle"), request(seq=True), request(seq=0),
                   request(v=True), request(kind="ack"), request(session="not-a-session"),
                   request(seq=2147483648), request(mode={"follow": 1})]
        for packet in invalid:
            self.send(packet)
        self.assertEqual(self.applied, [])

    def test_old_button_heartbeat_does_not_undo_voice_and_new_click_wins(self):
        self.send(request(seq=1, mode="interact"))
        self.send(request(seq=1, mode="follow", session="b"*32))
        self.send(request(seq=1, mode="interact"))
        self.assertEqual(self.receiver.mode, "follow")
        self.assertEqual(self.applied, ["interact", "follow"])
        self.send(request(seq=2, mode="interact"))
        self.assertEqual(self.receiver.mode, "interact")
        self.client.sendto(request(kind="get", mode=None), self.destination)
        self.receiver.poll(self.apply)
        responses = [json.loads(self.client.recv(256)) for _ in range(5)]
        self.assertEqual(responses[-1]["kind"], "state")
        self.assertEqual(responses[-1]["mode"], "interact")
        self.assertEqual(responses[-1]["revision"], 3)

    def test_allowed_host_rejects_other_sources(self):
        self.receiver.allowed_host = "192.0.2.1"
        self.client.bind(("127.0.0.2", 0))
        self.send(request())
        self.assertEqual(self.applied, [])
        self.send(request(kind="get", mode=None))
        with self.assertRaises(socket.timeout):
            self.client.recv(256)

    def test_board_can_read_authoritative_mode_without_changing_it(self):
        self.receiver.allowed_host = "127.0.0.2"
        self.client.bind(("127.0.0.2", 0))
        self.send(request(kind="get", mode=None, seq=7))
        state = json.loads(self.client.recv(256))
        self.assertEqual((state["kind"], state["mode"], state["seq"]), ("state", "follow", 7))
        self.assertEqual(self.applied, [])

    def test_session_cache_is_bounded(self):
        for index in range(24):
            self.send(request(session="{:032x}".format(index)))
        self.assertEqual(len(self.receiver.sequences), 16)

    def test_vmc_mode_changes_are_immediate_and_heartbeats_do_not_spam(self):
        sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(sink.close)
        sink.bind(("127.0.0.1", 0))
        sink.settimeout(0.1)
        sender = VmcSender(*sink.getsockname())
        self.addCleanup(sender.socket.close)
        sender.send_mode("follow")
        sender.send_mode("interact")
        sender.send_mode("interact")
        self.assertEqual(decode_osc(sink.recv(2048)), ("/Avatar/Mode", ["follow"]))
        self.assertEqual(decode_osc(sink.recv(2048)), ("/Avatar/Mode", ["interact"]))
        with self.assertRaises(socket.timeout):
            sink.recv(2048)


class RunningBridgeTests(unittest.TestCase):
    def setUp(self):
        self.sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sink.bind(("127.0.0.1", 0))
        self.sink.settimeout(0.1)
        self.addCleanup(self.sink.close)
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client.settimeout(1.0)
        self.addCleanup(self.client.close)
        self.motion_port, self.gesture_port, self.control_port = free_port(), free_port(), free_port()
        self.process = subprocess.Popen([
            sys.executable, "-B", "-u", str(REFERENCE / "motion_bridge.py"),
            "--bind", "127.0.0.1", "--port", str(self.motion_port), "--no-fingers",
            "--gesture-port", str(self.gesture_port), "--mode-control-port", str(self.control_port),
            "--vmc-host", "127.0.0.1", "--vmc-port", str(self.sink.getsockname()[1]), "--quiet",
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.addCleanup(self.stop)
        self.wait_for(lambda a, v: a == "/Avatar/Mode" and v == ["follow"])

    def stop(self):
        self.process.terminate()
        try:
            self.output = self.process.communicate(timeout=5)[0].decode("utf-8", errors="replace")
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.output = self.process.communicate()[0].decode("utf-8", errors="replace")

    def wait_for(self, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                address, values = decode_osc(self.sink.recv(2048))
            except socket.timeout:
                if self.process.poll() is not None:
                    self.fail("bridge exited: " + self.process.stdout.read().decode(errors="replace"))
                continue
            if predicate(address, values):
                return values
        self.fail("expected OSC not received")

    def set_mode(self, mode, seq):
        self.client.sendto(request(seq=seq, mode=mode), ("127.0.0.1", self.control_port))
        ack = json.loads(self.client.recv(256))
        self.assertEqual((ack["seq"], ack["mode"]), (seq, mode))

    def test_switch_without_camera_and_repeated_requests(self):
        self.set_mode("interact", 1)
        self.wait_for(lambda a, v: a == "/Avatar/Mode" and v == ["interact"])
        self.set_mode("interact", 1)
        self.set_mode("follow", 2)
        self.wait_for(lambda a, v: a == "/Avatar/Mode" and v == ["follow"])
        # Resync continues even when the board camera has stopped completely.
        self.wait_for(lambda a, v: a == "/Avatar/Mode" and v == ["follow"])

    def test_gesture_stale_release_without_motion_frames(self):
        self.set_mode("interact", 1)
        frame = GestureFrame(sequence=1, capture_time_ns=1, emit_time_ns=1,
                             frame_width=1920, frame_height=1080, source_id=0,
                             records=(GestureRecord(0, "touch_head", 1, 1.0, 0.0, True),))
        self.client.sendto(pack_gesture(frame), ("127.0.0.1", self.gesture_port))
        self.wait_for(lambda a, v: a == "/Avatar/Gesture" and v == ["touch_head", 1])
        self.wait_for(lambda a, v: a == "/Avatar/Gesture" and v == ["touch_head", 0])

    def test_live_mode_changes_switch_neutral_and_solved_bones(self):
        points = [(0.5, 0.5, 0.9)] * 17
        points[5:11] = [(0.6, 0.3, 0.9), (0.4, 0.3, 0.9), (0.8, 0.3, 0.9),
                        (0.2, 0.3, 0.9), (0.9, 0.3, 0.9), (0.1, 0.3, 0.9)]
        body = Detection(0.9, (0.1, 0.1, 0.8, 0.8), tuple(points))
        frame = MotionFrame(sequence=1, capture_time_ns=0, emit_time_ns=0, body=body, face=None)
        def arm():
            self.client.sendto(pack_frame(frame), ("127.0.0.1", self.motion_port))
            values = self.wait_for(lambda a, v: a == "/VMC/Ext/Bone/Pos" and v[0] == "LeftUpperArm")
            # Drain the rest of this frame before requesting another one.
            self.wait_for(lambda a, v: a == "/VMC/Ext/T")
            return values[4:]
        self.set_mode("interact", 1)
        self.wait_for(lambda a, v: a == "/Avatar/Mode" and v == ["interact"])
        neutral = arm()
        self.set_mode("follow", 2)
        self.wait_for(lambda a, v: a == "/Avatar/Mode" and v == ["follow"])
        self.assertNotEqual(arm(), neutral)


if __name__ == "__main__":
    unittest.main()
