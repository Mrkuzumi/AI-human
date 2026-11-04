"""Offline board PCM -> real TCP relay -> mocked Tencent WebSocket checks."""

import contextlib
import io
import json
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "tools"))
sys.path.insert(0, str(PACKAGE / "pc_voice"))
import board_voice_stream as board
import stream_receiver as relay


class FakeCloud:
    def __init__(self):
        self.messages = queue.Queue()
        self.messages.put(json.dumps({"code": 0}))
        self.audio = bytearray()
        self.closed = False
        self.audio_received = threading.Event()

    def recv(self):
        return self.messages.get(timeout=5)

    def settimeout(self, value):
        pass

    def send_binary(self, data):
        self.audio.extend(data)
        self.audio_received.set()
        self.messages.put(json.dumps({"code": 0, "result": {
            "slice_type": 2, "voice_text_str": "你好，数字人。"}}))

    def send(self, data):
        if json.loads(data).get("type") == "end":
            self.messages.put(json.dumps({"code": 0, "final": 1}))

    def close(self):
        self.closed = True
        self.messages.put("")


class VoiceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="runtime-voice-")
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "voice.json"
        self.path.write_text(json.dumps(dict(
            app_id="123456", secret_id="test-id", secret_key="test-key",
            upload_token="test-token", server_port=8766, reconnect_seconds=0.1,
            audio_device="plughw:0,0")), encoding="utf-8")
        self.env = patch.dict(os.environ, {}, clear=True)
        self.env.start()
        self.addCleanup(self.env.stop)

    def start_relay(self, cloud):
        relay.StreamHandler.token = "test-token"
        relay.StreamHandler.app_id = "123456"
        relay.StreamHandler.secret_id = "test-id"
        relay.StreamHandler.secret_key = "test-key"
        server = relay.StreamServer(("127.0.0.1", 0), relay.StreamHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        cloud_patch = patch.object(relay.websocket, "create_connection", return_value=cloud)
        cloud_patch.start()
        thread.start()
        def cleanup():
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
            cloud_patch.stop()
        self.addCleanup(cleanup)
        return server.server_address

    def test_environment_overrides_pc_and_board_config(self):
        with patch.dict(os.environ, {
            "TENCENTCLOUD_SECRET_KEY": "new-key", "VOICE_UPLOAD_TOKEN": "中文口令",
            "VOICE_PORT": "8877", "AUDIO_DEVICE": "plughw:1,0",
        }):
            self.assertEqual(relay.load_config(self.path)["secret_key"], "new-key")
            config = board.load_config(self.path)
            self.assertEqual(config["upload_token"], "中文口令")
            self.assertEqual(config["server_port"], 8877)
            self.assertEqual(config["audio_device"], "plughw:1,0")

    def test_rejects_missing_keys_bad_token_port_and_enabled(self):
        data = json.loads(self.path.read_text())
        self.path.write_text("[]")
        with self.assertRaises(ValueError):
            relay.load_config(self.path)
        for field, value in (("secret_key", ""), ("upload_token", "bad\nvalue"), ("app_id", "bad")):
            with self.subTest(pc=field):
                changed = dict(data, **{field: value})
                self.path.write_text(json.dumps(changed))
                with self.assertRaises(ValueError):
                    relay.load_config(self.path)
        for field, value in (("upload_token", ""), ("server_port", 70000),
                             ("reconnect_seconds", 0), ("enabled", "yes")):
            with self.subTest(board=field):
                self.path.write_text(json.dumps(dict(data, **{field: value})))
                with self.assertRaises(ValueError):
                    board.load_config(self.path)

    def test_rejects_wrong_upload_token_without_cloud_call(self):
        cloud = FakeCloud()
        host, port = self.start_relay(cloud)
        with socket.create_connection((host, port), timeout=2) as client:
            client.sendall(b"wrong-token\n")
            self.assertIn(b"Unauthorized", client.recv(4096))
        self.assertFalse(cloud.audio)

    def test_final_result_is_drained_after_pcm_eof(self):
        cloud = FakeCloud()
        host, port = self.start_relay(cloud)
        with socket.create_connection((host, port), timeout=3) as client:
            reader = client.makefile("rb")
            self.addCleanup(reader.close)
            client.sendall(b"test-token\n")
            self.assertIn("已连接", reader.readline().decode())
            client.sendall(b"\0" * board.CHUNK_BYTES)
            client.shutdown(socket.SHUT_WR)
            lines = reader.read().decode()
            self.assertIn("[最终] 你好，数字人。", lines)
            self.assertEqual(len(cloud.audio), board.CHUNK_BYTES)

    def test_real_tcp_board_capture_results_and_recorder_cleanup(self):
        cloud = FakeCloud()
        host, port = self.start_relay(cloud)
        config = board.load_config(self.path)
        config["server_port"] = port
        client = board.VoiceClient(host, config)
        recorder = subprocess.Popen([
            sys.executable, "-u", "-c",
            "import sys,time; sys.stdout.buffer.write(b'\\x01\\x00'*6400); "
            "sys.stdout.buffer.flush(); time.sleep(30)",
        ], stdout=subprocess.PIPE)
        output = io.StringIO()
        with patch.object(board.subprocess, "Popen", return_value=recorder), contextlib.redirect_stdout(output):
            thread = threading.Thread(target=client.run, daemon=True)
            thread.start()
            try:
                self.assertTrue(cloud.audio_received.wait(4))
                deadline = time.monotonic() + 3
                while "[最终]" not in output.getvalue() and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertIn("[最终] 你好，数字人。", output.getvalue())
                client.stop()
                thread.join(timeout=4)
                self.assertFalse(thread.is_alive())
                self.assertIsNotNone(recorder.poll())
                self.assertEqual(cloud.audio[:board.CHUNK_BYTES], b"\x01\x00" * 3200)
            finally:
                client.stop()
                if recorder.poll() is None:
                    recorder.kill()
                recorder.wait(timeout=3)
                thread.join(timeout=4)

    def test_connection_failure_retries_and_stop_interrupts_delay(self):
        config = board.load_config(self.path)
        client = board.VoiceClient("127.0.0.1", config)
        attempts = threading.Event()
        calls = []
        def unavailable():
            calls.append(1)
            if len(calls) >= 2:
                attempts.set()
            raise OSError("offline")
        with patch.object(client, "stream_once", side_effect=unavailable):
            thread = threading.Thread(target=client.run, daemon=True)
            thread.start()
            try:
                self.assertTrue(attempts.wait(2))
            finally:
                client.stop()
                thread.join(timeout=2)
            self.assertFalse(thread.is_alive())
            self.assertGreaterEqual(len(calls), 2)

    def test_provider_error_and_empty_responses(self):
        self.assertIsNone(relay.parse_result('{"code":0}'))
        self.assertEqual(relay.parse_result('{"code":0,"result":{"slice_type":1,"voice_text_str":"你"}}'), ("你", False))
        with self.assertRaisesRegex(RuntimeError, "腾讯云错误"):
            relay.parse_result('{"code":4001,"message":"test denied"}')

    def test_disabled_capture_does_not_require_arecord(self):
        data = json.loads(self.path.read_text())
        data["enabled"] = False
        self.path.write_text(json.dumps(data))
        with patch.object(sys, "argv", ["board_voice_stream.py", "--config", str(self.path)]), patch.object(board.shutil, "which", return_value=None):
            board.main()

    def test_board_and_full_are_synced_and_board_has_no_cloud_credentials(self):
        sibling = PACKAGE.with_name(PACKAGE.name.replace("_full", "_board"))
        for relative in ("tools/board_voice_stream.py", "configs/voice.json", "scripts/run_board.sh", "CLOUD_VOICE.md"):
            self.assertEqual((PACKAGE / relative).read_bytes(), (sibling / relative).read_bytes(), relative)
        self.assertFalse((sibling / "pc_voice" / "voice.local.json").exists())
        self.assertNotIn("secret_key", (sibling / "configs" / "voice.json").read_text())


if __name__ == "__main__":
    unittest.main()
