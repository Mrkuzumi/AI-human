"""Local UDP playback ownership checks; no cloud calls or audible audio."""
import contextlib
import json
from pathlib import Path
import socket
import threading
import time
import unittest
from unittest.mock import patch

from pc_speech import Speaker, UnitySpeech
from test_dialogue import wav_audio


@contextlib.contextmanager
def renderer(respond):
    events = []
    stop = threading.Event()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
        server.bind(("127.0.0.1", 0))
        server.settimeout(0.03)
        unity = UnitySpeech(server.getsockname()[1])
        def run():
            while not stop.is_set():
                try:
                    data, remote = server.recvfrom(12000)
                except socket.timeout:
                    continue
                message = json.loads(data)
                events.append(message)
                reply = respond(message)
                if reply:
                    server.sendto(json.dumps(dict(version=1, session=message["session"],
                        utterance=message.get("utterance"), kind=reply)).encode(), remote)
        thread = threading.Thread(target=run)
        thread.start()
        try:
            yield unity, events
        finally:
            stop.set()
            thread.join(2)
            unity.close()


class UnityAudioTests(unittest.TestCase):
    def test_unity_owns_audio_and_temporary_file_until_done(self):
        paths = []
        def respond(message):
            if message["kind"] == "prepare":
                paths.append(Path(message["audio_path"]))
                self.assertTrue(paths[-1].is_file())
                return "ready"
            return {"play": "started", "heartbeat": "done"}.get(message["kind"])
        with renderer(respond) as (unity, events), patch("winsound.PlaySound") as sound:
            self.assertTrue(Speaker(unity).play(wav_audio(0.2), "你好", threading.Event()))
            self.assertFalse(any(call.args[0] is not None for call in sound.call_args_list))
            self.assertEqual(sum(item["kind"] == "play" for item in events), 1)
            self.assertFalse(any(item["kind"] in ("start", "mouth") for item in events))
        self.assertTrue(paths)
        self.assertTrue(all(not path.exists() for path in paths))

    def test_no_ready_never_commits_and_falls_back_to_windows(self):
        with renderer(lambda message: None) as (unity, events), patch("winsound.PlaySound") as sound:
            self.assertTrue(Speaker(unity).play(wav_audio(), "回退", threading.Event()))
            self.assertFalse(any(item["kind"] == "play" for item in events))
            self.assertEqual(sum(call.args[0] is not None for call in sound.call_args_list), 1)
            self.assertTrue(any(item["kind"] == "start" for item in events))

    def test_prepare_error_falls_back_without_play_commit(self):
        with renderer(lambda message: "error" if message["kind"] == "prepare" else None) as (unity, events), \
                patch("winsound.PlaySound") as sound:
            self.assertTrue(Speaker(unity).play(wav_audio(), "回退", threading.Event()))
            self.assertFalse(any(item["kind"] == "play" for item in events))
            self.assertEqual(sum(call.args[0] is not None for call in sound.call_args_list), 1)

    def test_lost_commit_ack_never_falls_back_and_duplicates_audio(self):
        with renderer(lambda message: "ready" if message["kind"] == "prepare" else None) as (unity, events), \
                patch("winsound.PlaySound") as sound:
            with self.assertRaisesRegex(RuntimeError, "失去响应"):
                Speaker(unity).play(wav_audio(0.2), "确认丢失", threading.Event())
            self.assertTrue(any(item["kind"] == "play" for item in events))
            self.assertFalse(any(call.args[0] is not None for call in sound.call_args_list))

    def test_cancel_after_ready_stops_without_windows_fallback(self):
        cancelled = threading.Event()
        def respond(message):
            if message["kind"] == "prepare":
                return "ready"
            if message["kind"] == "play":
                cancelled.set()
                return "started"
        with renderer(respond) as (unity, events), patch("winsound.PlaySound") as sound:
            self.assertFalse(Speaker(unity).play(wav_audio(), "取消", cancelled))
            self.assertFalse(any(call.args[0] is not None for call in sound.call_args_list))

    def test_wrong_source_or_utterance_ack_is_ignored(self):
        with renderer(lambda message: None) as (unity, events):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as other:
                other.sendto(json.dumps(dict(version=1, session=unity.session, utterance="test", kind="done")).encode(),
                             unity.socket.getsockname())
                time.sleep(0.02)
                self.assertIsNone(unity.receive("test"))


if __name__ == "__main__":
    unittest.main()
