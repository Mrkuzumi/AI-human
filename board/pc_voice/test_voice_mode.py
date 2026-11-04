"""Offline ASR commands -> real local mode socket -> dialogue/caption gating."""
import json
from pathlib import Path
import queue
import socket
import sys
import threading
import time
import unittest
from unittest.mock import patch

from stream_receiver import StreamHandler, StreamServer
from voice_mode import VoiceModeLink, command_mode
from dialogue import DialogueSession
from test_dialogue import FakeCloud, FakeSpeaker, FakeUnity, wait_for

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "AI_humen/AI_humen/pc/reference"))
from mode_control import ModeControlReceiver
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "AI_humen/AI_humen/pc/tests"))
import test_mode_control as bridge_checks


class VoiceModeTests(unittest.TestCase):
    def test_real_tcp_asr_to_bridge_osc_with_live_pcm_in_follow_and_during_playback(self):
        bridge = bridge_checks.RunningBridgeTests()
        self.addCleanup(bridge.doCleanups)
        bridge.setUp()
        class ASR:
            def __init__(self):
                self.messages, self.audio = queue.Queue(), bytearray()
                self.messages.put('{"code":0}')
            def recv(self): return self.messages.get(timeout=5)
            def settimeout(self, _): pass
            def send_binary(self, data): self.audio.extend(data)
            def send(self, _): self.messages.put('{"code":0,"final":1}')
            def close(self): self.messages.put("")
            def final(self, text, index):
                self.messages.put(json.dumps(dict(code=0, result=dict(
                    index=index, start_time=index * 100, slice_type=2, voice_text_str=text))))
        class Dialogue:
            def __init__(self): self.questions, self.paused, self.echo = [], False, False
            def set_paused(self, value): self.paused = value
            def suppress_microphone(self): return self.echo
            def submit(self, text):
                if not self.paused: self.questions.append(text)
            def close(self): pass
        cloud, dialogue = ASR(), Dialogue()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as captions:
            captions.bind(("127.0.0.1", 0))
            captions.settimeout(.1)
            with StreamServer(("127.0.0.1", 0), StreamHandler) as server:
                server.dialogue_factory = lambda _: dialogue
                server.enable_mode_control(bridge.control_port, captions.getsockname()[1])
                with patch("stream_receiver.websocket.create_connection", return_value=cloud), patch.object(StreamHandler, "token", "test-token"):
                    worker = threading.Thread(target=server.serve_forever, daemon=True)
                    worker.start()
                    try:
                        with socket.create_connection(server.server_address, timeout=2) as client:
                            client.sendall(b"test-token\n")
                            self.assertIn("已连接", client.recv(4096).decode())
                            wait_for(lambda: server.mode == "follow")
                            client.sendall(b"\x01\x00" * 3200)
                            wait_for(lambda: len(cloud.audio) == 6400)
                            cloud.final("不要显示这句话", 1)
                            cloud.final("跟我聊天", 2)
                            bridge.wait_for(lambda a, v: a == "/Avatar/Mode" and v == ["interact"])
                            wait_for(lambda: server.mode == "interact")
                            cloud.final("你好数字人", 3)
                            wait_for(lambda: dialogue.questions == ["你好数字人"])
                            self.assertIn("你好数字人", client.recv(4096).decode())
                            dialogue.echo = True
                            client.sendall(b"\x02\x00" * 3200)
                            wait_for(lambda: len(cloud.audio) == 12800)
                            self.assertEqual(cloud.audio, b"\x01\x00" * 3200 + b"\x02\x00" * 3200)
                            cloud.final("跟着我的动作", 4)
                            bridge.wait_for(lambda a, v: a == "/Avatar/Mode" and v == ["follow"])
                            wait_for(lambda: server.mode == "follow" and dialogue.paused)
                            self.assertEqual(dialogue.questions, ["你好数字人"])
                            events = []
                            while True:
                                try: events.append(json.loads(captions.recv(12000)))
                                except socket.timeout: break
                            self.assertEqual([e["text"] for e in events if e["kind"] == "subtitle"], ["你好数字人"])
                            self.assertEqual(events[-1]["text"], "follow")
                    finally:
                        server.shutdown()
                        worker.join(2)

    def test_exact_commands_only(self):
        self.assertEqual(command_mode(" 跟着我的动作！"), "follow")
        self.assertEqual(command_mode("跟我聊天。"), "interact")
        for text in ("不要跟着我的动作", "他说跟我聊天", "跟我聊聊天", "跟着我的动作是什么意思", "你好"):
            self.assertIsNone(command_mode(text))

    def test_real_mode_link_button_heartbeat_reconnect_and_timeout(self):
        receiver = ModeControlReceiver("127.0.0.1", 0, allowed_host="192.0.2.1", initial_mode="interact")
        self.addCleanup(receiver.close)
        changes, applied = [], []
        stopped = threading.Event()
        def pump():
            while not stopped.wait(.005):
                receiver.poll(lambda mode: applied.append(mode) or mode)
        worker = threading.Thread(target=pump, daemon=True)
        worker.start()
        def stop():
            stopped.set(); worker.join(2)
        self.addCleanup(stop)
        link = VoiceModeLink(receiver.socket.getsockname()[1], changes.append)
        self.addCleanup(link.close)
        wait_for(lambda: link.mode == "interact")
        link.request("follow")
        wait_for(lambda: link.mode == "follow")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as button:
            packet = json.dumps(dict(v=1, kind="set", session="a"*32, seq=1, mode="interact")).encode()
            button.sendto(packet, receiver.socket.getsockname())
            wait_for(lambda: link.mode == "interact")
            link.request("follow")
            wait_for(lambda: link.mode == "follow")
            for _ in range(4):
                button.sendto(packet, receiver.socket.getsockname())
                time.sleep(.05)
            self.assertEqual(link.mode, "follow")
            self.assertEqual(receiver.mode, "follow")
        stop()
        wait_for(lambda: link.mode is None, timeout=3)

    def test_pause_cancels_reply_playback_and_resume_accepts_new_question(self):
        cloud, speaker, unity = FakeCloud(), FakeSpeaker(), FakeUnity()
        entered = threading.Event()
        def play(audio, text, cancelled):
            entered.set()
            cancelled.wait(3)
            return not cancelled.is_set()
        speaker.play = play
        session = DialogueSession(dict(system_prompt="test", history_turns=2, echo_tail_seconds=1.2), {},
                                  lambda _: None, cloud=cloud, speaker=speaker, unity=unity)
        self.addCleanup(session.close)
        session.submit("你好")
        self.assertTrue(entered.wait(2))
        session.set_paused(True)
        wait_for(lambda: not session.busy)
        self.assertFalse(session.submit("跟随中的普通话"))
        self.assertFalse(session.suppress_microphone())
        self.assertTrue(cloud.cancel_calls)
        self.assertEqual(session.history, [])
        session.set_paused(False)
        speaker.play = lambda *args: True
        self.assertTrue(session.submit("恢复聊天"))
        wait_for(lambda: len(session.history) == 2)

    def test_asr_final_commands_not_questions_follow_is_silent_and_asr_stays_alive(self):
        class Link:
            def __init__(self, server): self.server, self.requests = server, []
            def request(self, mode):
                self.requests.append(mode)
                self.server.mode_changed(mode)
        class Dialogue:
            def __init__(self): self.questions, self.pauses, self.echo = [], [], False
            def suppress_microphone(self): return self.echo
            def submit(self, text): self.questions.append(text)
            def set_paused(self, paused): self.pauses.append(paused); self.echo = False
        class Display:
            def __init__(self): self.events = []
            def send(self, *event): self.events.append(event)
        class ASR:
            def __init__(self, messages): self.messages = iter(messages)
            def recv(self): return next(self.messages, "")
        with StreamServer(("127.0.0.1", 0), StreamHandler) as server:
            link, dialogue, display = Link(server), Dialogue(), Display()
            server.mode_link, server.unity_display = link, display
            server.mode = "interact"
            server.register_dialogue(dialogue)
            handler = StreamHandler.__new__(StreamHandler)
            handler.server, handler.dialogue = server, dialogue
            lines = []
            handler.send_line = lines.append
            handler.request = type("Socket", (), {"shutdown": lambda *args: None})()
            def final(text, index, slice_type=2):
                return json.dumps(dict(code=0, result=dict(index=index, start_time=index*100,
                                                           slice_type=slice_type, voice_text_str=text)))
            messages = [final("跟着我的动作", 1, 1), final("跟着我的动作", 1),
                        final("跟着我的动作", 1), final("不要显示这句话", 2),
                        final("跟我聊天", 3), final("跟我聊天", 3), final("你好数字人", 4),
                        json.dumps(dict(code=0, final=1))]
            # Even an active answer must not hide a mode command.
            dialogue.echo = True
            handler.forward_results(ASR(messages), threading.Event())
            self.assertEqual(link.requests, ["follow", "interact"])
            self.assertEqual(dialogue.questions, ["你好数字人"])
            self.assertEqual(lines, ["[最终] 你好数字人"])
            self.assertEqual([e for e in display.events if e[0] == "subtitle"], [("subtitle", "你好数字人")])
            self.assertIn(True, dialogue.pauses)
            self.assertIn(False, dialogue.pauses)
            # Fake link/display are not real sockets.
            server.mode_link = server.unity_display = None


if __name__ == "__main__":
    unittest.main()
