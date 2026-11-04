"""Offline dialogue, audio, protocol and lifetime checks. Never calls paid APIs."""

import base64
import contextlib
import io
import json
import os
import queue
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import wave

import dialogue as d
import stream_receiver as relay
from pc_speech import Speaker, UnitySpeech, mouth_level, wav_frames


def wav_audio(duration=0.08, amplitude=0):
    target = io.BytesIO()
    with wave.open(target, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(16000)
        wav.writeframes(int(amplitude).to_bytes(2, "little", signed=True) * int(duration * 16000))
    return target.getvalue()


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("timed out")
        time.sleep(0.01)


class FakeUnity:
    def __init__(self):
        self.events = []
        self.closed = False

    def send(self, *args, **kwargs):
        self.events.append((args, kwargs))

    def close(self):
        self.closed = True


class FakeCloud:
    def __init__(self):
        self.messages = []
        self.synthesized = []
        self.answer = "你好。很高兴见到你！"
        self.cancel_calls = 0

    def reply(self, messages, cancelled):
        self.messages.append(list(messages))
        yield self.answer

    def synthesize(self, text, cancelled):
        self.synthesized.append(text)
        return wav_audio()

    def cancel(self):
        self.cancel_calls += 1


class FakeSpeaker:
    def __init__(self):
        self.spoken = []

    def play(self, audio, text, cancelled):
        self.spoken.append(text)
        return not cancelled.is_set()


class DialogueTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "dialogue.local.json"
        self.path.write_text(json.dumps(dict(enabled=True, api_key="private-test-key")))
        self.config = d.load_dialogue_config(self.path)

    def session(self, cloud=None, speaker=None):
        self.cloud = cloud or FakeCloud()
        self.speaker = speaker or FakeSpeaker()
        self.unity = FakeUnity()
        self.logs = []
        session = d.DialogueSession(self.config, {}, self.logs.append, cloud=self.cloud,
                                    speaker=self.speaker, unity=self.unity)
        self.addCleanup(session.close)
        return session

    def test_local_key_wins_over_stale_environment_and_validation(self):
        with patch.dict(os.environ, {"HUNYUAN_API_KEY": "old"}):
            self.assertEqual(d.load_dialogue_config(self.path)["api_key"], "private-test-key")
        for field, value in (("enabled", "yes"), ("api_key", "bad\nkey"), ("api_key", "sk-masked***"),
                             ("api_key", "sk-masked..."), ("api_key", "sk-encoded&#x77;"), ("unity_port", True),
                             ("history_turns", 100), ("echo_tail_seconds", float("nan")),
                             ("speed", -10), ("model", "")):
            self.path.write_text(json.dumps(dict(enabled=True, api_key="x", **{field: value}) if field not in ("enabled", "api_key")
                                             else {"enabled": True, "api_key": "x", field: value}))
            with self.subTest(field=field), self.assertRaises(ValueError):
                d.load_dialogue_config(self.path)

    def test_missing_config_disables_without_breaking_asr(self):
        self.assertFalse(d.load_dialogue_config(self.path.with_name("missing.json"))["enabled"])

    def test_tokenhub_environment_and_default_model(self):
        self.path.write_text('{"enabled":true,"api_key":""}')
        with patch.dict(os.environ, {"TOKENHUB_API_KEY": "new-test-key", "HUNYUAN_API_KEY": "old-test-key"}):
            config = d.load_dialogue_config(self.path)
        self.assertEqual(config["api_key"], "new-test-key")
        self.assertEqual(config["model"], "hy-role")
        self.assertEqual(config["voice_type"], 601009)
        self.assertEqual(config["history_turns"], 4)

    def test_reply_uses_tokenhub_bearer_stream_and_disables_hy_thinking(self):
        cloud = d.CloudDialogue(self.config, {})
        calls = []
        def request(host, path, payload, headers, cancelled):
            calls.append((host, path, json.loads(payload), headers))
            yield io.BytesIO('data: {"choices":[{"delta":{"reasoning_content":"不应朗读"}}]}\n\ndata: {"choices":[{"delta":{"content":"你好。"},"finish_reason":"stop"}]}\n\n'.encode())
        with patch.object(cloud, "request", side_effect=request):
            self.assertEqual("".join(cloud.reply([{"role":"user","content":"你好"}], threading.Event())), "你好。")
        self.assertEqual(calls[0][0:2], ("tokenhub.tencentmaas.com", "/v1/chat/completions"))
        self.assertEqual(calls[0][2]["model"], "hy-role")
        self.assertNotIn("thinking", calls[0][2])
        self.assertEqual(calls[0][2]["max_tokens"], 160)
        self.assertTrue(calls[0][2]["stream"])
        self.assertEqual(calls[0][3]["Authorization"], "Bearer private-test-key")
        cloud.config = dict(self.config, model="another-test-model")
        with patch.object(cloud, "request", side_effect=request):
            list(cloud.reply([], threading.Event()))
        self.assertNotIn("thinking", calls[1][2])
        for model in ("hy3", "hy4-preview"):
            cloud.config = dict(self.config, model=model)
            with patch.object(cloud, "request", side_effect=request):
                list(cloud.reply([], threading.Event()))
            self.assertEqual(calls[-1][2]["thinking"], {"type": "disabled"})

    def test_sse_unicode_comments_multiline_and_stop(self):
        raw = ': ping\n\ndata: {"choices":\ndata: [{"delta":{"content":"你好"}}]}\n\ndata: {"choices":[{"delta":{"content":"！"},"finish_reason":"stop"}]}\n\n'
        self.assertEqual("".join(d.stream_text(io.BytesIO(raw.encode()), threading.Event())), "你好！")
        self.assertEqual(list(d.stream_text(io.BytesIO(b'data: [DONE]\n\n'), threading.Event())), [])
        for raw in (b'data: {"error": {"message":"secret"}}\n\n', b'data: {}\n\n', b'data: {'):
            with self.assertRaises(RuntimeError) as result:
                list(d.stream_text(io.BytesIO(raw), threading.Event()))
            self.assertNotIn("secret", str(result.exception))

    def test_sentence_stream_is_bounded_and_emits_before_complete(self):
        reached_end = []
        def tokens():
            yield "你好。"
            reached_end.append(True)
            yield "我在这里！" + "长" * 130
        parts = d.sentences(tokens())
        self.assertEqual(next(parts), "你好。")
        self.assertFalse(reached_end)
        rest = list(parts)
        self.assertEqual("".join(rest), "我在这里！" + "长" * 130)
        self.assertTrue(all(len(part) <= 60 for part in rest))

    def test_first_meaningful_clause_is_emitted_before_rest_of_stream(self):
        advanced = []
        first = "建议你先重新启动语音服务，"
        def tokens():
            yield first
            advanced.append(True)
            yield "然后检查网络连接。"
        parts = d.sentences(tokens())
        self.assertEqual(next(parts), first)
        self.assertEqual(advanced, [])
        self.assertEqual("".join(parts), "然后检查网络连接。")

    def test_first_segment_is_shorter_without_losing_text_or_splitting_tiny_ack(self):
        text = "好，" + "长" * 140 + "。下一句。"
        parts = list(d.sentences(iter(text)))
        self.assertEqual("".join(parts), text)
        self.assertEqual(len(parts[0]), 24)
        self.assertTrue(all(len(part) <= 60 for part in parts[1:]))
        self.assertNotEqual(parts[0], "好，")
        with self.assertRaises(RuntimeError):
            list(d.sentences(["长" * 3001]))

    def test_two_turns_remember_context_and_close_discards_it(self):
        session = self.session()
        session.submit("我叫小明")
        wait_for(lambda: len(session.history) == 2)
        session.echo_until = 0
        session.submit("我叫什么？")
        wait_for(lambda: len(session.history) == 4)
        self.assertEqual(self.cloud.messages[1][1]["content"], "我叫小明")
        self.assertEqual(len(self.speaker.spoken), 4)
        session.close()
        self.assertFalse(session.worker.is_alive())
        self.assertTrue(self.unity.closed)

    def test_history_is_bounded_and_cannot_include_failed_answers(self):
        self.config["history_turns"] = 1
        session = self.session()
        for index in range(3):
            session.echo_until = 0
            session.submit(str(index))
            wait_for(lambda: session.history and session.history[0]["content"] == str(index))
        self.assertEqual(len(session.history), 2)
        self.cloud.synthesize = lambda *args: (_ for _ in ()).throw(ValueError("private-test-key"))
        session.echo_until = 0
        session.submit("fail")
        wait_for(lambda: any("对话错误" in line for line in self.logs))
        self.assertNotIn("private-test-key", "".join(self.logs))
        self.assertEqual(session.history[0]["content"], "2")
        wait_for(lambda: not session.busy)
        self.cloud.synthesize = lambda *args: wav_audio()
        session.submit("恢复后再问")
        wait_for(lambda: session.history[0]["content"] == "恢复后再问")

    def test_playback_and_tail_block_echo_from_becoming_new_turn(self):
        entered, release = threading.Event(), threading.Event()
        speaker = FakeSpeaker()
        def play(audio, text, cancelled):
            entered.set()
            release.wait(2)
            return True
        speaker.play = play
        session = self.session(speaker=speaker)
        session.submit("你好")
        self.assertTrue(entered.wait(2))
        self.assertTrue(session.suppress_microphone())
        session.submit("音箱回声")
        self.assertTrue(session.pending.empty())
        release.set()
        wait_for(lambda: len(session.history) == 2)
        self.assertTrue(session.suppress_microphone())
        self.assertEqual(len(self.cloud.messages), 1)

    def test_busy_noise_and_other_questions_do_not_replace_original_question(self):
        cloud = FakeCloud()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def reply(messages, cancelled):
            cloud.messages.append(messages)
            if len(cloud.messages) == 1:
                entered.set()
                release.wait(3)
            yield "原问题的回答。"
        cloud.reply = reply
        session = self.session(cloud=cloud)
        session.submit("你知道武盟DX是什么游戏吗？")
        self.assertTrue(entered.wait(2))
        for noise in ("嗯。", "啊！", "哦，嗯。", "旁边的人提出了另一个问题", "我重新问是什么意思？"):
            session.submit(noise)
        self.assertFalse(session.cancelled.is_set())
        self.assertTrue(session.pending.empty())
        self.assertEqual(len(cloud.messages), 1)
        self.assertEqual(cloud.cancel_calls, 0)
        release.set()
        wait_for(lambda: len(session.history) == 2)
        self.assertEqual(self.speaker.spoken, ["原问题的回答。"])
        self.assertEqual(session.history[0]["content"], "你知道武盟DX是什么游戏吗？")
        self.assertTrue(any("已忽略" in line for line in self.logs))

    def test_turn_is_reserved_before_worker_dequeues(self):
        gate = threading.Event()
        original = d.DialogueSession.run_turns
        def delayed(session):
            gate.wait(3)
            original(session)
        with patch.object(d.DialogueSession, "run_turns", delayed):
            session = self.session()
        self.addCleanup(gate.set)
        self.assertTrue(session.submit("第一个问题"))
        self.assertFalse(session.submit("嗯。"))
        self.assertTrue(session.busy)
        self.assertEqual(session.pending.qsize(), 1)
        self.assertFalse(session.cancelled.is_set())
        gate.set()
        wait_for(lambda: len(session.history) == 2)
        self.assertEqual(session.history[0]["content"], "第一个问题")

    def test_explicit_cancel_while_thinking_then_valid_short_question(self):
        for command in ("取消刚才的问题。", "请取消当前问题！", "我重新问。"):
            with self.subTest(command=command):
                cloud = FakeCloud()
                entered = threading.Event()
                def reply(messages, cancelled):
                    cloud.messages.append(messages)
                    if len(cloud.messages) == 1:
                        entered.set()
                        cancelled.wait(3)
                    yield "回答。"
                cloud.reply = reply
                session = self.session(cloud=cloud)
                self.assertTrue(session.submit("原问题"))
                self.assertTrue(entered.wait(2))
                self.assertFalse(session.submit(command))
                wait_for(lambda: not session.busy)
                self.assertTrue(session.cancelled.is_set())
                self.assertEqual(session.history, [])
                self.assertEqual(self.speaker.spoken, [])
                self.assertEqual(cloud.cancel_calls, 1)
                self.assertTrue(session.submit("为什么？"))
                wait_for(lambda: len(session.history) == 2)
                self.assertEqual(session.history[0]["content"], "为什么？")
                self.assertEqual(self.speaker.spoken, ["回答。"])
                session.close()

    def test_cancel_before_worker_dequeues_does_not_call_cloud(self):
        gate = threading.Event()
        original = d.DialogueSession.run_turns
        def delayed(session):
            gate.wait(3)
            original(session)
        with patch.object(d.DialogueSession, "run_turns", delayed):
            session = self.session()
        self.addCleanup(gate.set)
        session.submit("原问题")
        session.submit("取消刚才的问题")
        gate.set()
        wait_for(lambda: not session.busy)
        self.assertEqual(self.cloud.messages, [])
        self.assertEqual(session.history, [])

    def test_synthesis_is_protected_and_cancellation_does_not_start_a_new_reply(self):
        entered, release = threading.Event(), threading.Event()
        cloud = FakeCloud()
        def synthesize(text, cancelled):
            entered.set()
            release.wait(3)
            return wav_audio()
        cloud.synthesize = synthesize
        session = self.session(cloud=cloud)
        self.addCleanup(release.set)
        session.submit("原问题")
        self.assertTrue(entered.wait(2))
        session.submit("另一个问题")
        self.assertFalse(session.cancelled.is_set())
        session.submit("我要重新问")
        release.set()
        wait_for(lambda: not session.busy)
        self.assertEqual(len(cloud.messages), 1)
        self.assertEqual(self.speaker.spoken, [])

    def test_gap_between_spoken_sentences_stays_locked_even_after_echo_tail(self):
        gap, release = threading.Event(), threading.Event()
        cloud = FakeCloud()
        def reply(messages, cancelled):
            cloud.messages.append(messages)
            yield "第一句。"
            gap.set()
            release.wait(3)
            yield "第二句。"
        cloud.reply = reply
        session = self.session(cloud=cloud)
        self.addCleanup(release.set)
        session.submit("原问题")
        self.assertTrue(gap.wait(2))
        session.echo_until = 0
        self.assertFalse(session.suppress_microphone())
        session.submit("取消刚才的问题")
        session.submit("嗯")
        self.assertFalse(session.cancelled.is_set())
        self.assertTrue(session.busy)
        release.set()
        wait_for(lambda: not session.busy)
        self.assertEqual(self.speaker.spoken, ["第一句。", "第二句。"])
        session.echo_until = 0
        self.assertTrue(session.submit("不"))
        wait_for(lambda: len(session.history) == 4)
        self.assertEqual(session.history[2]["content"], "不")

    def test_idle_cancel_and_punctuation_do_not_call_model(self):
        session = self.session()
        self.assertFalse(session.submit("我重新问。"))
        self.assertFalse(session.submit("……？！"))
        self.assertFalse(session.busy)
        self.assertTrue(session.pending.empty())
        self.assertEqual(self.cloud.messages, [])
        self.assertTrue(session.submit("嗯"))  # An idle acknowledgement can be meaningful.
        wait_for(lambda: len(session.history) == 2)

    def test_audio_validation_envelope_and_loopback_event_order(self):
        rate, frames = wav_frames(wav_audio(amplitude=10000))
        self.assertEqual(rate, 16000)
        self.assertGreater(mouth_level(frames), 0.5)
        self.assertEqual(mouth_level(b'\0' * 200), 0)
        with self.assertRaises(ValueError):
            wav_frames(b"bad")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(1)
            unity = UnitySpeech(receiver.getsockname()[1])
            self.addCleanup(unity.close)
            unity.send("start", "你好")
            one = json.loads(receiver.recv(12000))
            unity.send("end")
            two = json.loads(receiver.recv(12000))
            self.assertEqual(one["session"], two["session"])
            self.assertEqual(two["sequence"], one["sequence"] + 1)
            self.assertEqual(one["text"], "你好")

    @unittest.skipUnless(os.name == "nt", "Windows speaker API")
    def test_real_windows_silent_wav_playback_and_cancel(self):
        unity = FakeUnity()
        speaker = Speaker(unity)
        event = threading.Event()
        self.assertTrue(speaker.play(wav_audio(), "静音检查", event))
        self.assertEqual(unity.events[0][0][0], "start")
        self.assertEqual(unity.events[-1][0][0], "end")
        timer = threading.Timer(0.1, event.set)
        timer.start()
        try:
            start = time.monotonic()
            self.assertFalse(speaker.play(wav_audio(duration=2), "取消检查", event))
            self.assertLess(time.monotonic() - start, 1)
        finally:
            timer.join()

    def test_tc3_signature_deterministic_and_payload_sensitive(self):
        one = d.tencent_headers(b'{}', "test-id", "test-key", 1700000000)
        self.assertEqual(one, d.tencent_headers(b'{}', "test-id", "test-key", 1700000000))
        self.assertNotEqual(one["Authorization"], d.tencent_headers(b'{ }', "test-id", "test-key", 1700000000)["Authorization"])
        self.assertIn("2023-11-14/tts/tc3_request", one["Authorization"])
        self.assertNotIn("test-key", str(one))

    def test_http_tts_payload_audio_and_error_redaction(self):
        cloud = d.CloudDialogue(self.config, dict(secret_id="test-id", secret_key="test-key"))
        calls = []
        def request(host, path, payload, headers, cancelled):
            calls.append((host, json.loads(payload), headers))
            yield io.BytesIO(json.dumps({"Response": {"Audio": base64.b64encode(wav_audio()).decode()}}).encode())
        with patch.object(cloud, "request", side_effect=request):
            self.assertEqual(cloud.synthesize("你好", threading.Event()), wav_audio())
        self.assertEqual(calls[0][1]["Codec"], "wav")
        self.assertEqual(calls[0][1]["SampleRate"], 16000)
        self.assertEqual(calls[0][1]["VoiceType"], 601009)
        self.assertEqual(calls[0][2]["X-TC-Action"], "TextToVoice")
        def denied(*args):
            yield io.BytesIO(b'{"Response":{"Error":{"Code":"AuthFailure","Message":"private-test-key"}}}')
        with patch.object(cloud, "request", side_effect=denied), self.assertRaisesRegex(RuntimeError, "AuthFailure") as error:
            cloud.synthesize("你好", threading.Event())
        self.assertNotIn("private-test-key", str(error.exception))

    def test_cloud_connections_use_tls_and_cancel_both_llm_and_tts(self):
        class Connection:
            sock = None
            def __init__(self, host, timeout):
                self.host, self.timeout = host, timeout
            def request(self, *args, **kwargs):
                pass
            def getresponse(self):
                return type("Response", (), {"status": 401, "close": lambda self: None})()
            def close(self):
                pass
        cloud = d.CloudDialogue(self.config, {})
        with patch.object(d.http.client, "HTTPSConnection", Connection), self.assertRaisesRegex(RuntimeError, "HTTP 401"):
            list(cloud.request("tokenhub.tencentmaas.com", "/", b"{}", {}, threading.Event()))
        self.assertFalse(cloud.connections)
        class Socket:
            count = 0
            def shutdown(self, how):
                self.count += 1
        connections = [Connection("", 12), Connection("", 12)]
        for connection in connections:
            connection.sock = Socket()
            cloud.connections.add(connection)
        cloud.cancel()
        self.assertEqual([item.sock.count for item in connections], [1, 1])

    def test_gesture_interrupts_reply_and_speaks_fixed_line_without_model(self):
        started = threading.Event()
        class BlockingSpeaker:
            spoken = []
            def play(self, audio, text, cancelled):
                self.spoken.append(text)
                if text != d.GESTURE_REPLIES["hug"]:
                    started.set()
                    cancelled.wait(2)
                return not cancelled.is_set()
        speaker = BlockingSpeaker()
        session = self.session(speaker=speaker)
        self.assertTrue(session.submit("原来的问题"))
        wait_for(started.is_set)
        self.assertTrue(session.respond_to_gesture("hug"))
        wait_for(lambda: len(speaker.spoken) == 2 and not session.busy)
        self.assertEqual(speaker.spoken[1], d.GESTURE_REPLIES["hug"])
        self.assertEqual(len(self.cloud.messages), 1)
        self.assertEqual(self.cloud.synthesized, ["你好。"])
        self.assertFalse(session.history)
        self.assertFalse(session.respond_to_gesture("unknown"))

    def test_all_gesture_audio_files_are_valid_local_wav(self):
        for name in d.GESTURE_REPLIES:
            with self.subTest(name=name):
                path = d.GESTURE_AUDIO_DIR / (name + ".wav")
                rate, frames = wav_frames(path.read_bytes())
                self.assertGreater(len(frames) / (rate * 2), 0.5)


class RelayModeTests(unittest.TestCase):
    def run_main(self, mode, dialogue=None):
        class Server:
            def enable_mode_control(self, *args):
                pass
            def enable_gesture_responses(self):
                pass
            def __enter__(self):
                return self
            def __exit__(self, *args):
                pass
            def serve_forever(self):
                pass
        server = Server()
        output = io.StringIO()
        credentials = dict(upload_token="fixture-token", app_id="123", secret_id="fixture-id", secret_key="fixture-key")
        with patch.object(sys, "argv", ["stream_receiver.py", mode]), \
             patch.object(relay, "load_config", return_value=credentials), \
             patch.object(relay, "load_dialogue_config", return_value=dialogue) as load, \
             patch.object(relay, "StreamServer", return_value=server), \
             patch.object(relay, "DialogueSession") as session, contextlib.redirect_stdout(output):
            relay.main()
            if mode == "--asr-only":
                load.assert_not_called()
                session.assert_not_called()
                self.assertFalse(hasattr(server, "dialogue_factory"))
            else:
                self.assertTrue(hasattr(server, "dialogue_factory"))
                server.dialogue_factory(lambda message: None)
                self.assertTrue(session.call_args.args[0]["enabled"])
        return output.getvalue()

    def test_asr_only_never_loads_dialogue_secrets_or_creates_paid_session(self):
        output = self.run_main("--asr-only")
        self.assertIn("仅字幕模式", output)
        self.assertIn("禁止大模型和 TTS", output)

    def test_required_dialogue_overrides_local_disabled_setting(self):
        config = dict(enabled=False, api_key="private-test-key", model="hy3", system_prompt="test", unity_port=39545)
        self.assertIn("对话已启用", self.run_main("--require-dialogue", config))

    def test_required_dialogue_without_key_fails_before_server_or_cloud_call(self):
        with patch.object(sys, "argv", ["stream_receiver.py", "--require-dialogue", "--check"]), \
             patch.object(relay, "load_config", return_value={}), \
             patch.object(relay, "load_dialogue_config", return_value=dict(enabled=False, api_key="")), \
             patch.object(relay, "StreamServer") as server, contextlib.redirect_stderr(io.StringIO()), \
             self.assertRaises(SystemExit) as error:
            relay.main()
        self.assertEqual(error.exception.code, 2)
        server.assert_not_called()

    def test_asr_only_and_required_dialogue_are_mutually_exclusive(self):
        with patch.object(sys, "argv", ["stream_receiver.py", "--asr-only", "--require-dialogue"]), \
             contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            relay.main()
        self.assertEqual(error.exception.code, 2)


class RelayDialogueTests(unittest.TestCase):
    def test_loopback_gesture_only_reaches_interaction_session(self):
        received = queue.Queue()
        class Session:
            def respond_to_gesture(self, name):
                received.put(name)
                return True
        class ModeLink:
            def close(self):
                pass
        with relay.StreamServer(("127.0.0.1", 0), relay.StreamHandler) as server:
            server.mode_link = ModeLink()
            server.sessions.add(Session())
            server.enable_gesture_responses(0)
            target = server.gesture_socket.getsockname()
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(b"hug", target)
                time.sleep(0.25)
                self.assertTrue(received.empty())
                server.mode = "interact"
                sender.sendto(b"hug", target)
                self.assertEqual(received.get(timeout=1), "hug")

    def test_real_tcp_asr_final_drives_dialogue_mutes_echo_and_rejects_second_board(self):
        class ASR:
            def __init__(self):
                self.messages = queue.Queue()
                self.messages.put('{"code":0}')
                self.audio = []
            def recv(self):
                return self.messages.get(timeout=3)
            def settimeout(self, value):
                pass
            def send_binary(self, audio):
                self.audio.append(audio)
                if any(audio):
                    index = len(self.audio) - 1
                    text = "你好" if index == 0 else "嗯。"
                    message = json.dumps({"code": 0, "result": {"index": index, "slice_type": 2, "voice_text_str": text}})
                    self.messages.put(message)
                    self.messages.put(message)  # Provider duplicate must not create another reply.
            def send(self, data):
                self.messages.put('{"code":0,"final":1}')
            def close(self):
                self.messages.put("")
        asr = ASR()
        speaker, unity, cloud = FakeSpeaker(), FakeUnity(), FakeCloud()
        release = threading.Event()
        def reply(messages, cancelled):
            cloud.messages.append(messages)
            release.wait(3)
            yield cloud.answer
        cloud.reply = reply
        config = dict(system_prompt="test", history_turns=2, echo_tail_seconds=1.2, unity_port=39545)
        sessions = []
        logs = []
        def factory(log):
            def record(message):
                logs.append(message)
                log(message)
            session = d.DialogueSession(config, {}, record, cloud=cloud, speaker=speaker, unity=unity)
            sessions.append(session)
            return session
        relay.StreamHandler.token = "test-token"
        server = relay.StreamServer(("127.0.0.1", 0), relay.StreamHandler)
        server.dialogue_factory = factory
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with patch.object(relay.websocket, "create_connection", return_value=asr) as connect:
                with socket.create_connection(server.server_address, timeout=3) as client:
                    client.sendall(b'test-token\n')
                    self.assertIn("已连接", client.recv(4096).decode())
                    with socket.create_connection(server.server_address, timeout=3) as other:
                        other.sendall(b'test-token\n')
                        self.assertIn("另一块板子", other.recv(4096).decode())
                    self.assertEqual(connect.call_count, 1)
                    client.sendall(b'\x01\x00' * 3200)
                    wait_for(lambda: len(cloud.messages) == 1)
                    client.sendall(b'\x01\x00' * 3200)  # A second ASR final while thinking.
                    wait_for(lambda: any("已忽略" in line for line in logs))
                    self.assertFalse(sessions[0].cancelled.is_set())
                    self.assertEqual(cloud.cancel_calls, 0)
                    release.set()
                    wait_for(lambda: sessions and len(sessions[0].history) == 2)
                    self.assertEqual(speaker.spoken, ["你好。", "很高兴见到你！"])
                    client.sendall(b'\x01\x00' * 3200)
                    wait_for(lambda: len(asr.audio) >= 3)
                    self.assertFalse(any(asr.audio[-1]))
                    self.assertEqual(len(cloud.messages), 1)
                    client.shutdown(socket.SHUT_WR)
                    while client.recv(4096):
                        pass
                wait_for(lambda: unity.closed)
                self.assertFalse(sessions[0].worker.is_alive())
        finally:
            release.set()
            for session in sessions:
                session.close()
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
