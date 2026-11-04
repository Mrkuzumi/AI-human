"""Unity-owned audio/lip sync, with Windows playback when Unity is unavailable."""

from array import array
import io
import json
import math
import os
from pathlib import Path
import socket
import tempfile
import time
import uuid
import wave


def wav_frames(audio):
    if not 44 <= len(audio) <= 8_000_000:
        raise ValueError("TTS WAV 大小异常")
    with wave.open(io.BytesIO(audio), "rb") as wav:
        if wav.getnchannels() != 1 or wav.getsampwidth() != 2 or wav.getcomptype() != "NONE":
            raise ValueError("TTS 必须返回单声道 16-bit PCM WAV")
        rate = wav.getframerate()
        if rate not in (8000, 16000, 24000):
            raise ValueError("TTS WAV 采样率异常")
        frames = wav.readframes(wav.getnframes())
        if not frames or len(frames) % 2 or len(frames) / (rate * 2) > 90:
            raise ValueError("TTS WAV 时长异常")
    return rate, frames


def mouth_level(frames):
    samples = array("h", frames)
    rms = math.sqrt(sum(value * value for value in samples) / max(1, len(samples))) / 32768
    return min(1.0, max(0.0, (rms - 0.008) * 7.0))


class UnitySpeech:
    def __init__(self, port):
        self.target = ("127.0.0.1", port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.setblocking(False)
        self.session = uuid.uuid4().hex
        self.sequence = 0

    def send(self, kind, text="", level=0.0, **fields):
        self.sequence += 1
        message = dict(version=1, session=self.session, sequence=self.sequence,
                       kind=kind, text=text[:1600], level=level)
        message.update(fields)
        try:
            self.socket.sendto(json.dumps(message, ensure_ascii=False).encode("utf-8"), self.target)
        except OSError:
            pass  # Unity may be stopped while the PC continues its conversation.

    def receive(self, utterance):
        for _ in range(32):
            try:
                data, remote = self.socket.recvfrom(4096)
            except (BlockingIOError, OSError):
                return None
            try:
                message = json.loads(data)
                if (remote == self.target and isinstance(message, dict) and message.get("version") == 1
                        and message.get("session") == self.session and message.get("utterance") == utterance
                        and message.get("kind") in ("ready", "started", "playing", "done", "error")):
                    return message["kind"]
            except (ValueError, UnicodeError):
                continue
        return None

    def close(self):
        self.socket.close()


class Speaker:
    def __init__(self, unity):
        self.unity = unity

    def play_in_unity(self, path, text, duration, cancelled):
        utterance = uuid.uuid4().hex
        ready_deadline = time.monotonic() + 0.8
        next_send = 0.0
        while not cancelled.is_set() and time.monotonic() < ready_deadline:
            if time.monotonic() >= next_send:
                self.unity.send("prepare", text, audio_path=str(path), utterance=utterance)
                next_send = time.monotonic() + 0.1
            reply = self.unity.receive(utterance)
            if reply == "ready":
                break
            if reply == "error":
                return None
            cancelled.wait(0.01)
        else:
            # No commit was sent: a late prepare can never start duplicate audio.
            return False if cancelled.is_set() else None
        self.unity.send("play", utterance=utterance)
        started = False
        now = time.monotonic()
        deadline, heard, next_send = now + duration + 3, now, now + 0.1
        while not cancelled.is_set() and time.monotonic() < deadline:
            reply = self.unity.receive(utterance)
            if reply == "done":
                return True
            if reply == "error":
                raise RuntimeError("Unity 语音播放失败；本句不会重复播音")
            if reply in ("started", "playing"):
                started, heard = True, time.monotonic()
            now = time.monotonic()
            if now - heard > 1.2:
                raise RuntimeError("Unity 播音失去响应；已停止本句，请检查 Unity 是否仍在 Play")
            if now >= next_send:
                self.unity.send("heartbeat" if started else "play", utterance=utterance)
                next_send = now + 0.1
            cancelled.wait(0.01)
        if cancelled.is_set():
            return False
        raise RuntimeError("Unity 播音超时，已停止本句")

    def play(self, audio, text, cancelled):
        if os.name != "nt":
            raise RuntimeError("电脑扬声器播放需要 Windows")
        import winsound
        rate, frames = wav_frames(audio)
        if cancelled.is_set():
            return False
        # The local file stays alive until Unity acknowledges completion or fallback playback ends.
        with tempfile.TemporaryDirectory(prefix="digitalman-speech-") as directory:
            path = Path(directory) / "reply.wav"
            path.write_bytes(audio)
            try:
                duration = len(frames) / (rate * 2)
                if isinstance(self.unity, UnitySpeech):
                    result = self.play_in_unity(path, text, duration, cancelled)
                    if result is not None:
                        return result
                if cancelled.is_set():
                    return False
                winsound.PlaySound(str(path), winsound.SND_FILENAME | winsound.SND_ASYNC | winsound.SND_NODEFAULT)
                started = time.monotonic()
                self.unity.send("start", text)
                while not cancelled.is_set():
                    elapsed = time.monotonic() - started
                    if elapsed >= duration:
                        break
                    offset = int(elapsed * rate) * 2
                    block = frames[offset:offset + int(rate * 0.04) * 2]
                    self.unity.send("mouth", level=mouth_level(block))
                    cancelled.wait(min(0.04, duration - elapsed))
                return not cancelled.is_set()
            finally:
                winsound.PlaySound(None, 0)
                self.unity.send("end")
