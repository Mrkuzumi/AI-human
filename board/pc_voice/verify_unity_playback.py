"""Run only against a test Unity Play session; silent WAVs, no cloud calls."""
import io
import math
import struct
from pathlib import Path
import tempfile
import threading
import time
import uuid
import wave
from unittest.mock import patch

from pc_speech import Speaker, UnitySpeech


def test_wav(duration=1):
    stream = io.BytesIO()
    with wave.open(stream, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(16000)
        # The Unity test harness mutes output after analysis, not the AudioSource input.
        wav.writeframes(b"".join(struct.pack("<h", int(6000 * math.sin(i * 2 * math.pi * 600 / 16000) +
            3000 * math.sin(i * 2 * math.pi * 1200 / 16000))) for i in range(16000 * duration)))
    return stream.getvalue()


def main():
    unity = UnitySpeech(39545)
    def no_fallback(path, flags):
        assert path is None, "Unity did not own the audio; unexpected Windows fallback"
    try:
        with patch("winsound.PlaySound", side_effect=no_fallback):
            assert Speaker(unity).play(test_wav(), "本地静音播音验证", threading.Event())
            cancelled = threading.Event()
            timer = threading.Timer(0.3, cancelled.set)
            timer.start()
            try:
                assert not Speaker(unity).play(test_wav(), "停止播音验证", cancelled)
            finally:
                timer.join()
        with tempfile.TemporaryDirectory(prefix="digitalman-speech-") as directory:
            path = Path(directory) / "reply.wav"
            path.write_bytes(test_wav(3))
            utterance = uuid.uuid4().hex
            def wait_reply(expected):
                deadline = time.monotonic() + 0.8
                while time.monotonic() < deadline:
                    reply = unity.receive(utterance)
                    if reply == expected:
                        return
                    time.sleep(0.01)
                raise AssertionError("Expected Unity reply: " + expected)
            unity.send("prepare", "心跳停止验证", audio_path=str(path), utterance=utterance)
            wait_reply("ready")
            unity.send("play", utterance=utterance)
            wait_reply("started")
            time.sleep(1)  # Simulate a killed backend: no heartbeat and no end packet.
            unity.send("heartbeat", utterance=utterance)
            wait_reply("error")  # Watchdog stopped it before the 3-second clip completed.
        time.sleep(0.2)
        print("UNITY_PLAYBACK_OK real-Unity silent-AudioSource handoff completion cancellation watchdog no-double-play", flush=True)
    finally:
        unity.close()


if __name__ == "__main__":
    main()
