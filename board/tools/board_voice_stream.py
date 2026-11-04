#!/usr/bin/env python3
"""Capture board ALSA PCM and reconnect to the PC ASR relay; stdlib only."""

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import threading
import time


CHUNK_BYTES = 6400  # 200 ms, 16 kHz, signed 16-bit little-endian, mono.


def load_config(path):
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict) or not isinstance(data.get("enabled", True), bool):
        raise ValueError("voice config must be an object with a boolean enabled")
    data["server_port"] = int(os.getenv("VOICE_PORT") or data.get("server_port", 8766))
    data["audio_device"] = os.getenv("AUDIO_DEVICE") or data.get("audio_device", "plughw:0,0")
    data["upload_token"] = os.getenv("VOICE_UPLOAD_TOKEN") or data.get("upload_token", "")
    data["reconnect_seconds"] = float(data.get("reconnect_seconds", 3))
    if not 1 <= data["server_port"] <= 65535 or not 0.1 <= data["reconnect_seconds"] <= 60:
        raise ValueError("invalid voice port or reconnect_seconds (0.1..60)")
    token = data["upload_token"]
    if not isinstance(token, str) or not token or "\n" in token or "\r" in token or len(token.encode("utf-8")) > 1024:
        raise ValueError("upload_token must be nonempty, <=1024 bytes and without newlines")
    if not isinstance(data["audio_device"], str) or not data["audio_device"]:
        raise ValueError("audio_device must be a nonempty ALSA device name")
    return data


class VoiceClient:
    def __init__(self, host, config):
        self.host = host
        self.config = config
        self.stopped = threading.Event()
        self.connection = None
        self.recorder = None

    def stop(self):
        self.stopped.set()
        connection, recorder = self.connection, self.recorder
        if connection:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if recorder and recorder.poll() is None:
            recorder.terminate()

    def receive(self, connection, ready, ended):
        pending = b""
        try:
            while not self.stopped.is_set() and not ended.is_set():
                try:
                    packet = connection.recv(4096)
                except socket.timeout:
                    continue
                if not packet:
                    break
                pending += packet
                if len(pending) > 65536:
                    raise ValueError("oversized voice server response")
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    text = line.decode("utf-8", errors="replace")
                    print("[VOICE] " + text, flush=True)
                    if text.startswith("[已连接]"):
                        ready.set()
                    if text.startswith("[错误]"):
                        return
        except (OSError, ValueError) as exc:
            if not self.stopped.is_set():
                print("[VOICE] receive: {}".format(exc), flush=True)
        finally:
            ended.set()
            recorder = self.recorder
            if recorder and recorder.poll() is None:
                recorder.terminate()

    def stream_once(self):
        connection = socket.create_connection((self.host, self.config["server_port"]), timeout=5)
        self.connection = connection
        ready, ended = threading.Event(), threading.Event()
        receiver = None
        try:
            if self.stopped.is_set():
                return
            connection.sendall((self.config["upload_token"] + "\n").encode("utf-8"))
            receiver = threading.Thread(target=self.receive, args=(connection, ready, ended), daemon=True)
            receiver.start()
            deadline = time.monotonic() + 15
            while not ready.is_set():
                if self.stopped.is_set():
                    return
                if ended.is_set() or time.monotonic() >= deadline:
                    raise RuntimeError("PC/cloud handshake failed; check the PC voice console")
                ready.wait(0.1)
            if ended.is_set() or self.stopped.is_set():
                return
            self.recorder = subprocess.Popen([
                "arecord", "-q", "-D", self.config["audio_device"], "-t", "raw",
                "-f", "S16_LE", "-r", "16000", "-c", "1", "-",
            ], stdout=subprocess.PIPE)
            print("[VOICE] recording {} -> {}:{}".format(
                self.config["audio_device"], self.host, self.config["server_port"]), flush=True)
            while not self.stopped.is_set() and not ended.is_set():
                chunk = self.recorder.stdout.read(CHUNK_BYTES)
                if not chunk:
                    raise RuntimeError("arecord stopped; check ALSA device (arecord -l)")
                connection.sendall(chunk)
        finally:
            ended.set()
            recorder = self.recorder
            if recorder:
                if recorder.poll() is None:
                    recorder.terminate()
                try:
                    recorder.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    recorder.kill()
                    recorder.wait()
                recorder.stdout.close()
            self.recorder = None
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
            self.connection = None
            if receiver:
                receiver.join(timeout=2)

    def run(self):
        while not self.stopped.is_set():
            try:
                self.stream_once()
            except (OSError, RuntimeError, ValueError) as exc:
                if not self.stopped.is_set():
                    print("[VOICE] {}".format(exc), flush=True)
            if not self.stopped.is_set():
                print("[VOICE] reconnecting in {}s".format(self.config["reconnect_seconds"]), flush=True)
                self.stopped.wait(self.config["reconnect_seconds"])


def main():
    parser = argparse.ArgumentParser(description="Board microphone -> PC -> Tencent ASR")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "configs" / "voice.json")
    parser.add_argument("--host", default=os.getenv("VOICE_HOST") or "192.168.126.100")
    args = parser.parse_args()
    try:
        config = load_config(args.config)
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    if not config.get("enabled", True):
        print("[VOICE] disabled by configs/voice.json", flush=True)
        return
    if not shutil.which("arecord"):
        parser.error("missing arecord; install alsa-utils on the board")
    client = VoiceClient(args.host, config)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: client.stop())
    try:
        client.run()
    finally:
        client.stop()


if __name__ == "__main__":
    main()
