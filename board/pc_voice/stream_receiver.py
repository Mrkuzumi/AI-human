import argparse
import base64
import hashlib
import hmac
import json
import os
import socket
from pathlib import Path
import secrets
import socketserver
import threading
import time
import uuid
from urllib.parse import quote, urlencode

import websocket

from dialogue import DialogueSession, load_dialogue_config
from pc_speech import UnitySpeech
from voice_mode import VoiceModeLink, command_mode


CHUNK_BYTES = 6400  # 16kHz * 16-bit * mono * 200ms


def build_asr_url(
    app_id: str,
    secret_id: str,
    secret_key: str,
    vad_silence_ms: int,
    *,
    timestamp: int | None = None,
    nonce: int | None = None,
    voice_id: str | None = None,
) -> str:
    timestamp = timestamp or int(time.time())
    params = {
        "engine_model_type": "16k_zh_en_2.0",
        "expired": timestamp + 86400,
        "needvad": 1,
        "nonce": nonce or secrets.randbelow(1_000_000_000) + 1,
        "secretid": secret_id,
        "timestamp": timestamp,
        "vad_silence_time": vad_silence_ms,
        "voice_format": 1,
        "voice_id": voice_id or str(uuid.uuid4()),
    }
    query = urlencode(sorted(params.items()))
    target = f"asr.cloud.tencent.com/asr/v2/{app_id}?{query}"
    signature = base64.b64encode(
        hmac.new(secret_key.encode(), target.encode(), hashlib.sha1).digest()
    ).decode()
    return f"wss://{target}&signature={quote(signature, safe='')}"


def parse_result(message: str) -> tuple[str, bool] | None:
    data = json.loads(message)
    if data.get("code", 0) != 0:
        raise RuntimeError(f"腾讯云错误 {data.get('code')}: {data.get('message', '')}")
    sentences = data.get("sentences")
    if isinstance(sentences, dict) and sentences.get("sentence"):
        return sentences["sentence"], sentences.get("sentence_type") == 1
    result = data.get("result")
    if not result or not result.get("voice_text_str"):
        return None
    final = result.get("slice_type") == 2
    return result["voice_text_str"], final


class StreamHandler(socketserver.StreamRequestHandler):
    token = ""
    handshake_timeout = 15
    app_id = ""
    secret_id = ""
    secret_key = ""
    vad_silence_ms = 800

    def setup(self):
        super().setup()
        self.write_lock = threading.Lock()
        self.dialogue = None

    def send_line(self, text: str) -> None:
        if (getattr(self.server, "mode_link", None) and self.server.mode != "interact"
                and text.startswith(("[识别中]", "[最终]", "[数字人]", "[对话]", "[已忽略]"))):
            return
        print(f"[VOICE] {text}", flush=True)
        try:
            with self.write_lock:
                self.request.sendall((text + "\n").encode("utf-8"))
        except OSError:
            pass

    def forward_results(self, cloud, stopped: threading.Event) -> None:
        previous = None
        seen_finals = []
        try:
            while not stopped.is_set():
                message = cloud.recv()
                if not message:
                    break
                parsed = parse_result(message)
                if parsed:
                    text, final = parsed
                    data = json.loads(message)
                    result = data.get("sentences") or data.get("result") or {}
                    identity = (result.get("sentence_id", result.get("index")), result.get("start_time"), text)
                    fresh = final and identity not in seen_finals
                    if fresh:
                        seen_finals.append(identity)
                        seen_finals = seen_finals[-128:]
                    mode_link = getattr(self.server, "mode_link", None)
                    requested = command_mode(text) if mode_link else None
                    if requested:
                        if fresh:
                            mode_link.request(requested)
                            print("[VOICE MODE] requested " + requested, flush=True)
                        # Commands never become questions or captions, including partials.
                        continue
                    if self.dialogue and self.dialogue.suppress_microphone():
                        continue
                    if mode_link and self.server.mode != "interact":
                        continue
                    line = f"[最终] {text}" if final else f"[识别中] {text}"
                    if line != previous:
                        self.send_line(line)
                        self.server.send_subtitle(text)
                        previous = line
                    if fresh and self.dialogue:
                        self.dialogue.submit(text)
                if json.loads(message).get("final") == 1:
                    break
        except Exception as exc:
            self.send_line(f"[错误] {exc}")
        finally:
            stopped.set()
            # Wake the PCM reader when the cloud session ends or fails.
            try:
                self.request.shutdown(socket.SHUT_RD)
            except OSError:
                pass

    def handle(self) -> None:
        self.request.settimeout(self.handshake_timeout)
        try:
            line = self.rfile.readline(1025)
        except (OSError, TimeoutError):
            return
        token = line.rstrip(b"\r\n")
        if not line.endswith(b"\n") or len(token) > 1024 or not hmac.compare_digest(token, self.token.encode("utf-8")):
            self.send_line("[错误] Unauthorized")
            return

        self.request.settimeout(None)
        cloud = None
        stopped = threading.Event()
        receiver = None
        claimed = False
        try:
            factory = getattr(self.server, "dialogue_factory", None)
            if factory:
                claimed = self.server.dialogue_lock.acquire(blocking=False)
                if not claimed:
                    self.send_line("[错误] 电脑正在与另一块板子对话，请停止另一连接后重试")
                    return
                self.dialogue = factory(self.send_line)
                self.server.register_dialogue(self.dialogue)
            url = build_asr_url(
                self.app_id,
                self.secret_id,
                self.secret_key,
                self.vad_silence_ms,
            )
            cloud = websocket.create_connection(url, timeout=10)
            ack = json.loads(cloud.recv())
            if ack.get("code") != 0:
                raise RuntimeError(
                    f"腾讯云握手失败 {ack.get('code')}: {ack.get('message', '')}"
                )
            cloud.settimeout(None)
            self.send_line("[已连接] 开始实时识别，按 Ctrl+C 停止")
            receiver = threading.Thread(
                target=self.forward_results, args=(cloud, stopped), daemon=True
            )
            receiver.start()

            while not stopped.is_set():
                chunk = self.rfile.read(CHUNK_BYTES)
                if not chunk:
                    break
                if self.dialogue and self.dialogue.suppress_microphone() and not self.server.mode_link:
                    # Keep ASR timing/connection alive, but do not feed the speaker's echo back.
                    chunk = bytes(len(chunk))
                cloud.send_binary(chunk)
        except Exception as exc:
            self.send_line(f"[错误] {exc}")
        finally:
            if cloud:
                try:
                    cloud.send(json.dumps({"type": "end"}))
                except Exception:
                    pass
                if receiver:
                    receiver.join(timeout=5)
                stopped.set()
                cloud.close()
                if receiver:
                    receiver.join(timeout=1)
            if self.dialogue:
                self.server.unregister_dialogue(self.dialogue)
                self.dialogue.close()
            if claimed:
                self.server.dialogue_lock.release()


class StreamServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, handler):
        super().__init__(address, handler)
        self.dialogue_lock = threading.Lock()
        self.mode_lock = threading.Lock()
        self.sessions = set()
        self.mode = None
        self.mode_link = None
        self.unity_display = None
        self.gesture_socket = None
        self.gesture_stop = threading.Event()
        self.gesture_thread = None

    def enable_gesture_responses(self, port=39546):
        self.gesture_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gesture_socket.bind(("127.0.0.1", port))
        self.gesture_socket.settimeout(0.2)
        self.gesture_thread = threading.Thread(target=self.receive_gestures, daemon=True)
        self.gesture_thread.start()

    def receive_gestures(self):
        while not self.gesture_stop.is_set():
            try:
                data, remote = self.gesture_socket.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break
            if remote[0] != "127.0.0.1":
                continue
            try:
                gesture = data.decode("ascii")
            except UnicodeError:
                continue
            with self.mode_lock:
                sessions = list(self.sessions) if not self.mode_link or self.mode == "interact" else []
            for dialogue in sessions:
                if dialogue.respond_to_gesture(gesture):
                    print("[VOICE] Gesture reply: " + gesture, flush=True)

    def register_dialogue(self, dialogue):
        with self.mode_lock:
            self.sessions.add(dialogue)
            if self.mode_link:
                dialogue.set_paused(self.mode != "interact")

    def unregister_dialogue(self, dialogue):
        with self.mode_lock:
            self.sessions.discard(dialogue)

    def mode_changed(self, mode):
        with self.mode_lock:
            self.mode = mode
            for dialogue in self.sessions:
                dialogue.set_paused(mode != "interact")
            if self.unity_display:
                self.unity_display.send("display_mode", mode or "pending")
        print("[VOICE MODE] {}: ASR listening; captions/assistant {}".format(
            mode or "waiting for motion bridge", "enabled" if mode == "interact" else "paused"), flush=True)

    def send_subtitle(self, text):
        with self.mode_lock:
            if self.mode == "interact" and self.unity_display:
                # Unity may have started after the initial mode notification.
                self.unity_display.send("display_mode", "interact")
                self.unity_display.send("subtitle", text)

    def enable_mode_control(self, port, unity_port):
        self.unity_display = UnitySpeech(unity_port)
        self.unity_display.send("display_mode", "pending")
        self.mode_link = VoiceModeLink(port, self.mode_changed)

    def server_close(self):
        self.gesture_stop.set()
        if self.gesture_socket:
            self.gesture_socket.close()
        if self.gesture_thread:
            self.gesture_thread.join(timeout=1)
        if self.mode_link:
            self.mode_link.close()
        if self.unity_display:
            self.unity_display.close()
        super().server_close()


def load_config(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8-sig")) if path.is_file() else {}
    if not isinstance(data, dict):
        raise ValueError("PC 语音配置必须是 JSON 对象")
    fields = {
        "app_id": "TENCENTCLOUD_APP_ID",
        "secret_id": "TENCENTCLOUD_SECRET_ID",
        "secret_key": "TENCENTCLOUD_SECRET_KEY",
        "upload_token": "VOICE_UPLOAD_TOKEN",
    }
    for field, env in fields.items():
        data[field] = os.getenv(env) or data.get(field, "")
        if not isinstance(data[field], str) or not data[field].strip():
            raise ValueError(f"请在 {path.name} 或环境变量 {env} 中配置 {field}")
    if not data["app_id"].isdigit():
        raise ValueError("app_id 必须为数字")
    if "\n" in data["upload_token"] or "\r" in data["upload_token"] or len(data["upload_token"].encode("utf-8")) > 1024:
        raise ValueError("upload_token 必须为不含换行、至多 1024 字节的非空字符串")
    return data


def main() -> None:
    parser = argparse.ArgumentParser(description="三模型 runtime 腾讯云实时语音识别中转服务")
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("voice.local.json"))
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--vad-silence-ms", type=int, default=800)
    parser.add_argument("--check", action="store_true", help="仅检查本地配置与依赖，不调用云端")
    parser.add_argument("--dialogue-config", type=Path, default=Path(__file__).with_name("dialogue.local.json"))
    parser.add_argument("--voice-mode-control", action="store_true", default=True, help="语音口令与串口屏共用动捕/交互模式；默认启用")
    parser.add_argument("--no-voice-mode-control", action="store_false", dest="voice_mode_control", help="禁用语音口令切换模式")
    parser.add_argument("--mode-control-port", type=int, default=39544)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--asr-only", action="store_true", help="仅语音字幕，不调用对话和语音合成")
    mode.add_argument("--require-dialogue", action="store_true", help="开启语音对话；未配置完整 Key 时启动失败，不静默降级")
    args = parser.parse_args()
    if not 1 <= args.mode_control_port <= 65535:
        parser.error("--mode-control-port 需为 1..65535")
    if not 1 <= args.port <= 65535 or not 500 <= args.vad_silence_ms <= 2000:
        parser.error("端口需为 1..65535；--vad-silence-ms 需为 500..2000")
    try:
        config = load_config(args.config)
        dialogue = {"enabled": False} if args.asr_only else load_dialogue_config(args.dialogue_config)
        if args.require_dialogue:
            if not dialogue.get("api_key") or not dialogue.get("model", "").strip() or not dialogue.get("system_prompt", "").strip():
                raise ValueError("启动器已开启语音对话，请先在 dialogue.local.json 配置完整 TokenHub api_key、model 和 system_prompt")
            dialogue["enabled"] = True
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    StreamHandler.token = config["upload_token"]
    StreamHandler.app_id = config["app_id"]
    StreamHandler.secret_id = config["secret_id"]
    StreamHandler.secret_key = config["secret_key"]
    StreamHandler.vad_silence_ms = args.vad_silence_ms
    if args.check:
        print("VOICE_CONFIG_OK (未调用腾讯云)", flush=True)
        if dialogue.get("enabled") and not args.asr_only:
            print("DIALOGUE_CONFIG_OK" if dialogue["api_key"] else "DIALOGUE_PENDING: 请在 dialogue.local.json 填写混元 api_key；目前仅字幕", flush=True)
        return

    if args.asr_only:
        print("[VOICE] 仅字幕模式：只调用 ASR，禁止大模型和 TTS（ASR 仍可能计费）", flush=True)
    with StreamServer((args.host, args.port), StreamHandler) as server:
        if args.voice_mode_control:
            server.enable_mode_control(args.mode_control_port, dialogue.get("unity_port", 39545))
            print("[VOICE MODE voice-mode-20261023] 跟着我的动作 -> follow；跟我聊天 -> interact", flush=True)
        if dialogue.get("enabled") and not args.asr_only:
            if dialogue["api_key"]:
                server.dialogue_factory = lambda log: DialogueSession(dialogue, config, log)
                server.enable_gesture_responses()
                print("[VOICE] 对话已启用：TokenHub/" + dialogue["model"] + " → 腾讯 TTS → 电脑扬声器；Unity 嘴型 UDP 127.0.0.1:" + str(dialogue["unity_port"]), flush=True)
            else:
                print("[VOICE] [待配置] dialogue.local.json 缺少混元 api_key；保留语音字幕，不调用对话/TTS", flush=True)
        print(f"[VOICE] TCP {args.host}:{args.port} -> 腾讯云实时 ASR；等待板端录音", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
