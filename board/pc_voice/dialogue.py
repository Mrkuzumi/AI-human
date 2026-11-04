"""ASR final sentences -> streaming Hunyuan -> short TTS sentences -> PC speakers."""

import base64
import datetime
import hashlib
import hmac
import http.client
import json
import os
from pathlib import Path
import queue
import re
import threading
import time
import uuid

from pc_speech import Speaker, UnitySpeech

GESTURE_REPLIES = {
    "touch_head": "谢谢你的摸摸，我很开心。",
    "touch_face": "轻轻碰我的脸，我会害羞的。",
    "hug": "给你一个温暖的拥抱。",
}
GESTURE_AUDIO_DIR = Path(__file__).with_name("gesture_audio")


def load_dialogue_config(path):
    data = json.loads(Path(path).read_text(encoding="utf-8-sig")) if Path(path).is_file() else {"enabled": False}
    if not isinstance(data, dict) or not isinstance(data.get("enabled", False), bool):
        raise ValueError("dialogue.local.json 的 enabled 必须为布尔值")
    data["api_key"] = data.get("api_key") or os.getenv("TOKENHUB_API_KEY") or os.getenv("HUNYUAN_API_KEY", "")
    for key in ("api_key", "model", "system_prompt"):
        if not isinstance(data.get(key, ""), str):
            raise ValueError("对话配置字段必须是字符串: " + key)
    for key, default, low, high in (("history_turns", 4, 1, 15), ("voice_type", 601009, 1, 9999999),
                                    ("unity_port", 39545, 1024, 65535)):
        value = data.setdefault(key, default)
        if type(value) is not int or not low <= value <= high:
            raise ValueError("对话配置数字范围错误: " + key)
    for key, default, low, high in (("speed", 0, -2, 6), ("echo_tail_seconds", 1.2, 0.4, 5)):
        value = data.setdefault(key, default)
        if type(value) not in (int, float) or not low <= value <= high:
            raise ValueError("对话配置数字范围错误: " + key)
    if any(mark in data["api_key"] for mark in ("\r", "\n", "*", "…", "...", "&#")):
        raise ValueError("请填写复制得到的完整 API Key，不能填写带星号/省略号的脱敏 Key 或含换行的内容")
    data["api_key"] = data["api_key"].strip()
    data.setdefault("model", "hy-role")
    data.setdefault("system_prompt", "你是AI数字人小语，用自然中文口语直接回答。第一句尽量不超过16个汉字，默认最多两句、总计50字，不用占位话开头，不假装看到了画面或控制了身体。")
    if data.get("enabled") and (not data["model"].strip() or not data["system_prompt"].strip()):
        raise ValueError("model 和 system_prompt 不得为空")
    return data


def tencent_headers(payload, secret_id, secret_key, timestamp=None):
    """Tencent TC3-HMAC-SHA256, signing the exact bytes sent to TextToVoice."""
    stamp = int(time.time()) if timestamp is None else timestamp
    date = datetime.datetime.fromtimestamp(stamp, datetime.timezone.utc).strftime("%Y-%m-%d")
    host = "tts.tencentcloudapi.com"
    content_type = "application/json; charset=utf-8"
    canonical = ("POST\n/\n\ncontent-type:" + content_type + "\nhost:" + host +
                 "\n\ncontent-type;host\n" + hashlib.sha256(payload).hexdigest())
    scope = date + "/tts/tc3_request"
    string_to_sign = "TC3-HMAC-SHA256\n" + str(stamp) + "\n" + scope + "\n" + hashlib.sha256(canonical.encode()).hexdigest()
    sign = lambda key, message: hmac.new(key, message.encode(), hashlib.sha256).digest()
    key = sign(sign(sign(("TC3" + secret_key).encode(), date), "tts"), "tc3_request")
    signature = hmac.new(key, string_to_sign.encode(), hashlib.sha256).hexdigest()
    return {"Content-Type": content_type, "Host": host, "X-TC-Action": "TextToVoice",
            "X-TC-Version": "2019-08-23", "X-TC-Timestamp": str(stamp),
            "Authorization": "TC3-HMAC-SHA256 Credential=" + secret_id + "/" + scope +
                             ", SignedHeaders=content-type;host, Signature=" + signature}


def stream_text(response, cancelled):
    """Read SSE events, including multiline data, without printing provider payloads."""
    lines = []
    while not cancelled.is_set():
        line = response.readline(262145)
        if len(line) > 262144:
            raise RuntimeError("对话服务返回过大的 SSE 数据")
        if not line:
            if lines:
                raise RuntimeError("对话流意外中断")
            raise RuntimeError("对话流未正常结束")
        line = line.decode("utf-8").rstrip("\r\n")
        if line:
            if line.startswith("data:"):
                lines.append(line[5:].lstrip())
                if sum(map(len, lines)) > 262144:
                    raise RuntimeError("对话服务返回过大的 SSE 事件")
            continue
        if not lines:
            continue
        data = "\n".join(lines)
        lines.clear()
        if data == "[DONE]":
            return
        event = json.loads(data)
        if event.get("error"):
            raise RuntimeError("对话服务返回错误；请检查模型、余额及服务权限")
        choices = event.get("choices", [])
        if choices:
            text = choices[0].get("delta", {}).get("content")
            if isinstance(text, str) and text:
                yield text
            if choices[0].get("finish_reason") in ("stop", "length", "content_filter"):
                return


def sentences(tokens):
    buffer = ""
    total = 0
    first = True
    for token in tokens:
        buffer += token
        total += len(token)
        if total > 3000:
            raise RuntimeError("对话回复超长，已停止")
        while buffer:
            limit = 24 if first else 60
            match = re.search(r"[。！？!?；;\n]", buffer)
            if first and (not match or match.end() > limit):
                match = next((item for item in re.finditer(r"[，：]", buffer[:limit]) if item.end() >= 8), match)
            end = match.end() if match and match.end() <= limit else (limit if len(buffer) >= limit else 0)
            if not end:
                break
            text, buffer = buffer[:end].strip(), buffer[end:]
            if text:
                first = False
                yield text
    if buffer.strip():
        yield buffer.strip()


class CloudDialogue:
    def __init__(self, config, credentials):
        self.config = config
        self.credentials = credentials
        self.connections = set()
        self.lock = threading.Lock()

    def cancel(self):
        with self.lock:
            connections = list(self.connections)
        for connection in connections:
            try:
                stream_socket = connection.sock or getattr(connection, "_dialogue_socket", None)
                if stream_socket:
                    import socket
                    stream_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def request(self, host, path, payload, headers, cancelled):
        connection = http.client.HTTPSConnection(host, timeout=12)
        response = None
        with self.lock:
            self.connections.add(connection)
        try:
            if cancelled.is_set():
                raise InterruptedError()
            connection.request("POST", path, body=payload, headers=headers)
            # HTTPResponse can detach a Connection: close socket; retain it for cancellation.
            connection._dialogue_socket = connection.sock
            if cancelled.is_set():
                raise InterruptedError()
            response = connection.getresponse()
            if response.status != 200:
                # Never log response bodies or headers: they may contain credentials or private text.
                hint = "模型服务尚未开通或额度不足，请检查控制台" if response.status == 402 else "请检查密钥、模型、服务权限和余额"
                raise RuntimeError(f"{host} HTTP {response.status}；{hint}")
            yield response
        finally:
            if response:
                response.close()
            connection.close()
            with self.lock:
                self.connections.discard(connection)

    def reply(self, messages, cancelled):
        body = dict(model=self.config["model"], messages=messages, stream=True,
                    max_tokens=160, temperature=0.7)
        if self.config["model"] in ("hy4-preview", "hy3"):
            # Voice conversation needs the answer, not a long reasoning prelude.
            body["thinking"] = {"type": "disabled"}
        payload = json.dumps(body, ensure_ascii=False).encode()
        headers = {"Content-Type": "application/json", "Authorization": "Bearer " + self.config["api_key"]}
        for response in self.request("tokenhub.tencentmaas.com", "/v1/chat/completions", payload, headers, cancelled):
            yield from stream_text(response, cancelled)

    def synthesize(self, text, cancelled):
        payload = json.dumps(dict(Text=text, SessionId=uuid.uuid4().hex, ModelType=1,
                                  VoiceType=self.config["voice_type"], Speed=self.config["speed"],
                                  PrimaryLanguage=1, SampleRate=16000, Codec="wav"), ensure_ascii=False).encode()
        headers = tencent_headers(payload, self.credentials["secret_id"], self.credentials["secret_key"])
        for response in self.request("tts.tencentcloudapi.com", "/", payload, headers, cancelled):
            body = response.read(12_000_001)
            if len(body) > 12_000_000:
                raise RuntimeError("TTS 返回音频过大")
            result = json.loads(body).get("Response", {})
            if result.get("Error"):
                code = result["Error"].get("Code", "Unknown")
                code = re.sub(r"[^A-Za-z0-9_.]", "", str(code))[:100]
                raise RuntimeError("腾讯 TTS " + code + "；请确认已开通语音合成、有余额且音色可用")
            return base64.b64decode(result["Audio"], validate=True)


class DialogueSession:
    """One board connection owns one bounded history, worker, and speaker stream."""
    def __init__(self, config, credentials, log, cloud=None, speaker=None, unity=None):
        self.config = config
        self.log = log
        self.cloud = cloud or CloudDialogue(config, credentials)
        self.unity = unity or UnitySpeech(config["unity_port"])
        self.speaker = speaker or Speaker(self.unity)
        self.pending = queue.Queue(maxsize=1)
        self.lock = threading.Lock()
        self.cancelled = threading.Event()
        self.closed = threading.Event()
        self.history = []
        self.busy = False
        self.paused = False
        self.answer_started = False
        self.echo_until = 0.0
        self.playing = threading.Event()
        self.worker = threading.Thread(target=self.run, name="digitalman-dialogue", daemon=True)
        self.worker.start()

    def suppress_microphone(self):
        return self.playing.is_set() or time.monotonic() < self.echo_until

    def interrupt(self):
        with self.lock:
            self.cancelled.set()
        self.cloud.cancel()

    def respond_to_gesture(self, gesture):
        text = GESTURE_REPLIES.get(gesture)
        if not text:
            return False
        with self.lock:
            if self.closed.is_set() or self.paused:
                return False
            self.cancelled.set()
            self.cloud.cancel()
            try:
                self.pending.get_nowait()
            except queue.Empty:
                pass
            self.cancelled = threading.Event()
            self.busy = True
            self.answer_started = False
            self.pending.put_nowait((gesture, self.cancelled, True))
        return True

    def set_paused(self, paused):
        with self.lock:
            changed = self.paused != paused
            self.paused = paused
            if paused:
                self.cancelled.set()
                self.echo_until = 0.0
        if changed and paused:
            self.cloud.cancel()

    def submit(self, text):
        text = text.strip()[:1000]
        words = re.sub(r"[\W_]+", "", text)
        if not words:
            return False
        command = words in ("取消刚才的问题", "取消上一个问题", "取消当前问题", "我重新问", "我要重新问",
                            "请取消刚才的问题", "请取消上一个问题", "请取消当前问题")
        with self.lock:
            if self.closed.is_set() or self.paused or self.suppress_microphone():
                return False
            if self.busy:
                if command and not self.answer_started:
                    self.cancelled.set()
                    self.cloud.cancel()
                    notice = "[对话] 正在取消本轮，请稍候…"
                elif re.fullmatch(r"[嗯啊呃哦唉额唔]+", words):
                    notice = "[已忽略] 本轮处理中，附和短词不会打断原问题"
                else:
                    notice = "[已忽略] 本轮处理中，保留原问题；请等本轮回答结束后再提问"
            elif command:
                notice = "[对话] 当前没有正在处理的问题，可直接提问"
            else:
                # Reserve the turn before the worker dequeues it; a second final must not replace it.
                self.busy = True
                self.answer_started = False
                self.cancelled = threading.Event()
                self.pending.put_nowait((text, self.cancelled, False))
                return True
        self.log(notice)
        return False

    def run(self):
        try:
            self.run_turns()
        finally:
            self.unity.send("end")
            self.unity.close()
            with self.lock:
                self.busy = False

    def run_turns(self):
        while not self.closed.is_set():
            try:
                text, cancelled, gesture_reply = self.pending.get(timeout=0.1)
            except queue.Empty:
                continue
            messages = [{"role": "system", "content": self.config["system_prompt"]}] + self.history + [{"role": "user", "content": text}]
            spoken = []
            try:
                if cancelled.is_set() or self.closed.is_set():
                    continue
                if not gesture_reply:
                    self.log("[对话] 正在思考…")
                # Gesture replies use the existing TTS and Unity lip sync, without a model request.
                replies = [GESTURE_REPLIES[text]] if gesture_reply else sentences(self.cloud.reply(messages, cancelled))
                for sentence in replies:
                    if cancelled.is_set():
                        break
                    if gesture_reply:
                        path = GESTURE_AUDIO_DIR / (text + ".wav")
                        if not path.is_file():
                            raise RuntimeError("动作语音文件缺失: " + path.name)
                        audio = path.read_bytes()
                    else:
                        audio = self.cloud.synthesize(sentence, cancelled)
                    with self.lock:
                        if cancelled.is_set() or self.closed.is_set():
                            break
                        self.answer_started = True
                        self.playing.set()
                    try:
                        self.log(("[动作回应] " if gesture_reply else "[数字人] ") + sentence)
                        complete = self.speaker.play(audio, sentence, cancelled)
                        if complete:
                            spoken.append(sentence)
                    finally:
                        self.echo_until = 0.0 if self.paused else time.monotonic() + self.config["echo_tail_seconds"]
                        self.playing.clear()
                if not spoken and not cancelled.is_set():
                    raise RuntimeError("没有可播放的回答")
            except Exception as error:
                if not cancelled.is_set() and not self.closed.is_set():
                    # Allow-listed public errors only; SDK/HTTP details can contain credentials.
                    message = str(error) if type(error) is RuntimeError else type(error).__name__
                    self.log("[对话错误] " + message)
            finally:
                self.unity.send("end")
                with self.lock:
                    if spoken and not gesture_reply:
                        self.history.extend([{"role": "user", "content": text},
                                             {"role": "assistant", "content": "".join(spoken)}])
                        self.history = self.history[-self.config["history_turns"] * 2:]
                    self.busy = not self.pending.empty()
                    self.answer_started = False
                if cancelled.is_set() and not self.closed.is_set() and not self.paused:
                    self.log("[对话] 本轮已取消，可重新提问")

    def close(self):
        self.closed.set()
        self.interrupt()
        self.worker.join(timeout=2)
