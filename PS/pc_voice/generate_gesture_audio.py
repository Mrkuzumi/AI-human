"""Generate the three fixed gesture WAV files once using the configured Tencent voice."""

import threading
from pathlib import Path

from dialogue import CloudDialogue, GESTURE_REPLIES, load_dialogue_config
from pc_speech import wav_frames
from stream_receiver import load_config


def main():
    directory = Path(__file__).resolve().parent
    config = load_dialogue_config(directory / "dialogue.local.json")
    credentials = load_config(directory / "voice.local.json")
    cloud = CloudDialogue(config, credentials)
    output = directory / "gesture_audio"
    output.mkdir(exist_ok=True)
    for name, text in GESTURE_REPLIES.items():
        audio = cloud.synthesize(text, threading.Event())
        rate, frames = wav_frames(audio)
        target = output / (name + ".wav")
        temporary = output / (name + ".wav.tmp")
        temporary.write_bytes(audio)
        temporary.replace(target)
        print(f"{name}: {len(frames) / (rate * 2):.2f}s, {target.name}", flush=True)


if __name__ == "__main__":
    main()
