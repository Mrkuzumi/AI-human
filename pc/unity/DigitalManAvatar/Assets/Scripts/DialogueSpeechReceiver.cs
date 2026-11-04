using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using VRM;

namespace DigitalMan
{
    // Only the mouth is overridden, after VMC/animations; bones and blinks remain untouched.
    [DefaultExecutionOrder(32000)]
    public sealed class DialogueSpeechReceiver : MonoBehaviour
    {
        const int CaptionFontSize = 72  ;
        const float CaptionCenterFromTop = 0.65f;

        [Serializable]
        public sealed class SpeechEvent
        {
            public int version;
            public string session;
            public int sequence;
            public string kind;
            public string text;
            public float level;
            public string audio_path;
            public string utterance;
        }

        UdpClient socket;
        VRMBlendShapeProxy proxy;
        string session, subtitle = "";
        string displaySession, recognized = "";
        int displaySequence;
        bool displayInteractive = true;
        float recognizedUntil;
        int sequence;
        bool speaking, releaseMouth;
        float lastPacket, subtitleUntil, target, value, nextFind;
        GUIStyle style;
        AudioSource speaker;
        uLipSync.uLipSync lipSync;
        IPEndPoint audioOwner;
        string utterance;
        bool prepared, clipPlaying, audioFinished;
        float playbackAt;
        readonly float[] phonemes = new float[5], mouthWeights = new float[5];
        static readonly string[] Vowels = { "A", "I", "U", "E", "O" };
        static readonly BlendShapeKey A = BlendShapeKey.CreateFromPreset(BlendShapePreset.A);
        static readonly BlendShapeKey I = BlendShapeKey.CreateFromPreset(BlendShapePreset.I);
        static readonly BlendShapeKey U = BlendShapeKey.CreateFromPreset(BlendShapePreset.U);
        static readonly BlendShapeKey E = BlendShapeKey.CreateFromPreset(BlendShapePreset.E);
        static readonly BlendShapeKey O = BlendShapeKey.CreateFromPreset(BlendShapePreset.O);
        static readonly BlendShapeKey[] MouthKeys = { A, I, U, E, O };

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void Install()
        {
            if (FindObjectOfType<DialogueSpeechReceiver>() != null) return;
            var host = new GameObject("Dialogue Speech (PC speakers)");
            DontDestroyOnLoad(host);
            host.AddComponent<DialogueSpeechReceiver>();
        }

        void OnEnable()
        {
            Application.runInBackground = true;
            try
            {
                socket = new UdpClient();
                socket.Client.ExclusiveAddressUse = true;
                socket.Client.Bind(new IPEndPoint(IPAddress.Loopback, 39545));
                socket.Client.Blocking = false;
            }
            catch (SocketException error)
            {
                if (socket != null) socket.Close();
                socket = null;
                Debug.LogWarning("Dialogue mouth receiver could not bind loopback UDP 39545: " + error.SocketErrorCode);
            }
        }

        public void Accept(SpeechEvent message)
        { Accept(message, null); }

        void Accept(SpeechEvent message, IPEndPoint remote)
        {
            if (message == null || message.version != 1 || string.IsNullOrEmpty(message.session) ||
                message.session.Length > 64 || message.sequence < 1 ||
                float.IsNaN(message.level) || float.IsInfinity(message.level)) return;
            if (message.kind == "display_mode" || message.kind == "subtitle")
            {
                if (displaySession == message.session && message.sequence <= displaySequence) return;
                if (displaySession != message.session && message.kind != "display_mode") return;
                displaySession = message.session;
                displaySequence = message.sequence;
                if (message.kind == "display_mode") SetInteraction(message.text == "interact");
                else if (displayInteractive)
                {
                    recognized = SafeText(message.text);
                    recognizedUntil = Time.unscaledTime + 6;
                }
                return;
            }
            if (!displayInteractive && message.kind != "end") return;
            if (message.kind != "start" && message.kind != "mouth" && message.kind != "end" &&
                message.kind != "prepare" && message.kind != "play" && message.kind != "heartbeat") return;
            if (session == message.session && message.sequence <= sequence) return;
            // An orphan level/end from an old session must not cancel the current speaker.
            if (session != message.session && message.kind != "start" && message.kind != "prepare") return;
            if ((message.kind == "play" || message.kind == "heartbeat") &&
                (message.utterance != utterance || audioOwner == null || !audioOwner.Equals(remote))) return;
            session = message.session;
            sequence = message.sequence;
            lastPacket = Time.unscaledTime;
            if (message.kind == "prepare")
            {
                if (remote == null || !IPAddress.IsLoopback(remote.Address) ||
                    string.IsNullOrEmpty(message.utterance) || message.utterance.Length != 32) return;
                if (prepared && utterance == message.utterance && audioOwner != null && audioOwner.Equals(remote))
                { Reply("ready"); return; }
                EndSpeech();
                audioOwner = remote;
                utterance = message.utterance;
                try
                {
                    EnsureAudio();
                    speaker.clip = ReadLocalWav(message.audio_path);
                    prepared = true;
                    audioFinished = false;
                    subtitle = SafeText(message.text);
                    Reply("ready");
                }
                catch (Exception error) when (error is IOException || error is ArgumentException ||
                    error is UnauthorizedAccessException || error is InvalidOperationException)
                {
                    EndSpeech(); Reply("error");
                    Debug.LogWarning("Dialogue audio preparation failed: " + error.GetType().Name);
                }
            }
            else if (message.kind == "play")
            {
                if (audioFinished) { Reply("done"); return; }
                if (clipPlaying) { Reply("started"); return; }
                if (!prepared || speaker == null || speaker.clip == null) { Reply("error"); return; }
                speaker.Play();
                prepared = false;
                clipPlaying = speaking = true;
                releaseMouth = false;
                playbackAt = Time.unscaledTime;
                subtitleUntil = playbackAt + 120;
                Reply("started");
            }
            else if (message.kind == "heartbeat")
            { Reply(audioFinished ? "done" : clipPlaying ? "playing" : "error"); }
            else if (message.kind == "start")
            {
                EndSpeech();
                speaking = true;
                releaseMouth = false;
                subtitle = SafeText(message.text);
                subtitleUntil = Time.unscaledTime + 120;
                target = 0;
            }
            else if (message.kind == "mouth")
            {
                if (speaking) target = Mathf.Clamp01(message.level);
            }
            else EndSpeech();
        }

        public void SetInteraction(bool interaction)
        {
            displayInteractive = interaction;
            if (!interaction)
            {
                EndSpeech();
                subtitle = recognized = "";
                subtitleUntil = recognizedUntil = 0;
            }
        }

        void EndSpeech()
        {
            if (speaking) releaseMouth = true;
            if (speaker != null)
            {
                speaker.Stop();
                if (speaker.clip != null)
                {
                    if (Application.isPlaying) Destroy(speaker.clip);
                    else DestroyImmediate(speaker.clip);
                }
                speaker.clip = null;
            }
            prepared = clipPlaying = false;
            Array.Clear(phonemes, 0, phonemes.Length);
            speaking = false;
            target = 0;
            subtitleUntil = Time.unscaledTime + 3;
        }

        static string SafeText(string text)
        { return string.IsNullOrEmpty(text) ? "" : text.Substring(0, Math.Min(1600, text.Length)); }

        void EnsureAudio()
        {
            if (speaker != null && lipSync != null) return;
            var profile = Resources.Load<uLipSync.Profile>("DialogueLipSyncProfile");
            if (profile == null) throw new InvalidOperationException("Lip sync profile missing");
            var audio = new GameObject("Dialogue audio + uLipSync");
            audio.SetActive(false);
            audio.transform.SetParent(transform, false);
            speaker = audio.AddComponent<AudioSource>();
            speaker.playOnAwake = false;
            speaker.spatialBlend = 0;
            speaker.loop = false;
            speaker.ignoreListenerPause = true;
            bool hasListener = false;
            foreach (var listener in FindObjectsOfType<AudioListener>())
                if (listener.isActiveAndEnabled) { hasListener = true; break; }
            if (!hasListener) audio.AddComponent<AudioListener>();
            lipSync = audio.AddComponent<uLipSync.uLipSync>();
            lipSync.profile = profile;
            lipSync.onLipSyncUpdate.AddListener(OnLipSync);
            audio.SetActive(true);
        }

        public void OnLipSync(uLipSync.LipSyncInfo info)
        {
            if (!clipPlaying) return;
            float volume = info.rawVolume > 0.003f ? Mathf.Clamp01(info.volume) : 0;
            for (int i = 0; i < phonemes.Length; i++)
            {
                float ratio;
                phonemes[i] = info.phonemeRatios != null && info.phonemeRatios.TryGetValue(Vowels[i], out ratio) &&
                    !float.IsNaN(ratio) && !float.IsInfinity(ratio) ? Mathf.Clamp01(ratio) * volume : 0;
            }
        }

        void Reply(string kind)
        {
            if (socket == null || audioOwner == null) return;
            var message = new SpeechEvent { version = 1, session = session, utterance = utterance, kind = kind };
            byte[] bytes = Encoding.UTF8.GetBytes(JsonUtility.ToJson(message));
            try { socket.Send(bytes, bytes.Length, audioOwner); }
            catch (SocketException) { }
        }

        public static AudioClip ReadLocalWav(string path)
        {
            if (string.IsNullOrEmpty(path)) throw new ArgumentException("Missing WAV path");
            string full = Path.GetFullPath(path);
            string temporary = Path.GetFullPath(Path.GetTempPath()).TrimEnd(Path.DirectorySeparatorChar) + Path.DirectorySeparatorChar;
            var parent = Directory.GetParent(full);
            if (!full.StartsWith(temporary, StringComparison.OrdinalIgnoreCase) || Path.GetFileName(full) != "reply.wav" ||
                parent == null || !parent.Name.StartsWith("digitalman-speech-", StringComparison.Ordinal) ||
                !string.Equals(parent.Parent.FullName.TrimEnd(Path.DirectorySeparatorChar),
                    temporary.TrimEnd(Path.DirectorySeparatorChar), StringComparison.OrdinalIgnoreCase))
                throw new ArgumentException("Only our temporary reply WAV is allowed");
            if ((File.GetAttributes(parent.FullName) & FileAttributes.ReparsePoint) != 0 ||
                (File.GetAttributes(full) & FileAttributes.ReparsePoint) != 0)
                throw new ArgumentException("WAV links are not allowed");
            long size = new FileInfo(full).Length;
            if (size < 44 || size > 8000000) throw new ArgumentException("WAV size invalid");
            using (var reader = new BinaryReader(File.OpenRead(full)))
            {
                if (new string(reader.ReadChars(4)) != "RIFF") throw new ArgumentException("Not RIFF");
                reader.ReadUInt32();
                if (new string(reader.ReadChars(4)) != "WAVE") throw new ArgumentException("Not WAVE");
                ushort format = 0, channels = 0, bits = 0;
                int rate = 0;
                byte[] data = null;
                while (reader.BaseStream.Position + 8 <= size)
                {
                    string id = new string(reader.ReadChars(4));
                    uint length = reader.ReadUInt32();
                    long start = reader.BaseStream.Position, end = start + length;
                    if (end > size) throw new ArgumentException("Truncated WAV");
                    if (id == "fmt " && length >= 16)
                    {
                        format = reader.ReadUInt16(); channels = reader.ReadUInt16(); rate = reader.ReadInt32();
                        reader.ReadUInt32(); reader.ReadUInt16(); bits = reader.ReadUInt16();
                    }
                    else if (id == "data") data = reader.ReadBytes((int)length);
                    reader.BaseStream.Position = end + (length % 2);
                }
                if (format != 1 || channels != 1 || bits != 16 || (rate != 8000 && rate != 16000 && rate != 24000) ||
                    data == null || data.Length < 2 || data.Length % 2 != 0 || data.Length / (rate * 2f) > 90)
                    throw new ArgumentException("Expected bounded mono PCM16 WAV");
                float[] samples = new float[data.Length / 2];
                for (int i = 0; i < samples.Length; i++) samples[i] = (short)(data[i * 2] | data[i * 2 + 1] << 8) / 32768f;
                AudioClip clip = AudioClip.Create("Dialogue reply", samples.Length, 1, rate, false);
                clip.SetData(samples, 0);
                return clip;
            }
        }

        void Update()
        {
            if (socket != null)
            {
                for (int count = 0; count < 64 && socket.Available > 0; ++count)
                {
                    try
                    {
                        IPEndPoint remote = new IPEndPoint(IPAddress.Loopback, 0);
                        byte[] bytes = socket.Receive(ref remote);
                        if (bytes.Length <= 12000 && IPAddress.IsLoopback(remote.Address))
                            Accept(JsonUtility.FromJson<SpeechEvent>(Encoding.UTF8.GetString(bytes)), remote);
                    }
                    catch (ArgumentException) { }
                    catch (SocketException) { break; }
                }
            }
            // Lost end packet, killed backend, or network error must not leave the mouth open.
            if ((speaking || prepared) && Time.unscaledTime - lastPacket > 0.7f) EndSpeech();
            if (clipPlaying && speaker != null && Time.unscaledTime - playbackAt > 0.1f && !speaker.isPlaying)
            {
                audioFinished = true;
                Reply("done");
                EndSpeech();
            }
            if (proxy == null && Time.unscaledTime >= nextFind)
            {
                proxy = FindObjectOfType<VRMBlendShapeProxy>();
                nextFind = Time.unscaledTime + 1;
            }
        }

        void LateUpdate()
        {
            if ((!speaking && !releaseMouth) || proxy == null) return;
            float smoothing = 1 - Mathf.Exp(-25 * Time.unscaledDeltaTime);
            value = speaking ? Mathf.Lerp(value, target, smoothing) : 0;
            for (int i = 0; i < MouthKeys.Length; i++)
            {
                float weight = clipPlaying ? phonemes[i] : i == 0 ? value : 0;
                mouthWeights[i] = !speaking ? 0 : clipPlaying ? Mathf.Lerp(mouthWeights[i], weight, smoothing) : weight;
                proxy.ImmediatelySetValue(MouthKeys[i], mouthWeights[i]);
            }
            releaseMouth = false;
        }

        void OnGUI()
        {
            if (!displayInteractive) return;
            string caption = subtitle.Length > 0 && Time.unscaledTime <= subtitleUntil ? "AI · " + subtitle
                : recognized.Length > 0 && Time.unscaledTime <= recognizedUntil ? "你 · " + recognized : "";
            if (caption.Length == 0) return;
            if (style == null)
            {
                style = new GUIStyle(GUI.skin.box) { wordWrap = true, richText = false,
                    alignment = TextAnchor.MiddleCenter, fontSize = CaptionFontSize };
                style.normal.textColor = Color.white;
            }
            float width = Mathf.Min(Screen.width - 32, 1000);
            float height = Mathf.Min(240, style.CalcHeight(new GUIContent(caption), width) + 20);
            var previousMatrix = GUI.matrix;
            try
            {
                GUI.matrix = previousMatrix * Matrix4x4.TRS(new Vector3(Screen.width, 0, 0),
                    Quaternion.identity, new Vector3(-1, 1, 1));
                GUI.Box(new Rect((Screen.width - width) / 2,
                    Screen.height * CaptionCenterFromTop - height / 2, width, height), caption, style);
            }
            finally { GUI.matrix = previousMatrix; }
        }

        void OnDisable()
        {
            EndSpeech();
            LateUpdate();
            if (socket != null) socket.Close();
            socket = null;
        }
    }
}
