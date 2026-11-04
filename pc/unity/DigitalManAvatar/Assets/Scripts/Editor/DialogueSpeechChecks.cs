using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using VRM;

namespace DigitalMan.Editor
{
    public static class DialogueSpeechChecks
    {
        const BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;
        static FieldInfo Field(string name) { return typeof(DialogueSpeechReceiver).GetField(name, Private); }
        static void Call(DialogueSpeechReceiver target, string name)
        { typeof(DialogueSpeechReceiver).GetMethod(name, Private).Invoke(target, null); }
        static void Require(bool condition, string message)
        { if (!condition) throw new InvalidOperationException(message); }

        public static void RunBatch()
        {
            try { Run(); EditorApplication.Exit(0); }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }

        static void Run()
        {
            EditorSceneManager.OpenScene("Assets/Scenes/AvatarPreview.unity", OpenSceneMode.Single);
            var proxy = UnityEngine.Object.FindObjectOfType<VRMBlendShapeProxy>();
            Require(proxy != null, "Real avatar scene has no VRM0 blendshape proxy");
            proxy.Reinitialize();
            var animator = proxy.GetComponent<Animator>();
            Require(animator != null, "No avatar Animator");
            var arm = animator.GetBoneTransform(HumanBodyBones.LeftUpperArm);
            Quaternion armBefore = arm.localRotation;
            var blink = BlendShapeKey.CreateFromPreset(BlendShapePreset.Blink_L);
            var mouth = BlendShapeKey.CreateFromPreset(BlendShapePreset.A);
            proxy.ImmediatelySetValue(blink, 0.42f);
            GameObject host = new GameObject("Dialogue check - unsaved");
            try
            {
                var target = host.AddComponent<DialogueSpeechReceiver>();
                Field("proxy").SetValue(target, proxy);
                target.Accept(new DialogueSpeechReceiver.SpeechEvent { version = 1, session = "test", sequence = 1,
                    kind = "start", text = "测试字幕 <b>literal</b>" });
                target.Accept(new DialogueSpeechReceiver.SpeechEvent { version = 1, session = "test", sequence = 2,
                    kind = "mouth", level = 0.8f });
                Require((bool)Field("speaking").GetValue(target), "Start was not accepted");
                Require(Mathf.Abs((float)Field("target").GetValue(target) - 0.8f) < 0.001f, "Mouth level missing");
                target.Accept(new DialogueSpeechReceiver.SpeechEvent { version = 1, session = "test", sequence = 1,
                    kind = "mouth", level = 0 });
                Require(Mathf.Abs((float)Field("target").GetValue(target) - 0.8f) < 0.001f, "Stale event was accepted");
                target.Accept(new DialogueSpeechReceiver.SpeechEvent { version = 1, session = "old", sequence = 99, kind = "end" });
                Require((bool)Field("speaking").GetValue(target), "Orphan end cancelled active speech");
                Field("value").SetValue(target, 0.8f);
                Call(target, "LateUpdate");
                Require(proxy.GetValue(mouth) > 0, "Real avatar mouth did not open");
                Require(Mathf.Abs(proxy.GetValue(blink) - 0.42f) < 0.001f, "Speech overwrote blink");
                Require(Quaternion.Angle(armBefore, arm.localRotation) < 0.001f, "Speech overwrote arm pose");
                target.Accept(new DialogueSpeechReceiver.SpeechEvent { version = 1, session = "test", sequence = 3, kind = "end" });
                Call(target, "LateUpdate");
                Require(proxy.GetValue(mouth) == 0, "End did not close mouth");
                proxy.ImmediatelySetValue(mouth, 0.33f);
                Call(target, "LateUpdate");
                Require(Mathf.Abs(proxy.GetValue(mouth) - 0.33f) < 0.001f, "Mouth capture was not released");
                target.Accept(new DialogueSpeechReceiver.SpeechEvent { version = 1, session = "test", sequence = 4, kind = "start" });
                Field("lastPacket").SetValue(target, Time.unscaledTime - 2);
                Call(target, "Update");
                Require(!(bool)Field("speaking").GetValue(target), "Lost end watchdog failed");
                CheckPhonemes(target, proxy);
                CheckAudioFileAndPrepare(target);
                Require(Mathf.Abs(proxy.GetValue(blink) - 0.42f) < 0.001f, "Phoneme sync overwrote blink");
                Require(Quaternion.Angle(armBefore, arm.localRotation) < 0.001f, "Phoneme sync overwrote arm pose");
                Debug.Log("DIALOGUE_SPEECH_CHECKS_OK real VRM five-vowels, MFCC analysis, local-WAV, prepare-without-play, subtitle, sequence, stale session, blink, bones, release, watchdog");
            }
            finally { UnityEngine.Object.DestroyImmediate(host); }
        }

        static void CheckPhonemes(DialogueSpeechReceiver target, VRMBlendShapeProxy proxy)
        {
            Field("speaking").SetValue(target, true);
            Field("clipPlaying").SetValue(target, true);
            target.OnLipSync(new uLipSync.LipSyncInfo { volume = 0.8f, rawVolume = 0.1f,
                phonemeRatios = new Dictionary<string, float> { { "A", 0 }, { "I", 0.75f }, { "U", 0.25f } } });
            var weights = (float[])Field("phonemes").GetValue(target);
            Require(Mathf.Abs(weights[1] - 0.6f) < 0.001f && Mathf.Abs(weights[2] - 0.2f) < 0.001f,
                    "Phoneme weights not driven by MFCC ratios");
            Array.Copy(weights, (float[])Field("mouthWeights").GetValue(target), weights.Length);
            Call(target, "LateUpdate");
            Require(proxy.GetValue(BlendShapeKey.CreateFromPreset(BlendShapePreset.I)) > 0,
                    "Real avatar I mouth not driven");
            target.OnLipSync(new uLipSync.LipSyncInfo { volume = 1, rawVolume = 0,
                phonemeRatios = new Dictionary<string, float> { { "A", 1 } } });
            Require(weights.All(value => value == 0), "Silence did not close phoneme targets");
            target.Accept(new DialogueSpeechReceiver.SpeechEvent { version = 1, session = "test", sequence = 5, kind = "end" });
            Call(target, "LateUpdate");
            foreach (var vowel in new[] { BlendShapePreset.A, BlendShapePreset.I, BlendShapePreset.U, BlendShapePreset.E, BlendShapePreset.O })
                Require(proxy.GetValue(BlendShapeKey.CreateFromPreset(vowel)) == 0, "End left a vowel open");
            var profile = Resources.Load<uLipSync.Profile>("DialogueLipSyncProfile");
            Require(profile != null && new[] { "A", "I", "U", "E", "O" }.All(profile.GetPhonemeNames().Contains),
                    "Female MFCC profile missing vowels");
            var analyzerHost = new GameObject("MFCC offline check");
            analyzerHost.SetActive(false);
            try
            {
                var analyzer = analyzerHost.AddComponent<uLipSync.uLipSync>();
                analyzer.profile = profile;
                analyzer.OnBakeStart(profile);
                try
                {
                    float[] audio = new float[8192];
                    for (int i = 0; i < audio.Length; i++) audio[i] = Mathf.Sin(i * 2 * Mathf.PI * 600 / AudioSettings.outputSampleRate) * 0.2f;
                    analyzer.OnBakeUpdate(audio, 1);
                    Require(analyzer.result.rawVolume > 0 && analyzer.result.phonemeRatios.Count >= 5,
                            "Actual MFCC job produced no analysis");
                    Require(analyzer.result.phonemeRatios.Values.All(value => !float.IsNaN(value) && !float.IsInfinity(value)),
                            "MFCC produced non-finite weights");
                }
                finally { analyzer.OnBakeEnd(); }
            }
            finally { UnityEngine.Object.DestroyImmediate(analyzerHost); }
        }

        static void CheckAudioFileAndPrepare(DialogueSpeechReceiver target)
        {
            if (Field("socket").GetValue(target) == null) Call(target, "OnEnable");
            string directory = Path.Combine(Path.GetTempPath(), "digitalman-speech-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(directory);
            string path = Path.Combine(directory, "reply.wav");
            try
            {
                using (var writer = new BinaryWriter(File.Create(path)))
                {
                    writer.Write(Encoding.ASCII.GetBytes("RIFF")); writer.Write(36 + 3200);
                    writer.Write(Encoding.ASCII.GetBytes("WAVEfmt ")); writer.Write(16);
                    writer.Write((ushort)1); writer.Write((ushort)1); writer.Write(16000); writer.Write(32000);
                    writer.Write((ushort)2); writer.Write((ushort)16);
                    writer.Write(Encoding.ASCII.GetBytes("data")); writer.Write(3200); writer.Write(new byte[3200]);
                }
                var clip = DialogueSpeechReceiver.ReadLocalWav(path);
                Require(clip.samples == 1600 && clip.frequency == 16000, "WAV decoded incorrectly");
                UnityEngine.Object.DestroyImmediate(clip);
                bool rejected = false;
                try { DialogueSpeechReceiver.ReadLocalWav("C:/Windows/win.ini"); }
                catch (ArgumentException) { rejected = true; }
                Require(rejected, "Arbitrary file path accepted");
                using (var client = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0)))
                {
                    client.Client.ReceiveTimeout = 1000;
                    var prepare = new DialogueSpeechReceiver.SpeechEvent { version = 1, session = "test", sequence = 6,
                        utterance = Guid.NewGuid().ToString("N"), kind = "prepare", audio_path = path, text = "静音检查" };
                    byte[] data = Encoding.UTF8.GetBytes(JsonUtility.ToJson(prepare));
                    client.Send(data, data.Length, new IPEndPoint(IPAddress.Loopback, 39545));
                    System.Threading.Thread.Sleep(30);
                    Call(target, "Update");
                    IPEndPoint remote = null;
                    var response = JsonUtility.FromJson<DialogueSpeechReceiver.SpeechEvent>(Encoding.UTF8.GetString(client.Receive(ref remote)));
                    Require(response.kind == "ready" && response.utterance == prepare.utterance, "Audio prepare not acknowledged");
                    Require((bool)Field("prepared").GetValue(target) && !(bool)Field("clipPlaying").GetValue(target),
                            "Prepare started audio before commit");
                    Require(!((AudioSource)Field("speaker").GetValue(target)).isPlaying, "Prepare emitted audio");
                    Field("lastPacket").SetValue(target, Time.unscaledTime - 2);
                    Call(target, "Update");
                    Require(!(bool)Field("prepared").GetValue(target), "Expired audio prepare was not discarded");
                }
            }
            finally { File.Delete(path); Directory.Delete(directory); }
        }
    }
}
