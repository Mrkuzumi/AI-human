using System;
using System.Diagnostics;
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using VRM;

namespace DigitalMan.Editor
{
    public static class DialoguePlaybackChecks
    {
        const string Pending = "DigitalMan.DialoguePlaybackChecks";
        static Process child;
        static double started;
        static uLipSync.uLipSync analyzer;
        static VRMBlendShapeProxy avatar;
        static float maxVolume, maxMouth;

        [InitializeOnLoadMethod]
        static void Register() { EditorApplication.update += Tick; }

        public static void RunBatch()
        {
            EditorSceneManager.OpenScene("Assets/Scenes/AvatarPreview.unity", OpenSceneMode.Single);
            SessionState.SetBool(Pending, true);
            EditorApplication.EnterPlaymode();
        }

        static string Argument(string name)
        {
            string[] arguments = Environment.GetCommandLineArgs();
            int index = Array.IndexOf(arguments, name);
            if (index < 0 || index + 1 >= arguments.Length) throw new ArgumentException("Missing " + name);
            return arguments[index + 1];
        }

        static void Tick()
        {
            if (!SessionState.GetBool(Pending, false) || !EditorApplication.isPlaying || EditorApplication.isCompiling) return;
            try
            {
                if (child == null)
                {
                    var receiver = UnityEngine.Object.FindObjectOfType<DialogueSpeechReceiver>();
                    if (receiver == null) return;
                    typeof(DialogueSpeechReceiver).GetMethod("EnsureAudio", BindingFlags.Instance | BindingFlags.NonPublic).Invoke(receiver, null);
                    analyzer = receiver.GetComponentInChildren<uLipSync.uLipSync>();
                    analyzer.outputSoundGain = 0; // Analyze real samples while keeping this check silent.
                    avatar = UnityEngine.Object.FindObjectOfType<VRMBlendShapeProxy>();
                    started = EditorApplication.timeSinceStartup;
                    child = Process.Start(new ProcessStartInfo(Argument("--dialogue-test-python"),
                        "-u -B \"" + Argument("--dialogue-test-script") + "\"") {
                        UseShellExecute = false, CreateNoWindow = true, RedirectStandardOutput = true, RedirectStandardError = true });
                    return;
                }
                maxVolume = Mathf.Max(maxVolume, analyzer.result.rawVolume);
                foreach (var vowel in new[] { BlendShapePreset.A, BlendShapePreset.I, BlendShapePreset.U, BlendShapePreset.E, BlendShapePreset.O })
                    maxMouth = Mathf.Max(maxMouth, avatar.GetValue(BlendShapeKey.CreateFromPreset(vowel)));
                if (!child.HasExited)
                {
                    if (EditorApplication.timeSinceStartup - started > 20) throw new TimeoutException("Unity playback check timed out");
                    return;
                }
                string output = child.StandardOutput.ReadToEnd() + child.StandardError.ReadToEnd();
                if (child.ExitCode != 0 || !output.Contains("UNITY_PLAYBACK_OK")) throw new InvalidOperationException(output);
                if (maxVolume < 0.01f || maxMouth < 0.01f) throw new InvalidOperationException("Actual audio did not drive MFCC and avatar mouth: " + maxVolume + "/" + maxMouth);
                UnityEngine.Debug.Log(output);
                UnityEngine.Debug.Log("UNITY_AUDIO_MOUTH_OK volume=" + maxVolume + " mouth=" + maxMouth);
                Finish(0);
            }
            catch (Exception error) { UnityEngine.Debug.LogException(error); Finish(1); }
        }

        static void Finish(int code)
        {
            SessionState.SetBool(Pending, false);
            if (child != null)
            {
                if (!child.HasExited) child.Kill();
                child.Dispose(); child = null;
            }
            EditorApplication.Exit(code);
        }
    }
}
