using System;
using System.Linq;
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Animations;
using UnityEngine.Playables;

namespace DigitalMan.Editor
{
    public static class InteractionVoiceChecks
    {
        const BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;
        static void Call(object target, string method, params object[] args) =>
            target.GetType().GetMethod(method, Private).Invoke(target, args);
        static object Field(object target, string name) => target.GetType().GetField(name, Private).GetValue(target);
        static void Require(bool condition, string message)
        { if (!condition) throw new InvalidOperationException(message); }

        public static void RunBatch()
        {
            try { Run(); EditorApplication.Exit(0); }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }

        static void Run()
        {
            // Reuse the existing ownership/held-gesture checks before sampling all clips.
            typeof(ModeSwitchChecks).GetMethod("Run", BindingFlags.Static | BindingFlags.NonPublic).Invoke(null, null);
            EditorSceneManager.OpenScene("Assets/Scenes/AvatarPreview.unity", OpenSceneMode.Single);
            var target = UnityEngine.Object.FindObjectOfType<GestureTriggerReceiver>();
            Call(target, "Awake");
            var animator = (Animator)Field(target, "animator");
            var bones = (Transform[])Field(target, "standingBones");
            var positions = bones.Select(t => t == null ? Vector3.zero : t.localPosition).ToArray();
            var rotations = bones.Select(t => t == null ? Quaternion.identity : t.localRotation).ToArray();
            var hand = animator.GetBoneTransform(HumanBodyBones.LeftHand);
            var head = animator.GetBoneTransform(HumanBodyBones.Head);
            Call(target, "ApplyMode", true);
            animator.runtimeAnimatorController = null; // Sample the same clips through a deterministic playable.
            float maxResponse = 0;
            Vector3 idleHand = Vector3.zero;
            int sampled = 0;
            foreach (var name in new[] { "HeadPatIdle", "HeadPatAction", "TouchFaceAction", "HugAction" })
            {
                var clip = AssetDatabase.LoadAssetAtPath<AnimationClip>("Assets/Animations/" + name + ".anim");
                var graph = PlayableGraph.Create("Interaction standing check");
                try
                {
                    var playable = AnimationClipPlayable.Create(graph, clip);
                    AnimationPlayableOutput.Create(graph, "clip", animator).SetSourcePlayable(playable);
                    graph.Play();
                    for (int step = 0; step <= 6; step++)
                    {
                        playable.SetTime(clip.length * step / 6);
                        graph.Evaluate(0);
                        Call(target, "LateUpdate");
                        for (int i = 0; i < bones.Length; i++)
                        {
                            if (bones[i] == null) continue;
                            Require(Vector3.Distance(bones[i].localPosition, positions[i]) < .0001f, "Standing translation lost: " + bones[i].name);
                            Require(Quaternion.Angle(bones[i].localRotation, rotations[i]) < .05f, "Standing leg rotated: " + bones[i].name);
                        }
                        foreach (var t in new[] { head, bones[3], bones[7] })
                        {
                            var v = Camera.main.WorldToViewportPoint(t.position);
                            Require(v.z > 0 && v.x > .02f && v.x < .98f && v.y > .02f && v.y < .98f,
                                "Head/feet outside frame: " + name + " " + t.name + " " + v);
                        }
                        if (name == "HeadPatIdle") idleHand = hand.position;
                        else maxResponse = Mathf.Max(maxResponse, Vector3.Distance(hand.position, idleHand));
                        sampled++;
                    }
                }
                finally { graph.Destroy(); }
            }
            Require(maxResponse > .08f, "Standing correction stopped upper-body reactions");
            Call(target, "ApplyMode", false);
            var changed = bones[1].localRotation * Quaternion.Euler(0, 0, 20);
            bones[1].localRotation = changed;
            Call(target, "LateUpdate");
            Require(Quaternion.Angle(bones[1].localRotation, changed) < .05f, "Follow mode still constrained the legs");

            var host = new GameObject("Unsaved subtitle check");
            host.SetActive(false); // No sockets or audio: leave the user's production receiver alone.
            try
            {
                var speech = host.AddComponent<DialogueSpeechReceiver>();
                Action<string, string, int> display = (kind, text, seq) => speech.Accept(new DialogueSpeechReceiver.SpeechEvent {
                    version = 1, session = "display", sequence = seq, kind = kind, text = text });
                display("display_mode", "interact", 1);
                display("subtitle", "识别字幕", 2);
                Require((string)Field(speech, "recognized") == "识别字幕", "ASR subtitle missing");
                speech.Accept(new DialogueSpeechReceiver.SpeechEvent { version = 1, session = "speaker", sequence = 1, kind = "start", text = "回答" });
                Require((bool)Field(speech, "speaking"), "Display channel overwrote speech session");
                display("display_mode", "follow", 3);
                Require(!(bool)Field(speech, "speaking") && (string)Field(speech, "subtitle") == "" &&
                        (string)Field(speech, "recognized") == "", "Follow failed to stop/hide speech and captions");
                display("subtitle", "必须隐藏", 4);
                speech.Accept(new DialogueSpeechReceiver.SpeechEvent { version = 1, session = "speaker", sequence = 2, kind = "start", text = "旧回答" });
                Require(!(bool)Field(speech, "speaking") && (string)Field(speech, "recognized") == "", "Stale reply escaped follow mode");
                display("display_mode", "interact", 5);
                display("subtitle", "恢复字幕", 6);
                Require((string)Field(speech, "recognized") == "恢复字幕", "Interaction failed to restore captions");
            }
            finally { UnityEngine.Object.DestroyImmediate(host); }
            Debug.Log("INTERACTION_VOICE_CHECKS_OK samples=" + sampled + " upperBodyResponse=" + maxResponse +
                " full-body-in-frame, three gestures, follow release, caption/audio gating");
        }
    }
}
