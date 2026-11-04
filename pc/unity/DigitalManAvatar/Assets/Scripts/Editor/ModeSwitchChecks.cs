using System;
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace DigitalMan.Editor
{
    public static class ModeSwitchChecks
    {
        const BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;

        static object Field(GestureTriggerReceiver target, string name)
        {
            return typeof(GestureTriggerReceiver).GetField(name, Private).GetValue(target);
        }

        static void Call(GestureTriggerReceiver target, string method, params object[] args)
        {
            typeof(GestureTriggerReceiver).GetMethod(method, Private).Invoke(target, args);
        }

        static void Require(bool condition, string message)
        {
            if (!condition) throw new InvalidOperationException(message);
        }

        public static void RunBatch()
        {
            try { Run(); EditorApplication.Exit(0); }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }

        static void Run()
        {
            // Open without saving: the test uses the real scene's controller and bindings.
            EditorSceneManager.OpenScene("Assets/Scenes/AvatarPreview.unity", OpenSceneMode.Single);
            var target = UnityEngine.Object.FindObjectOfType<GestureTriggerReceiver>();
            Require(target != null, "Scene is missing GestureTriggerReceiver");
            Call(target, "Awake");
            var animator = (Animator)Field(target, "animator");
            var controller = (RuntimeAnimatorController)Field(target, "cachedController");
            var receiver = (EVMC4U.ExternalReceiver)Field(target, "receiver");
            var bindings = (GestureTriggerReceiver.GestureBinding[])Field(target, "gestures");
            Require(animator != null && controller != null && receiver != null, "Missing scene dependencies");
            Require(bindings.Length == 3, "Expected the three board gesture bindings");

            Call(target, "ApplyMode", false);
            Require(animator.runtimeAnimatorController == null, "Follow must release Animator ownership");
            foreach (var binding in bindings) binding.lastActive = 1;
            Call(target, "ApplyMode", true);
            Require(animator.runtimeAnimatorController == controller, "Interact must restore Controller");
            foreach (var binding in bindings)
            {
                Require(binding.waitForRelease, "A held gesture must not replay on mode entry");
                Call(target, "HandleGesture", binding, 1);
                Require(!(bool)Field(target, "saved"), "Held gesture unexpectedly froze capture");
                Call(target, "HandleGesture", binding, 0);
                Require(!binding.waitForRelease, "Release must rearm the next gesture");
            }

            bool freeze = receiver.Freeze;
            bool blend = receiver.BlendShapeSynchronize;
            Call(target, "Begin", GestureTriggerReceiver.HandOver.WholeBody);
            Require(receiver.Freeze, "Interaction did not take bone ownership");
            Call(target, "ApplyMode", true);
            Require(receiver.Freeze && (bool)Field(target, "saved"), "Duplicate mode interrupted animation");
            Call(target, "ApplyMode", false);
            Require(receiver.Freeze == freeze && receiver.BlendShapeSynchronize == blend,
                    "Mid-action switch failed to restore receiver state");
            Require(!(bool)Field(target, "saved") && animator.runtimeAnimatorController == null,
                    "Follow retained interaction ownership");

            Call(target, "ApplyMode", true);
            Call(target, "Begin", GestureTriggerReceiver.HandOver.FaceOnly);
            Require(!receiver.BlendShapeSynchronize, "Face-only handover did not start");
            Call(target, "OnDisable");
            Require(receiver.BlendShapeSynchronize == blend && !(bool)Field(target, "saved"),
                    "Disabling component leaked receiver ownership");
            Debug.Log("MODE_SWITCH_CHECKS_OK follow/interact, duplicate, held gesture, mid-action restore, disable");
        }
    }
}
