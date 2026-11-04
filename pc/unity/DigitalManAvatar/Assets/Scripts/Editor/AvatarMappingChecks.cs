using System;
using System.IO;
using System.Linq;
using EVMC4U;
using UnityEditor;
using UnityEngine;
using VRM;

namespace DigitalMan.Editor
{
    // Tests the Python output against the actual imported rig and VMC receiver.
    public static class AvatarMappingChecks
    {
        [Serializable] public class Bone { public string name; public Quaternion rotation; }
        [Serializable] public class Segment { public string start, end; public Vector2 imageDirection; }
        [Serializable] public class Radial { public string side; public Vector3 direction; }
        [Serializable] public class Pose { public Bone[] bones; public Segment[] segments; public Radial[] handEdges; }
        [Serializable] public class Cases { public Pose[] poses; }
        [Serializable] public class Binding
        {
            public string humanoid, transform, parent;
            public Vector3 localPosition, modelPosition;
            public Quaternion localRotation;
        }
        [Serializable] public class Morph { public string clip, path, target; public int index; public float weight; }
        [Serializable] public class Report { public Binding[] bones; public Morph[] morphs; public int posesChecked, handEdgesChecked; public float maxProjectionErrorDegrees, maxHandEdgeErrorDegrees; }

        static void Require(bool condition, string message)
        {
            if (!condition) throw new InvalidOperationException(message);
        }

        static Transform BoneTransform(Animator animator, string name) =>
            animator.GetBoneTransform((HumanBodyBones)Enum.Parse(typeof(HumanBodyBones), name));

        static void Send(ExternalReceiver receiver, string address, params object[] values)
        {
            var message = new uOSC.Message { address = address, values = values };
            receiver.MessageDaisyChain(ref message, 0);
        }

        public static void RunBatch()
        {
            try { Run(); EditorApplication.Exit(0); }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }

        [MenuItem("DigitalMan/Verify landmark mapping")]
        public static void Run()
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Models/model.prefab");
            Require(prefab != null, "Avatar prefab is missing");
            var avatar = UnityEngine.Object.Instantiate(prefab);
            var go = new GameObject("Landmark mapping check receiver");
            try
            {
                avatar.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
                var animator = avatar.GetComponent<Animator>();
                Require(animator != null && animator.isHuman, "The model must have a valid Humanoid Avatar");
                animator.runtimeAnimatorController = null;
                var report = new Report();
                report.bones = Enum.GetValues(typeof(HumanBodyBones)).Cast<HumanBodyBones>()
                    .Where(id => id != HumanBodyBones.LastBone && animator.GetBoneTransform(id) != null)
                    .Select(id => {
                        var bone = animator.GetBoneTransform(id);
                        return new Binding { humanoid = id.ToString(), transform = bone.name, parent = bone.parent.name,
                            localPosition = bone.localPosition, localRotation = bone.localRotation,
                            modelPosition = avatar.transform.InverseTransformPoint(bone.position) };
                    }).ToArray();
                var proxy = avatar.GetComponent<VRMBlendShapeProxy>();
                Require(proxy != null, "VRM0 face proxy is missing");
                proxy.Reinitialize();
                report.morphs = proxy.BlendShapeAvatar.Clips.SelectMany(clip => clip.Values.Select(binding => {
                    var mesh = avatar.transform.Find(binding.RelativePath).GetComponent<SkinnedMeshRenderer>().sharedMesh;
                    return new Morph { clip = clip.BlendShapeName, path = binding.RelativePath, index = binding.Index,
                        target = mesh.GetBlendShapeName(binding.Index), weight = binding.Weight };
                })).ToArray();
                var receiver = go.AddComponent<ExternalReceiver>();
                receiver.Model = avatar;
                receiver.BonePositionSynchronize = false;
                receiver.BoneRotationFilterEnable = false;
                receiver.BlendShapeFilterEnable = false;
                Require(!receiver.CorrectArmBodyPenetration, "Avoidance must be opt-in for measured alignment");
                receiver.Start();
                receiver.Process();
                var data = AssetDatabase.LoadAssetAtPath<TextAsset>("Assets/Editor/AvatarMappingCases.json");
                Require(data != null, "Python mapping cases are missing");
                foreach (var pose in JsonUtility.FromJson<Cases>(data.text).poses)
                {
                    foreach (var bone in pose.bones)
                        Send(receiver, "/VMC/Ext/Bone/Pos", bone.name, 0f, 0f, 0f,
                            bone.rotation.x, bone.rotation.y, bone.rotation.z, bone.rotation.w);
                    receiver.Process();
                    foreach (var bone in pose.bones)
                        Require(Quaternion.Angle(BoneTransform(animator, bone.name).localRotation, bone.rotation) < .05f,
                            "Receiver changed measured rotation: " + bone.name);
                    foreach (var segment in pose.segments)
                    {
                        var direction = BoneTransform(animator, segment.end).position - BoneTransform(animator, segment.start).position;
                        float error = Vector2.Angle(new Vector2(direction.x, direction.y), segment.imageDirection);
                        report.maxProjectionErrorDegrees = Mathf.Max(report.maxProjectionErrorDegrees, error);
                        Require(error < .2f, "Real rig projection differs from input: " + segment.start + " " + error);
                    }
                    if (pose.handEdges != null)
                        foreach (var edge in pose.handEdges)
                        {
                            var hand = BoneTransform(animator, edge.side + "Hand");
                            var radial = BoneTransform(animator, edge.side + "IndexProximal").position -
                                         BoneTransform(animator, edge.side + "LittleProximal").position;
                            radial = Vector3.ProjectOnPlane(radial, hand.right).normalized;
                            float error = Vector3.Angle(radial, edge.direction);
                            report.maxHandEdgeErrorDegrees = Mathf.Max(report.maxHandEdgeErrorDegrees, error);
                            Require(error < .1f, "Actual index/little edge reversed: " + edge.side + " " + error);
                            report.handEdgesChecked++;
                        }
                    report.posesChecked++;
                }
                foreach (var bind in report.bones)
                    Require(Vector3.Distance(BoneTransform(animator, bind.humanoid).localPosition, bind.localPosition) < .00001f,
                        "VMC zero positions stretched the rig: " + bind.humanoid);

                var face = avatar.transform.Find("Face").GetComponent<SkinnedMeshRenderer>();
                string[] controls = { "Blink_L", "Blink_R", "A", "BrowUp", "BrowDown" };
                int[] indices = { 15, 14, 39, 10, 9 };
                for (int active = 0; active < controls.Length; active++)
                {
                    for (int i = 0; i < controls.Length; i++)
                        Send(receiver, "/VMC/Ext/Blend/Val", controls[i], i == active ? .72f : 0f);
                    Send(receiver, "/VMC/Ext/Blend/Apply");
                    for (int i = 0; i < controls.Length; i++)
                        Require(Mathf.Abs(face.GetBlendShapeWeight(indices[i]) - (i == active ? 72f : 0f)) < .05f,
                            "VMC facial control bound incorrectly: " + controls[i]);
                }
                string output = Path.GetFullPath(Path.Combine(Application.dataPath, "../AvatarMappingCheck.json"));
                File.WriteAllText(output, JsonUtility.ToJson(report, true));
                Debug.Log("AVATAR_MAPPING_CHECKS_OK poses=" + report.posesChecked +
                    " maxProjectionErrorDegrees=" + report.maxProjectionErrorDegrees +
                    " handEdges=" + report.handEdgesChecked + " maxHandEdgeErrorDegrees=" + report.maxHandEdgeErrorDegrees + " report=" + output);
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(go);
                UnityEngine.Object.DestroyImmediate(avatar);
            }
        }
    }
}
