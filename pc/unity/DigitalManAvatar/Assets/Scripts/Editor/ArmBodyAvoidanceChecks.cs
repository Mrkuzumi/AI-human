using System;
using System.Collections.Generic;
using System.IO;
using EVMC4U;
using UnityEditor;
using UnityEngine;

namespace DigitalMan.Editor
{
    public static class ArmBodyAvoidanceChecks
    {
        public static void RunBatch()
        {
            try { Run(); EditorApplication.Exit(0); }
            catch (Exception e) { Debug.LogException(e); EditorApplication.Exit(1); }
        }
        static void Require(bool value, string message) { if (!value) throw new InvalidOperationException(message); }
        static void MeshClear(GameObject avatar, Transform[] bones, string label)
        {
            try { CheckGarmentMesh(avatar, bones); }
            catch (Exception e) { throw new InvalidOperationException(label + ": " + e.Message); }
        }
        static void Pose(Transform[] b, float leftRoll, float leftBend, float leftDepth,
            float rightRoll, float rightBend, float rightDepth, Quaternion leftPalm, Quaternion rightPalm)
        {
            b[0].localRotation = Quaternion.AngleAxis(leftRoll, Vector3.forward) * Quaternion.AngleAxis(leftDepth, Vector3.up);
            b[1].localRotation = Quaternion.AngleAxis(leftBend, Vector3.forward);
            b[2].localRotation = leftPalm;
            b[3].localRotation = Quaternion.AngleAxis(rightRoll, Vector3.forward) * Quaternion.AngleAxis(-rightDepth, Vector3.up);
            b[4].localRotation = Quaternion.AngleAxis(rightBend, Vector3.forward);
            b[5].localRotation = rightPalm;
        }
        [MenuItem("DigitalMan/Verify arm body avoidance")]
        public static void Run()
        {
            var avatar = UnityEngine.Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Models/model.prefab"));
            try
            {
                var animator = avatar.GetComponent<Animator>();
                var ids = new[] { HumanBodyBones.LeftUpperArm, HumanBodyBones.LeftLowerArm, HumanBodyBones.LeftHand,
                    HumanBodyBones.RightUpperArm, HumanBodyBones.RightLowerArm, HumanBodyBones.RightHand };
                Transform[] b = Array.ConvertAll(ids, id => animator.GetBoneTransform(id));
                Vector3[] positions = Array.ConvertAll(b, bone => bone.localPosition);
                VerifyReceiver(avatar, b);
                Debug.Log("CONTINUOUS_RECEIVER_OK Process/LateUpdate/Freeze");
                foreach (float depth in new[] { -15f, 0f, 10f, 30f })
                {
                    var solver = new ArmBodyAvoidance();
                    for (int frame = 0; frame < 50; frame++)
                    {
                        Pose(b, 90f, 0f, depth, -88f, -5f, 0f, Quaternion.Euler(0f, 0f, 180f), Quaternion.identity);
                        Quaternion lp = b[2].rotation, rp = b[5].rotation;
                        solver.Apply(animator, true, true, 1f / 30f);
                        Require(Quaternion.Angle(lp, b[2].rotation) < 0.05f && Quaternion.Angle(rp, b[5].rotation) < 0.05f,
                            "Standing palm changed");
                    }
                    float leftAngle = Vector3.Angle(Vector3.down, b[1].position - b[0].position);
                    float rightAngle = Vector3.Angle(Vector3.down, b[4].position - b[3].position);
                    Debug.Log($"CONTINUOUS_STANDING depth={depth} left={leftAngle:F2} right={rightAngle:F2}");
                    Require(leftAngle < 40f && rightAngle < 28f, "Asymmetric standing arm was raised");
                    MeshClear(avatar, b, "Standing depth=" + depth);
                    if (depth == 0f)
                    {
                        Render(avatar, "continuous_standing_front.png", false);
                        Render(avatar, "continuous_standing_side.png", true);
                    }
                }
                foreach (int moving in new[] { 0, 1 })
                {
                    var solver = new ArmBodyAvoidance();
                    Quaternion pu = Quaternion.identity, pl = Quaternion.identity;
                    Vector3 pw = Vector3.zero;
                    float maxAngle = 0f, maxDistance = 0f;
                    for (int i = -40; i <= 360; i++)
                    {
                        float roll = i < 0 ? 90f : i <= 180 ? 90f - i : i - 270f;
                        Pose(b, moving == 0 ? roll : 90f, 0f, 10f,
                            moving == 1 ? -roll : -90f, 0f, 10f, Quaternion.identity, Quaternion.identity);
                        Quaternion lp = b[2].rotation, rp = b[5].rotation;
                        solver.Apply(animator, true, true, 1f / 30f);
                        int k = moving == 0 ? 0 : 3;
                        if (i > -40)
                        {
                            maxAngle = Mathf.Max(maxAngle, Quaternion.Angle(pu, b[k].localRotation),
                                Quaternion.Angle(pl, b[k + 1].localRotation));
                            maxDistance = Mathf.Max(maxDistance, Vector3.Distance(pw, b[k + 2].position));
                            Require(maxAngle < 6f, $"Slow raise jumped: side={moving} frame={i} max={maxAngle:F2}");
                            Require(maxDistance < 0.035f, "Slow raise wrist jumped");
                        }
                        Require(Quaternion.Angle(lp, b[2].rotation) < 0.05f && Quaternion.Angle(rp, b[5].rotation) < 0.05f,
                            "Slow raise palm changed");
                        pu = b[k].localRotation; pl = b[k + 1].localRotation; pw = b[k + 2].position;
                        if (i >= 0) MeshClear(avatar, b, $"Slow raise side={moving} frame={i}");
                        if (i == 0 || i == 45 || i == 90 || i == 180)
                            Render(avatar, $"continuous_arm{moving}_{i}.png", false);
                        if (i == 90) Require(Mathf.Abs(b[k + 2].position.y - b[k].position.y) < 0.05f,
                            "Slow raise stayed down instead of reaching horizontal");
                        if (i == 180) Require(b[k + 2].position.y - b[k].position.y > 0.35f,
                            "Slow raise did not reach above head");
                    }
                    Debug.Log($"CONTINUOUS_RAISE side={moving} maxJointStep={maxAngle:F3} maxWristStep={maxDistance:F4}");
                }
                {
                    var solver = new ArmBodyAvoidance();
                    Quaternion pu = Quaternion.identity, pl = Quaternion.identity;
                    float maxStep = 0f;
                    for (int i = -40; i <= 160; i++)
                    {
                        float t = i < 0 ? 0f : i <= 80 ? i / 80f : (160 - i) / 80f;
                        float roll = Mathf.Lerp(90f, 78f, t), bend = Mathf.Lerp(0f, 55f, t);
                        Pose(b, roll, bend, 10f, -roll, -bend, 10f, Quaternion.identity, Quaternion.identity);
                        solver.Apply(animator, true, true, 1f / 30f);
                        if (i > -40) maxStep = Mathf.Max(maxStep, Quaternion.Angle(pu, b[0].localRotation),
                            Quaternion.Angle(pl, b[1].localRotation));
                        pu = b[0].localRotation; pl = b[1].localRotation;
                        Require(maxStep < 6f, $"Clasp transition jumped: frame={i} max={maxStep:F2}");
                        if (i >= 0) MeshClear(avatar, b, "Clasp frame=" + i);
                        if (i == 80)
                        {
                            Debug.Log($"CONTINUOUS_CLASP_POSITION left={b[2].position:F4} right={b[5].position:F4}");
                            Require(Mathf.Abs(b[2].position.x) < 0.10f && Mathf.Abs(b[5].position.x) < 0.10f &&
                                b[2].position.z > 0.10f && b[5].position.z > 0.10f,
                                "Clasp stayed at sides instead of reaching the front");
                            Render(avatar, "continuous_clasp_front.png", false);
                            Render(avatar, "continuous_clasp_side.png", true);
                        }
                    }
                    Debug.Log($"CONTINUOUS_CLASP maxJointStep={maxStep:F3}");
                }
                foreach (int touching in new[] { 0, 1 })
                {
                    var solver = new ArmBodyAvoidance();
                    Vector3 goal = Vector3.zero;
                    for (int i = 0; i < 60; i++)
                    {
                        Pose(b, touching == 0 ? 0f : 90f, touching == 0 ? -110f : 0f, 10f,
                            touching == 1 ? 0f : -90f, touching == 1 ? 110f : 0f, 10f,
                            Quaternion.identity, Quaternion.identity);
                        goal = b[touching == 0 ? 2 : 5].position;
                        solver.Apply(animator, true, true, 1f / 30f);
                    }
                    Require(Vector3.Distance(goal, b[touching == 0 ? 2 : 5].position) < 0.015f, "Head-touch target moved");
                    MeshClear(avatar, b, "Head touch side=" + touching);
                }
                Debug.Log("CONTINUOUS_HEAD_TOUCH_OK both sides");
                {
                    var solver = new ArmBodyAvoidance();
                    Quaternion prev = Quaternion.identity;
                    float maxStep = 0f;
                    for (int i = -40; i <= 200; i++)
                    {
                        float depth = i < 0 ? 20f : 20f + 10f * Mathf.Sin(i * Mathf.PI / 200f);
                        Pose(b, 90f, 0f, depth, -90f, 0f, 0f,
                            Quaternion.Euler(0f, 0f, i < 0 ? 0f : i * 0.5f), Quaternion.identity);
                        solver.Apply(animator, true, true, 1f / 30f);
                        if (i > -40) maxStep = Mathf.Max(maxStep, Quaternion.Angle(prev, b[0].localRotation));
                        prev = b[0].localRotation;
                        Require(maxStep < 4f, "25-degree depth boundary jumped");
                        Require(Vector3.Angle(Vector3.down, b[1].position - b[0].position) < 38f,
                            "Depth noise raised standing left arm");
                        if (i >= 0) MeshClear(avatar, b, "Depth boundary frame=" + i);
                    }
                    Debug.Log($"CONTINUOUS_DEPTH_BOUNDARY maxJointStep={maxStep:F3}");
                }
                Transform hips = animator.GetBoneTransform(HumanBodyBones.Hips);
                Quaternion hipsBind = hips.localRotation;
                Vector3 reference = Vector3.zero;
                foreach (float yaw in new[] { 0f, 90f, 180f, -90f })
                {
                    var solver = new ArmBodyAvoidance();
                    hips.localRotation = Quaternion.AngleAxis(yaw, Vector3.up) * hipsBind;
                    for (int i = 0; i < 60; i++)
                    {
                        Pose(b, 78f, 55f, 10f, -78f, -55f, 10f, Quaternion.identity, Quaternion.identity);
                        solver.Apply(animator, true, true, 1f / 30f);
                    }
                    MeshClear(avatar, b, "Turned clasp yaw=" + yaw);
                    Vector3 relative = hips.InverseTransformPoint(b[2].position);
                    if (yaw == 0f) reference = relative;
                    else Require(Vector3.Distance(reference, relative) < 0.001f, "Clasp changed with body yaw");
                }
                hips.localRotation = hipsBind;
                Debug.Log("CONTINUOUS_TURN_OK 0/90/180/-90");
                var random = new System.Random(39540);
                for (int i = 0; i < 120; i++)
                {
                    avatar.transform.SetPositionAndRotation(new Vector3(1f, 0.2f, -2f), Quaternion.Euler(0f, 37f, 8f));
                    avatar.transform.localScale = Vector3.one * 1.3f;
                    float lu = (float)random.NextDouble() * 130f - 20f, ll = (float)random.NextDouble() * 220f - 110f;
                    float ld = (float)random.NextDouble() * 120f - 60f;
                    float ru = -(float)random.NextDouble() * 130f + 20f, rl = (float)random.NextDouble() * 220f - 110f;
                    float rd = (float)random.NextDouble() * 120f - 60f;
                    Quaternion lp = Quaternion.Euler((float)random.NextDouble() * 180f, (float)random.NextDouble() * 180f,
                        (float)random.NextDouble() * 180f);
                    Quaternion rp = Quaternion.Euler((float)random.NextDouble() * 180f, (float)random.NextDouble() * 180f,
                        (float)random.NextDouble() * 180f);
                    var solver = new ArmBodyAvoidance();
                    for (int j = 0; j < 30; j++)
                    {
                        Pose(b, lu, ll, ld, ru, rl, rd, lp, rp);
                        Quaternion leftPalm = b[2].rotation, rightPalm = b[5].rotation;
                        solver.Apply(animator, true, true, 1f / 30f);
                        Require(Quaternion.Angle(leftPalm, b[2].rotation) < 0.05f &&
                            Quaternion.Angle(rightPalm, b[5].rotation) < 0.05f, "Random palm changed");
                    }
                    MeshClear(avatar, b, $"Random pose={i} L={lu:F1},{ll:F1},{ld:F1} R={ru:F1},{rl:F1},{rd:F1}");
                }
                Debug.Log("CONTINUOUS_RANDOM_OK 120 poses including world rotation and scale");
                Pose(b, 78f, 55f, 10f, -78f, -55f, 10f, Quaternion.identity, Quaternion.identity);
                Quaternion cutUpper = b[0].localRotation, cutLower = b[1].localRotation;
                new ArmBodyAvoidance().Apply(animator, false, true, 1f / 30f);
                Require(Quaternion.Angle(cutUpper, b[0].localRotation) < 0.01f &&
                    Quaternion.Angle(cutLower, b[1].localRotation) < 0.01f, "Cutoff arm changed");
                for (int i = 0; i < b.Length; i++)
                    Require(Vector3.Distance(positions[i], b[i].localPosition) < 0.000001f, "Bone length changed");
                Debug.Log("ARM_CONTINUITY_OK: asymmetric standing, both slow raises and returns, clasp transition, actual garment triangles, palm orientation, bone lengths");
            }
            finally { UnityEngine.Object.DestroyImmediate(avatar); }
        }
        static void CheckGarmentMesh(GameObject avatar, Transform[] bones)
        {
            // Ray tests use the real torso/coat triangles, independently of the avoidance capsule.
            foreach (var skin in avatar.GetComponentsInChildren<SkinnedMeshRenderer>())
            {
                if (skin.sharedMesh == null) continue;
                var baked = new Mesh();
                skin.BakeMesh(baked);
                var weights = skin.sharedMesh.boneWeights;
                var body = Array.ConvertAll(skin.bones, bone => bone != null &&
                    (bone.name.Contains("C_Hips") || bone.name.Contains("C_Spine") ||
                     bone.name.Contains("C_Chest") || bone.name.Contains("C_UpperChest")));
                Func<int, bool> torso = vi => {
                    if (vi >= weights.Length) return false;
                    BoneWeight w = weights[vi];
                    float sum = (body[w.boneIndex0] ? w.weight0 : 0f) +
                        (body[w.boneIndex1] ? w.weight1 : 0f) +
                        (body[w.boneIndex2] ? w.weight2 : 0f) +
                        (body[w.boneIndex3] ? w.weight3 : 0f);
                    return sum > 0.75f;
                };
                var triangles = new List<int>();
                int[] all = baked.triangles;
                for (int i = 0; i < all.Length; i += 3)
                    if (torso(all[i]) && torso(all[i + 1]) && torso(all[i + 2]))
                    { triangles.Add(all[i]); triangles.Add(all[i + 1]); triangles.Add(all[i + 2]); }
                if (triangles.Count == 0) { UnityEngine.Object.DestroyImmediate(baked); continue; }
                baked.triangles = triangles.ToArray();
                var probe = new GameObject("Torso triangle check");
                probe.transform.SetParent(skin.transform, false);
                var collider = probe.AddComponent<MeshCollider>();
                collider.sharedMesh = baked;
                Physics.SyncTransforms();
                try
                {
                    foreach (int i in new[] { 1, 4 })
                    {
                        Vector3 delta = bones[i + 1].position - bones[i].position;
                        RaycastHit hit;
                        Require(!collider.Raycast(new Ray(bones[i].position, delta.normalized), out hit, delta.magnitude) &&
                            !collider.Raycast(new Ray(bones[i + 1].position, -delta.normalized), out hit, delta.magnitude),
                            "Forearm crosses actual garment triangles");
                    }
                }
                finally
                {
                    UnityEngine.Object.DestroyImmediate(probe);
                    UnityEngine.Object.DestroyImmediate(baked);
                }
            }
        }

        static void Render(GameObject avatar, string name, bool side = false)
        {
            var cameraObject = new GameObject("Avoidance test camera");
            var lightObject = new GameObject("Avoidance test light");
            var target = new RenderTexture(600, 900, 24);
            var pixels = new Texture2D(600, 900, TextureFormat.RGB24, false);
            var bakedObjects = new List<GameObject>();
            var bakedMeshes = new List<Mesh>();
            var skins = new List<SkinnedMeshRenderer>();
            RenderTexture previous = RenderTexture.active;
            try
            {
                // Two renders in one editor frame otherwise reuse the GPU's previous skinning pose.
                foreach (var skin in avatar.GetComponentsInChildren<SkinnedMeshRenderer>())
                {
                    if (!skin.enabled) continue;
                    var mesh = new Mesh();
                    skin.BakeMesh(mesh);
                    var baked = new GameObject("Baked avoidance preview");
                    baked.transform.SetParent(skin.transform, false);
                    baked.AddComponent<MeshFilter>().sharedMesh = mesh;
                    baked.AddComponent<MeshRenderer>().sharedMaterials = skin.sharedMaterials;
                    bakedObjects.Add(baked);
                    bakedMeshes.Add(mesh);
                    skins.Add(skin);
                    skin.enabled = false;
                }
                var camera = cameraObject.AddComponent<Camera>();
                camera.clearFlags = CameraClearFlags.SolidColor;
                camera.backgroundColor = new Color(0.12f, 0.14f, 0.18f);
                camera.transform.position = side ? new Vector3(3.2f, 0.86f, 0f) : new Vector3(0f, 0.86f, 3.2f);
                camera.transform.LookAt(new Vector3(0f, 0.86f, 0f));
                camera.orthographic = true;
                camera.orthographicSize = 0.9f;
                lightObject.AddComponent<Light>().type = LightType.Directional;
                lightObject.transform.rotation = Quaternion.Euler(35f, -30f, 0f);
                camera.targetTexture = target;
                camera.Render();
                RenderTexture.active = target;
                pixels.ReadPixels(new Rect(0, 0, 600, 900), 0, 0);
                pixels.Apply();
                Directory.CreateDirectory("Artifacts");
                File.WriteAllBytes(Path.Combine("Artifacts", name), pixels.EncodeToPNG());
            }
            finally
            {
                RenderTexture.active = previous;
                foreach (var skin in skins) skin.enabled = true;
                foreach (var baked in bakedObjects) UnityEngine.Object.DestroyImmediate(baked);
                foreach (var mesh in bakedMeshes) UnityEngine.Object.DestroyImmediate(mesh);
                UnityEngine.Object.DestroyImmediate(cameraObject);
                UnityEngine.Object.DestroyImmediate(lightObject);
                UnityEngine.Object.DestroyImmediate(target);
                UnityEngine.Object.DestroyImmediate(pixels);
            }
        }
        static void VerifyReceiver(GameObject avatar, Transform[] bones)
        {
            var bind = Array.ConvertAll(bones, bone => bone.localRotation);
            var go = new GameObject("Receiver integration check");
            try
            {
                var receiver = go.AddComponent<ExternalReceiver>();
                receiver.Model = avatar;
                receiver.CorrectArmBodyPenetration = true;
                receiver.BonePositionSynchronize = false;
                receiver.BlendShapeSynchronize = false;
                receiver.Start();
                var flags = System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic;
                var names = (Dictionary<string, HumanBodyBones>)typeof(ExternalReceiver)
                    .GetField("HumanBodyBonesTable", flags).GetValue(receiver);
                var positions = (Dictionary<HumanBodyBones, Vector3>)typeof(ExternalReceiver)
                    .GetField("HumanBodyBonesPositionTable", flags).GetValue(receiver);
                var rotations = (Dictionary<HumanBodyBones, Quaternion>)typeof(ExternalReceiver)
                    .GetField("HumanBodyBonesRotationTable", flags).GetValue(receiver);
                Pose(bones, 78f, 55f, 10f, -78f, -55f, 10f, Quaternion.identity, Quaternion.identity);
                var ids = new[] { HumanBodyBones.LeftUpperArm, HumanBodyBones.LeftLowerArm, HumanBodyBones.LeftHand,
                    HumanBodyBones.RightUpperArm, HumanBodyBones.RightLowerArm, HumanBodyBones.RightHand };
                for (int i = 0; i < ids.Length; i++)
                {
                    names[ids[i].ToString()] = ids[i];
                    positions[ids[i]] = bones[i].localPosition;
                    rotations[ids[i]] = bones[i].localRotation;
                    bones[i].localRotation = bind[i];
                }
                receiver.Process();
                MeshClear(avatar, bones, "Receiver overwrote collision correction");
                receiver.Freeze = true;
                bones[0].localRotation = Quaternion.AngleAxis(78f, Vector3.forward);
                Quaternion frozen = bones[0].localRotation;
                receiver.Process();
                Require(Quaternion.Angle(frozen, bones[0].localRotation) < 0.01f, "Frozen animation was modified");
                receiver.Freeze = false;
                receiver.EnableLateUpdateForOverwriteAnimationResult = true;
                receiver.LateUpdate();
                MeshClear(avatar, bones, "LateUpdate correction is missing");
            }
            finally { UnityEngine.Object.DestroyImmediate(go); }
        }



    }
}
