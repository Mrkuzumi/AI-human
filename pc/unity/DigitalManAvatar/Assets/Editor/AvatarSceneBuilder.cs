using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;

namespace DigitalMan.Editor
{
    public static class AvatarSceneBuilder
    {
        private const string ModelPath = "Assets/Models/model.vrm";
        private const string PrefabPath = "Assets/Models/model.prefab";
        private const string ReceiverPath = "Assets/EVMC4U/ExternalReceiver.prefab";
        private const string ScenePath = "Assets/Scenes/AvatarPreview.unity";
        private static double _importStartedAt;
        private const string VerifyActiveKey = "DigitalMan.VmcVerify.Active";
        private const string VerifyStartedKey = "DigitalMan.VmcVerify.Started";
        private const string VerifyMaxAngleKey = "DigitalMan.VmcVerify.MaxAngle";
        private const string VerifyAvailableKey = "DigitalMan.VmcVerify.Available";

        public static void Build()
        {
            try
            {
                var model = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
                if (model != null)
                {
                    BuildScene(model);
                    EditorApplication.Exit(0);
                    return;
                }

                var projectRoot = Directory.GetParent(Application.dataPath)?.FullName
                                  ?? throw new InvalidOperationException("Cannot resolve Unity project root.");
                var fullModelPath = Path.Combine(projectRoot, ModelPath);
                VRM.vrmAssetPostprocessor.ImportVrmAndCreatePrefab(
                    fullModelPath,
                    UniGLTF.UnityPath.FromUnityPath(PrefabPath));

                _importStartedAt = EditorApplication.timeSinceStartup;
                EditorApplication.update += WaitForPrefab;
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
                EditorApplication.Exit(1);
            }
        }

        public static void VerifyVmcReception()
        {
            EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            SessionState.SetBool(VerifyActiveKey, true);
            SessionState.SetFloat(VerifyStartedKey, (float)EditorApplication.timeSinceStartup);
            SessionState.SetFloat(VerifyMaxAngleKey, 0f);
            SessionState.SetBool(VerifyAvailableKey, false);
            ResumeVmcVerification();
            EditorApplication.isPlaying = true;
        }

        [InitializeOnLoadMethod]
        private static void ResumeVmcVerification()
        {
            if (!SessionState.GetBool(VerifyActiveKey, false))
                return;
            EditorApplication.update -= PollVmcReception;
            EditorApplication.update += PollVmcReception;
        }

        private static void PollVmcReception()
        {
            if (!SessionState.GetBool(VerifyActiveKey, false) || !EditorApplication.isPlaying)
                return;

            var avatar = GameObject.Find("Avatar");
            var animator = avatar != null ? avatar.GetComponent<Animator>() : null;
            var arm = animator != null ? animator.GetBoneTransform(HumanBodyBones.LeftUpperArm) : null;
            if (arm != null)
                SessionState.SetFloat(
                    VerifyMaxAngleKey,
                    Mathf.Max(SessionState.GetFloat(VerifyMaxAngleKey, 0f),
                        Quaternion.Angle(Quaternion.identity, arm.localRotation)));

            var receiverObject = GameObject.Find("VMC Receiver");
            var receiver = receiverObject != null
                ? Array.Find(receiverObject.GetComponents<MonoBehaviour>(),
                    component => component != null && component.GetType().FullName == "EVMC4U.ExternalReceiver")
                : null;
            var getAvailable = receiver?.GetType().GetMethod("GetAvailable");
            if (getAvailable != null && (int)getAvailable.Invoke(receiver, null) == 1)
                SessionState.SetBool(VerifyAvailableKey, true);

            if (EditorApplication.timeSinceStartup - SessionState.GetFloat(VerifyStartedKey, 0f) < 8d)
                return;

            EditorApplication.update -= PollVmcReception;
            SessionState.SetBool(VerifyActiveKey, false);
            var available = SessionState.GetBool(VerifyAvailableKey, false);
            var maxArmAngle = SessionState.GetFloat(VerifyMaxAngleKey, 0f);
            var success = available && maxArmAngle > 5f;
            Debug.Log($"[DIGITAL_MAN] VMC_NETWORK_{(success ? "OK" : "FAIL")} " +
                      $"available={available} maxLeftUpperArm={maxArmAngle:F1}");
            EditorApplication.isPlaying = false;
            EditorApplication.delayCall += () => EditorApplication.Exit(success ? 0 : 1);
        }

        private static void WaitForPrefab()
        {
            try
            {
                var model = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
                if (model != null)
                {
                    EditorApplication.update -= WaitForPrefab;
                    BuildScene(model);
                    EditorApplication.Exit(0);
                    return;
                }

                if (EditorApplication.timeSinceStartup - _importStartedAt > 120d)
                    throw new TimeoutException($"Timed out waiting for VRM prefab: {PrefabPath}");
            }
            catch (Exception exception)
            {
                EditorApplication.update -= WaitForPrefab;
                Debug.LogException(exception);
                EditorApplication.Exit(1);
            }
        }

        private static void BuildScene(GameObject model)
        {

            var receiverPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(ReceiverPath);
            if (receiverPrefab == null)
                throw new InvalidOperationException($"EVMC4U receiver prefab is missing: {ReceiverPath}");

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var avatar = PrefabUtility.InstantiatePrefab(model) as GameObject ?? UnityEngine.Object.Instantiate(model);
            avatar.name = "Avatar";
            avatar.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

            var receiverObject = PrefabUtility.InstantiatePrefab(receiverPrefab) as GameObject
                                 ?? UnityEngine.Object.Instantiate(receiverPrefab);
            receiverObject.name = "VMC Receiver";
            var receiverComponent = Array.Find(
                receiverObject.GetComponents<MonoBehaviour>(),
                component => component != null && component.GetType().FullName == "EVMC4U.ExternalReceiver");
            if (receiverComponent == null)
                throw new InvalidOperationException("EVMC4U ExternalReceiver component is missing.");
            var receiver = new SerializedObject(receiverComponent);
            receiver.FindProperty("Model").objectReferenceValue = avatar;
            receiver.FindProperty("RootPositionSynchronize").boolValue = false;
            receiver.FindProperty("RootRotationSynchronize").boolValue = false;
            receiver.FindProperty("BonePositionSynchronize").boolValue = false;
            receiver.FindProperty("BlendShapeSynchronize").boolValue = true;
            receiver.FindProperty("BoneRotationFilterEnable").boolValue = false;
            receiver.FindProperty("BlendShapeFilterEnable").boolValue = false;
            receiver.FindProperty("enableAutoLoadVRM").boolValue = false;
            receiver.ApplyModifiedPropertiesWithoutUndo();

            var oscComponent = Array.Find(
                receiverObject.GetComponents<MonoBehaviour>(),
                component => component != null && component.GetType().FullName == "uOSC.uOscServer");
            if (oscComponent == null)
                throw new InvalidOperationException("uOSC server component is missing.");
            var oscServer = new SerializedObject(oscComponent);
            oscServer.FindProperty("port").intValue = 39539;
            oscServer.ApplyModifiedPropertiesWithoutUndo();

            var renderers = avatar.GetComponentsInChildren<Renderer>(true);
            if (renderers.Length == 0)
                throw new InvalidOperationException("Imported VRM contains no renderers.");

            var bounds = renderers[0].bounds;
            for (var i = 1; i < renderers.Length; i++)
                bounds.Encapsulate(renderers[i].bounds);

            var cameraObject = new GameObject("Main Camera");
            var camera = cameraObject.AddComponent<Camera>();
            cameraObject.tag = "MainCamera";
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.12f, 0.14f, 0.18f, 1f);
            camera.fieldOfView = 30f;
            camera.nearClipPlane = 0.01f;
            camera.farClipPlane = 100f;

            var halfHeight = Mathf.Max(bounds.extents.y, 0.5f);
            var distance = halfHeight / Mathf.Tan(camera.fieldOfView * 0.5f * Mathf.Deg2Rad) + bounds.extents.z + 0.25f;
            cameraObject.transform.position = bounds.center + Vector3.forward * distance;
            cameraObject.transform.LookAt(bounds.center);

            var lightObject = new GameObject("Directional Light");
            var light = lightObject.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.2f;
            lightObject.transform.rotation = Quaternion.Euler(35f, -30f, 0f);
            RenderSettings.ambientMode = AmbientMode.Flat;
            RenderSettings.ambientLight = new Color(0.65f, 0.65f, 0.65f, 1f);

            Directory.CreateDirectory(Path.GetDirectoryName(ScenePath) ?? "Assets");
            if (!EditorSceneManager.SaveScene(scene, ScenePath))
                throw new InvalidOperationException($"Failed to save scene: {ScenePath}");

            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
            RenderPreview(camera);

            var animator = avatar.GetComponent<Animator>();
            var skinnedMeshes = avatar.GetComponentsInChildren<SkinnedMeshRenderer>(true);
            var blendShapeCount = 0;
            foreach (var meshRenderer in skinnedMeshes)
                if (meshRenderer.sharedMesh != null)
                    blendShapeCount += meshRenderer.sharedMesh.blendShapeCount;

            Debug.Log("[DIGITAL_MAN] VRM_VMC_OK port=39539 bonePosition=False " +
                      $"renderers={renderers.Length} skinned={skinnedMeshes.Length} " +
                      $"blendShapes={blendShapeCount} humanoid={(animator != null && animator.isHuman)} " +
                      $"scene={ScenePath} preview=Artifacts/avatar_preview.png");
        }

        private static void RenderPreview(Camera camera)
        {
            var projectRoot = Directory.GetParent(Application.dataPath)?.FullName
                              ?? throw new InvalidOperationException("Cannot resolve Unity project root.");
            var outputPath = Path.Combine(projectRoot, "Artifacts", "avatar_preview.png");
            Directory.CreateDirectory(Path.GetDirectoryName(outputPath) ?? projectRoot);

            var previous = RenderTexture.active;
            var target = new RenderTexture(1024, 1024, 24, RenderTextureFormat.ARGB32);
            var image = new Texture2D(1024, 1024, TextureFormat.RGBA32, false);
            try
            {
                target.Create();
                camera.targetTexture = target;
                camera.Render();
                RenderTexture.active = target;
                image.ReadPixels(new Rect(0, 0, 1024, 1024), 0, 0);
                image.Apply();
                File.WriteAllBytes(outputPath, image.EncodeToPNG());
            }
            finally
            {
                camera.targetTexture = null;
                RenderTexture.active = previous;
                target.Release();
                UnityEngine.Object.DestroyImmediate(target);
                UnityEngine.Object.DestroyImmediate(image);
            }
        }
    }
}
