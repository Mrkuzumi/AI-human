using System.Text;
using UnityEditor;
using UnityEngine;

namespace DigitalMan.Editor
{
    /// <summary>
    /// Prints where the hands actually end up for each gesture clip, so the
    /// direction of every baked muscle can be judged without watching the
    /// editor: a cupped face needs the hands up near the head, a hug needs them
    /// together in front of the chest.
    ///
    /// Run from the menu, or in batch mode:
    ///   Unity.exe -batchmode -quit -projectPath &lt;proj&gt; \
    ///     -executeMethod DigitalMan.Editor.GestureAnimationProbe.Probe -logFile &lt;log&gt;
    /// </summary>
    public static class GestureAnimationProbe
    {
        const string ScenePath = "Assets/Scenes/AvatarPreview.unity";

        [MenuItem("DigitalMan/检查三个手势动画的骨骼走向")]
        public static void Probe()
        {
            EVMC4U.ExternalReceiver receiver = Object.FindObjectOfType<EVMC4U.ExternalReceiver>();
            if (receiver == null && System.IO.File.Exists(ScenePath))
            {
                UnityEditor.SceneManagement.EditorSceneManager.OpenScene(
                    ScenePath, UnityEditor.SceneManagement.OpenSceneMode.Single);
                receiver = Object.FindObjectOfType<EVMC4U.ExternalReceiver>();
            }
            GameObject avatar = receiver != null && receiver.Model != null ? receiver.Model : null;
            if (avatar == null)
            {
                Debug.LogError("GestureAnimationProbe: 找不到数字人。");
                return;
            }

            Animator animator = avatar.GetComponent<Animator>();
            Transform head = animator.GetBoneTransform(HumanBodyBones.Head);
            Transform chest = animator.GetBoneTransform(HumanBodyBones.Chest);
            Transform left = animator.GetBoneTransform(HumanBodyBones.LeftHand);
            Transform right = animator.GetBoneTransform(HumanBodyBones.RightHand);
            Transform lElbow = animator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
            if (head == null || chest == null || left == null || right == null)
            {
                Debug.LogError("GestureAnimationProbe: 骨骼解析失败。");
                return;
            }

            // Rest pose, for reference: everything below is a delta from this.
            float restHeadY = head.position.y;
            float restSpread = Mathf.Abs(left.position.x - right.position.x);
            float restHandY = 0.5f * (left.position.y + right.position.y);
            float restForward = 0.5f * ((left.position.z - chest.position.z) + (right.position.z - chest.position.z));
            Debug.Log(string.Format(
                "PROBE rest: headY={0:F3} handY={1:F3} spread={2:F3} forward={3:F3}",
                restHeadY, restHandY, restSpread, restForward));

            string[] names =
            {
                "HeadPatAction", "TouchFaceAction", "HugAction",
            };
            foreach (string name in names)
            {
                AnimationClip clip = AssetDatabase.LoadAssetAtPath<AnimationClip>(
                    "Assets/Animations/" + name + ".anim");
                if (clip == null)
                {
                    Debug.LogWarning("PROBE " + name + ": 片段不存在");
                    continue;
                }

                StringBuilder report = new StringBuilder();
                report.AppendLine("PROBE " + name + " length=" + clip.length.ToString("F2"));
                int steps = 6;
                for (int i = 0; i <= steps; ++i)
                {
                    float t = clip.length * i / steps;
                    clip.SampleAnimation(avatar, t);
                    // SampleAnimation writes the pose, but the bone transforms only
                    // settle once the Animator evaluates; without this the probe
                    // reported the same positions for every clip.
                    animator.Update(0f);
                    float handY = 0.5f * (left.position.y + right.position.y);
                    float spread = Mathf.Abs(left.position.x - right.position.x);
                    float forward = 0.5f * ((left.position.z - chest.position.z) + (right.position.z - chest.position.z));
                    report.AppendLine(string.Format(
                        "  t={0:F2} handY={1:+0.000;-0.000} (d{2:+0.000;-0.000}) spread={3:F3} (d{4:+0.000;-0.000}) forward={5:F3} (d{6:+0.000;-0.000}) headDist={7:F3}",
                        t, handY, handY - restHandY, spread, spread - restSpread, forward, forward - restForward,
                        Mathf.Min(Vector3.Distance(left.position, head.position),
                                  Vector3.Distance(right.position, head.position))));
                }
                if (lElbow != null)
                {
                    report.AppendLine(string.Format("  elbowY={0:F3}", lElbow.position.y));
                }
                Debug.Log(report.ToString());
            }

            // Leave the rig in its rest pose so the scene is not saved posed.
            AnimationClip idle = AssetDatabase.LoadAssetAtPath<AnimationClip>("Assets/Animations/HeadPatIdle.anim");
            if (idle != null) idle.SampleAnimation(avatar, 0f);
        }
    }
}
