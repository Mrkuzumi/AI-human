using System;
using UnityEditor;
using UnityEditor.Animations;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace DigitalMan.Editor
{
    /// <summary>
    /// 一键生成"被摸头"动画片段和 Animator Controller，并接到 Avatar 的 Animator 上。
    ///
    /// 菜单：DigitalMan / 生成摸头动画并接线
    ///
    /// 注意本文件必须放在 Assets/Scripts/Editor/ 而不是 Assets/Editor/：
    /// Assets/Editor 下有 DigitalMan.Editor.asmdef，asmdef 程序集无法引用
    /// Assembly-CSharp，而 EVMC4U.ExternalReceiver 与 GestureTriggerReceiver
    /// 都在 Assembly-CSharp 里。放在这里会进入 Assembly-CSharp-Editor，可以正常引用。
    ///
    /// 动画使用 Humanoid 肌肉曲线（Head Tilt / Head Nod / Neck Turn），
    /// 不依赖具体 VRM 模型的骨骼层级；重复运行会覆盖这两个资产。
    /// </summary>
    public static class HeadPatAnimationBuilder
    {
        const string Folder = "Assets/Animations";
        const string ClipPath = Folder + "/HeadPatAction.anim";
        const string IdleClipPath = Folder + "/HeadPatIdle.anim";
        const string ControllerPath = Folder + "/HeadPatController.controller";
        const string ParameterName = "HeadPat";
        const string StateName = "HeadPatAction";
        const float ClipLength = 1.6f;

        [MenuItem("DigitalMan/生成摸头动画并接线")]
        public static void Build()
        {
            var receiver = FindReceiver();
            if (receiver == null)
            {
                Fail("场景里找不到 EVMC4U.ExternalReceiver。请确认打开的是 AvatarPreview 场景。");
                return;
            }

            var avatar = receiver.Model;
            if (avatar == null) avatar = FindHumanoidAvatar();
            if (avatar == null)
            {
                Fail("找不到 Avatar。请先把 VMC Receiver 上 ExternalReceiver 的 Model 指向模型。");
                return;
            }

            var animator = avatar.GetComponent<Animator>();
            if (animator == null)
            {
                Fail("Avatar 上没有 Animator 组件，无法播放动画。");
                return;
            }
            if (animator.avatar == null || !animator.avatar.isHuman)
            {
                Fail("Avatar 的 Animator 不是 Humanoid 类型，肌肉曲线无法生效。");
                return;
            }

            if (!AssetDatabase.IsValidFolder(Folder))
            {
                AssetDatabase.CreateFolder("Assets", "Animations");
            }

            // 只读一次静止姿势，两个 clip 共用。
            var baseline = ReadBaseline(animator);

            var clip = BuildActionClip(baseline);
            AssetDatabase.CreateAsset(clip, ClipPath);

            // 交互模式下必须有一个"自然站姿"的 Idle，否则 Unity 会把没被
            // 动画驱动的肌肉写回默认值（T 姿势），看起来就很奇怪。
            var idleClip = BuildIdleClip(baseline);
            AssetDatabase.CreateAsset(idleClip, IdleClipPath);
            AssetDatabase.SaveAssets();

            var controller = BuildController(idleClip, clip);
            AssetDatabase.SaveAssets();

            animator.runtimeAnimatorController = controller;
            EditorUtility.SetDirty(animator);

            var wiring = WireGestureReceiver();

            // MarkSceneDirty 在某些情况下会抛异常（例如拿到的 scene 无效），
            // 但它只是"提示保存"，不该影响前面的成果，所以这里做防御式处理。
            var scene = animator.gameObject.scene;
            if (scene.IsValid() && scene.isLoaded)
            {
                EditorSceneManager.MarkSceneDirty(scene);
            }
            else
            {
                Debug.LogWarning("[HeadPatBuilder] 场景句柄无效，跳过 MarkSceneDirty；请手动 Ctrl+S 保存。");
            }
            AssetDatabase.Refresh();

            Debug.Log(string.Format(
                "[HeadPatBuilder] 完成。\n  动画: {0} (时长 {1:0.0}s)\n  控制器: {2} (Trigger 参数 {3})\n" +
                "  Avatar: {4}\n  {5}\n请按 Ctrl+S 保存场景，然后 Play 并做摸头动作。",
                ClipPath, ClipLength, ControllerPath, ParameterName, avatar.name, wiring));

            EditorUtility.DisplayDialog(
                "生成完成",
                "已生成动画和 Animator Controller，并接到 " + avatar.name + " 上。\n\n" +
                wiring + "\n\n请按 Ctrl+S 保存场景，然后 Play 测试。",
                "好");
        }

        static EVMC4U.ExternalReceiver FindReceiver()
        {
            var all = UnityEngine.Object.FindObjectsOfType<EVMC4U.ExternalReceiver>();
            return all.Length > 0 ? all[0] : null;
        }

        static GameObject FindHumanoidAvatar()
        {
            var animators = UnityEngine.Object.FindObjectsOfType<Animator>();
            foreach (var candidate in animators)
            {
                if (candidate.avatar != null && candidate.avatar.isHuman) return candidate.gameObject;
            }
            return null;
        }

        // ------------------------------------------------------------- 动画片段

        struct Reaction
        {
            public string Muscle;
            public float[] Times;
            public float[] Values;
        }

        // 自然感的关键：各组肌肉用不同的时间轴，头先动、肩和躯干延迟跟随
        // （动画里叫 overlapping action）。所有肌肉同时到达峰值会显得很机械。
        // 时间是秒，值是在静止姿势上的偏移（humanoid 肌肉归一化值）。
        static readonly float[] HeadTimes = { 0f, 0.20f, 0.38f, 0.62f, 1.15f, 1.60f };
        static readonly float[] NeckTimes = { 0f, 0.15f, 0.32f, 0.56f, 1.10f, 1.60f };
        static readonly float[] ShoulderTimes = { 0f, 0.13f, 0.28f, 0.52f, 1.08f, 1.60f };
        static readonly float[] TorsoTimes = { 0f, 0.30f, 0.52f, 0.80f, 1.26f, 1.60f };

        static readonly Reaction[] Reactions =
        {
            // 头：快起 → 轻微过冲 0.34 → 回稳在 0.29 停一会儿 → 慢慢松开
            new Reaction { Muscle = "Head Tilt", Times = HeadTimes,
                           Values = new[] { 0f, 0.22f, 0.30f, 0.26f, 0.11f, 0f } },
            new Reaction { Muscle = "Head Nod", Times = HeadTimes,
                           Values = new[] { 0f, 0.07f, 0.12f, 0.10f, 0.04f, 0f } },

            // 颈：比头早一点点启动（先缩脖子），幅度更小
            new Reaction { Muscle = "Neck Nod", Times = NeckTimes,
                           Values = new[] { 0f, 0.08f, 0.13f, 0.11f, 0.04f, 0f } },
            new Reaction { Muscle = "Neck Tilt", Times = NeckTimes,
                           Values = new[] { 0f, 0.04f, 0.06f, 0.05f, 0.02f, 0f } },

            // 肩：轻微耸起，左右错开一点点，避免完全对称的机械感
            new Reaction { Muscle = "Left Shoulder Down-Up", Times = ShoulderTimes,
                           Values = new[] { 0f, 0.09f, 0.15f, 0.12f, 0.05f, 0f } },
            new Reaction { Muscle = "Right Shoulder Down-Up", Times = ShoulderTimes,
                           Values = new[] { 0f, 0.07f, 0.12f, 0.10f, 0.04f, 0f } },

            // 躯干：明显延迟，幅度最小，只做很轻的跟随
            new Reaction { Muscle = "Spine Tilt", Times = TorsoTimes,
                           Values = new[] { 0f, 0.03f, 0.07f, 0.06f, 0.02f, 0f } },
            new Reaction { Muscle = "Chest Tilt", Times = TorsoTimes,
                           Values = new[] { 0f, 0.02f, 0.04f, 0.03f, 0.01f, 0f } },
        };

        static AnimationClip BuildActionClip(float[] baseline)
        {
            var clip = new AnimationClip { name = "HeadPatAction", frameRate = 60f };
            var settings = AnimationUtility.GetAnimationClipSettings(clip);
            settings.loopTime = false;
            AnimationUtility.SetAnimationClipSettings(clip, settings);

            // 关键：动画期间只有 Animator 在驱动角色，所以 clip 必须覆盖全部
            // humanoid 肌肉。否则没被写到的部位会掉回默认（T 姿势），看上去
            // 就像"只有上半身是活的"。基准取模型当前的静止姿势。
            int applied = 0;
            for (int i = 0; i < HumanTrait.MuscleCount; i++)
            {
                string muscle = HumanTrait.MuscleName[i];
                int index = ReactionIndex(muscle);
                var curve = new AnimationCurve();

                if (index >= 0)
                {
                    // 参与反应的肌肉：用这一组自己的时间轴，做出层次感。
                    applied++;
                    var times = Reactions[index].Times;
                    var values = Reactions[index].Values;
                    for (int k = 0; k < times.Length; k++)
                    {
                        curve.AddKey(times[k], baseline[i] + values[k]);
                    }
                }
                else
                {
                    // 其余肌肉全程保持静止姿势。必须写进 clip，否则 Unity 会用
                    // 默认值填充，角色就掉成 T 姿势了。
                    curve.AddKey(0f, baseline[i]);
                    curve.AddKey(ClipLength, baseline[i]);
                }

                // Auto 切线 = Animation 窗口的默认手感：缓入缓出、不会乱过冲。
                for (int k = 0; k < curve.length; k++)
                {
                    AnimationUtility.SetKeyLeftTangentMode(curve, k, AnimationUtility.TangentMode.Auto);
                    AnimationUtility.SetKeyRightTangentMode(curve, k, AnimationUtility.TangentMode.Auto);
                }

                AnimationUtility.SetEditorCurve(
                    clip,
                    EditorCurveBinding.FloatCurve(string.Empty, typeof(Animator), muscle),
                    curve);
            }

            Debug.Log(string.Format(
                "[HeadPatBuilder] 动作动画覆盖 {0} 条肌肉，其中 {1} 条带反应动作。",
                HumanTrait.MuscleCount, applied));

            if (applied == 0)
            {
                Debug.LogWarning("[HeadPatBuilder] 没有任何反应肌肉匹配成功，动作动画只是静止姿势。");
            }
            return clip;
        }

        /// <summary>
        /// 维持模型自然站姿的 Idle。交互模式下数字人站着不动，靠这个 clip 让它
        /// 保持正常姿势，而不是 Unity 的默认（T 姿势）值。
        /// </summary>
        static AnimationClip BuildIdleClip(float[] baseline)
        {
            var clip = new AnimationClip { name = "HeadPatIdle", frameRate = 60f };
            var settings = AnimationUtility.GetAnimationClipSettings(clip);
            settings.loopTime = true;
            AnimationUtility.SetAnimationClipSettings(clip, settings);

            for (int i = 0; i < HumanTrait.MuscleCount; i++)
            {
                var curve = new AnimationCurve();
                curve.AddKey(0f, baseline[i]);
                curve.AddKey(1f, baseline[i]);
                AnimationUtility.SetEditorCurve(
                    clip,
                    EditorCurveBinding.FloatCurve(string.Empty, typeof(Animator), HumanTrait.MuscleName[i]),
                    curve);
            }
            return clip;
        }

        static float[] ReadBaseline(Animator animator)
        {
            try
            {
                var handler = new HumanPoseHandler(animator.avatar, animator.transform);
                var pose = new HumanPose();
                handler.GetHumanPose(ref pose);
                return (float[])pose.muscles.Clone();
            }
            catch (Exception error)
            {
                Debug.LogWarning("[HeadPatBuilder] 读取静止姿势失败，改用全 0 基准: " + error.Message);
                return new float[HumanTrait.MuscleCount];
            }
        }

        static int ReactionIndex(string muscle)
        {
            for (int i = 0; i < Reactions.Length; i++)
            {
                if (muscle.IndexOf(Reactions[i].Muscle, StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    return i;
                }
            }
            return -1;
        }

        // ----------------------------------------------------------- 状态机

        static AnimatorController BuildController(AnimationClip idleClip, AnimationClip actionClip)
        {
            var controller = AnimatorController.CreateAnimatorControllerAtPath(ControllerPath);
            controller.AddParameter(ParameterName, AnimatorControllerParameterType.Trigger);

            var machine = controller.layers[0].stateMachine;
            var idle = machine.AddState("Idle", new Vector3(260f, 120f, 0f));
            idle.motion = idleClip;
            machine.defaultState = idle;

            var action = machine.AddState(StateName, new Vector3(540f, 120f, 0f));
            action.motion = actionClip;

            // Trigger 一进来立刻切过去，不等 Idle 播完。
            var enter = idle.AddTransition(action);
            enter.hasExitTime = false;
            enter.duration = 0.10f;
            enter.AddCondition(AnimatorConditionMode.If, 0f, ParameterName);

            // 动画播完自动回 Idle。
            var leave = action.AddTransition(idle);
            leave.hasExitTime = true;
            leave.exitTime = 1f;
            leave.duration = 0.15f;

            return controller;
        }

        // ------------------------------------------------- 顺手对齐冻结时长

        static string WireGestureReceiver()
        {
            var receivers = UnityEngine.Object.FindObjectsOfType<GestureTriggerReceiver>();
            if (receivers.Length == 0)
            {
                return "提示：场景里没找到 GestureTriggerReceiver，记得把它挂到 VMC Receiver 上。";
            }

            var receiver = receivers[0];
            var so = new SerializedObject(receiver);
            var hold = so.FindProperty("holdSeconds");
            var trigger = so.FindProperty("triggerName");
            if (hold != null) hold.floatValue = ClipLength;
            if (trigger != null) trigger.stringValue = ParameterName;
            so.ApplyModifiedProperties();
            EditorUtility.SetDirty(receiver);

            return string.Format(
                "已把 GestureTriggerReceiver 的 Hold Seconds 设为 {0:0.0}s（等于动画时长），Trigger Name 设为 {1}。",
                ClipLength, ParameterName);
        }

        static void Fail(string message)
        {
            Debug.LogError("[HeadPatBuilder] " + message);
            EditorUtility.DisplayDialog("生成失败", message, "好");
        }
    }
}
