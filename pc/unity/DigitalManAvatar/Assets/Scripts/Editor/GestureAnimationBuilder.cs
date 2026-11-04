using System.Collections.Generic;
using System.Text;
using UnityEditor;
using UnityEditor.Animations;
using UnityEngine;

namespace DigitalMan.Editor
{
    /// <summary>
    /// 生成交互模式的三个回应动作并接线。
    ///
    /// 摸头那一段沿用已有的 HeadPatAction.anim（它已经调好了，不重新生成），
    /// 这里只新建捧脸和拥抱两段，然后把控制器扩成 4 个状态、3 个 Trigger，
    /// 最后把三条绑定写进 GestureTriggerReceiver。
    ///
    /// 为什么必须覆盖全部 95 条 Humanoid muscle：Unity 的 Animator 只写动画
    /// 里出现过的肌肉，其余会回到默认姿势（T-pose）。所以每段动画都要把
    /// HumanTrait.MuscleName 全部铺一遍，动作组之外的用基线值占位。
    ///
    /// 放在 Assets/Scripts/Editor 而不是 Assets/Editor：后者带 asmdef，引用不到
    /// Assembly-CSharp 里的 GestureTriggerReceiver / EVMC4U.ExternalReceiver。
    /// </summary>
    public static class GestureAnimationBuilder
    {
        const string Folder = "Assets/Animations";
        const string IdlePath = Folder + "/HeadPatIdle.anim";
        const string HeadPatPath = Folder + "/HeadPatAction.anim";
        const string TouchFacePath = Folder + "/TouchFaceAction.anim";
        const string HugPath = Folder + "/HugAction.anim";

        // 沿用原路径：场景里 Avatar 的 Animator 已经指向它，不用改场景。
        const string ControllerPath = Folder + "/HeadPatController.controller";

        /// <summary>Avatar 所在的场景，批处理模式下需要自己打开。</summary>
        const string ScenePath = "Assets/Scenes/AvatarPreview.unity";

        const string TouchHeadTrigger = "touch_head";        const string TouchFaceTrigger = "touch_face";
        const string HugTrigger = "hug";

        const float HeadPatLength = 1.6f;
        const float TouchFaceLength = 2.5f;
        const float HugLength = 3.0f;

        // ------------------------------------------------------------------
        // 动作定义
        //
        // Muscle 是 Humanoid 肌肉名的子串，大小写不敏感。写成 "Arm Front-Back"
        // 会同时命中 Left/Right 两条 —— 这正是双手对称动作想要的。
        //
        // 数值约定：Unity 的 muscle 值域约 -1..1，正负方向与左右镜像都由
        // Unity 自己处理，所以左右同名肌肉给同一个值就是对称动作。
        // 如果实测发现某个关节方向反了，只改下面表格里的正负号即可。
        // ------------------------------------------------------------------

        struct Reaction
        {
            public string Muscle;
            public float[] Times;
            public float[] Values;
        }

        // 摸脸回应 = **双手在胸前合拢 + 娇羞地低头耸肩膀 + 左右摇头**。
        // 所有值都是相对基线（手臂下垂 = -0.50）的增量：
        //   大臂略外展（+0.24）让手肘离开躯干，不至于把前臂压进胸口；
        //   手肘深弯（-0.82）把前臂收到胸前合拢；
        //   上臂内旋 + 锁骨前送让两手向中线靠拢。
        // 手指不做特殊处理（这个模型的 Avatar 没有手指骨骼映射）。
        static readonly float[] FaceTimes = { 0f, 0.70f, 0.95f, 2.20f, 2.45f, TouchFaceLength };
        static readonly float[] FaceShakeTimes = { 0f, 0.95f, 1.25f, 1.55f, 1.85f, 2.20f, 2.45f, TouchFaceLength };
        static readonly Reaction[] TouchFaceReactions =
        {
            // ---- 双手在胸前合拢 ----
            new Reaction { Muscle = "Arm Down-Up", Times = FaceTimes, Values = new[] { 0f, 0.18f, 0.24f, 0.24f, 0.08f, 0f } },
            new Reaction { Muscle = "Arm Front-Back", Times = FaceTimes, Values = new[] { 0f, -0.32f, -0.42f, -0.42f, -0.14f, 0f } },
            new Reaction { Muscle = "Arm Twist In-Out", Times = FaceTimes, Values = new[] { 0f, 0.42f, 0.55f, 0.55f, 0.18f, 0f } },
            new Reaction { Muscle = "Forearm Stretch", Times = FaceTimes, Values = new[] { 0f, -0.70f, -0.82f, -0.82f, -0.26f, 0f } },
            new Reaction { Muscle = "Forearm Twist In-Out", Times = FaceTimes, Values = new[] { 0f, 0.20f, 0.28f, 0.28f, 0.09f, 0f } },
            // 锁骨前送：把整条手臂转向身体中线（手肘自身角度不变）
            new Reaction { Muscle = "Left Shoulder Front-Back", Times = FaceTimes, Values = new[] { 0f, 0.38f, 0.48f, 0.48f, 0.15f, 0f } },
            new Reaction { Muscle = "Right Shoulder Front-Back", Times = FaceTimes, Values = new[] { 0f, 0.38f, 0.48f, 0.48f, 0.15f, 0f } },
            // ---- 娇羞体态：头微低 + 头轻侧倾 + 双肩微耸 ----
            new Reaction { Muscle = "Head Nod Down-Up", Times = FaceTimes, Values = new[] { 0f, 0.10f, 0.14f, 0.14f, 0.05f, 0f } },
            new Reaction { Muscle = "Head Tilt Left-Right", Times = FaceTimes, Values = new[] { 0f, 0.05f, 0.08f, 0.08f, 0.03f, 0f } },
            new Reaction { Muscle = "Left Shoulder Down-Up", Times = FaceTimes, Values = new[] { 0f, 0.05f, 0.08f, 0.08f, 0.03f, 0f } },
            new Reaction { Muscle = "Right Shoulder Down-Up", Times = FaceTimes, Values = new[] { 0f, 0.05f, 0.08f, 0.08f, 0.03f, 0f } },
            // ---- 合手之后左右摇头（手保持不动，头单独摆两下）----
            new Reaction { Muscle = "Head Turn Left-Right", Times = FaceShakeTimes, Values = new[] { 0f, 0f, 0.18f, -0.16f, 0.13f, -0.08f, 0.03f, 0f } },
            new Reaction { Muscle = "Neck Turn Left-Right", Times = FaceShakeTimes, Values = new[] { 0f, 0f, 0.09f, -0.08f, 0.06f, -0.04f, 0.01f, 0f } },
        };

        // 拥抱：**张开双臂**（斜向下张开，像邀请对方过来抱），手肘只轻轻一弯。
        //
        // 教训：手肘深弯 + 上臂向前环（"环抱"）会让前臂横过胸口，在这套模型上
        // 必然穿模 —— 胸口宽、外套厚。所以方向改成"张开"而不是"环抱"：
        //   上臂斜向下张开（0 = 水平 T 字，-0.5 = 自然下垂，这里取 -0.28）
        //   上臂略向前，手肘只弯 12%，不前伸
        static readonly float[] HugTimes = { 0f, 0.80f, 1.05f, 2.40f, 2.75f, HugLength };
        static readonly Reaction[] HugReactions =
        {
            // 从自然垂手（-0.50）抬到斜下方 -0.28，双臂在两侧张开
            new Reaction { Muscle = "Arm Down-Up", Times = HugTimes, Values = new[] { 0f, 0.22f, 0.25f, 0.25f, 0.08f, 0f } },
            // 略微向前（负值 = 向前），有"要抱住对方"的倾向但不环过去
            new Reaction { Muscle = "Arm Front-Back", Times = HugTimes, Values = new[] { 0f, -0.22f, -0.28f, -0.28f, -0.09f, 0f } },
            // 手肘只轻轻一弯 —— 弯多了前臂就横过胸口穿模
            new Reaction { Muscle = "Forearm Stretch", Times = HugTimes, Values = new[] { 0f, -0.08f, -0.12f, -0.12f, -0.04f, 0f } },
            // 前臂内旋一点，手掌朝向对方
            new Reaction { Muscle = "Forearm Twist In-Out", Times = HugTimes, Values = new[] { 0f, 0.10f, 0.15f, 0.15f, 0.05f, 0f } },
            // 上身微微前倾
            new Reaction { Muscle = "Spine Front-Back", Times = HugTimes, Values = new[] { 0f, -0.04f, -0.06f, -0.06f, -0.02f, 0f } },
            new Reaction { Muscle = "Chest Front-Back", Times = HugTimes, Values = new[] { 0f, -0.03f, -0.05f, -0.05f, -0.02f, 0f } },
            // 头轻轻低一点
            new Reaction { Muscle = "Head Nod Down-Up", Times = HugTimes, Values = new[] { 0f, 0.04f, 0.06f, 0.06f, 0.02f, 0f } },
        };

        // 摸头：沿用原来调好的参数（各组肌肉不同时间轴 = overlapping action，
        // 头先动、肩和躯干延迟跟随）。搬进来是为了让它也用统一的"手臂下垂"
        // 基线 —— 否则播放期间手臂会跳回 T 字。
        static readonly float[] HeadTimes = { 0f, 0.20f, 0.38f, 0.62f, 1.15f, 1.60f };
        static readonly float[] NeckTimes = { 0f, 0.15f, 0.32f, 0.56f, 1.10f, 1.60f };
        static readonly float[] ShoulderTimes = { 0f, 0.13f, 0.28f, 0.52f, 1.08f, 1.60f };
        static readonly float[] TorsoTimes = { 0f, 0.30f, 0.52f, 0.80f, 1.26f, 1.60f };
        static readonly Reaction[] TouchHeadReactions =
        {
            new Reaction { Muscle = "Head Tilt", Times = HeadTimes, Values = new[] { 0f, 0.22f, 0.30f, 0.26f, 0.11f, 0f } },
            new Reaction { Muscle = "Head Nod", Times = HeadTimes, Values = new[] { 0f, 0.07f, 0.12f, 0.10f, 0.04f, 0f } },
            new Reaction { Muscle = "Neck Nod", Times = NeckTimes, Values = new[] { 0f, 0.08f, 0.13f, 0.11f, 0.04f, 0f } },
            new Reaction { Muscle = "Neck Tilt", Times = NeckTimes, Values = new[] { 0f, 0.04f, 0.06f, 0.05f, 0.02f, 0f } },
            new Reaction { Muscle = "Left Shoulder Down-Up", Times = ShoulderTimes, Values = new[] { 0f, 0.09f, 0.15f, 0.12f, 0.05f, 0f } },
            new Reaction { Muscle = "Right Shoulder Down-Up", Times = ShoulderTimes, Values = new[] { 0f, 0.07f, 0.12f, 0.10f, 0.04f, 0f } },
            new Reaction { Muscle = "Spine Tilt", Times = TorsoTimes, Values = new[] { 0f, 0.03f, 0.07f, 0.06f, 0.02f, 0f } },
            new Reaction { Muscle = "Chest Tilt", Times = TorsoTimes, Values = new[] { 0f, 0.02f, 0.04f, 0.03f, 0.01f, 0f } },
        };

        [MenuItem("DigitalMan/生成三个交互手势动画并接线")]
        public static void Build()
        {
            // Batch mode (-batchmode -executeMethod) starts with no scene in memory,
            // so open the avatar scene before looking for anything.
            EVMC4U.ExternalReceiver receiver = Object.FindObjectOfType<EVMC4U.ExternalReceiver>();
            if (receiver == null) receiver = OpenAvatarScene();
            GameObject avatar = receiver != null && receiver.Model != null
                ? receiver.Model
                : FindHumanoidAvatar();
            if (avatar == null)
            {
                Fail("场景里找不到数字人模型。请先运行 DigitalMan 菜单里的场景构建。");
                return;
            }

            Animator animator = avatar.GetComponent<Animator>();
            if (animator == null || animator.avatar == null || !animator.avatar.isHuman)
            {
                Fail("数字人的 Animator 必须是 Humanoid 且已赋值 Avatar。");
                return;
            }

            EnsureFolder();

            float[] baseline = ReadBaseline(animator);

            // 站立基线：所有肌肉固定在基线、不带动任何动作。每次重建 ——
            // 它承载"手臂自然下垂"的姿势，旧的 T 字基线必须被覆盖掉。
            AnimationClip idle = BuildClip("HeadPatIdle", new Reaction[0], 1.0f, baseline, true);
            AssetDatabase.CreateAsset(idle, IdlePath);

            // 摸头每次也重建：动作参数已经搬进来，基线必须和其它三个片段一致。
            AnimationClip headPat = BuildClip("TouchHeadAction", TouchHeadReactions,
                HeadPatLength, baseline, false);
            AssetDatabase.CreateAsset(headPat, HeadPatPath);

            AnimationClip touchFace = BuildClip("TouchFaceAction", TouchFaceReactions,
                TouchFaceLength, baseline, false);
            AssetDatabase.CreateAsset(touchFace, TouchFacePath);

            AnimationClip hug = BuildClip("HugAction", HugReactions, HugLength, baseline, false);
            AssetDatabase.CreateAsset(hug, HugPath);

            AnimatorController controller = BuildController(idle, headPat, touchFace, hug);
            animator.runtimeAnimatorController = controller;

            string wiring = WireReceiver(TouchFaceLength, HugLength, HeadPatLength);

            EditorUtility.SetDirty(animator);
            if (receiver != null) EditorUtility.SetDirty(receiver);
            MarkSceneDirty();
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            Debug.Log("GestureAnimationBuilder: 三个动作已生成并接线。\n" + wiring);
            if (!Application.isBatchMode)
            {
                EditorUtility.DisplayDialog(
                    "三个交互手势动画",
                    "已生成：\n" +
                    "  " + HeadPatPath + "（沿用）\n" +
                    "  " + TouchFacePath + "\n" +
                    "  " + HugPath + "\n" +
                    "控制器：" + ControllerPath + "（4 状态 / 3 Trigger）\n\n" + wiring,
                    "好");
            }
        }

        static void EnsureFolder()
        {
            if (!AssetDatabase.IsValidFolder(Folder))
                AssetDatabase.CreateFolder("Assets", "Animations");
        }

        /// <summary>
        /// Batch mode loads no scene at all.  Opening it here means the same entry
        /// point works from the menu and from -executeMethod.
        /// </summary>
        static EVMC4U.ExternalReceiver OpenAvatarScene()
        {
            if (!System.IO.File.Exists(ScenePath)) return null;
            UnityEditor.SceneManagement.EditorSceneManager.OpenScene(
                ScenePath, UnityEditor.SceneManagement.OpenSceneMode.Single);
            Debug.Log("GestureAnimationBuilder: opened " + ScenePath + " because no scene was loaded.");
            return Object.FindObjectOfType<EVMC4U.ExternalReceiver>();
        }

        static GameObject FindHumanoidAvatar()
        {
            foreach (Animator candidate in Object.FindObjectsOfType<Animator>())
            {
                if (candidate.avatar != null && candidate.avatar.isHuman) return candidate.gameObject;
            }
            return null;
        }

        /// <summary>当前站立姿势的 95 条肌肉值，作为动画基线。</summary>
        static float[] ReadBaseline(Animator animator)
        {
            float[] muscles;
            try
            {
                HumanPose pose = new HumanPose();
                HumanPoseHandler handler = new HumanPoseHandler(animator.avatar, animator.transform);
                handler.GetHumanPose(ref pose);
                handler.Dispose();
                muscles = pose.muscles.Clone() as float[];
            }
            catch (System.Exception error)
            {
                Debug.LogWarning("GestureAnimationBuilder: 读取基线姿势失败，改用全零基线。" + error.Message);
                muscles = new float[HumanTrait.MuscleCount];
            }
            RelaxArms(muscles);
            return muscles;
        }

        /// <summary>
        /// Humanoid 的静止姿势是 T 字（手臂平伸），直接当基线会让数字人在交互模式下
        /// 一直张开双臂。这里把手臂肌肉改成"自然下垂"。
        /// </summary>
        static void RelaxArms(float[] muscles)
        {
            // 探针与实测都表明：Arm Down-Up 负值 = 手臂向下（正值 = 抬起）。
            // 幅度不能太大：-0.55 时胳膊会贴进身体、前臂交叉穿模，-0.50 是
            // 手自然垂在体侧的位置。
            SetMuscle(muscles, "Left Arm Down-Up", -0.50f);
            SetMuscle(muscles, "Right Arm Down-Up", -0.50f);
            // 手肘**不做弯曲**：Curve 上给负值会把前臂往身前带，直接穿过身体。
            // 保持骨骼静止值（伸直）反而最干净。
            SetMuscle(muscles, "Left Forearm Stretch", 0f);
            SetMuscle(muscles, "Right Forearm Stretch", 0f);
        }

        static void SetMuscle(float[] muscles, string name, float value)
        {
            for (int i = 0; i < HumanTrait.MuscleCount; ++i)
            {
                if (string.Equals(HumanTrait.MuscleName[i], name, System.StringComparison.OrdinalIgnoreCase))
                {
                    muscles[i] = value;
                    return;
                }
            }
            Debug.LogWarning("GestureAnimationBuilder: 找不到肌肉 " + name);
        }

        /// <summary>
        /// 生成一段动画。所有 95 条肌肉都会写入曲线：动作组按自己的时间轴，
        /// 其余固定为基线值，否则它们会回落到 T-pose。
        /// </summary>
        static AnimationClip BuildClip(string name,
                                       Reaction[] reactions,
                                       float length,
                                       float[] baseline,
                                       bool loop)
        {
            AnimationClip clip = new AnimationClip { name = name, frameRate = 60f };

            AnimationClipSettings settings = AnimationUtility.GetAnimationClipSettings(clip);
            settings.loopTime = loop;
            AnimationUtility.SetAnimationClipSettings(clip, settings);

            for (int index = 0; index < HumanTrait.MuscleCount; ++index)
            {
                string muscle = HumanTrait.MuscleName[index];
                EditorCurveBinding binding =
                    EditorCurveBinding.FloatCurve(string.Empty, typeof(Animator), muscle);

                int reaction = FindReaction(reactions, muscle);
                if (reaction >= 0 && reactions[reaction].Times != null && reactions[reaction].Values != null)
                {
                    Reaction source = reactions[reaction];
                    Keyframe[] keys = new Keyframe[source.Times.Length];
                    for (int k = 0; k < source.Times.Length; ++k)
                    {
                        float value = baseline[index] + source.Values[k];
                        keys[k] = new Keyframe(source.Times[k], value);
                    }
                    AnimationCurve curve = new AnimationCurve(keys);
                    SmoothTangents(curve);
                    AnimationUtility.SetEditorCurve(clip, binding, curve);
                }
                else
                {
                    float value = baseline[index];
                    AnimationCurve curve = new AnimationCurve(
                        new Keyframe(0f, value), new Keyframe(length, value));
                    AnimationUtility.SetEditorCurve(clip, binding, curve);
                }
            }

            return clip;
        }

        static int FindReaction(Reaction[] reactions, string muscle)
        {
            if (reactions == null) return -1;
            for (int i = 0; i < reactions.Length; ++i)
            {
                if (!string.IsNullOrEmpty(reactions[i].Muscle) &&
                    muscle.IndexOf(reactions[i].Muscle, System.StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    return i;
                }
            }
            return -1;
        }

        static void SmoothTangents(AnimationCurve curve)
        {
            for (int i = 0; i < curve.length; ++i)
            {
                AnimationUtility.SetKeyLeftTangentMode(curve, i, AnimationUtility.TangentMode.Auto);
                AnimationUtility.SetKeyRightTangentMode(curve, i, AnimationUtility.TangentMode.Auto);
            }
        }

        /// <summary>4 个状态、3 个 Trigger。回到 Idle 用 exitTime，播完自动回落。</summary>
        static AnimatorController BuildController(AnimationClip idle,
                                                  AnimationClip headPat,
                                                  AnimationClip touchFace,
                                                  AnimationClip hug)
        {
            AssetDatabase.DeleteAsset(ControllerPath);
            AnimatorController controller = AnimatorController.CreateAnimatorControllerAtPath(ControllerPath);
            controller.AddParameter(TouchHeadTrigger, AnimatorControllerParameterType.Trigger);
            controller.AddParameter(TouchFaceTrigger, AnimatorControllerParameterType.Trigger);
            controller.AddParameter(HugTrigger, AnimatorControllerParameterType.Trigger);

            AnimatorStateMachine machine = controller.layers[0].stateMachine;
            AnimatorState idleState = machine.AddState("Idle", new Vector3(260f, 120f, 0f));
            idleState.motion = idle;
            machine.defaultState = idleState;

            AddAction(machine, idleState, "TouchHeadAction", headPat, TouchHeadTrigger, 380f);
            AddAction(machine, idleState, "TouchFaceAction", touchFace, TouchFaceTrigger, 560f);
            AddAction(machine, idleState, "HugAction", hug, HugTrigger, 740f);
            return controller;
        }

        static void AddAction(AnimatorStateMachine machine,
                              AnimatorState idleState,
                              string stateName,
                              AnimationClip clip,
                              string trigger,
                              float x)
        {
            AnimatorState state = machine.AddState(stateName, new Vector3(x, 120f, 0f));
            state.motion = clip;

            AnimatorStateTransition into = idleState.AddTransition(state);
            into.hasExitTime = false;
            into.duration = 0.1f;
            into.AddCondition(AnimatorConditionMode.If, 0f, trigger);

            AnimatorStateTransition back = state.AddTransition(idleState);
            back.hasExitTime = true;
            back.exitTime = 1f;
            back.duration = 0.15f;
        }

        /// <summary>把三条绑定写进 GestureTriggerReceiver，保持时长与片段一致。</summary>
        static string WireReceiver(float touchFaceLength, float hugLength, float headPatLength)
        {
            GestureTriggerReceiver[] receivers = Object.FindObjectsOfType<GestureTriggerReceiver>();
            if (receivers == null || receivers.Length == 0)
                return "警告：场景里没有 GestureTriggerReceiver，绑定未写入。";

            StringBuilder report = new StringBuilder();
            foreach (GestureTriggerReceiver target in receivers)
            {
                SerializedObject serialized = new SerializedObject(target);
                SerializedProperty array = serialized.FindProperty("gestures");
                if (array == null)
                {
                    report.AppendLine("  跳过 " + target.name + "：找不到 gestures 字段。");
                    continue;
                }

                array.arraySize = 3;
                Bind(array.GetArrayElementAtIndex(0), TouchHeadTrigger, TouchHeadTrigger, headPatLength);
                Bind(array.GetArrayElementAtIndex(1), TouchFaceTrigger, TouchFaceTrigger, touchFaceLength);
                Bind(array.GetArrayElementAtIndex(2), HugTrigger, HugTrigger, hugLength);
                serialized.ApplyModifiedPropertiesWithoutUndo();
                report.AppendLine("  " + target.name + "：写入 3 条绑定。");
            }
            return report.ToString().TrimEnd();
        }

        static void Bind(SerializedProperty element,
                         string gestureName,
                         string triggerName,
                         float holdSeconds)
        {
            element.FindPropertyRelative("gestureName").stringValue = gestureName;
            element.FindPropertyRelative("triggerName").stringValue = triggerName;
            element.FindPropertyRelative("holdSeconds").floatValue = holdSeconds;
            element.FindPropertyRelative("mode").enumValueIndex = 0;      // OneShotTrigger
            element.FindPropertyRelative("handOver").enumValueIndex = 0;  // WholeBody
        }

        static void MarkSceneDirty()
        {
            UnityEngine.SceneManagement.Scene scene =
                UnityEngine.SceneManagement.SceneManager.GetActiveScene();
            if (!scene.IsValid()) return;
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(scene);
            // Batch mode (-batchmode -executeMethod) has no UI and quits right
            // after this returns, so the wiring has to be written to disk here or
            // it would be thrown away.
            UnityEditor.SceneManagement.EditorSceneManager.SaveOpenScenes();
        }

        static void Fail(string message)
        {
            Debug.LogError("GestureAnimationBuilder: " + message);
            if (!Application.isBatchMode)
                EditorUtility.DisplayDialog("三个交互手势动画", message, "好");
        }
    }
}
