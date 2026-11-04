using System.Collections;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using uOSC;

namespace DigitalMan
{
    /// <summary>
    /// 把 Python 桥接器发来的 /Avatar/Gesture 信号位变成 Animator 动作。
    ///
    /// 桥接器发的是「状态」（按住手势时 1，松开后 0），不是一次性事件，所以这里
    /// 既能当边沿触发的一次性 Trigger 用，也能直接切到持续 bool 模式，
    /// Python 侧一个字都不用改（见 GestureMode）。
    ///
    /// 动作播放期间必须让 EVMC4U 停止写入同一批骨骼/表情，否则两边每帧互相覆盖，
    /// 表现为抖动或者动画根本看不见。让权范围由 HandOver 决定。
    ///
    /// 挂载位置：场景里带 uOSC.uOscServer 的那个 "VMC Receiver" 物体上。
    /// </summary>
    [DefaultExecutionOrder(31000)]
    public class GestureTriggerReceiver : MonoBehaviour
    {
        public enum GestureMode
        {
            /// <summary>一次性：信号位 0→1 时触发一次，播够 holdSeconds 后交还控制权。</summary>
            OneShotTrigger,
            /// <summary>持续：信号位为 1 期间保持动作，归 0 就交还（后续要改成持续状态时用这个）。</summary>
            SustainedBool,
        }

        public enum HandOver
        {
            /// <summary>整体冻结 EVMC4U，动画完全接管（最可靠，但动作期间身体不动）。</summary>
            WholeBody,
            /// <summary>只交出表情（BlendShape），身体和头部继续动捕。</summary>
            FaceOnly,
            /// <summary>交出头部+颈部+表情，身体继续动捕。需要先在 Inspector 里配好 CutBonesEnable。</summary>
            HeadAndFace,
        }

        /// <summary>
        /// 一个手势到一条 Animator 动作的绑定。
        ///
        /// 三个手势共用同一个组件，而不是挂三个组件实例：EVMC4U 的 Freeze/BlendShape/
        /// CutBones 是接收器上的共享状态，两个实例各自 Begin/End 会互相覆盖快照，
        /// 典型后果是数字人被永久 Freeze（且没有任何报错）。
        /// </summary>
        [System.Serializable]
        public class GestureBinding
        {
            [Tooltip("桥接器发来的信号名：touch_head / touch_face / hug。")]
            public string gestureName = "touch_head";

            public GestureMode mode = GestureMode.OneShotTrigger;

            [Tooltip("Animator 上的 Trigger 参数名。")]
            public string triggerName = "touch_head";

            [Tooltip("持续模式（SustainedBool）使用的 Bool 参数名。")]
            public string boolName = "";

            [Tooltip("一次性模式下让出控制权的时长，填该动画片段的长度。")]
            public float holdSeconds = 1.6f;

            public HandOver handOver = HandOver.WholeBody;

            /// <summary>上一次收到的信号位，用来只对边沿做出反应。</summary>
            [System.NonSerialized] public int lastActive = -1;
            [System.NonSerialized] public bool waitForRelease;
        }

        [Header("OSC")]
        [SerializeField] string address = "/Avatar/Gesture";
        [SerializeField] string modeAddress = "/Avatar/Mode";
        [SerializeField] bool logMessages = true;

        [Header("模式")]
        [Tooltip("跟随模式会摘掉 Animator 的 Controller，让动捕完全控制角色；交互模式再挂回去。")]
        [SerializeField] bool interactive;

        [Header("动作")]
        [Tooltip("每个手势一条绑定。板端同一时刻只会拉高一个手势，播放本身也是独占的。")]
        [SerializeField] GestureBinding[] gestures = new GestureBinding[0];
        [SerializeField] Animator animator;

        [Header("旧字段（仅在 gestures 为空时用于迁移）")]
        [SerializeField] string gestureName = "touch_head";
        [SerializeField] GestureMode mode = GestureMode.OneShotTrigger;
        [SerializeField] string triggerName = "touch_head";
        [SerializeField] string boolName = "";
        [SerializeField] float holdSeconds = 1.6f;
        [SerializeField] HandOver handOver = HandOver.WholeBody;

        [Header("EVMC4U")]
        [SerializeField] EVMC4U.ExternalReceiver receiver;

        uOscServer server;
        Coroutine playing;
        // 当前正在播放的那条绑定（独占播放，保证 EVMC4U 保存/还原成对）。
        GestureBinding playingBinding;
        bool saved;
        bool warnedAboutCutBones;
        bool warnedAboutAnimator;
        bool modeInitialized;

        // Animator 上的原始 Controller。跟随模式会把它摘掉，交互模式再装回来。
        RuntimeAnimatorController cachedController;
        Transform[] standingBones;
        Vector3 standingHipsPosition;
        Quaternion[] standingRotations;

        // 让权前保存的 EVMC4U 状态，结束时精确还原。
        bool savedFreeze, savedBlendShape, savedCutBones, savedCutNeck, savedCutHead;

        void Awake()
        {
            server = GetComponent<uOscServer>();
            if (server == null) server = FindObjectOfType<uOscServer>();
            if (server == null)
            {
                Debug.LogError("GestureTriggerReceiver 需要挂在带 uOSC.uOscServer 的物体上。");
                enabled = false;
                return;
            }
            server.onDataReceived.AddListener(OnOsc);
            if (animator == null) animator = GetComponentInChildren<Animator>();
            if (receiver == null) receiver = GetComponent<EVMC4U.ExternalReceiver>();
            if (receiver == null) receiver = FindObjectOfType<EVMC4U.ExternalReceiver>();
            EnsureAnimator();
            CaptureStandingPose();
            if (animator != null && cachedController == null)
            {
                cachedController = animator.runtimeAnimatorController;
            }
            if ((gestures == null || gestures.Length == 0) && !string.IsNullOrEmpty(gestureName))
            {
                // 迁移旧场景：这一版之前只支持一个手势，把旧字段变成唯一一条绑定。
                gestures = new[]
                {
                    new GestureBinding
                    {
                        gestureName = gestureName,
                        mode = mode,
                        triggerName = triggerName,
                        boolName = boolName,
                        holdSeconds = holdSeconds,
                        handOver = handOver,
                    },
                };
            }
            // 默认跟随模式：摘掉 Controller，让 EVMC4U 完全控制角色。
            // bridge 每秒会重发 /Avatar/Mode，所以交互模式也能自动切过来。
            ApplyMode(interactive);
            if (logMessages)
            {
                var names = new System.Text.StringBuilder();
                if (gestures != null)
                {
                    for (int i = 0; i < gestures.Length; i++)
                    {
                        if (gestures[i] == null) continue;
                        if (names.Length > 0) names.Append(", ");
                        names.Append(gestures[i].gestureName);
                    }
                }
                // 无条件打一条，用于确认组件真的活起来了（否则出错时完全静默）。
                Debug.Log(string.Format(
                    "[Gesture] ready: server={0} port={1} animator={2} receiver={3} interactive={4} gestures=[{5}]",
                    server.name, server.port, animator != null ? animator.name : "none",
                    receiver != null ? receiver.name : "none", interactive,
                    names.Length > 0 ? names.ToString() : "none"));
            }
        }

        /// <summary>
        /// 跟随模式摘掉 Animator 的 Controller：Unity 的 Animator 会在 EVMC4U
        /// 之后执行并覆盖动捕，戴着它数字人就不跟随了（而且会掉成默认姿势）。
        /// 交互模式再挂回来，让数字人站立并播放摸头回应。
        /// </summary>
        void ApplyMode(bool interaction)
        {
            EnsureAnimator();
            if (animator == null) return;
            if (cachedController == null && animator.runtimeAnimatorController != null)
                cachedController = animator.runtimeAnimatorController;

            bool changed = !modeInitialized || interactive != interaction;
            if (changed)
            {
                // A mode switch must return bone ownership even mid-animation.
                StopPlaying();
                ClearGestureParameters();
                if (gestures != null)
                    foreach (var binding in gestures)
                        if (binding != null)
                            binding.waitForRelease = interaction && binding.lastActive != 0;
            }

            if (interaction)
            {
                if (animator.runtimeAnimatorController == null && cachedController != null)
                {
                    animator.runtimeAnimatorController = cachedController;
                }
            }
            else if (animator.runtimeAnimatorController != null)
            {
                animator.runtimeAnimatorController = null;
            }
            if (changed) ClearGestureParameters();
            foreach (var speech in FindObjectsOfType<DialogueSpeechReceiver>()) speech.SetInteraction(interaction);
            modeInitialized = true;

            if (interactive != interaction)
            {
                interactive = interaction;
                if (logMessages)
                {
                    Debug.Log(interaction
                        ? "[Gesture] 交互模式：数字人站立不跟随，摸头给回应"
                        : "[Gesture] 跟随模式：数字人跟随你的动作");
                }
            }
        }

        void ClearGestureParameters()
        {
            if (animator == null || animator.runtimeAnimatorController == null || gestures == null) return;
            foreach (var binding in gestures)
            {
                if (binding == null) continue;
                if (!string.IsNullOrEmpty(binding.triggerName)) animator.ResetTrigger(binding.triggerName);
                if (binding.mode == GestureMode.SustainedBool && !string.IsNullOrEmpty(binding.boolName))
                    animator.SetBool(binding.boolName, false);
            }
        }

        void CaptureStandingPose()
        {
            if (animator == null || standingBones != null) return;
            var ids = new[] { HumanBodyBones.Hips, HumanBodyBones.LeftUpperLeg, HumanBodyBones.LeftLowerLeg,
                HumanBodyBones.LeftFoot, HumanBodyBones.LeftToes, HumanBodyBones.RightUpperLeg,
                HumanBodyBones.RightLowerLeg, HumanBodyBones.RightFoot, HumanBodyBones.RightToes };
            standingBones = new Transform[ids.Length];
            standingRotations = new Quaternion[ids.Length];
            for (int i = 0; i < ids.Length; i++)
            {
                standingBones[i] = animator.GetBoneTransform(ids[i]);
                if (standingBones[i] != null) standingRotations[i] = standingBones[i].localRotation;
            }
            if (standingBones[0] != null) standingHipsPosition = standingBones[0].localPosition;
        }

        void LateUpdate()
        {
            if (!interactive || standingBones == null) return;
            // Legacy Humanoid clips omit root height and contain bad leg curves.
            // Keep only the lower body at the imported standing pose; all three
            // upper-body responses and their handover remain under Animator.
            if (standingBones[0] != null) standingBones[0].localPosition = standingHipsPosition;
            for (int i = 0; i < standingBones.Length; i++)
                if (standingBones[i] != null) standingBones[i].localRotation = standingRotations[i];
        }

        /// <summary>Avatar is referenced by Model, rather than parented to the receiver.</summary>
        void EnsureAnimator()
        {
            if (animator == null && receiver != null && receiver.Model != null)
            {
                animator = receiver.Model.GetComponent<Animator>();
            }
        }

        void OnDestroy()
        {
            if (server != null) server.onDataReceived.RemoveListener(OnOsc);
        }

        void OnDisable()
        {
            StopPlaying();
        }

        void OnOsc(Message message)
        {
            if (!isActiveAndEnabled) return;
            // 诊断：凡是 /Avatar/* 的消息都记一条，用来确认 OSC 到底有没有进来。
            if (logMessages && message.address != null &&
                message.address.StartsWith("/Avatar/", System.StringComparison.Ordinal))
            {
                Debug.Log("[Gesture] OSC in: " + message.address);
            }

            if (message.address == modeAddress)
            {
                if (message.values.Length >= 1 && message.values[0] is string)
                {
                    string requested = (string)message.values[0];
                    if (requested == "follow" || requested == "interact")
                        ApplyMode(requested == "interact");
                }
                return;
            }

            if (message.address != address || message.values.Length < 2) return;
            if (!(message.values[0] is string) || !(message.values[1] is int)) return;

            string name = (string)message.values[0];
            int active = (int)message.values[1];
            if (active != 0 && active != 1) return;
            if (gestures == null) return;
            for (int i = 0; i < gestures.Length; i++)
            {
                GestureBinding binding = gestures[i];
                if (binding != null && binding.gestureName == name)
                {
                    HandleGesture(binding, active);
                    return;
                }
            }
        }

        /// <summary>把一条绑定的信号位变成动作。只对边沿做出反应。</summary>
        void HandleGesture(GestureBinding binding, int active)
        {
            if (logMessages) Debug.Log("[Gesture] " + binding.gestureName + " active=" + active);

            if (binding.waitForRelease)
            {
                binding.lastActive = active;
                if (active == 0) binding.waitForRelease = false;
                return;
            }

            // 只在状态跳变时动作，避免每帧重复 SetTrigger。
            if (active == binding.lastActive) return;
            binding.lastActive = active;

            // 跟随模式下 Controller 被摘掉了，触发不会有任何动画，反而会让
            // Begin() 白白冻结动捕。所以跟随模式直接忽略手势信号。
            if (!interactive)
            {
                if (logMessages) Debug.Log("[Gesture] 当前是跟随模式，忽略手势信号");
                return;
            }

            EnsureAnimator();
            if (animator == null && !warnedAboutAnimator)
            {
                warnedAboutAnimator = true;
                Debug.LogWarning(
                    "GestureTriggerReceiver: 找不到 Animator，信号收到了但无法触发动画。" +
                    "请把 Avatar 的 Animator 拖到本组件的 Animator 字段上。");
            }

            if (binding.mode == GestureMode.SustainedBool)
            {
                if (animator != null && !string.IsNullOrEmpty(binding.boolName))
                    animator.SetBool(binding.boolName, active != 0);
                if (active != 0) StartAction(binding);
                else StopPlaying();
                return;
            }

            if (active != 0) StartAction(binding);
            else if (playingBinding == binding) StopPlaying();
        }

        void StartAction(GestureBinding binding)
        {
            // 独占播放。板端同一时刻只拉高一个手势，这里再保证一次，这样
            // Begin/End 永远成对，不会出现两个快照互相覆盖导致永久 Freeze。
            StopPlaying();
            try
            {
                using (var notify = new UdpClient())
                {
                    byte[] name = Encoding.UTF8.GetBytes(binding.gestureName);
                    notify.Send(name, name.Length, new IPEndPoint(IPAddress.Loopback, 39546));
                }
            }
            catch (SocketException error) { Debug.LogWarning("Gesture voice notification failed: " + error.SocketErrorCode); }
            playingBinding = binding;
            playing = StartCoroutine(PlayOnce(binding));
        }

        void StopPlaying()
        {
            if (playing != null) StopCoroutine(playing);
            playing = null;
            playingBinding = null;
            End();
        }

        IEnumerator PlayOnce(GestureBinding binding)
        {
            Begin(binding.handOver);
            if (animator != null && !string.IsNullOrEmpty(binding.triggerName))
                animator.SetTrigger(binding.triggerName);
            if (binding.holdSeconds > 0f) yield return new WaitForSeconds(binding.holdSeconds);
            End();
            playing = null;
            playingBinding = null;
        }

        void Begin(HandOver scope)
        {
            if (receiver == null || saved) return;

            savedFreeze = receiver.Freeze;
            savedBlendShape = receiver.BlendShapeSynchronize;
            savedCutBones = receiver.CutBonesEnable;
            savedCutNeck = receiver.CutBoneNeck;
            savedCutHead = receiver.CutBoneHead;
            saved = true;

            switch (scope)
            {
                case HandOver.WholeBody:
                    receiver.Freeze = true;
                    break;

                case HandOver.FaceOnly:
                    receiver.BlendShapeSynchronize = false;
                    break;

                case HandOver.HeadAndFace:
                    receiver.BlendShapeSynchronize = false;
                    if (receiver.CutBonesEnable)
                    {
                        receiver.CutBoneNeck = true;
                        receiver.CutBoneHead = true;
                    }
                    else if (!warnedAboutCutBones)
                    {
                        // CutBonesEnable 关闭时逐骨骼拦截不生效。不能在这里擅自打开它：
                        // EVMC4U 的 cut 列表默认切掉了 Hips/Spine/Chest/双腿，一旦启用
                        // 身体会立刻停止同步。所以只交出表情，并提示手动配置。
                        warnedAboutCutBones = true;
                        Debug.LogWarning(
                            "GestureTriggerReceiver: HandOver.HeadAndFace 需要先在 ExternalReceiver 上勾选 " +
                            "CutBonesEnable，并确认 Hips/Spine/Chest/双腿 的 Cut 未被勾选、只勾选 Head 和 Neck。" +
                            "当前未配置，已降级为 FaceOnly（仅表情让权）。");
                    }
                    break;
            }
        }

        void End()
        {
            if (receiver == null || !saved) return;
            receiver.Freeze = savedFreeze;
            receiver.BlendShapeSynchronize = savedBlendShape;
            receiver.CutBonesEnable = savedCutBones;
            receiver.CutBoneNeck = savedCutNeck;
            receiver.CutBoneHead = savedCutHead;
            saved = false;
        }
    }
}
