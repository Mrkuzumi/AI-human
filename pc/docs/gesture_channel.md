# 交互手势通道（板端 PS 判定 → Unity）

interact 模式下数字人不跟随，只对三个动作做出回应。判定**全部在板子的 PS 侧完成**，
上位机只负责转发和边沿处理。

## 数据流

```
板端 PS                          电脑端                           Unity
┌─────────────────────┐   GST1   ┌──────────────────┐   OSC    ┌──────────────────────┐
│ head_pat::Trigger   │  UDP     │ gesture_packet   │  39539   │ GestureTriggerReceiver│
│ gesture_region      │ ─39543─> │  解码 + 名字映射  │ ───────> │  gestureName 查表      │
│  （三个判定器）      │  每帧    │  模式门控/边沿    │          │  → Animator Trigger   │
└─────────────────────┘          └──────────────────┘          └──────────────────────┘
```

## 动作表（唯一契约，两边必须同步）

| id | 信号名 | Trigger | 判定方式 | 时长 |
|---|---|---|---|---|
| 0 | `touch_head` | `touch_head` | 单手在头顶区域停留 | 1.6s |
| 1 | `touch_face` | `touch_face` | **双手**贴脸颊两侧（一左一右） | 2.5s |
| 2 | `hug` | `hug` | **双臂展开**：手腕与肩同高、两手远离中线 | 3.0s |

- 板端常量：`include/gesture_udp_sender.hpp` 的 `ID_HEAD_PAT` / `ID_TOUCH_FACE` / `ID_HUG`
- 电脑端映射：`pc/reference/gesture_packet.py` 的 `GESTURE_IDS`
- 改名字只改这两处，协议本身不用动

**同一时刻只有一个动作会被拉高**，优先级 `hug > touch_face > touch_head`。
（手从头顶滑到脸颊时会同时命中两个区域，所以必须有优先级。）

## GST1 报文

小端，48 字节头 + N×12 字节记录 + 4 字节 CRC32：

| 偏移 | 长度 | 字段 |
|---|---|---|
| 0 | 4 | magic `GST1` |
| 4 | 1 | version = 1 |
| 5 | 1 | flags（保留 0） |
| 6 | 2 | header_bytes = 48 |
| 8 | 4 | packet_bytes |
| 12 | 4 | sequence |
| 16 | 8 | capture_time_ns |
| 24 | 8 | emit_time_ns |
| 32 | 2 | frame_width |
| 34 | 2 | frame_height |
| 36 | 1 | source_id |
| 37 | 1 | reserved |
| 38 | 2 | count |
| 40 | 4 | reserved |
| 44 | 4 | reserved |
| 48 | count×12 | `{ id u16, active u8, flags u8, progress f32, metric f32 }` |
| 末尾 | 4 | CRC32 |

`flags` 的 bit0 = `FLAG_METRIC_VALID`：置位时 `metric` 有效，表示手离区域边界
还有多远（0 = 已在区域内，单位是头宽/肩宽）。三个动作每帧都发，`active=0` 也发，
这样上位机能随时看到进度。

## 板端判定器

两个实现，都在 PS 上逐帧跑：

| 文件 | 用途 |
|---|---|
| `include/head_pat_trigger.hpp` | 摸头专用（带鼻子缓存、头宽 EMA、左右耳估计） |
| `include/gesture_region.hpp` | 通用"区域 + 停留"，锚点可选**鼻子**或**双肩中点** |

通用判定器的锚点：

- **鼻子** → 尺度取耳距（头宽）。`touch_face` 用它，区域在鼻子**下方**（脸颊）
- **双肩中点** → 尺度取肩宽。`hug` 用它，区域是胸前一块窄条（贴近中线）

可以调的量（全部在 yaml 里，**改完重启即可，不用重新编译**）：

```yaml
  gesture_udp:
    enabled: true
    host: 192.168.126.91
    port: 39543
    head_pat:
      enabled: true
    dwell_seconds: 0.5        # touch_head 停留时间
    region_above: 1.3         # touch_head 区域上边界（头宽倍数）
    region_half_width: 0.8
    min_confidence: 0.3
    touch_face:
      enabled: true
      dwell_seconds: 0.4
      grace_seconds: 0.35     # 手短暂丢失时进度不清零的容忍时间
      region_above: 0.1       # 相对鼻子的上边界（越小则不含鼻子以上）
      region_below: 0.9       # 下边界（覆盖到下颌）
      region_half_width: 1.0
      min_hands: 2
      require_both_sides: true
    hug:
      enabled: true
      dwell_seconds: 0.5
      grace_seconds: 0.40
      region_above: 0.15       # 手腕最多比肩高 0.15 肩宽（约 6cm）
      region_below: 0.20       # 最多比肩低 0.20 肩宽（约 8cm）
      region_inner_half_width: 0.45   # 手要离中线超过 0.45 肩宽（约 18cm）
      region_half_width: 2.50  # 外边界放到 100cm，不误挡
      min_hands: 2
      require_both_sides: true
```

**姿势对称的动作（拥抱、捧脸）靠肌肉名子串匹配左右同名肌肉**，所以不需要给左右不同数值。

## 电脑端

```powershell
C:\Users\11201\miniconda3\python.exe .\pc\reference\motion_bridge.py --mode interact
```

- `--gesture-source {board,pc,both}`：默认 `board`（用板端判定）。`pc` 回到电脑端
  的 `gesture_trigger.py`，`both` 两边同时跑，用于逐帧比对
- 状态行里的两个调参字段：

  | 字段 | 含义 |
  |---|---|
  | `board_prog=touch_face:0.45` | 当前进度最高的动作及其停留进度（0→1） |
  | `board_out=0.00` | 该动作的手离区域边界多远（0 = 已在区域内；`-` = 没检测到手） |

  读法：`board_prog` 一直是 `0.00` 且 `board_out` 很大 → 区域位置/大小不对；
  涨到 0.5 又归零 → 手部检测掉帧，该调大 `grace_seconds`

## Unity

一个 `GestureTriggerReceiver` 挂三个 `GestureBinding`（**不要挂三个组件实例** —— 见下）：

```csharp
[Serializable] public class GestureBinding {
    public string gestureName;   // touch_head / touch_face / hug
    public string triggerName;   // Animator Trigger 参数名
    public float holdSeconds;    // 必须等于该动画片段长度
    public HandOver handOver;    // 动作期间让出 EVMC4U 控制权的范围
    public GestureMode mode;     // OneShotTrigger / SustainedBool
}
```

动画与控制器由编辑器菜单生成：

```
菜单栏 → DigitalMan → 生成三个交互手势动画并接线
```

它会生成 `TouchFaceAction.anim` / `HugAction.anim`、把 `HeadPatController.controller`
重建为 4 状态 3 Trigger、并把三条绑定写进组件。**摸头那一段沿用已有的
`HeadPatAction.anim`，不重新生成。**

### 三个必须知道的坑

1. **绝不能挂两个组件实例。** EVMC4U 的 `Freeze` / `BlendShapeSynchronize` /
   `CutBonesEnable` 是接收器上的共享状态，两个实例各自 `Begin`/`End` 会互相覆盖快照，
   典型后果是数字人被**永久冻结且没有任何报错**。必须用一个组件 + 多条绑定。
2. **动画必须写满全部 95 条 Humanoid muscle 曲线。** 只写动作涉及的那几条，
   其余会回落到默认姿势（T-pose）。构建器已经处理，手改动画时要注意。
3. **`holdSeconds` 必须等于片段长度。** 短了会在动画播到一半时交还 EVMC4U（画面弹跳），
   长了会把末尾姿势多停一会儿。

## 防火墙

上位机监听 **39540 / 39541 / 39542 / 39543** 四个 UDP 端口，Windows 入站默认阻止，
必须逐条放行（管理员 PowerShell，一次即可）：

```powershell
New-NetFirewallRule -DisplayName "AI_humen UDP 39543" -Direction Inbound -Protocol UDP -LocalPort 39543 -Action Allow -Profile Any
```

漏了 39543 的现象是：其它数据都正常，只有 `board_rx=0/0` 一直不动。

## 板端只能跑一个实例

HWPRE 是独占硬件，第二个实例会在预检阶段退出：
`[FATAL][PREFLIGHT] HWPRE busy=1. Abort.`
启动前用 `pgrep -f '[s]dicamera'` 确认。

另外：**不要用 `pkill -9` 之后再立刻启动**，相机/传感器可能停在异常状态，
表现为两个端口都不出数据 —— 停掉后等几秒，或干脆重启板子。
