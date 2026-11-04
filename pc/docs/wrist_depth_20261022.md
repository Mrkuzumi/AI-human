# 手腕旋转与可选手臂深度：wrist-depth-20261022

修改位置：`runtimeapp/release_1_0_1`。原版 `release_1_0_0` 未修改。本轮运行改动在 Python 桥接器，Unity 模型与运行脚本不需要重新构建。新增的 Unity Editor 检查只用于验证。

## 已确认的问题与修复

| 问题 | 代码证据 | 本次处理 |
| --- | --- | --- |
| 手腕不能翻转 | 默认路径的 `_twists` 始终为零，只能绕 Z 做二维偏转 | 增加 `-WristOrientation`，从 RGB 的三维手掌补充绕手部局部 X 的旋转 |
| 拇指侧／小指侧可能反向 | 旧可选三维路径把固定摄像头向上方向当作参考，没有消除前臂旋转；实际模型合成姿态中，手臂抬起后能出现接近 180° 的误差 | 新路径直接使用关键点 17→5（小指掌指关节到食指掌指关节），对应实际模型两手的局部 +Z 食指／拇指侧；经过当前躯干、肩、上臂、前臂、二维手腕旋转的逆变换后求剩余翻转 |
| 手臂只有固定前后倾角 | 默认没有三维输入，只使用固定上臂前倾 10°、前臂相对深度 0° | 增加 `-ArmDepthOnly`，只读取 RGB 肩肘腕的带符号深度，不替换二维平面方向，不启用旧身体转向、腿深度或手指弯曲融合 |

第二项确认的是上述可复现的代码缺陷；没有把用户描述的每一次侧面反向都认定为同一种问题，也没有对所有左右手做统一反号。新路径不使用 MediaPipe 原始 Left/Right 标签决定身体侧别，而是匹配板端已分配的手腕与中指根部；两手交叉、身份接近时拒绝不明确匹配。

深度采用本项目已有的 MediaPipe GHUM RGB 模型。查阅的 GitHub 资料：[MediaPipe Pose](https://github.com/google-ai-edge/mediapipe/blob/master/docs/solutions/pose.md)、[MediaPipe Hands](https://github.com/google-ai-edge/mediapipe/blob/master/docs/solutions/hands.md)、[Kalidokit HandSolver](https://github.com/yeemachine/kalidokit/blob/main/src/HandSolver/index.ts)。没有照搬其他 VRM 的旋转系数或增加新的依赖库。单凭二维上下臂长度比不能唯一决定前后：改变深度符号可以保留相同二维投影。

主要源码：`pc/reference/motion_bridge.py` 的 `FingerSolver._wrist_twist`、手部关联与 `BodySolver._camera_arm_depths`；`pc/reference/rgb_pose_receiver.py` 的图像／三维推理与采集时间信息；`pc/start_3d_bridge.ps1` 的新开关、解释器选择和版本提示。检查代码为 `pc/tests/test_wrist_depth_mapping.py`、`pc/tests/build_wrist_depth_cases.py`、Unity 的 `Assets/Scripts/Editor/AvatarMappingChecks.cs` 与生成的测试 JSON。

## 保留与代价

- 不带新开关时仍为原来的二维映射。新增手腕模式不修改 30 个手指骨骼的弯曲或拇指对掌计算；手腕的二维偏转也继续使用板端点。
- 手腕旋转在当前父骨骼坐标系中求解；手臂前后旋转不会被重复叠加到手腕。
- 本轮补充的是轴向翻掌；保留原二维腕偏转，尚未让 RGB 接管腕关节向前／后折叠的全部三维自由度，以免同时改变当前手指与手腕的投影方向。
- 深度模式保留所有二维投影方向、躯干和腿部计算。上下臂的绝对深度分别平滑，再转换成父子局部旋转，避免相对角跨 ±180° 时绕远路。
- 新增的深度会改变手臂在屏幕中的投影长度和手腕位置。这是固定骨长加入深度后的实际变化，不能保证每种动作的屏幕端点都比二维版本更贴合，因此独立开关验证。
- 上下臂深度限制在 ±75°，优先避免单目错误造成大幅姿态翻转；正对镜头伸直的动作可能仍然伸不到完全 90°。
- 没有增加额外平滑层。RGB 推理在后台运行，不阻塞 DMV/FNG/Face；但新增的手腕旋转和深度仍受图像发送频率及 CPU 推理延迟影响。一次离线图像的两模型推理约 39 ms，不能当作实拍端到端延迟。
- 默认只接收与当前 DMV 同一来源 IP、source_id 且采集时间差不超过 150 ms 的三维数据，并按各手臂的图像位置、置信度继续筛选。JPEG 接收端跳过积压旧图像，只推理当前最新完整图像。
- 无效数据短暂保持后逐渐回到旧映射的固定深度／零手腕翻转。遮挡、过小的手或过快动作可能触发回退；若 RGB 长期过时，应先查图像链路和负载，不能把增大时间窗口当作准确性修复。
- 这是正面使用时的单目 RGB 估计，不能保证双臂完全藏到身体后方时判断正确，也没有开启全身转向重建。

## 正确启动

1. 在现在运行旧桥接的 PowerShell 中按 `Ctrl+C` 退出。只保留一个桥接；不要同时点 Runner 的旧启动按钮。已经运行的 Python 不会自动重载本轮改动。本次检查发现原二维桥接仍在占用 39540、39542、39544，未替用户终止它。
2. Unity 继续使用 `release_1_0_1/AI_humen/AI_humen/pc/unity/DigitalManAvatar` 工程的 `Assets/Scenes/AvatarPreview.unity`，进入 Play，使用 `follow` 模式。本轮不必重新构建播放器。若使用其他旧工程或旧 Runner 自带播放器，先确认其具有之前的模型映射修复；优先使用这个工程验证。
3. 先只验证手腕：

```powershell
Set-Location 'D:\FudanMicro\AAAAAnew\runtimeapp\release_1_0_1\AI_humen\AI_humen'
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\pc\start_3d_bridge.ps1 -Mode follow -WristOrientation -FingerLog .\pc\logs\wrist_only_20261022.jsonl -Hand3DLog .\pc\logs\rgb_wrist_only_20261022.jsonl
```

确认手腕方向后，`Ctrl+C` 退出该桥接，再启用手臂深度：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\pc\start_3d_bridge.ps1 -Mode follow -WristOrientation -ArmDepthOnly -FingerLog .\pc\logs\wrist_depth_20261022.jsonl -Hand3DLog .\pc\logs\rgb_wrist_depth_20261022.jsonl
```

启动必须显示 `Mapping build: wrist-depth-20261022`，以及包含 `release_1_0_1` 的 `Bridge source`。Python 行也会显示 `[MAPPING wrist-depth-20261022] ... wrist_only=True arm_depth_only=True/False`。仅使用过去的 `-Mode follow` 命令不会开启新功能。不要同时加旧的 `-Hand3DDrive`；它还会改变手指弯曲。仅加旧的 `-EnableRgb3D` 也不是本次保守增强模式。

本机已验证可用的三维 Python 是：

```text
C:\Users\11201\Desktop\AI-Humen-Runner-v1.4.1-win-x64\runtime\python.exe
```

脚本会在默认 Python 缺少 RGB 依赖时自动尝试这个已安装的解释器，用它执行 **当前 release 的 Python 文件**。这不启动 Runner 的旧桥接。也可在上述命令末尾显式添加：

```powershell
-PythonExe 'C:\Users\11201\Desktop\AI-Humen-Runner-v1.4.1-win-x64\runtime\python.exe'
```

不要将该参数作为单独一条命令执行。复制来的 `.venv-3d` 仍可能指向不存在的 `C:\Python314`，本次已验证自动回退可以绕过它。

## 开发板 RGB 前提与状态判断

新开关需要原始 RGB 图像，仅有 DMV/FNG 点数据不能恢复真实手掌翻转或唯一的前后深度。无需增加摄像头；现有板端程序支持 DMI1 JPEG。

本机 `.10` 配置文件已包含所需设置：

`runtimeapp/release_1_0_1/yolov8-pose+yolov8-face+yolov8-finger_full/configs/ZG/yolov8-pose+yolov8-face+yolov8-finger+hdmi-192.168.126.10.yaml`

```yaml
pipeline:
  frame_udp:
    enabled: true
    host: 192.168.126.10
    port: 39541
    width: 640
    height: 360
    quality: 55
    stride: 3
```

请确认开发板实际加载的是这份配置或相同设置，启动日志应包含 `Frame UDP target: 192.168.126.10:39541 ...`。若配置文件已更新而板端仍载入旧配置，仅重新启动 PC 桥接不会改变板端发送行为。本次曾在未被其他进程占用的 39541 上监听 5 秒，未收到 RGB 包；这只能说明该观察窗口没有图像，不能据此认定板端永久未发送。

上位机状态解释：

- `rgb=.../...`、`hand3d=.../...` 应连续增长；始终 0/0 时检查板端 frame_udp、网线和 UDP 39541 入站规则。
- 可见且匹配的手应出现 `wrist3d=Left:OK/Right:OK`；只看见一只手时另一只显示 `hold/fallback` 是正常的。
- 开启深度后，`pose3d=True` 才表示当前帧确实接受了三维深度；`rgb_skew_ms` 是 RGB 与 DMV 的采集时间差，单位毫秒。图像计数增长但长期 `pose3d=False`，应检查时间差、RGB 人体是否与板端主用户相同、手臂是否可见。
- 深度模式无需启动时 30 帧归零，状态 `arm_cal=0/0`；上臂 `arm_depth` 正值朝镜头，负值背离镜头。它只打印上臂深度，不代表整只手臂的前后位置。

## 已完成验证与实拍补充

- Python：147 项单元测试通过；包含实际 rig 点位生成的 300 组左右手、镜像、前臂方向与手掌翻转组合；无 RGB 时输出与当前二维路径相同，单只手臂遮挡时独立保持／回退。
- Unity：36 组真实模型／VMC 接收检查通过，肩肘腕投影方向误差最大约 0.0198°，30 次两手食指／小指侧检查误差低于 Unity 浮点测量分辨率。该结果验证数学变换，不代表实拍模型误差为零。
- UDP 集成：实际本地 RGB 模型 + 一张已有图像 + 与之匹配的模拟 DMV/FNG，120 帧输出 6730 个 VMC 包，118 帧接受有效手腕三维结果；期间未干扰现有用户桥接／Unity。
- Windows PowerShell 5：自动选用有效 RGB 解释器和两个新增开关的传参检查通过。

请分阶段拍摄：先固定双肘只翻掌（掌心、手背、拇指侧、小指侧朝镜头），再做手臂下垂／侧平举／抬高并重复翻掌，最后分别向前伸、向后收。左右手各做一次，最后才测试交叉和遮挡。每阶段保持 2 秒，使用上述独立日志文件避免覆盖此前记录。

若仍有偏差，提供同步的真人画面与 Unity 画面、所用启动命令、板端 `Frame UDP target` 行，以及两份 JSONL。RGB 日志现在同时包含手部三维点和 `pose_world_h36m`／`pose_image_h36m`，Finger 日志增加完整的 `body_keypoints` 17 点，可按 `capture_time_ns` 对齐板端日志，区分是三维模型前后估错、时间过旧、手部身份不明，还是父子骨骼转换问题。无需访问训练机。

完整离线证据与修改前备份在 `D:\FudanMicro\AAAAAnew\work\wrist_depth_20261022`。
