# 单摄像头手臂前后深度

> 2026-10-22 更新：为保留当前已验证的二维映射，推荐使用 [手腕与深度更新说明](wrist_depth_20261022.md) 中的 `-ArmDepthOnly`。此模式保留肩肘腕在画面中的方向，采用绝对带符号深度，不执行下文旧模式的 30 帧深度归零或身体转向混合。同步指标改为采集时间差 `rgb_skew_ms`；RGB 与 DMV 序号来自不同计数器，旧 `pose_lag` 序号差不能用于判断延迟。

## 实机结论

仅把 YOLOv8-Pose 的 COCO-17 二维点送入 VideoPose3D，无法稳定区分“礼仪式双手在身前”和“跨立式双手在身后”。两种动作在正面摄像头下的二维投影接近，且手腕被身体遮挡时检测置信度仍可能接近 1.0，因此不能用置信度或二维几何规则可靠补救。

可选三维增强链路为（默认启动只使用板端二维点）：

```text
板端 YOLOv8-Pose/Face ── DMV1:39540 ──┐
板端 RGB565→640×360 JPEG ─ DMI1:39541 ├→ 上位机融合 → VMC:39539 → Unity
                                      └→ MediaPipe GHUM 33 点世界坐标（只取手臂 Z）
```

这仍然只有一个物理摄像头。板端每 3 个后处理帧发送一张 JPEG，UDP 分片不超过 1228 字节；丢失任一分片时整帧自动丢弃，不阻塞现有动作包。RGB 3D 结果超过 0.5 秒未更新时，短暂保持上一深度，随后回到双臂统一向前的安全姿势。

## 启动

板卡程序需要以 `sudo` 运行，以访问 `/dev/udmabuf0`。要启用三维增强，板卡开始向上位机 `:39540` 和 `:39541` 发包后，在项目根目录执行：

```powershell
.\pc\start_3d_bridge.ps1 -EnableRgb3D
```

启动后的前 30 个有效三维人体帧用于左右臂中立姿态标定。请自然站立、双臂放在身体两侧约 2 秒，等状态显示 `arm_cal=30/30` 后再测试前后动作。`rgb=成功推理帧/完整图像帧`；`arm_depth=L+12.3/R-8.4` 中正值表示手臂朝摄像头，负值表示朝身体后方。

如果动作前后完全相反：

```powershell
.\pc\start_3d_bridge.ps1 -ArmDepthScale -1
```

## 环境重建

```powershell
python -m venv .venv-3d
.\.venv-3d\Scripts\python.exe -m pip install -r .\pc\requirements-3d.txt
Invoke-WebRequest `
  -Uri "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task" `
  -OutFile ".\pc\models\pose_landmarker_full.task"
```

模型位置：`pc\models\pose_landmarker_full.task`。当前文件 SHA256 为 `4EAA5EB7A98365221087693FCC286334CF0858E2EB6E15B506AA4A7ECDCEC4AD`。Full 模型在当前上位机 CPU 上的黑帧热态基准约为 22 ms/帧。

## 限制

MediaPipe Pose Landmarker 直接读取 RGB，并输出 33 个三维世界坐标；相较只读取二维骨架的提升模型，它能利用遮挡、纹理和人体外观判断深度。不过单目三维仍是模型估计，不是真实深度传感器，而且它与板端动捕包分开推理，可能对应不同采集帧。因此它改为显式选项；启用后实际验收应同时观察 `rgb` 连续增长、`pose_lag` 是否较小，以及两种目标动作的 `arm_depth` 符号。双手完全被躯干遮住时，没有任何单摄像头算法能保证每帧正确。

VideoPose3D 代码与权重仍保留为命令行回退项，但不再由默认启动脚本使用。
