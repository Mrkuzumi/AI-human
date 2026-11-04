# yolov8-pose + yolov8-face + yolov8-finger（开发源码包）

## 融合版（2026-10-20）

本源码包保留腾讯 ASR、混元对话、腾讯 TTS、字幕和音量嘴型链路，并合入队友的 Pose 手腕兜底、摸脸备用区域和持续手势脉冲逻辑。board/full 源码及 AArch64 可执行文件已经同步；融合后的可执行文件 SHA-256 为 `699f18b2b2b646becd5a1de9eaca54a288c05fc24567b5109a2855351b528d41`。

所有 `.10/.91/.100/动态 IP` 配置均采用相同手势区域。持续保持时，摸头、摸脸、拥抱分别按 1.6 秒、2.5 秒、3.0 秒高电平和 0.4 秒低电平重复触发。

## 数字人语音对话（2026-10-20）

上位机已增加“腾讯 ASR → 混元对话 → 腾讯 TTS → 电脑扬声器”，并接入 Unity 字幕/音量嘴型。
首次使用需要在 `pc_voice/dialogue.local.json` 填混元 API Key，并开通腾讯 TTS；未配置时保持原有字幕。
原启动器 EXE 和板端均不需更新；用更新后的 Unity 工程点 Play。当前播音采用半双工防回声，暂不支持播音中插话。
配置、启动、验收和限制见 [pc_voice/DIALOGUE.md](pc_voice/DIALOGUE.md)。

本目录 `full` 用于开发和交叉编译，部署目录为同级 `yolov8-pose+yolov8-face+yolov8-finger_board`。

## 串口屏实时切换（2026-10-18）

TJC 屏幕以 `115200 / 8N1` 发送 `@MODE=follow` 或 `@MODE=interact`，末尾为实际 `0D 0A`。
新增 `tools/uart_mode_bridge.py` 将指令通过 UDP `39544` 转发给 PC，带确认、重试和串口重连；
同时读取 PC 已应用的模式，向屏幕发送 `page page2`（动捕）或 `page page3`（交互），
以三个 `FF` 字节结束；语音切换和串口重连后也会同步页面。
Python 3 标准库即可运行。本次没有修改三模型 C++ 或 AArch64 可执行文件。

PC 使用更新后的 `pc/start_3d_bridge.ps1` 和 Unity 工程，只需启动一次。
板端直接启动，默认串口 `/dev/ttyUSB0`、PC `192.168.126.100`，必要时自动通过 sudo 重新运行整个脚本：
CH340（1a86:7523）使用随包附带的 5.4.52 内核驱动；启动脚本会自动加载。

```bash
bash scripts/run_board.sh
```

如果 runtime 已运行，只另开串口程序，避免重复启动推理：

```bash
sudo bash scripts/load_ch340_driver.sh
sudo python3 -u tools/uart_mode_bridge.py --serial /dev/ttyUSB0 --host 192.168.126.100
```

退出时会清理串口监听；`UART_DEVICE='' bash scripts/run_board.sh` 可禁用串口。
完整部署、状态日志、PC 环境和测试说明见包内 `UART_MODE_CONTROL.md`。

本包包含三个模型的 AArch64 可执行文件、JSON/RAW、YAML 配置和源码。三个模型的输入均为 640×352，配置阈值均为 `0.35`。Pose/Face 平滑逻辑：框和置信度按 50% 前帧 + 50% 当前帧平滑（IoU > 0.80）；关键点按 60% 前帧 + 40% 当前帧平滑（IoU > 0.50）。

## 启动

将整个目录复制到板卡后，在目录内执行：

```bash
chmod +x scripts/run_board.sh
./scripts/run_board.sh
```

默认配置为 `configs/ZG/yolov8-pose+yolov8-face+yolov8-finger+hdmi-192.168.126.100.yaml`。启动脚本检查必要文件是否存在，但不自动校验 SHA-256，因此修改 YAML 阈值后仍可直接启动。也可以指定同目录下的另一个 YAML：

```bash
./scripts/run_board.sh configs/ZG/yolov8-pose+yolov8-face+yolov8-finger+hdmi-192.168.126.100.yaml
```

## 校验

```bash
sha256sum -c SHA256SUMS
```

所有项目均显示 `OK` 表示包内文件与清单匹配。`logs/` 是运行日志，不纳入清单。

## 交互手势（三个，全部在板端 PS 判定）

板端以主用户 `track_id=1` 的 pose 关键点（必要时回退到 face）、双肩和 finger 21 点，
逐帧判定三个动作，通过独立 GST1 UDP 端口 `39543` 发往上位机。**同一时刻只拉高一个**，
优先级 `hug > touch_face > touch_head`（手从头顶滑到脸颊会同时命中两个区域）。

| id | 信号名 | 判定 | 停留 |
|---|---|---|---|
| 0 | `touch_head` | Finger 手部或 Pose 手腕位于鼻子水平线附近或上方 | 0.5s |
| 1 | `touch_face` | 双手掌心位于鼻子两侧；Finger 漏检时由 Pose 双手腕颈胸区域兜底 | 0.4s |
| 2 | `hug` | **双手掌心**在双肩中点附近的肩胸高度带两侧张开；中间区域不计入 | 0.5s |

不是单纯的手腕固定半径检测：`touch_head` 用 `include/head_pat_trigger.hpp`（带鼻点缓存、
头宽 EMA），另外两个用 `include/gesture_region.hpp`（通用"区域 + 停留"，锚点可选鼻子或双肩中点）。
默认几何由代码提供；可选在 YAML 的 `pipeline.gesture_udp.touch_face` 和 `.hug` 下覆盖各动作参数，
**改完重启即可，不需要重新编译**。

持续保持动作时信号会脉冲化，使 Unity 可以重复播放动画：摸头、摸脸、拥抱高电平分别为
1.6 秒、2.5 秒、3.0 秒，动作仍保持时每次以 0.4 秒低电平重新触发。

完整的协议与调参说明见 `AI_humen/AI_humen/pc/docs/gesture_channel.md`。

使用前确认 YAML 中 `motion_udp`、`finger_udp`、`frame_udp`、`gesture_udp` 的 `host` 都是上位机 IP，
并允许 Windows 入站 UDP `39543`（管理员 PowerShell：`New-NetFirewallRule -DisplayName "DigitalHuman GST1 UDP 39543" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 39543 -Profile Any`）。
同一开发板上先停止其他占用 HWPRE 的 runtime；另外**不要 `pkill -9` 之后立刻重启**，
相机/传感器可能停在异常状态（表现为两个端口都不出数据），停掉后等几秒或重启板子。

上位机先在 Unity 打开 `AI_humen/AI_humen/pc/unity/DigitalManAvatar/Assets/Scenes/AvatarPreview.unity`
并进入 Play，再在 `AI_humen/AI_humen` 目录运行：

```powershell
python .\pc\reference\motion_bridge.py --mode interact --gesture-source board
```

上位机输出 `[GESTURE] touch_head ON/OFF` 之类的行表示 GST1 事件状态变化；
状态行里的 `board_prog=<动作>:<进度>` 和 `board_out=<离区域多远>` 用来调参。

注意 `interact` 会让数字人保持站立；Unity `GestureTriggerReceiver` 在 `follow` 模式会忽略手势信号。
此 runtime 保留该骨骼控制权设计，因此手势动画验证需使用 `--mode interact`。

## 腾讯云实时语音识别（2026-10-20）

默认 bash scripts/run_board.sh 会同时启动三模型、串口屏和板端麦克风上传。
上位机在 full 目录运行：

~~~powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\pc_voice\start_voice_server.ps1
~~~

语音目标默认 192.168.126.100:8766（TCP），本次可用 VOICE_HOST 覆盖。
板端仅需 Python 3 标准库与 arecord；云端凭据仅在 full/pc_voice/voice.local.json。
语音故障自动重试，三模型继续运行；临时禁用用 VOICE_ENABLED=0 bash scripts/run_board.sh。
部署到 /root 的仍是整个 board 文件夹；详细配置、日志与命令见 [CLOUD_VOICE.md](CLOUD_VOICE.md)。
