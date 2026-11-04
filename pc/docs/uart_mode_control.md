# TJC 串口屏实时切换动捕 / 交互模式

板端串口监听会读取 PC 已确认的模式，并向屏幕写入页面指令：
`follow` → `page page2` + `FF FF FF`，`interact` → `page page3` + `FF FF FF`。
语音口令和屏幕按钮使用同一模式状态；板端串口重连后再次同步页面。
若使用 `-ModeControlHost`，设置为板卡 IP，允许板卡查询当前模式。

当前部署：板卡 `fmsh@192.168.126.49`，上位机 `192.168.126.100`。
启动脚本默认选用 `.100.yaml`；不填写第三个参数时，串口控制目标也默认为 `192.168.126.100`。
原 `.91.yaml` 保留，`.100.yaml` 仅在对应 `.91.yaml` 基础上修改四路 host，保留该包原有识别和手势参数。

屏幕：TJC1060X5A1_011C_I，建议 UART 115200、8N1。已验证按钮输出：

```text
@MODE=follow\r\n
@MODE=interact\r\n
```

这里 `\r\n` 代表实际字节 `0D 0A`。屏幕弹起事件分别使用
`prints "@MODE=follow",0` / `prints "@MODE=interact",0`，随后执行 `printh 0D 0A`。
关闭按钮默认的“发送键值”；进入、返回按钮只跳页。

## 1. 部署位置与连接

开发修改在 `yolov8-pose+yolov8-face+yolov8-finger_full`；上传板卡的是
`yolov8-pose+yolov8-face+yolov8-finger_board`。本次新增 Python 标准库控制程序，
三模型 C++、模型、阈值和原 AArch64 可执行文件不需要修改或交叉编译。

推荐屏幕通过匹配实际接口电平的 USB 转串口模块接入板卡 USB Host。
TTL 接口交叉接 TX/RX 并共地，屏幕按铭牌要求独立供电。
不要把屏幕 TX/RX 接到 Mini-USB 的 USB 数据线上；板载 PS UART 侧为 1.8V。

在板卡上确认 USB 串口设备名：

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB* /dev/ttyACM*
python3 --version
```

优先使用 `/dev/serial/by-id/...` 的稳定路径，避免 USB 重新插拔后编号变化。
程序依赖 Python 3 标准库的 `termios`、`fcntl`，不依赖 pyserial、yaml 或 pip 包。
运行账号必须能读写串口，例如已有串口权限的用户或板卡 root。

## 2. PC 只启动一次

在 `AI_humen/AI_humen` 目录，使用 Python 3 环境启动桥接器。默认仅用板端 NPU 输出，不需要 MediaPipe：

```powershell
.\pc\start_3d_bridge.ps1 -Mode follow
```

`-Mode` 现在是初始模式，按钮可以运行中覆盖它。默认监听 UDP `39544`。
也可用 `-Mode interact` 启动。需要单目身体深度时添加 `-EnableRgb3D`；需要用 MediaPipe 三维手部结果参与指骨驱动时再添加 `-Hand3DDrive`。
Unity 使用本工程更新后的 `GestureTriggerReceiver.cs`，打开 `AvatarPreview.unity` 并进入 Play。
若使用独立打包的 Unity 播放器，需要重新构建播放器使 C# 改动生效。

若启用 `-EnableRgb3D` 或 `-Hand3DDrive`，需要安装原 MediaPipe 依赖并提供模型文件；此时如果拷贝来的 `.venv-3d` 指向另一台电脑的 Python，需重建该环境，或传入已安装 numpy/MediaPipe 的 Python 完整路径：

```powershell
.\pc\start_3d_bridge.ps1 -Mode follow -PythonExe 'D:\your-3d-env\Scripts\python.exe'
```

可使用 `-ModeControlHost <板卡IPv4地址>` 限制控制消息来源。
控制协议面向可信局域网，无身份认证；不要将控制端口映射到公网。
若只需旧的固定模式启动行为，可加 `-ModeControlPort 0`。

PC 防火墙除原数据端口外，需要允许 UDP `39544`。如 Windows 防火墙拦截，
由用户在管理员 PowerShell 中执行（此文档不会自动修改防火墙）：

```powershell
New-NetFirewallRule -DisplayName 'DigitalHuman mode UDP 39544' -Direction Inbound -Action Allow -Protocol UDP -LocalPort 39544 -Profile Private
```

## 3. 板卡启动

进入上传后的 `yolov8-pose+yolov8-face+yolov8-finger_board` 目录，直接启动：

```bash
bash scripts/run_board.sh
```

默认使用 `.100.yaml`、`/dev/ttyACM0` 和 PC `192.168.126.100`，自动同时启动串口监听。
非 root 用户启动时，整个脚本会通过 sudo 重新运行，可能提示输入当前用户密码；
这样监听程序及退出清理使用同一权限。无需另外启动串口程序。
参数仍依次为配置、屏幕设备、PC IP；第三个参数必须与 YAML 中四路 UDP 的 host 一致。
若只想启动推理、不启用串口，使用 `UART_DEVICE='' bash scripts/run_board.sh`。
Ctrl+C / runtime 退出时，脚本会清理它启动的串口控制程序。

如果三模型 runtime 已经在运行，不要再开第二个 runtime；另开一个板卡终端，只运行控制程序：

```bash
sudo python3 -u tools/uart_mode_bridge.py --serial /dev/ttyACM0 --host 192.168.126.100
```

串口程序启动后不主动改变 PC 初始模式，等待首次按钮选择；后续每 0.5 秒同步最新选择。
PC 进程重启后会自动恢复到该选择。串口程序或板卡重启后，使用 PC 初始模式，需重新点击按钮。
USB 断开后每 2 秒尝试重新打开同一路径。

## 4. 如何确认已接通

点击交互按钮后，板端依次打印：

```text
[MODE] requested interact (seq=1)
[MODE] PC bridge confirmed interact (seq=1)
```

PC 在真正发生变化时打印：

```text
[MODE] applied interact (source=board)
```

数字人停止动捕跟随，三个板端手势仍可触发交互动画。点击动捕按钮后恢复跟随。
如果进入交互时某个手势已经保持高电平，需要先松开再做一次，避免旧动作被重放。
动画播放中切回动捕会停止交互协程、恢复 Freeze/表情同步状态并移除 Animator Controller。
重复点击同一模式不会中断正在播放的交互动画。

`PC bridge confirmed` 只确认 Python 已应用模式并发出 OSC，并非 Unity 播放器回执。
模式 OSC 会在模式变化时立即发送，并每秒补发，即使没有相机数据也会同步。
现有屏幕固件没有约定状态控件名，本次不向屏幕写不存在的控件，确认状态显示在终端和日志中。

若打印 `PC not confirmed`，检查 PC 进程、IP、防火墙和 UDP 39544。
若打印 `[UART] ... retry in 2 seconds`，检查设备路径、权限及串口是否被其他进程占用。
屏幕波特率、电平、供电仍需实机核对；背景图片里的波特率文字不会自动配置 UART。

## 5. 开发验证

PC 单测（包含真实桥接子进程、UDP、骨骼输出、无相机模式切换及手势超时释放）：

```powershell
python -B -m unittest discover -s pc/tests -p 'test_*.py'
```

Linux / Docker 中，从工程根目录运行串口测试：

```bash
python3 -B -m unittest discover -s yolov8-pose+yolov8-face+yolov8-finger_full/tests -p test_uart_mode_bridge.py -v
```

PTY 集成测试不依赖屏幕或 NPU，覆盖串口拆包、连续命令、PC 重启自动同步和 USB 重连。
Unity 批处理校验入口为 `DigitalMan.Editor.ModeSwitchChecks.RunBatch`，加载实际场景检查
Controller 切换、重复模式、保持中的手势以及中途恢复骨骼控制权，不保存场景。

部署包更新后执行 `sha256sum -c SHA256SUMS`。更改 YAML 后清单里的对应配置哈希自然会不匹配；
本次没有改变原三模型程序的二进制。
