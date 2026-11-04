# 腾讯云实时语音识别（2026-10-20）

## 启动上位机（Windows）

在同级的 full runtime 根目录打开 PowerShell，执行：

~~~powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\pc_voice\start_voice_server.ps1
~~~

首次运行自动建立 pc_voice/.venv 并安装 websocket-client，启动后监听 TCP 8766。
保持窗口打开；识别中的文字和最终文字会同时显示在此窗口与板端控制台。
只检查配置和依赖时，在命令末尾添加 -Check，不会调用腾讯云。
这一路语音服务与 AI_humen 的 motion_bridge、Unity 并行运行，仍按原来的方式启动数字人。
本次接入只输出识别文字，没有把文字接成数字人对话、语音合成或串口屏指令。

full/pc_voice/voice.local.json 已从旧模块迁入本机的 AppID、SecretId、SecretKey、上传口令。
该文件只留在 PC，不上传板子，也不写进 SHA256SUMS。
voice.example.json 是不含云端密钥的配置模板。
环境变量 TENCENTCLOUD_APP_ID、TENCENTCLOUD_SECRET_ID、TENCENTCLOUD_SECRET_KEY、
VOICE_UPLOAD_TOKEN 可覆盖 PC 本地配置。
实时服务不再依赖 D:/Qiansai/modelzoo 中的旧模块。

Windows 防火墙需允许专用局域网入站 TCP 8766；如果被拦截，在管理员 PowerShell 执行：

~~~powershell
New-NetFirewallRule -DisplayName "DigitalHuman Voice TCP 8766" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8766 -Profile Private -RemoteAddress LocalSubnet
~~~

## 上传与启动板端

由用户将整个 yolov8-pose+yolov8-face+yolov8-finger_board 文件夹上传到 /root。
不需要更换模型、三模型 AArch64 主程序或交叉编译。
在板端执行：

~~~bash
cd /root/yolov8-pose+yolov8-face+yolov8-finger_board
bash scripts/run_board.sh
~~~

默认同时启动三模型、串口屏控制和录音上传。
PC 地址沿用 run_board.sh 的第三个参数 / MODE_HOST，默认 192.168.126.100。
语音默认 TCP 8766，摄像头/手势仍使用原来的 UDP 端口。
configs/voice.json 中保存麦克风 audio_device、server_port、enabled、
upload_token 和重连间隔。这里没有腾讯云 SecretId / SecretKey。
上传口令必须与 PC 配置一致；云端凭据只由 PC 使用。
录音格式为 16kHz、16-bit、小端、单声道 PCM，每块 200ms。

板子需要 Python 3.8+（标准库即可）以及 arecord；运行 arecord -l 检查麦克风。
如果提示找不到 arecord，安装板端 alsa-utils。
麦克风型号不同可指定 AUDIO_DEVICE=plughw:1,0。
连接或录音失败会自动重试，语音故障不会停止三模型与串口屏控制。
Ctrl+C 或三模型进程退出时，会清理语音 helper 及 arecord。

~~~bash
# 仅本次覆盖语音地址/麦克风；串口与视频参数保持原有用法
VOICE_HOST=192.168.126.100 AUDIO_DEVICE=plughw:1,0 bash scripts/run_board.sh

# 暂时禁用语音
VOICE_ENABLED=0 bash scripts/run_board.sh

# 三模型已经运行时，只另开语音；不要同时再开第二份 run_board.sh
python3 -u tools/board_voice_stream.py --host 192.168.126.100
~~~

VOICE_PORT、AUDIO_DEVICE、VOICE_UPLOAD_TOKEN 可覆盖板端 JSON；
VOICE_ENABLED=0 是启动脚本开关，configs/voice.json 的 enabled=false 也可禁用录音。
不要同时启动两份语音进程占用同一个麦克风。
板子仅连接 PC，不要求板子直接访问腾讯云。

## 日志与检查

板端文字与录音错误写入 logs/trinet-时间.log；PC 窗口显示 [VOICE]、
[识别中] 和 [最终]。PC 没有启动时，板端会显示重连并继续视频推理。
无人在摄像头前不会阻止语音录音或识别。

~~~powershell
# 在 full 根目录；本地 TCP 和模拟云端，不产生云端调用
.\pc_voice\.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_cloud_voice.py -v
.\pc_voice\.venv\Scripts\python.exe -B -m unittest discover -s pc_voice -p test_stream_receiver.py -v
~~~

板端可用 sha256sum -c SHA256SUMS 检查部署包。
清单使用 LF；logs、虚拟环境和本地云端凭据不进入清单。
本次离线测试覆盖 PCM 上传、真实本地 TCP、云端结果转发、鉴权拒绝、
EOF 后最终文字、断线重试、录音子进程清理、配置覆盖及 full/board 同步。
实际麦克风、云端账号权限和现场网络仍需用户上板后验证。
