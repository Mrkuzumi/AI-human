# AI-human 源码

仓库包含 PC 端动捕桥接、Unity 工程、开发板三模型源码和语音服务。

## 目录与启动

| 路径 | 用途 |
| --- | --- |
| `pc/reference/`、`pc/start_3d_bridge.ps1` | PC 端 DMV/手势/表情桥接 |
| `pc/unity/DigitalManAvatar/` | Unity 工程，保留 `Assets`、`Packages`、`ProjectSettings` 及模型素材 |
| `board/src/`、`board/include/`、`board/CMakeLists.txt` | ZG330 三模型板端源码 |
| `board/pc_voice/` | PC 端腾讯 ASR、对话和 TTS 服务 |
| `fpga/hwpre/` | 960×512 视频预处理 RTL 与仿真用例 |

在 Windows PowerShell 中从仓库根目录启动桥接：

```powershell
python .\pc\reference\motion_bridge.py --mode interact --gesture-source board
```

如需 RGB 3D 功能，先安装 `pc/requirements-3d.txt`，并将 `pose_landmarker_full.task`、`hand_landmarker.task` 放在 `pc/models/`；然后运行 `pc/start_3d_bridge.ps1` 并传入所需开关。普通桥接只用 Python 标准库，不需要这两个模型。

用 Unity Hub 打开 `pc/unity/DigitalManAvatar`，加载 `Assets/Scenes/AvatarPreview.unity` 并点击 Play。Unity 首次打开会生成 `Library` 缓存。

## 板端部署与语音

仓库仅保存板端源码和配置。板端部署前，从原始发布包将三份 `imodel/ZG/*.raw`、AArch64 可执行文件及适用内核的 CH340 驱动复制到 `board/` 对应位置，或在本机配置 iCraft SDK 后重新编译。随后在开发板的 `board/` 目录运行 `bash scripts/run_board.sh`。缺少上述文件时脚本会报出具体文件名。

`pc/docs/` 中保留了原项目的技术记录，部分历史命令仍提到 Runner 和旧机器路径；以本页命令为当前启动方式。

语音配置请复制 `board/pc_voice/voice.example.json` 为 `voice.local.json`、复制 `dialogue.example.json` 为 `dialogue.local.json`，填写自己的云端凭据；板端语音上传配置请复制 `board/configs/voice.example.json` 为 `voice.json`，两端 `upload_token` 设为同一个自选随机值。`*.local.json` 和 `voice.json` 已被 Git 忽略。PC 端从仓库根目录启动：

```powershell
powershell -ExecutionPolicy Bypass -File .\board\pc_voice\start_voice_server.ps1
```

## Git 提交范围

保留源码、配置模板、测试、协议文档，以及 Unity 场景依赖的 VRM 和资源。FPGA 只保留自定义 HWPRE 模块与仿真，详情见 [fpga/hwpre/README.md](fpga/hwpre/README.md)。排除 Runner、三份顶层 `.pt`、板端 `.raw`、预编译程序、SDK/第三方大型编译依赖、旧版重复部署包、Unity `Library`、Python 虚拟环境、日志和真实密钥配置。需要部署产物时从原始发布包本地复制，不要用 `git add -f` 绕过忽略规则。
