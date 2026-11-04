# 数字人语音对话（2026-10-20）

## 先配置两项云服务

1. 在[腾讯 TokenHub 控制台](https://console.cloud.tencent.com/tokenhub)创建普通 API Key，并开通需要调用的模型服务；把完整 Key 填入本目录 `dialogue.local.json` 的 `api_key`（不能用带星号/省略号的脱敏显示值），不要改 `voice.local.json` 的 ASR SecretKey。两者不是同一种凭据。当前接口为 `https://tokenhub.tencentmaas.com/v1/chat/completions`，默认模型已改为 `hy-role`（Hy-Role 角色对话），可在同一文件改 `model`。旧混元 `hunyuan-turbos-latest` 不适用于本接口。优先读取本地 Key，也支持 `TOKENHUB_API_KEY`，兼容原环境变量 `HUNYUAN_API_KEY`；所有密钥均不进入启动器 EXE。参考[TokenHub 模型列表](https://cloud.tencent.com/document/product/1823/130051)。
2. 在[腾讯云语音合成](https://console.cloud.tencent.com/tts)开通 TTS，确认有可用额度、允许 `TextToVoice` 调用。它复用 `voice.local.json` 的腾讯 SecretId/SecretKey；仅开通 ASR 不等于已开通 TTS。本程序不会自动购买、开通云服务或充值。
3. `api_key` 未填写时程序明确显示 `[待配置]`，继续原来的语音字幕，不会伪造回答。正式调用对话与 TTS 会按云端规则消耗额度。密钥文件不能发给队友、提交仓库或放到板子。

Hy4/Hy3 的请求明确设置 `thinking.type=disabled`，用于短语音问答，避免默认长推理影响响应；流式返回中的 `reasoning_content` 不朗读。创建 Key 不等于模型服务已经开通，HTTP 402 时检查模型服务和额度。仅配置和离线测试不消耗云额度。

## 速度优先配置

当前使用 Hy-Role（`hy-role`），以角色聊天代替通用深度推理。已通过当前 Key 的只读 `GET /v1/models` 确认该 ID 为 online；这不代表已确认账户有调用额度，也不是实际推理速度测试。平台模型信息不提供首音延迟保证，不能只凭模型名字承诺快几秒。

默认提示词要求直接回答、第一句尽量不超过16个汉字，默认最多两句、总计50字；不是先播放“好的/让我想想”等占位话。生成输出预算从300降到160 tokens，历史默认从8轮减到4轮，以减少长回复和多轮输入负担；复杂知识推理和长篇解释不作为本配置的优化目标。

原先等到句号或最多60字才提交 TTS；现在首段遇到至少8字符后的中文逗号/冒号也可提前合成，没有自然停顿时最多24字符就提交首段，后续仍以完整句/最多60字处理。短回答不硬凑24字，收到句号或流结束即提交。首段切短可能在极长句中产生额外停顿/TTS请求，优先用短句提示词避免此情况。

本次不改 ASR 停顿检测、爱小芊音色、Unity 播音/口型、回合锁和回声抑制；也没有采用占位音频、跳过识别结果、或在关闭对话时发起云调用。仅需在启动器停止并重新启动语音服务，无需更新 EXE、Unity 或开发板。模型尚未启用/额度不足时需在 TokenHub 控制台检查本模型，不能自动退回慢模型，也不会自动开通后付费。

未调用付费推理/TTS进行测速；实际“说完话→听到回答”的时间还包含 ASR 判停、网络、云端排队与爱小芊合成，需现场对比。修改前备份：`D:\Qiansai\FACE_60x60\DialogueFastBaseline-20261020-220145`。

## 启动仍用原来的 EXE

当前默认音色已改为 **爱小芊（601009）**，本机 `dialogue.local.json` 与公开示例一致。它属于腾讯“大模型音色”，不是精品音色；需要对应类别的可用额度/余额，不会自动购买或退回其他音色。默认保持中性情感和正常语速，不主动添加撒娇等情绪参数。音色核对来源：https://cloud.tencent.com/document/product/1073/92668 。

启动器 v1.4.1 提供独立“语音字幕 / 语音对话”开关，对话每次打开默认关闭。关闭对话时强制 `--asr-only`，不读取对话密钥、不创建大模型或 TTS 会话；开启时强制 `--require-dialogue`，覆盖本地 `enabled=false`，但要求已配置完整 Key，缺失时明确启动失败。关闭“语音字幕”会停止整个语音服务并同时关闭对话；重新勾选后点击启动。运行中切换对话模式只重启语音服务，板端自动重连，数字人上位机不受影响；正在进行的语音轮次会中断、该连接聊天记忆清空。语音字幕 ASR 本身也可能计费，不是永久免费；已提交的调用不保证退费。旧版启动器没有此开关，手动命令仍沿用下文的本地配置行为。

保持选择现有 **full** 目录，不要选板端轻量包；启动器 EXE、本机路径设置和启动命令不变，本次不改启动器源码，也不需重新编译 EXE。若已有旧语音进程，先在原窗口 Ctrl+C 或在启动器停止该项，再启动。

Unity 打开同级 `AI_humen/AI_humen/pc/unity/DigitalManAvatar` 的 `AvatarPreview` 场景，等待脚本编译完成，再点 Play。新增嘴型/字幕接收器会自动创建，不用手工挂组件。若你使用的是以前导出的独立 Unity Player，它不会自动得到新的 C# 脚本，需要用更新后的工程重新构建 Player。

板端保持原来的 `run_board.sh`，无需改 YAML、编译或上传。电脑默认播放设备（Windows 声音设置中的扬声器/耳机）负责发声，不向板子发送声音。队友也可使用同样上位机文件，板子仍发往他电脑的 `.10` 地址。

手动启动仍为：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\pc_voice\start_voice_server.ps1
```

在 full 目录仅检查本地配置（不调用、不能证明云服务可用）：

```powershell
.\pc_voice\.venv\Scripts\python.exe -B .\pc_voice\stream_receiver.py --check
```

临时只要字幕：用 `--asr-only` 启动 Python 服务，或把 `dialogue.local.json` 的 `enabled` 改为 `false` 后重启语音项。

## 正常工作时

交互模式中，Unity 开始播放摸头、捧脸或拥抱动作时，会通过本机 UDP 39546 通知语音服务。正在生成或播放的云端回答会立即取消，随后直接读取 `pc_voice/gesture_audio` 中对应的 WAV，用现有 Unity 音频与嘴型链路播放；触发时不请求对话大模型或在线 TTS。文案在 `pc_voice/dialogue.py` 顶部的 `GESTURE_REPLIES` 中修改。改文案后运行 `pc_voice/.venv/Scripts/python.exe -B pc_voice/generate_gesture_audio.py` 重新生成三个 WAV（此命令调用三次在线 TTS）；复制给队友时要包含 `gesture_audio` 目录。语音服务必须启用“语音对话”，并重启以加载新代码；Unity 必须使用本次重新构建的 Player。

板子连续上传 16 kHz 麦克风音频 → 腾讯 ASR 最终句 → TokenHub 流式回答 → 每个短句调用腾讯 TTS → 本机 Unity 播音并驱动嘴型。默认保留当前板端连接最近 4 轮聊天；停止服务、板子断连后不保留聊天记录，不是长期记忆。默认每次简短回答，不上传摄像头画面给对话模型。

终端依次出现 `[最终] 你的话`、`[对话] 正在思考…`、`[数字人] 回答短句`。Unity 底部显示当前回答短句，通过 uLipSync 分析实际 AudioSource 音频的 MFCC 特征，混合驱动 VRM 的 A/I/U/E/O 五种嘴型，静音闭嘴。它只短暂接管嘴部，不改眨眼、身体、手指或动作模式；不是逐字、逐音素强制对齐，也不靠随机动画模拟说话。

## uLipSync 音频与口型同步

- 参考项目：https://github.com/hecomi/uLipSync （MIT）。工程内嵌 3.1.5，固定上游 commit `ece3272762f2148dd8d3e658ec04fcbd79d4d97e`；保留许可证，不复制第三方角色和语音示例。Unity Package Manager 首次导入会解析 Burst、Mathematics、Timeline 依赖。
- Python 将本句 WAV 写到 Windows 临时目录 `digitalman-speech-*/reply.wav`，只通过本机 UDP 提交路径。Unity 检查路径、文件大小与 PCM16 格式，回报准备完成后，Python 才提交播放。Unity 用同一 AudioSource 发声并分析口型；回答期间的麦克风抑制和回合锁保持有效。
- Unity 未运行/未更新时，在准备阶段无响应会回退原有 Windows 播音与音量事件。准备本身不发声，因此迟到的数据不会造成双重播音；提交播放后如确认丢失，宁可报错停止，也不再重复回退播音。旧 Player 需重新构建才能获得算法嘴型。
- 关闭语音服务、取消、断开连接时会请求停止；Unity 另有 0.7 秒无心跳保护，防止后端被结束后声音或张嘴遗留。只清理本句临时 WAV；历史字幕保留 3 秒再消失。
- `Assets/Resources/DialogueLipSyncProfile.asset` 使用上游通用女声校准作为初始配置，**尚未针对爱小芊的中文发音专门校准**，不是百分百精准口型。后续可用已合成的爱小芊语音在 uLipSync Profile 面板校准，不需要替换对话模型。口型算法本地运行，不调用额外云 API。

当前采用**回合锁定 + 轮流说话（半双工）**：问题入队时立即锁定本轮，排队、思考、合成、播放以及回答的句间停顿期间，普通新识别句不排队、不替换当前问题，也不额外调用大模型。忙时“嗯、啊、哦、呃”等孤立附和词显示 `[已忽略] 本轮处理中，附和短词不会打断原问题`；其他新句显示保留原问题的提示。空闲时不按字数过滤，“不”“为什么”“嗯”等有效短答仍可接续聊天；纯标点不触发对话。这不是声纹识别：空闲时的旁人说话仍可能被接收。

首次播音开始前，如需取消，单独说“取消刚才的问题”或“我重新问”，等日志出现 `[对话] 本轮已取消，可重新提问` 后再问新问题。也支持“取消上一个问题”“取消当前问题”“我要重新问”，前三条取消指令可加“请”。只匹配完整指令，去除空格和标点后判断；“我重新问是什么意思？”等普通问句不会取消当前回合。指令后直接连着新问题不会自动拆分或提交。失败、正常完成和取消都会释放回合锁。

播放时及其后默认 1.2 秒内，麦克风音频替换成静音送 ASR，并忽略返回的回声字幕，避免数字人听见自己的声音后自问自答。请等她说完后约 1.2 秒再回答。首次播音开始后，本轮的句间停顿也不接受语音取消；**播音时暂不支持自由插话**，不能把此机制当作回声消除（AEC）。后续需要同时说话时，应先加入带播放参考信号的 AEC，再开启真正的语音打断。`echo_tail_seconds` 可在本地配置改为 0.4–5 秒；房间回声长时应增加，不能承诺所有音箱距离下完全消除回声。

回合锁更新后，只需在原启动器中停止再启动“语音识别/语音对话”项，以加载新 Python 代码；不需重编译 EXE，不需重启 Unity 或上传板子。验收：先问完整问题，在 `[对话] 正在思考…` 之后说“嗯”，应看到忽略提示并听到原问题的回答，不应出现第二次“正在思考”。

## 故障定位

| 位置 / 日志 | 原因与处理 |
|---|---|
| `[待配置]` | 没有混元 Key；在 `dialogue.local.json` 填 `api_key`，保存后重启语音项 |
| 混元 HTTP 401/403 | Key 无效、服务权限或账号配置问题；ASR 的 SecretKey 不能替代混元 API Key |
| `腾讯 TTS …AppIdNotRegistered` / 权限 / 余额错误 | 检查 TTS 开通、额度、腾讯密钥权限，音色是否可用 |
| 有回答文字但没有声音 | 检查 Windows 默认播放设备/音量；查看 `[对话错误]`，声音不从板子播放 |
| 有声音但没字幕/嘴型 | 确认 Unity 使用更新后的工程并进入 Play，等待首次包导入/编译完成；本机 UDP 39545 未被其他项目占用 |
| `Unity 播音失去响应` | 提交播放后 Unity 停止响应，为避免重复发声不会回退重播；检查 Unity 是否暂停、退出 Play 或正在编译 |
| `腾讯云握手失败 4004` | 核对已购 ASR 资源包的引擎类型。此版本使用 `16k_zh_en_2.0`，对应“实时语音识别·大模型2.0版”资源包；购买其他引擎的包不能抵扣该请求。修改语音代码后需重启语音服务。 |

Unity 事件只监听 `127.0.0.1:39545`，不需要开放该端口的外网/局域网防火墙。修改 Unity 接收端口时必须同步修改 C# 常量和 `unity_port`；普通使用保持默认。

## 验证和当前边界

在本目录运行 `.\.venv\Scripts\python.exe -B -m unittest test_dialogue test_stream_receiver test_unity_audio -v`。测试使用假的云返回、真实本机 TCP/UDP、真实 Windows 静音 WAV 播放，不消耗云额度。Unity 的 `DigitalMan.Editor.DialogueSpeechChecks.RunBatch` 使用真实模型验证五嘴型、MFCC 运算、WAV 安全解析、准备阶段不播音和动作控制权；`DigitalMan.Editor.DialoguePlaybackChecks.RunBatch` 配合 `verify_unity_playback.py` 在真实 Play 下验证静音音频交接、完成/取消与不重复播音。

本次音色/算法更新未向云端发送合成或校准请求，爱小芊实际听感和中文嘴型仍需现场验收。修改前备份：`D:\Qiansai\FACE_60x60\DialogueLipSyncBaseline-20261020-211537`。板端无需更新或重新上传；Unity 需重新进入 Play，Python 语音项需停止后重启。

用户已确认真实云端链路跑通。本次回合锁修改通过离线模拟杂音及真实本机 TCP 回归，不调用付费云 API；尚需用更新后的进程做现场验收。没有实测所有现场音箱距离的回声或端到端响应时间。已有录音、ASR、板端和启动器保持原有接口。最初对话接入备份在 `D:\Qiansai\FACE_60x60\DialogueBaseline20261020`；回合锁修改前备份在同目录下的 `DialogueTurnLockBaseline-20261020-195932`。
