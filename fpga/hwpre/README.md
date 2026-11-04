# FPGA HWPRE 源码

本目录取自 `zg_one_display_25122301_detpost/PLIN.xpr/ps_in/ps_in.srcs/` 中的 `sources_1/rtl/yolo_hw_preprocess` 和 `sim_1/new`。`rtl/` 包含摄像头帧的双线性缩放、letterbox、NHWC 打包、AXI 寄存器接口及 MIG 写入/仲裁逻辑；`sim/` 包含完整链路和背压/连续帧压力测试。

测试默认几何为 1920×1080 输入、960×512 输出（中间内容 910×512，左右各填充 25 像素）。这与仓库内当前三模型板端配置的 640×352 输入不同，不能直接替换板端模型输入。

这些模块使用 Vivado XPM FIFO/存储器原语。运行仿真需要 Vivado 自带的 XPM 源码；两个 testbench 文件开头列出了 `xvlog`、`xelab`、`xsim` 命令。请从 `sim/` 目录运行，并将示例中的 XPM 路径改为本机 Vivado 安装目录。

原 Vivado 工程的顶层标有厂商保密声明，因此没有复制顶层、约束、厂商 IP、生成文件或 bitstream。本目录可以单独审阅和仿真 HWPRE 链路；综合完整板卡工程仍需本地原工程及其授权 IP。
