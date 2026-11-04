`timescale 1ns / 1ps
// hwpre_top_external 鈥? thin wrapper: x/y counter + hwpre_core_stream + hwpre_mig_master.
// Control FSM: IDLE 鈫? ARMED 鈫? CAPTURE 鈫? FLUSH 鈫? IDLE.
// All logic runs on a single clock (ddr_clk). No CDC inside this module.
//
// Stage 5A: standalone verification. Stage 5 will instance into ai7030_top.

module hwpre_top_external #(
    // 960x512 letterbox: 1920x1080 scaled by min(960/1920,512/1080)=0.4741 -> 910x512,
    // then 25 gray (PAD_VALUE=114) columns padded on each side -> 960x512.
    // One frame = 960*512*3 = 1,474,560 bytes = 23,040 beats x 64B.
    // 1920x1080 -> 910x512 -> 左右各补25列灰边(114) -> 960x512 (YOLO边长须为32的整数倍: 960=30*32, 512=16*32)
    parameter IN_W=1920, IN_H=1080,
    parameter RESIZE_W=910, RESIZE_H=512,
    parameter OUT_W=960, OUT_H=512,
    parameter PAD_LEFT=25, PAD_RIGHT=25, PAD_TOP=0, PAD_BOTTOM=0,
    parameter [7:0] PAD_VALUE = 8'd114,
    parameter LINE_BUF_COUNT=4
) (
    input  wire         clk,
    input  wire         rst_n,

    // Video input (from frame_trans, same clock domain)
    input  wire         img_start,
    input  wire         img_valid,
    input  wire [23:0]  img_data,

    // Control
    input  wire         hwpre_start,       // 0鈫?1 triggers frame capture
    input  wire [31:0]  base_byte_addr,    // PLDDR target byte address
    output wire         hwpre_done,        // 1-cycle pulse on frame complete
    output wire         busy,              // 1 = armed/capturing/flushing

    // MIG app_* write interface (to arbiter M1)
    output wire [28:0]  app_addr,
    output wire [2:0]   app_cmd,
    output wire         app_en,
    output wire [511:0] app_wdf_data,
    output wire         app_wdf_end,
    output wire [63:0]  app_wdf_mask,
    output wire         app_wdf_wren,
    output wire         app_request,
    input  wire         app_rdy,
    input  wire         app_wdf_rdy
);

    //==========================================================================
    // FSM states
    //==========================================================================
    localparam ST_IDLE    = 2'd0;  // waiting for hwpre_start
    localparam ST_ARMED   = 2'd1;  // hwpre_start received, waiting for next img_start
    localparam ST_CAPTURE = 2'd2;  // feeding pixels to core
    localparam ST_FLUSH   = 2'd3;  // last pixel sent, waiting for MIG done

    reg [1:0] state;

    // Forward declaration 鈥? wires from hwpre_mig_master / hwpre_core_stream
    wire mig_done;
    wire mig_busy;
    wire        beat_valid;
    wire [511:0] beat_data;
    wire        beat_last;
    wire        beat_ready;

    // Raw request remains visible when the arbiter deasserts ready.
    assign app_request = beat_valid;

    //==========================================================================
    // hwpre_start edge detection
    //==========================================================================
    reg hwpre_start_q;
    wire hwpre_start_posedge;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) hwpre_start_q <= 1'b0;
        else        hwpre_start_q <= hwpre_start;
    end
    assign hwpre_start_posedge = hwpre_start && !hwpre_start_q;

    //--------------------------------------------------------------------------
    // saw_start: latched "frame boundary seen" flag.
    // CAPTURE must NOT be entered on img_start (vsync) alone. The pixel feed comes
    // from axis_master, which forwards the camera vsync edge through
    // pulse_cross_camera while its async FIFO only ever stores active pixels
    // (wr_en = dvp_href), so img_valid stays LOW for the whole vertical blanking
    // interval (~133k ddr_clk cycles at 1080p60).
    // Entering at vsync lets the letterbox push PAD_LEFT pad pixels and then idle
    // for that entire blanking interval. When PAD_LEFT*BYTES_PER_PIX >= BEAT_BYTES
    // (25*3 = 75 >= 64) the pad alone flushes a first beat, which arms the packer
    // gap-timer; the blanking gap then trips it, the packer emits a spurious
    // beat_last and the frame is closed after ~2 beats instead of 23040.
    // Requiring img_valid makes the pad pixels come out back-to-back with real
    // data, so no idle interval exists and the gap-timer never fires.
    //--------------------------------------------------------------------------
    reg  saw_start;
    wire frame_start_seen;
    assign frame_start_seen = img_start || saw_start;

    // state_entering_capture: true when FSM enters CAPTURE this cycle.
    // First-pixel semantics: frame boundary already seen AND a real pixel is on the
    // bus, so the pixel-counter preload below takes its img_valid=1 (pix_x<=1) path.
    wire state_entering_capture;
    assign state_entering_capture = (state == ST_ARMED) && frame_start_seen && img_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) saw_start <= 1'b0;
        else if (state_entering_capture) saw_start <= 1'b0;
        else if (state == ST_ARMED && img_start) saw_start <= 1'b1;
    end

    //==========================================================================
    // Pixel counter (x/y)
    //==========================================================================
    reg [10:0] pix_x;
    reg [10:0] pix_y;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pix_x <= 11'd0;
            pix_y <= 11'd0;
        end else if (state_entering_capture) begin
            // new frame starts 鈥? reset counter
            // Mode A: img_valid=1 (first pixel captured), pre-set pix_x=1 for next pixel
            // Mode B: img_valid=0, first pixel not yet arrived, pix_x stays 0
            pix_x <= img_valid ? 11'd1 : 11'd0;
            pix_y <= 11'd0;
        end else if (state == ST_CAPTURE && img_valid) begin
            if (pix_x == (IN_W - 1)) begin
                pix_x <= 11'd0;
                pix_y <= pix_y + 11'd1;
            end else begin
                pix_x <= pix_x + 11'd1;
            end
        end
    end

    // Last-pixel detection
    wire last_pixel;
    assign last_pixel = (state == ST_CAPTURE) && img_valid &&
                        (pix_x == (IN_W - 1)) && (pix_y == (IN_H - 1));

    // mig_done latch: the resize finishes its output stream BEFORE the last
    // input pixel (output row IN_H-1 only needs input row ~2.93*IN_H/OUT_H),
    // so mig_done can pulse while the FSM is still in CAPTURE. Without this
    // latch that pulse is missed, the FSM hangs in FLUSH until the packer's
    // gap-timer flush (~65k cycles later) emits a spurious beat with
    // beat_last 鈥? overwriting the first 64 bytes of the frame buffer.
    reg mig_done_latched;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mig_done_latched <= 1'b0;
        end else if (mig_done) begin
            mig_done_latched <= 1'b1;
        end else if (state_entering_capture || state == ST_FLUSH) begin
            // new frame starting, or FSM has consumed the latch in FLUSH
            mig_done_latched <= 1'b0;
        end
    end

    //==========================================================================
    // FSM 鈥? single always @(posedge clk) block (no separate next_state)
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
        end else begin
            case (state)
                ST_IDLE: begin
                    // Always arm first; ST_ARMED then waits for a frame boundary AND
                    // a real pixel (see saw_start / state_entering_capture above).
                    if (hwpre_start_posedge)
                        state <= ST_ARMED;
                end
                ST_ARMED: begin
                    if (frame_start_seen && img_valid)
                        state <= ST_CAPTURE;
                end
                ST_CAPTURE: begin
                    if (last_pixel)
                        state <= ST_FLUSH;
                end
                ST_FLUSH: begin
                    if (mig_done || mig_done_latched)
                        state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end

    //==========================================================================
    // Outputs
    //==========================================================================
    assign busy       = (state != ST_IDLE);
    assign hwpre_done = (state == ST_FLUSH) && (mig_done || mig_done_latched);  // 1-cycle pulse

    //==========================================================================
    // Gated pixel feed to hwpre_core_stream
    //==========================================================================
    wire        core_in_valid;
    wire [10:0] core_in_x;
    wire [10:0] core_in_y;
    wire [23:0] core_in_rgb;

    assign core_in_valid = ((state == ST_CAPTURE) || state_entering_capture) && img_valid;
    // On entering CAPTURE, pix_x/pix_y NBA-reset hasn't taken effect yet,
    // so force (0,0) to avoid using stale previous-frame position.
    assign core_in_x     = state_entering_capture ? 11'd0 : pix_x;
    assign core_in_y     = state_entering_capture ? 11'd0 : pix_y;
    assign core_in_rgb   = img_data;

    //==========================================================================
    // hwpre_core_stream
    //==========================================================================
    hwpre_core_stream #(
        .IN_W(IN_W), .IN_H(IN_H),
        .RESIZE_W(RESIZE_W), .RESIZE_H(RESIZE_H),
        .OUT_W(OUT_W), .OUT_H(OUT_H),
        .PAD_LEFT(PAD_LEFT), .PAD_RIGHT(PAD_RIGHT),
        .PAD_TOP(PAD_TOP), .PAD_BOTTOM(PAD_BOTTOM),
        .PAD_VALUE(PAD_VALUE),
        .LINE_BUF_COUNT(LINE_BUF_COUNT)
    ) u_core (
        .clk            (clk),
        .rst_n          (rst_n),
        .frame_active   ((state == ST_CAPTURE) || (state == ST_FLUSH)),
        .in_valid       (core_in_valid),
        .in_x       (core_in_x),
        .in_y       (core_in_y),
        .in_rgb     (core_in_rgb),
        .beat_valid (beat_valid),
        .beat_data  (beat_data),
        .beat_last  (beat_last),
        .beat_ready (beat_ready)
    );

    //==========================================================================
    // hwpre_mig_master
    //==========================================================================
    hwpre_mig_master u_mig (
        .clk           (clk),
        .rst_n         (rst_n),
        .beat_valid    (beat_valid),
        .beat_data     (beat_data),
        .beat_last     (beat_last),
        .beat_ready    (beat_ready),
        .base_byte_addr(base_byte_addr),
        .app_addr      (app_addr),
        .app_cmd       (app_cmd),
        .app_en        (app_en),
        .app_wdf_data  (app_wdf_data),
        .app_wdf_end   (app_wdf_end),
        .app_wdf_mask  (app_wdf_mask),
        .app_wdf_wren  (app_wdf_wren),
        .app_rdy       (app_rdy),
        .app_wdf_rdy   (app_wdf_rdy),
        .busy          (mig_busy),
        .done          (mig_done)
    );

endmodule
