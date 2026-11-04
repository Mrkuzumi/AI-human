`timescale 1ns / 1ps
// resize_bilinear_stream_core v19 — fix v18 bilinear pixel regressions (3 bugs)
//
// v19 fixes (vs v18):
//   1. s3a_*/s3b_* 乘积寄存器 [17:0]→[31:0]: wxc*p 最大 4177920(22b) 被截断致像素偏暗
//   2. S3B 垂直 wy/wyc 接反修正: out = wyc*top(y0) + wy*bottom(y1) (原 wy/wyc 互换)
//   3. BRAM 读地址/cur_x0/cur_x1 [9:0]→[10:0]: IN_W>1023 时地址 1024..1919 被截断
// reduced(96x51) max≤1; full(960x512) golden diff PASS.
//
// Stage map (7 stages, pv[6:0]):
//   pv[0]=S0(BRAM addr)  pv[1]=S1(wait)  pv[2]=S2(BRAM cap+wxc+hprod)
//   pv[3]=S3A_add(hadd→s3a)  pv[4]=S3B(vprod→s3b)  pv[5]=S3C(vadd→s3c)  pv[6]=out
//   Metadata: pox/poy[0:5] align with pv[0:5], output at pv[6] uses pox[5]/poy[5]
//   Data: s2_p*→pv[2]  s3a_p*/q*→pv[2]  s3a_*→pv[3]  s3b_m*→pv[4]  s3c_*→pv[5]  out_rgb→pv[6]
//
// v18 fixes (vs v17):
//   A. pv[3] dead stage: S3A_add restored to if(pv[3]) (was incorrectly at pv[4])
//   B. pox/poy depth: [0:3]→[0:5], pwy: [0:3]→[0:4], full shift chain, output uses pox[5]/poy[5]
//   C. Y-coord timing: 2-stage raw_y_r→need_y_* breaks DSP→LUT→reg path (-0.486ns→expected≥0)
// Architecture constraint: out_ready must remain high during burst; pipeline does not stall

module resize_bilinear_stream_core #(
    parameter IN_W=1920, IN_H=1080,
    parameter OUT_W=910, OUT_H=512,
    parameter FRAC_BITS=14,
    parameter LINE_BUF_COUNT=4
) (
    input  wire         clk, rst_n,
    input  wire         frame_active,   // 1 = CAPTURE/FLUSH, 0 = idle → reset per-frame state
    input  wire         in_valid,
    input  wire [10:0]  in_x, in_y,
    input  wire [23:0]  in_rgb,
    output reg          out_valid,
    input  wire         out_ready,
    output reg  [9:0]   out_x,
    output reg  [8:0]   out_y,
    output reg  [23:0]  out_rgb,
    output reg          overflow
);

    localparam [31:0] SCALE_X  = (IN_W << FRAC_BITS) / OUT_W;
    localparam [31:0] SCALE_Y  = (IN_H << FRAC_BITS) / OUT_H;
    localparam [31:0] OFFSET_X = (SCALE_X >> 1) - 32'd8192;
    localparam [31:0] OFFSET_Y = (SCALE_Y >> 1) - 32'd8192;
    localparam [31:0] ROUND    = 32'd8192;
    localparam [31:0] ONE      = 32'd16384;

    // ================================================================
    // BRAM line buffer (external module, no reset, sync read)
    // ================================================================

    reg [1:0]  wr_slot;
    wire [10:0] ra_0A, ra_0B, ra_1A, ra_1B, ra_2A, ra_2B, ra_3A, ra_3B;  // v19: 11bit for IN_W>1023
    wire [23:0] rd_0A, rd_0B, rd_1A, rd_1B, rd_2A, rd_2B, rd_3A, rd_3B;

    // First pixel of a frame arrives while frame_active is still 0 (the FSM
    // enters CAPTURE on the same edge) and wr_slot has not been reset yet.
    // Force slot 0 for that cycle so pixel (0,0) lands in slot 0.
    wire [1:0] eff_wr_slot = frame_active ? wr_slot : 2'd0;

    bram_linebuf_4row #(.IN_W(IN_W)) u_bram (
        .clk        (clk),
        .wr_slot    (eff_wr_slot),
        .in_x       (in_x),
        .in_rgb     (in_rgb),
        .in_valid   (in_valid),
        .in_y       (in_y),
        .IN_H       (IN_H[10:0]),
        .ra_0A(ra_0A), .ra_0B(ra_0B), .ra_1A(ra_1A), .ra_1B(ra_1B),
        .ra_2A(ra_2A), .ra_2B(ra_2B), .ra_3A(ra_3A), .ra_3B(ra_3B),
        .rd_0A(rd_0A), .rd_0B(rd_0B), .rd_1A(rd_1A), .rd_1B(rd_1B),
        .rd_2A(rd_2A), .rd_2B(rd_2B), .rd_3A(rd_3A), .rd_3B(rd_3B)
    );

    // ================================================================
    // FSM / slot metadata
    // ================================================================
    localparam ST_IDLE=0, ST_RUN=1;
    reg        state, frame_started;
    reg        frame_active_q;
    wire       frame_start_rise = frame_active && !frame_active_q;
    reg [9:0]  next_out_y;  // v20: 10-bit for OUT_H=512 sentinel
    reg [9:0]  odx;
    reg        burst_active;
    reg [1:0]  y0_slot, y1_slot;
    reg [10:0] max_received_y, max_freed_y;
    reg [10:0] wr_row_id;
    reg [10:0] slot_row_id [0:LINE_BUF_COUNT-1];
    reg        slot_valid  [0:LINE_BUF_COUNT-1];

    // ================================================================
    // Pipeline: S0→S1→S2→S3A→S3B (5 stages)
    // ================================================================
    reg [6:0]  pv;           // S0=BRAMaddr S1=wait S2=BRAMcap+hprod S3=hadd S4=vprod S5=vadd S6=out
    reg [9:0]  pox [0:5];    // out_x: S0,S1,S2,S3A_mul,S3A_add,S3B → output at pv[6] uses pox[5]
    reg [8:0]  poy [0:5];    // out_y: S0,S1,S2,S3A_mul,S3A_add,S3B → output at pv[6] uses poy[5]
    reg [1:0]  ps0 [0:2];    // pipe y0 slot: S0,S1,S2
    reg [1:0]  ps1 [0:2];    // pipe y1 slot: S0,S1,S2
    reg [FRAC_BITS-1:0] pwx [0:2];  // pipe wx: S0,S1,S2
    reg [FRAC_BITS-1:0] pwy [0:4];  // pipe wy: S0,S1,S2,S3A_mul,S3B → S3B at pv[4] uses pwy[4]

    // BRAM read address registers — v19: [9:0]→[10:0]
    reg [10:0] ra_0A_r, ra_0B_r, ra_1A_r, ra_1B_r, ra_2A_r, ra_2B_r, ra_3A_r, ra_3B_r;
    assign ra_0A = ra_0A_r; assign ra_0B = ra_0B_r;
    assign ra_1A = ra_1A_r; assign ra_1B = ra_1B_r;
    assign ra_2A = ra_2A_r; assign ra_2B = ra_2B_r;
    assign ra_3A = ra_3A_r; assign ra_3B = ra_3B_r;

    // S2 capture registers
    reg [23:0] s2_p00, s2_p01, s2_p10, s2_p11;

    // S2: pre-compute wxc (wx complement) to break subtraction→DSP timing path
    (* DONT_TOUCH = "true" *) reg [FRAC_BITS-1:0] s2_wxc;

    // S3A: horizontal interpolation results (registered, DONT_TOUCH to prevent retiming)
    // v19: 乘积 wxc(14b)*p(8b) 最大 16384*255=4177920 需 22b, 原 [17:0] 截断致像素偏暗, 扩到 [31:0]
    (* DONT_TOUCH = "true" *) reg [31:0] s3a_ptr, s3a_ptg, s3a_ptb; // top products (wxc*p00)
    (* DONT_TOUCH = "true" *) reg [31:0] s3a_pbr, s3a_pbg, s3a_pbb; // bot products (wxc*p10)
    (* DONT_TOUCH = "true" *) reg [31:0] s3a_qtr, s3a_qtg, s3a_qtb; // top products (wx*p01)
    (* DONT_TOUCH = "true" *) reg [31:0] s3a_qbr, s3a_qbg, s3a_qbb; // bot products (wx*p11)

    // S3A_add: horizontal add+round results (DONT_TOUCH)
    (* DONT_TOUCH = "true" *) reg [7:0] s3a_tr, s3a_tg, s3a_tb;
    (* DONT_TOUCH = "true" *) reg [7:0] s3a_br, s3a_bg, s3a_bb;

    // S3B: vertical interpolation — registered DSP multiplier products (DONT_TOUCH)
    // v19: 同理 wy(14b)*s3a(8b) 需 22b, 扩到 [31:0]
    (* DONT_TOUCH = "true" *) reg [31:0] s3b_mtr, s3b_mtg, s3b_mtb; // wy * top
    (* DONT_TOUCH = "true" *) reg [31:0] s3b_mbr, s3b_mbg, s3b_mbb; // wyc * bottom

    // S3C: vertical interpolation — registered adder outputs (DONT_TOUCH)
    (* DONT_TOUCH = "true" *) reg [7:0] s3c_r, s3c_g, s3c_b;

    // ================================================================
    // Registered coordinate DSP outputs (v13: break DSP-to-DSP paths)
    // ================================================================
    reg [31:0] cur_src_x_fp_r;   // registered X coord (look-ahead: pre-loaded f(0), then f(odx+1))
    reg [10:0] raw_y_r;         // Stage-1: registered raw Y from DSP (breaks DSP→LUT path)
    reg [10:0] raw_y_next_r;    // Stage-1: registered raw Y+1 from DSP
    reg [10:0] need_y0_r;       // Stage-2: clamp from raw_y_r
    reg [10:0] need_y1_r;
    reg [10:0] need_y0_next_r;
    reg [10:0] cur_y0_r;        // Stage-2: clamp from raw_y_r
    reg [FRAC_BITS-1:0] burst_wy_r;

    // Next-X look-ahead: compute f(odx+1) for next cycle
    // v21: INCREMENTAL accumulate instead of (next_odx * SCALE_X) + OFFSET_X.
    //   The multiply fed a DSP48E1 and was the single 200 MHz setup critical path
    //   (odx_reg -> DSP -> adder -> cur_src_x_fp_r, ~5.06 ns).  Because odx advances
    //   by exactly +1 on every accepted output pixel and wraps to 0 at OUT_W-1
    //   (where f(0) = OFFSET_X), f(odx) + SCALE_X == f(odx+1) exactly, so this is
    //   bit-identical while leaving only a 32-bit adder on the path.
    //   Max value OFFSET_X + (OUT_W-1)*SCALE_X = 31,431,404 < 2^25 -> no overflow.
    wire [31:0] next_cur_src_x_fp = (odx == (OUT_W - 1)) ? OFFSET_X
                                                         : (cur_src_x_fp_r + SCALE_X);

    // Combinational Y coordinate DSP wires (registered at top of always block)
    wire [31:0] next_src_y_fp_comb  = (next_out_y * SCALE_Y) + OFFSET_Y;
    wire [31:0] next2_src_y_fp_comb = ((next_out_y + 10'd1) * SCALE_Y) + OFFSET_Y;  // v20: 10-bit operand

    // Extract from registered X coord — v19: cur_x0/x1 [9:0]→[10:0] 支持 IN_W>1023
    wire [10:0] cur_x0 = ((cur_src_x_fp_r[FRAC_BITS +: 11]) >= IN_W) ? (IN_W-1) : cur_src_x_fp_r[FRAC_BITS +: 11];
    wire [10:0] cur_x1 = ((cur_src_x_fp_r[FRAC_BITS +: 11] + 11'd1) >= IN_W) ? (IN_W-1) : (cur_src_x_fp_r[FRAC_BITS +: 11] + 11'd1);
    wire [FRAC_BITS-1:0] cur_wx = cur_src_x_fp_r[FRAC_BITS-1:0];

    // Slot matching uses registered need_y0_r / need_y1_r
    wire match_y0_0 = slot_valid[0] && (slot_row_id[0] == need_y0_r);
    wire match_y0_1 = slot_valid[1] && (slot_row_id[1] == need_y0_r);
    wire match_y0_2 = slot_valid[2] && (slot_row_id[2] == need_y0_r);
    wire match_y0_3 = slot_valid[3] && (slot_row_id[3] == need_y0_r);
    wire match_y1_0 = slot_valid[0] && (slot_row_id[0] == need_y1_r);
    wire match_y1_1 = slot_valid[1] && (slot_row_id[1] == need_y1_r);
    wire match_y1_2 = slot_valid[2] && (slot_row_id[2] == need_y1_r);
    wire match_y1_3 = slot_valid[3] && (slot_row_id[3] == need_y1_r);

    wire found_y0 = match_y0_0 || match_y0_1 || match_y0_2 || match_y0_3;
    wire found_y1 = match_y1_0 || match_y1_1 || match_y1_2 || match_y1_3;

    wire [1:0] slot_for_y0 = match_y0_0 ? 2'd0 : match_y0_1 ? 2'd1 : match_y0_2 ? 2'd2 : 2'd3;
    wire [1:0] slot_for_y1 = match_y1_0 ? 2'd0 : match_y1_1 ? 2'd1 : match_y1_2 ? 2'd2 : 2'd3;

    // min_needed_src_y uses registered cur_y0_r
    wire [10:0] min_needed_src_y = burst_active ? cur_y0_r : need_y0_r;

    // Burst start: pipeline must be fully empty (all 5 stages)
    wire burst_ready = !burst_active && state == ST_RUN && next_out_y < OUT_H
                       && found_y0 && found_y1 && !pv[0] && !pv[1] && !pv[2] && !pv[3] && !pv[4] && !pv[5] && !pv[6];

    // ================================================================
    // S3A horizontal interpolation wires
    // ================================================================
    wire [FRAC_BITS-1:0] s3a_wxc = s2_wxc;     // registered in S2 (breaks sub→DSP path)
    wire [FRAC_BITS-1:0] s3a_wx  = pwx[2];

    wire [31:0] h_top_r = (s3a_wxc * s2_p00[23:16]) + (s3a_wx * s2_p01[23:16]) + ROUND;
    wire [31:0] h_top_g = (s3a_wxc * s2_p00[15:8])  + (s3a_wx * s2_p01[15:8])  + ROUND;
    wire [31:0] h_top_b = (s3a_wxc * s2_p00[7:0])   + (s3a_wx * s2_p01[7:0])   + ROUND;
    wire [31:0] h_bot_r = (s3a_wxc * s2_p10[23:16]) + (s3a_wx * s2_p11[23:16]) + ROUND;
    wire [31:0] h_bot_g = (s3a_wxc * s2_p10[15:8])  + (s3a_wx * s2_p11[15:8])  + ROUND;
    wire [31:0] h_bot_b = (s3a_wxc * s2_p10[7:0])   + (s3a_wx * s2_p11[7:0])   + ROUND;

    // ================================================================
    // S3B vertical interpolation wires
    // ================================================================
    wire [FRAC_BITS-1:0] s3b_wyc = ONE[FRAC_BITS-1:0] - pwy[4];
    wire [FRAC_BITS-1:0] s3b_wy  = pwy[4];

    wire [31:0] v_out_r = (s3b_wyc * s3a_tr) + (s3b_wy * s3a_br) + ROUND;
    wire [31:0] v_out_g = (s3b_wyc * s3a_tg) + (s3b_wy * s3a_bg) + ROUND;
    wire [31:0] v_out_b = (s3b_wyc * s3a_tb) + (s3b_wy * s3a_bb) + ROUND;

    // ================================================================
    // Main always block
    // ================================================================
    integer xi;

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= ST_IDLE; overflow <= 0; frame_started <= 0;
            frame_active_q <= 0;
            wr_slot <= 0; wr_row_id <= 0;
            max_received_y <= 0; max_freed_y <= 0;
            next_out_y <= 0; burst_active <= 0; odx <= 0;
            y0_slot <= 0; y1_slot <= 0;
            out_valid <= 0; out_x <= 0; out_y <= 0; out_rgb <= 0;
            pv <= 7'b0;
            cur_src_x_fp_r <= OFFSET_X;  // f(0) = OFFSET_X (when odx=0, 0*SCALE_X + OFFSET_X)
            raw_y_r <= 0; raw_y_next_r <= 0;
            need_y0_r <= 0; need_y1_r <= 0; need_y0_next_r <= 0;
            cur_y0_r <= 0; burst_wy_r <= 0;
            for (xi = 0; xi < LINE_BUF_COUNT; xi = xi + 1) begin
                slot_valid[xi] <= 0; slot_row_id[xi] <= 0;
            end
        end else begin
            frame_active_q <= frame_active;
            if (!frame_active) begin
                // Between frames: full per-frame reset. This branch also runs
                // on the cycle the FSM enters CAPTURE (frame_active still 0
                // there), so slot 0 / row counters are clean for the first
                // pixel. 2026-09-22: re-arm (state<=ST_RUN) 恢复为首像素触发
                // (!frame_started) — 尾行伪再武装由"输出完成后 frame_started 保持 1"
                // 防住 (见 INPUT CAPTURE 段), 不依赖 frame_start_rise。
                state <= ST_IDLE; frame_started <= 0; overflow <= 0;
                wr_slot <= 0; wr_row_id <= 0;
                max_received_y <= 0; max_freed_y <= 0;
                next_out_y <= 0; burst_active <= 0; odx <= 0;
                y0_slot <= 0; y1_slot <= 0;
                out_valid <= 0;
                pv <= 7'b0;
                cur_src_x_fp_r <= OFFSET_X;
                for (xi = 0; xi < LINE_BUF_COUNT; xi = xi + 1) begin
                    slot_valid[xi] <= 0; slot_row_id[xi] <= 0;
                end
            end
            // ============================================================
            // Y coordinate: 2-stage pipeline (breaks DSP→LUT timing path)
            // Stage 1: register raw DSP outputs (DSP→register, ~3ns)
            // Stage 2: compare+clamp from registered raw values (LUT→register, ~2ns)
            // Total latency +1 cycle vs v17; safe because Y changes only at line boundaries
            // ============================================================
            raw_y_r      <= next_src_y_fp_comb >> FRAC_BITS;
            raw_y_next_r <= next2_src_y_fp_comb >> FRAC_BITS;
            burst_wy_r   <= next_src_y_fp_comb[FRAC_BITS-1:0];

            cur_y0_r      <= (raw_y_r >= IN_H) ? (IN_H - 1) : raw_y_r;
            need_y0_r     <= (raw_y_r >= IN_H) ? (IN_H - 1) : raw_y_r;
            need_y1_r     <= ((raw_y_r + 1) >= IN_H) ? (IN_H - 1) : (raw_y_r + 1);
            need_y0_next_r <= (raw_y_next_r >= IN_H) ? (IN_H - 1) : raw_y_next_r;

            // X coordinate: look-ahead (pre-load f(0) at burst start, then f(odx+1))
            if (burst_ready) begin
                cur_src_x_fp_r <= OFFSET_X;  // f(0): pre-load for odx=0
            end else if (burst_active && out_ready) begin
                cur_src_x_fp_r <= next_cur_src_x_fp;  // f(next_odx) for next cycle
            end

            // ============================================================
            // S3B: Vertical interpolation — register DSP multiplier products
            // ============================================================
            if (pv[4]) begin  // S3B_mul  (v19: 修正 wy/wyc 接反, out = wyc*top(y0) + wy*bottom(y1))
                s3b_mtr <= s3b_wyc * s3a_tr; s3b_mtg <= s3b_wyc * s3a_tg; s3b_mtb <= s3b_wyc * s3a_tb;
                s3b_mbr <= s3b_wy * s3a_br;  s3b_mbg <= s3b_wy * s3a_bg;  s3b_mbb <= s3b_wy * s3a_bb;
            end

            // ============================================================
            // S3C: Vertical interpolation — add + round → register → output
            // ============================================================
            if (pv[5]) begin  // S3C_add
                s3c_r <= (s3b_mtr + s3b_mbr + ROUND) >> FRAC_BITS;
                s3c_g <= (s3b_mtg + s3b_mbg + ROUND) >> FRAC_BITS;
                s3c_b <= (s3b_mtb + s3b_mbb + ROUND) >> FRAC_BITS;
            end
            if (pv[6] && out_ready) begin
                out_valid <= 1;
                out_x     <= pox[5];  // 6-cycle latency from S0 → aligned with pv[6]
                out_y     <= poy[5];
                out_rgb   <= {s3c_r, s3c_g, s3c_b};
            end else begin
                out_valid <= 0;
            end

            // ============================================================
            // S3A: Horizontal interpolation (from S2 data)
            // ============================================================
            // S3A_mul: register horizontal DSP products
            if (pv[2]) begin
                s3a_ptr <= s3a_wxc * s2_p00[23:16]; s3a_qtr <= s3a_wx * s2_p01[23:16];
                s3a_ptg <= s3a_wxc * s2_p00[15:8];  s3a_qtg <= s3a_wx * s2_p01[15:8];
                s3a_ptb <= s3a_wxc * s2_p00[7:0];   s3a_qtb <= s3a_wx * s2_p01[7:0];
                s3a_pbr <= s3a_wxc * s2_p10[23:16]; s3a_qbr <= s3a_wx * s2_p11[23:16];
                s3a_pbg <= s3a_wxc * s2_p10[15:8];  s3a_qbg <= s3a_wx * s2_p11[15:8];
                s3a_pbb <= s3a_wxc * s2_p10[7:0];   s3a_qbb <= s3a_wx * s2_p11[7:0];
            end

            // S3A_add: add+round → s3a registers (pv[3]=S3A_add)
            if (pv[3]) begin
                s3a_tr <= (s3a_ptr + s3a_qtr + ROUND) >> FRAC_BITS;
                s3a_tg <= (s3a_ptg + s3a_qtg + ROUND) >> FRAC_BITS;
                s3a_tb <= (s3a_ptb + s3a_qtb + ROUND) >> FRAC_BITS;
                s3a_br <= (s3a_pbr + s3a_qbr + ROUND) >> FRAC_BITS;
                s3a_bg <= (s3a_pbg + s3a_qbg + ROUND) >> FRAC_BITS;
                s3a_bb <= (s3a_pbb + s3a_qbb + ROUND) >> FRAC_BITS;
            end

            // Pipeline shift: S3A_add→S3B→S3C→output
            pv[6]   <= pv[5];   // S3C→output
            pv[5]   <= pv[4];   // S3B→S3C
            pv[4]   <= pv[3];   // S3A_add→S3B
            pox[5]  <= pox[4];  // metadata aligned with pv stages
            pox[4]  <= pox[3];
            pox[3]  <= pox[2];
            poy[5]  <= poy[4];
            poy[4]  <= poy[3];
            poy[3]  <= poy[2];
            pwy[4]  <= pwy[3];
            pwy[3]  <= pwy[2];

            // S2→S3A
            pv[3]   <= pv[2];

            // ============================================================
            // S2: Capture BRAM read data
            // ============================================================
            pv[2]   <= pv[1];
            pox[2]  <= pox[1];
            poy[2]  <= poy[1];
            pwx[2]  <= pwx[1];
            pwy[2]  <= pwy[1];
            if (pv[1]) begin
                s2_wxc <= ONE[FRAC_BITS-1:0] - pwx[1];  // pre-compute wx complement for S3A
                case (ps0[1])
                    0: begin s2_p00 <= rd_0A; s2_p01 <= rd_0B; end
                    1: begin s2_p00 <= rd_1A; s2_p01 <= rd_1B; end
                    2: begin s2_p00 <= rd_2A; s2_p01 <= rd_2B; end
                    3: begin s2_p00 <= rd_3A; s2_p01 <= rd_3B; end
                endcase
                case (ps1[1])
                    0: begin s2_p10 <= rd_0A; s2_p11 <= rd_0B; end
                    1: begin s2_p10 <= rd_1A; s2_p11 <= rd_1B; end
                    2: begin s2_p10 <= rd_2A; s2_p11 <= rd_2B; end
                    3: begin s2_p10 <= rd_3A; s2_p11 <= rd_3B; end
                endcase
            end

            // ============================================================
            // S1: Metadata passthrough (BRAM latency)
            // ============================================================
            pv[1]   <= pv[0];
            pox[1]  <= pox[0];
            poy[1]  <= poy[0];
            ps0[1]  <= ps0[0];
            ps1[1]  <= ps1[0];
            pwx[1]  <= pwx[0];
            pwy[1]  <= pwy[0];

            // ============================================================
            // S0: Issue BRAM read addresses + capture metadata
            //     Uses registered cur_src_x_fp_r, burst_wy_r
            // ============================================================
            if (burst_active && out_ready) begin
                if (y0_slot == 2'd0 || y1_slot == 2'd0) begin ra_0A_r <= cur_x0; ra_0B_r <= cur_x1; end
                if (y0_slot == 2'd1 || y1_slot == 2'd1) begin ra_1A_r <= cur_x0; ra_1B_r <= cur_x1; end
                if (y0_slot == 2'd2 || y1_slot == 2'd2) begin ra_2A_r <= cur_x0; ra_2B_r <= cur_x1; end
                if (y0_slot == 2'd3 || y1_slot == 2'd3) begin ra_3A_r <= cur_x0; ra_3B_r <= cur_x1; end

                pv[0]    <= 1;
                pox[0]   <= odx;
                poy[0]   <= next_out_y[8:0];  // v20: explicit 9-bit truncation from 10-bit counter
                ps0[0]   <= y0_slot;
                ps1[0]   <= y1_slot;
                pwx[0]   <= cur_wx;
                pwy[0]   <= burst_wy_r;

                if (odx == (OUT_W - 1)) begin
                    burst_active <= 0;
                    odx <= 0;
                    next_out_y <= next_out_y + 1;
                    if (need_y0_next_r > 0) max_freed_y <= need_y0_next_r - 1;
                    else max_freed_y <= 0;
                    for (xi = 0; xi < LINE_BUF_COUNT; xi = xi + 1) begin
                        if (slot_valid[xi] && (slot_row_id[xi] < need_y0_next_r))
                            slot_valid[xi] <= 0;
                    end
                end else begin
                    odx <= odx + 1;
                end
            end else begin
                pv[0] <= 0;
            end

            // ============================================================
            // INPUT CAPTURE
            // ============================================================
            if (in_valid && in_x < IN_W && in_y < IN_H) begin
                // 2026-09-22 BUGFIX: 原条件 frame_start_rise 只在 CAPTURE 进入后第 1 拍为 1,
                // 但 img_start 是 vsync 脉冲 (axi_master.v 的 pulse_cross_camera), 第一颗像素要等
                // 垂直消隐结束 (约 8800+ cam_clk 周期) 才从 afifo 出来 -> 硬件上永远错过,
                // state 恒 ST_IDLE -> burst_ready(需 ST_RUN) 永不成立 -> 零输出 -> packer
                // started=0/beat_emitted=0 (gap 定时器自锁) -> 零 beat -> mig_done 永不来 ->
                // hwpre FSM 永久卡 FLUSH (busy=1/done=0)。
                // 修复: 恢复 old project 的首像素再武装语义 (!frame_started)。
                // 帧间伪再武装不会发生: 输出完成后 frame_started 保持 1 (下面只清 state),
                // FLUSH 期间 core_in_valid 被顶层门控为 0, 下一帧进入时 !frame_active 分支
                // 才清 frame_started, 与首像素之间隔整个消隐期。
                if (!frame_started) begin
                    frame_started <= 1; state <= ST_RUN;
                end
                if (in_x == 0 && slot_valid[wr_slot] && (slot_row_id[wr_slot] >= min_needed_src_y))
                    overflow <= 1;
                if (in_x == (IN_W - 1)) begin
                    slot_valid[wr_slot] <= 1;
                    slot_row_id[wr_slot] <= wr_row_id;
                    max_received_y <= wr_row_id;
                    wr_slot <= (wr_slot + 1) % LINE_BUF_COUNT;
                    wr_row_id <= wr_row_id + 1;
                end
            end

            // ============================================================
            // SCHEDULE: start burst when source rows ready + pipeline empty
            // ============================================================
            if (burst_ready) begin
                y0_slot <= slot_for_y0; y1_slot <= slot_for_y1;
                burst_active <= 1; odx <= 0;
            end

            // Output rows complete (input tail may still be streaming; state 回 IDLE 但
            // frame_started 保持 1, 防止 CAPTURE 尾行伪再武装; 下一帧进入时 !frame_active
            // 分支清 frame_started, 首像素再武装)
            if (state == ST_RUN && !burst_active && next_out_y >= OUT_H
                && !pv[0] && !pv[1] && !pv[2] && !pv[3] && !pv[4] && !pv[5] && !pv[6]) begin
                state <= ST_IDLE;
            end
        end
    end

endmodule
