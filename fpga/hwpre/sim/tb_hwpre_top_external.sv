`timescale 1ns / 1ps
// tb_hwpre_top_external — full-pipeline functional testbench for the HWPRE chain.
//
// Geometry under test (DUT defaults, 960x512 letterbox):
//   1920x1080 -> resize 910x512 -> pad 25 cols of gray(114) each side -> 960x512
//   frame bytes = 960*512*3 = 1,474,560 = 23,040 beats x 64B (exact, no partial)
//
// Feeds two back-to-back 1920x1080 frames (valid every cycle), checks per frame:
//   A. Structural
//      1. beat count per frame == 23,040  (960*512*3/64)
//      2. first beat address == base_byte_addr
//      3. last  beat address == base + 23,039*64
//      4. app_cmd == 0 (write) on every beat
//      5. hwpre_done pulses once per frame, busy drops to 0 after
//      6. re-arm and run a 2nd frame (FSM returns to IDLE correctly)
//   B. Content — EVERY ONE of the 491,520 output pixels is compared against a
//      bit-exact integer reference model of the RTL datapath:
//        fx = ox*SCALE_X + OFFSET_X ;  x0 = fx>>14 ; wx = fx&16383
//        fy = oy*SCALE_Y + OFFSET_Y ;  y0 = fy>>14 ; wy = fy&16383
//        h  = (wxc*p(x0,y0) + wx*p(x1,y0) + 8192) >> 14
//        o  = (wyc*h_top   + wy*h_bot   + 8192) >> 14
//      Left/right 25 columns must be the pad value 0x727272.
//   C. resize line-buffer overflow flag (dut.u_core.u_resize.overflow) never fires
//
// Source pattern: R = x&255, G = y&255, B = (x&255)^(y&255)  — a pure function of
// (x,y), so the reference model needs no frame storage, only the index.
//
// Timing discipline: ALL stimulus is driven on clk negedge, so every input is
// stable 2.5 ns before each posedge the DUT samples. Driving at the posedge
// races the DUT (its FSM reads the img_start port directly while the
// state_entering_capture wire can still hold a stale 0 in the same timestep,
// which skipped the pix reset and collapsed frame 2 into a spurious FLUSH).
//
// Usage (Vivado 2018.3):
//   xvlog --sv ../rtl/*.v \
//         C:/Xilinx/Vivado/2018.3/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv \
//         C:/Xilinx/Vivado/2018.3/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv \
//         C:/Xilinx/Vivado/2018.3/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv \
//         tb_hwpre_top_external.sv
//   xelab tb_hwpre_top_external -s tb_snap
//   xsim tb_snap -R

module tb_hwpre_top_external;

    // DUT geometry, keep in sync with hwpre_top_external defaults
    localparam integer TB_OUT_W    = 960;
    localparam integer TB_OUT_H    = 512;
    localparam integer TB_RES_W    = 910;    // resize output width (content)
    localparam integer TB_PAD_LEFT = 25;
    localparam integer TB_PAD_RIGH  = 25;
    localparam integer TB_BEATS    = TB_OUT_W * TB_OUT_H * 3 / 64;   // 23,040
    localparam [23:0]  TB_PAD_RGB  = 24'h727272;                     // 8'd114 x3

    // Reference model geometry (identical to resize_bilinear_stream_core params)
    localparam integer FB = 14;
    localparam integer IN_W_C = 1920;
    localparam integer IN_H_C = 1080;
    localparam integer SCALE_X = (IN_W_C << FB) / TB_RES_W;          // 34568
    localparam integer SCALE_Y = (IN_H_C << FB) / TB_OUT_H;          // 34560
    localparam integer OFFSET_X = (SCALE_X >> 1) - 8192;             // 9092
    localparam integer OFFSET_Y = (SCALE_Y >> 1) - 8192;             // 9088

    //--------------------------------------------------------------------------
    // Clocks / reset (ddr_clk @ 200 MHz)
    //--------------------------------------------------------------------------
    reg clk = 1'b0;
    always #2.5 clk = ~clk;

    reg rst_n = 1'b0;

    //--------------------------------------------------------------------------
    // Video stimulus (frame_trans side)
    //--------------------------------------------------------------------------
    reg          img_start = 1'b0;
    reg          img_valid = 1'b0;
    reg  [23:0]  img_data  = 24'h0;

    //--------------------------------------------------------------------------
    // Control
    //--------------------------------------------------------------------------
    reg          hwpre_start  = 1'b0;
    reg  [31:0]  base_byte_addr = 32'h0;

    //--------------------------------------------------------------------------
    // MIG slave side (backpressure disabled: rdy tied high)
    //--------------------------------------------------------------------------
    reg          app_rdy     = 1'b1;
    reg          app_wdf_rdy = 1'b1;

    //--------------------------------------------------------------------------
    // DUT outputs
    //--------------------------------------------------------------------------
    wire         hwpre_done;
    wire         busy;
    wire [28:0]  app_addr;
    wire [ 2:0]  app_cmd;
    wire         app_en;
    wire [511:0] app_wdf_data;
    wire         app_wdf_end;
    wire [63:0]  app_wdf_mask;
    wire         app_wdf_wren;
    wire         app_request;

    //--------------------------------------------------------------------------
    // DUT
    //--------------------------------------------------------------------------
    hwpre_top_external dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .img_start      (img_start),
        .img_valid      (img_valid),
        .img_data       (img_data),
        .hwpre_start    (hwpre_start),
        .base_byte_addr (base_byte_addr),
        .hwpre_done     (hwpre_done),
        .busy           (busy),
        .app_addr       (app_addr),
        .app_cmd        (app_cmd),
        .app_en         (app_en),
        .app_wdf_data   (app_wdf_data),
        .app_wdf_end    (app_wdf_end),
        .app_wdf_mask   (app_wdf_mask),
        .app_wdf_wren   (app_wdf_wren),
        .app_request    (app_request),
        .app_rdy        (app_rdy),
        .app_wdf_rdy    (app_wdf_rdy)
    );

    //--------------------------------------------------------------------------
    // Frame capture + monitors
    //--------------------------------------------------------------------------
    reg  [511:0]    beat_mem [0:TB_BEATS-1];   // whole 1,474,560-byte frame
    integer         beat_cnt;
    integer         ovf_cnt;
    integer         k;
    reg  [31:0]     first_addr;
    reg  [31:0]     last_addr;
    reg             cmd_err;
    reg             wren_miss;
    reg             done_cnt;
    reg             done_latched;
    reg             busy_after_done_err;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            beat_cnt       <= 0;
            ovf_cnt        <= 0;
            first_addr     <= 32'h0;
            last_addr      <= 32'h0;
            cmd_err        <= 1'b0;
            wren_miss      <= 1'b0;
            done_cnt       <= 0;
            done_latched   <= 1'b0;
            busy_after_done_err <= 1'b0;
        end else begin
            if (dut.u_core.u_resize.overflow) ovf_cnt <= ovf_cnt + 1;

            if (app_en && app_wdf_wren) begin
                if (beat_cnt < TB_BEATS) beat_mem[beat_cnt] <= app_wdf_data;
                if (beat_cnt == 0) first_addr <= {app_addr, 3'b000};
                beat_cnt  <= beat_cnt + 1;
                last_addr <= {app_addr, 3'b000};
            end
            if (app_en) begin
                if (app_cmd != 3'b000) cmd_err <= 1'b1;
                if (!app_wdf_wren)     wren_miss <= 1'b1;
            end
            if (hwpre_done) begin
                done_cnt     <= done_cnt + 1;
                done_latched <= 1'b1;
            end
            if (done_latched) begin
                if (busy) busy_after_done_err <= 1'b1;
                done_latched <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Reference model
    //--------------------------------------------------------------------------
    // source pattern (must match the stimulus generator below)
    function integer src_r(input integer x, input integer y); src_r = x % 256; endfunction
    function integer src_g(input integer x, input integer y); src_g = y % 256; endfunction
    function integer src_b(input integer x, input integer y); src_b = (x % 256) ^ (y % 256); endfunction

    // 512-bit beat byte extraction: byte `off` of the captured frame
    function [7:0] get_byte(input integer off);
        integer bb, ww;
        begin
            bb = off / 64;
            ww = off % 64;
            get_byte = (beat_mem[bb] >> (ww*8)) & 64'hFF;
        end
    endfunction

    // bit-exact integer model of the resize datapath for scaled pixel (ox,oy)
    function [23:0] model_pix(input integer ox, input integer oy);
        integer fx, fy, x0, x1, y0, y1, wx, wy, wxc, wyc;
        integer htr, htg, htb, hbr, hbg, hbb;
        integer orr, org, orb;
        begin
            fx  = ox*SCALE_X + OFFSET_X;
            fy  = oy*SCALE_Y + OFFSET_Y;
            x0  = fx / 16384;
            wx  = fx % 16384;
            y0  = fy / 16384;
            wy  = fy % 16384;
            if (x0 > (IN_W_C-1)) x0 = IN_W_C-1;
            x1 = x0 + 1; if (x1 > (IN_W_C-1)) x1 = IN_W_C-1;
            if (y0 > (IN_H_C-1)) y0 = IN_H_C-1;
            y1 = y0 + 1; if (y1 > (IN_H_C-1)) y1 = IN_H_C-1;
            wxc = 16384 - wx;
            wyc = 16384 - wy;
            htr = (wxc*src_r(x0,y0) + wx*src_r(x1,y0) + 8192) / 16384;
            htg = (wxc*src_g(x0,y0) + wx*src_g(x1,y0) + 8192) / 16384;
            htb = (wxc*src_b(x0,y0) + wx*src_b(x1,y0) + 8192) / 16384;
            hbr = (wxc*src_r(x0,y1) + wx*src_r(x1,y1) + 8192) / 16384;
            hbg = (wxc*src_g(x0,y1) + wx*src_g(x1,y1) + 8192) / 16384;
            hbb = (wxc*src_b(x0,y1) + wx*src_b(x1,y1) + 8192) / 16384;
            orr = (wyc*htr + wy*hbr + 8192) / 16384;
            org = (wyc*htg + wy*hbg + 8192) / 16384;
            orb = (wyc*htb + wy*hbb + 8192) / 16384;
            model_pix = {orr[7:0], org[7:0], orb[7:0]};
        end
    endfunction

    //--------------------------------------------------------------------------
    // Full-frame content check
    //--------------------------------------------------------------------------
    integer errors;
    integer ox, oy, px, off, mism;
    reg [23:0] got, exp;

    task check_frame_content;
        begin
            mism = 0;
            for (oy = 0; oy < TB_OUT_H; oy = oy + 1) begin
                for (px = 0; px < TB_OUT_W; px = px + 1) begin
                    off = ((oy*TB_OUT_W) + px) * 3;
                    got = {get_byte(off), get_byte(off+1), get_byte(off+2)};   // byte0=R, 1=G, 2=B
                    if (px < TB_PAD_LEFT || px >= (TB_OUT_W - TB_PAD_RIGH))
                        exp = TB_PAD_RGB;
                    else
                        exp = model_pix(px - TB_PAD_LEFT, oy);
                    if (got !== exp) begin
                        mism = mism + 1;
                        if (mism <= 10)
                            $display("PIXMISMATCH oy=%0d px=%0d got=%06x exp=%06x",
                                     oy, px, got, exp);
                    end
                end
            end
            if (mism == 0)
                $display("OK: all %0d pixels match the reference model",
                         TB_OUT_W*TB_OUT_H);
            else begin
                $display("ERROR: %0d / %0d pixels mismatch", mism, TB_OUT_W*TB_OUT_H);
                errors = errors + 1;
            end
            // human-readable spot samples
            $display("  sample oy=0   px=0   -> %06x  (expect pad 727272)",
                     {get_byte(0),get_byte(1),get_byte(2)});
            off = ((0*TB_OUT_W) + 25) * 3;
            $display("  sample oy=0   px=25  -> %06x  model %06x",
                     {get_byte(off),get_byte(off+1),get_byte(off+2)}, model_pix(0,0));
            off = ((511*TB_OUT_W) + 934) * 3;
            $display("  sample oy=511 px=934 -> %06x  model %06x",
                     {get_byte(off),get_byte(off+1),get_byte(off+2)}, model_pix(909,511));
            off = ((256*TB_OUT_W) + 500) * 3;
            $display("  sample oy=256 px=500 -> %06x  model %06x",
                     {get_byte(off),get_byte(off+1),get_byte(off+2)}, model_pix(475,256));
        end
    endtask

    //--------------------------------------------------------------------------
    // Stimulus: run one full frame
    //--------------------------------------------------------------------------
    integer i, j;

    task run_one_frame;
        input [31:0] base_addr;
        input integer expect_beats;
        begin
            @(negedge clk);
            hwpre_start = 1'b1;
            @(negedge clk);
            hwpre_start = 1'b0;

            // wait for FSM to arm, then present the frame
            repeat (3) @(negedge clk);
            img_start = 1'b1;
            img_valid = 1'b1;
            for (j = 0; j < 1080; j = j + 1) begin
                for (i = 0; i < 1920; i = i + 1) begin
                    img_data = {i[7:0], j[7:0], i[7:0] ^ j[7:0]};
                    @(negedge clk);
                end
            end
            img_start = 1'b0;
            img_valid = 1'b0;

            // wait for completion
            wait (hwpre_done == 1'b1);
            repeat (5) @(posedge clk);

            // ---- structural checks ----
            if (beat_cnt != expect_beats) begin
                $display("ERROR: frame @0x%08x beat_cnt=%0d, expect %0d",
                         base_addr, beat_cnt, expect_beats);
                errors = errors + 1;
            end else begin
                $display("OK: frame @0x%08x beat_cnt=%0d", base_addr, beat_cnt);
            end
            if (first_addr != base_addr) begin
                $display("ERROR: first_addr=0x%08x, expect 0x%08x", first_addr, base_addr);
                errors = errors + 1;
            end
            if (last_addr != base_addr + expect_beats*64 - 64) begin
                $display("ERROR: last_addr=0x%08x, expect 0x%08x",
                         last_addr, base_addr + expect_beats*64 - 64);
                errors = errors + 1;
            end
            if (cmd_err) begin
                $display("ERROR: app_cmd != 0 seen"); errors = errors + 1; end
            if (wren_miss) begin
                $display("ERROR: app_en without app_wdf_wren"); errors = errors + 1; end
            if (done_cnt != 1) begin
                $display("ERROR: done_cnt=%0d, expect 1", done_cnt); errors = errors + 1; end
            if (busy_after_done_err) begin
                $display("ERROR: busy still high on done"); errors = errors + 1; end
            if (ovf_cnt != 0) begin
                $display("ERROR: resize line-buffer overflow fired %0d times", ovf_cnt);
                errors = errors + 1;
            end else begin
                $display("OK: resize line-buffer overflow never fired (4 rows sufficient)");
            end

            // ---- content check: every one of the 491,520 pixels ----
            check_frame_content;

            // reset per-frame monitors (NBA to stay out of the monitor's way)
            beat_cnt  <= 0;
            done_cnt  <= 0;
            cmd_err   <= 1'b0;
            wren_miss <= 1'b0;
            ovf_cnt   <= 0;
        end
    endtask

    initial begin
        errors = 0;
        $display("STIM: begin t=%0t  (geometry %0dx%0d, PAD_LEFT=%0d, %0d beats/frame)",
                 $time, TB_OUT_W, TB_OUT_H, TB_PAD_LEFT, TB_BEATS);
        $display("STIM: SCALE_X=%0d SCALE_Y=%0d OFFSET_X=%0d OFFSET_Y=%0d",
                 SCALE_X, SCALE_Y, OFFSET_X, OFFSET_Y);

        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);
        $display("STIM: rst released, arming frame 1 t=%0t", $time);

        // ---- frame 1 ----
        base_byte_addr = 32'h1000_0000;
        run_one_frame(base_byte_addr, TB_BEATS);

        // ---- frame 2: re-arm after IDLE ----
        base_byte_addr = 32'h2000_0000;
        run_one_frame(base_byte_addr, TB_BEATS);

        if (errors == 0) $display("=== TB PASS: both frames OK ===");
        else             $display("=== TB FAIL: %0d errors ===", errors);
        $finish;
    end

    // watchdog (2 frames x 2.0736M cycles x 5 ns ~ 21 ms + flush/setup margin)
    initial begin
        #200000000; $display("=== TB TIMEOUT ==="); $finish;
    end

endmodule
