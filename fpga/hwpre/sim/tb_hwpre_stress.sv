`timescale 1ns / 1ps
// tb_hwpre_stress — adversarial testbench for the HWPRE chain (960x512).
//
// Complements tb_hwpre_top_external.sv (which only drives an ideal,
// gap-free pixel stream).  This bench adds the conditions the real board
// creates but the golden TB never exercised:
//
//   S1  ideal timing            — baseline regression (= golden TB conditions)
//   S2  realistic timing        — 45 lines of vertical blanking before active
//                                 video, 280-cycle horizontal blanking after
//                                 every line, img_start as a 1-cycle vsync
//                                 pulse.  This is the shape of a real 1080p60
//                                 SDI feed.  Verifies:
//                                   * FSM ARMED -> CAPTURE happens only when
//                                     the real first pixel arrives (~99k
//                                     cycles after vsync)
//                                   * the packer gap timer (GAP_THRESH=65535)
//                                     cannot emit a spurious beat during the
//                                     inter-frame / vertical-blank gap
//                                   * per-line img_valid gaps do not corrupt
//                                     pix_x/pix_y counting
//                                   * full 491,520-pixel content still matches
//   S3  arm while busy          — hwpre_start pulsed during CAPTURE (what a PS
//                                 timeout-retry would do) must be ignored
//   S4  base change mid-frame   — PS rewriting BASE_ADDR while capturing must
//                                 not move the write pointer
//   S5  mid-frame reset         — rst_n asserted during CAPTURE; the next frame
//                                 must run cleanly (board power/reset robustness)
//   S6  post-done silence       — 300k idle cycles after done must produce zero
//                                 further beats (stale-flush / gap-timer guard)
//
// ALWAYS-ON INVARIANTS (checked every cycle of every scenario)
//   I1  resize must never present an output pixel while its out_ready is low.
//       resize_bilinear_stream_core drops such a pixel silently
//       (line 293: if (pv[6] && out_ready) else out_valid <= 0), so any
//       occurrence corrupts the row alignment.  With SCALE_X/SCALE_Y ~2.109
//       consecutive output rows always need fresh source rows, so a burst is
//       never back-pressured; this invariant proves that holds in practice.
//   I2  no X/Z on the MIG write data bus.
//   I3  write bursts stay 64-byte aligned and strictly mono-increasing.
//
// Usage (Vivado 2018.3, from this directory):
//   xvlog --sv ../rtl/*.v \
//         C:/Xilinx/Vivado/2018.3/data/ip/xpm/xpm_fifo/hdl/xpm_fifo.sv \
//         C:/Xilinx/Vivado/2018.3/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv \
//         C:/Xilinx/Vivado/2018.3/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv \
//         tb_hwpre_stress.sv
//   xelab tb_hwpre_stress -s stress_snap
//   xsim stress_snap -R

module tb_hwpre_stress;

    // DUT geometry (must match hwpre_top_external parameter defaults)
    localparam integer TB_IN_W     = 1920;
    localparam integer TB_IN_H     = 1080;
    localparam integer TB_OUT_W    = 960;
    localparam integer TB_OUT_H    = 512;
    localparam integer TB_RES_W    = 910;
    localparam integer TB_PAD_LEFT = 25;
    localparam integer TB_BEATS    = TB_OUT_W * TB_OUT_H * 3 / 64;   // 23,040
    localparam [23:0]  TB_PAD_RGB  = 24'h727272;

    // reference model geometry
    localparam integer FB       = 14;
    localparam integer RX_SCALE = (TB_IN_W << FB) / TB_RES_W;        // 34568
    localparam integer RY_SCALE = (TB_IN_H << FB) / TB_OUT_H;        // 34560
    localparam integer RX_OFF   = (RX_SCALE >> 1) - 8192;            // 9092
    localparam integer RY_OFF   = (RY_SCALE >> 1) - 8192;            // 9088

    //--------------------------------------------------------------------------
    reg clk = 1'b0;
    always #2.5 clk = ~clk;
    reg rst_n = 1'b0;

    reg  [23:0] img_data  = 24'h0;
    reg         img_valid = 1'b0;
    reg         img_start = 1'b0;
    reg         hwpre_start = 1'b0;
    reg  [31:0] base_byte_addr = 32'h0;

    reg  app_rdy = 1'b1;
    reg  app_wdf_rdy = 1'b1;

    wire hwpre_done, busy;
    wire [28:0]  app_addr;
    wire [2:0]   app_cmd;
    wire         app_en;
    wire [511:0] app_wdf_data;
    wire         app_wdf_end;
    wire [63:0]  app_wdf_mask;
    wire         app_wdf_wren;
    wire         app_request;

    hwpre_top_external dut (
        .clk(clk), .rst_n(rst_n),
        .img_start(img_start), .img_valid(img_valid), .img_data(img_data),
        .hwpre_start(hwpre_start), .base_byte_addr(base_byte_addr),
        .hwpre_done(hwpre_done), .busy(busy),
        .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en),
        .app_wdf_data(app_wdf_data), .app_wdf_end(app_wdf_end),
        .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren),
        .app_request(app_request), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy)
    );

    //--------------------------------------------------------------------------
    // Frame capture + always-on monitors
    //--------------------------------------------------------------------------
    reg  [511:0] beat_mem [0:TB_BEATS-1];
    integer beat_cnt, frame_beat_total, drop_cnt, x_cnt, misalign_cnt, cmd_err;
    integer back_step_cnt;
    reg [31:0] first_addr, last_addr, prev_addr;
    reg        prev_addr_vld;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            beat_cnt        <= 0;
            frame_beat_total<= 0;
            drop_cnt        <= 0;
            x_cnt           <= 0;
            misalign_cnt    <= 0;
            cmd_err         <= 0;
            back_step_cnt   <= 0;
            first_addr      <= 32'h0;
            last_addr       <= 32'h0;
            prev_addr       <= 32'h0;
            prev_addr_vld   <= 1'b0;
        end else begin
            // ---- I1: resize pixel presented while out_ready low ----
            if (dut.u_core.u_resize.pv[6] && !dut.u_core.u_resize.out_ready) begin
                drop_cnt <= drop_cnt + 1;
                if (drop_cnt < 5)
                    $display("INVARIANT I1 VIOLATION: resize out_valid dropped at t=%0t (oy=%0d px=%0d)",
                             $time, dut.u_core.u_resize.poy[5], dut.u_core.u_resize.pox[5]);
            end

            // ---- I2: X/Z on write data ----
            if (app_en && app_wdf_wren) begin
                if ((^app_wdf_data) === 1'bx) begin
                    x_cnt <= x_cnt + 1;
                    if (x_cnt < 5) $display("INVARIANT I2 VIOLATION: X on app_wdf_data t=%0t", $time);
                end
            end

            // ---- write beat capture ----
            if (app_en && app_wdf_wren) begin
                if (beat_cnt < TB_BEATS) beat_mem[beat_cnt] <= app_wdf_data;
                if (beat_cnt == 0) first_addr <= {app_addr, 3'b000};
                last_addr      <= {app_addr, 3'b000};
                frame_beat_total <= frame_beat_total + 1;

                // ---- I3: alignment + monotonic 64B ----
                if (app_addr[2:0] !== 3'b000) misalign_cnt <= misalign_cnt + 1;
                if (prev_addr_vld && ({app_addr, 3'b000} !== prev_addr + 32'd64))
                    back_step_cnt <= back_step_cnt + 1;
                prev_addr     <= {app_addr, 3'b000};
                prev_addr_vld <= 1'b1;

                beat_cnt <= beat_cnt + 1;
            end
            if (app_en && app_cmd != 3'b000) cmd_err <= cmd_err + 1;
        end
    end

    //--------------------------------------------------------------------------
    // Reference model
    //--------------------------------------------------------------------------
    function integer src_r(input integer x); src_r = x % 256; endfunction
    function integer src_g(input integer y); src_g = y % 256; endfunction
    function integer src_b(input integer x, input integer y); src_b = (x % 256) ^ (y % 256); endfunction

    function [7:0] get_byte(input integer off);
        integer bb, ww;
        begin
            bb = off / 64;
            ww = off % 64;
            get_byte = (beat_mem[bb] >> (ww*8)) & 64'hFF;
        end
    endfunction

    function [23:0] model_pix(input integer ox, input integer oy);
        integer fx, fy, x0, x1, y0, y1, wx, wy, wxc, wyc;
        integer htr, htg, htb, hbr, hbg, hbb, orr, org, orb;
        begin
            fx = ox*RX_SCALE + RX_OFF;
            fy = oy*RY_SCALE + RY_OFF;
            x0 = fx / 16384; wx = fx % 16384;
            y0 = fy / 16384; wy = fy % 16384;
            if (x0 > (TB_IN_W-1)) x0 = TB_IN_W-1;
            x1 = x0 + 1; if (x1 > (TB_IN_W-1)) x1 = TB_IN_W-1;
            if (y0 > (TB_IN_H-1)) y0 = TB_IN_H-1;
            y1 = y0 + 1; if (y1 > (TB_IN_H-1)) y1 = TB_IN_H-1;
            wxc = 16384 - wx; wyc = 16384 - wy;
            htr = (wxc*src_r(x0)     + wx*src_r(x1)     + 8192) / 16384;
            htg = (wxc*src_g(y0)     + wx*src_g(y0)     + 8192) / 16384;
            htb = (wxc*src_b(x0,y0)  + wx*src_b(x1,y0)  + 8192) / 16384;
            hbr = (wxc*src_r(x0)     + wx*src_r(x1)     + 8192) / 16384;
            hbg = (wxc*src_g(y1)     + wx*src_g(y1)     + 8192) / 16384;
            hbb = (wxc*src_b(x0,y1)  + wx*src_b(x1,y1)  + 8192) / 16384;
            orr = (wyc*htr + wy*hbr + 8192) / 16384;
            org = (wyc*htg + wy*hbg + 8192) / 16384;
            orb = (wyc*htb + wy*hbb + 8192) / 16384;
            model_pix = {orr[7:0], org[7:0], orb[7:0]};
        end
    endfunction

    //--------------------------------------------------------------------------
    integer errors;
    integer ox_, oy_, px_, off_, mism;
    reg [23:0] got, exp;

    task check_content(input [255:0] tag);
        begin
            mism = 0;
            for (oy_ = 0; oy_ < TB_OUT_H; oy_ = oy_ + 1) begin
                for (px_ = 0; px_ < TB_OUT_W; px_ = px_ + 1) begin
                    off_ = ((oy_*TB_OUT_W) + px_) * 3;
                    got = {get_byte(off_), get_byte(off_+1), get_byte(off_+2)};
                    if (px_ < TB_PAD_LEFT || px_ >= (TB_OUT_W - TB_PAD_LEFT))
                        exp = TB_PAD_RGB;
                    else
                        exp = model_pix(px_ - TB_PAD_LEFT, oy_);
                    if (got !== exp) begin
                        mism = mism + 1;
                        if (mism <= 5)
                            $display("  [%0s] PIXMISMATCH oy=%0d px=%0d got=%06x exp=%06x",
                                     tag, oy_, px_, got, exp);
                    end
                end
            end
            if (mism == 0)
                $display("  [%0s] OK %0d/%0d pixels bit-exact", tag, TB_OUT_W*TB_OUT_H, TB_OUT_W*TB_OUT_H);
            else begin
                $display("  [%0s] ERROR %0d pixels mismatch", tag, mism);
                errors = errors + 1;
            end
        end
    endtask

    //--------------------------------------------------------------------------
    // Stimulus driver
    //   vblank_lines : blank lines emitted (img_valid=0) after the vsync pulse
    //   hblank       : idle cycles after each of the 1080 active lines
    //   arm_line     : if >=0, pulse hwpre_start inside the hblank of that line
    //   bab_line     : if >=0, rewrite base_byte_addr inside the hblank of that line
    //   rst_line     : if >=0, assert rst_n low for 8 cycles in that line's hblank
    //--------------------------------------------------------------------------
    integer lj, li, lb;

    task drive_frame(input integer vblank_lines, input integer hblank,
                     input integer arm_line, input integer bab_line,
                     input integer rst_line);
        begin
            // vsync: 1-cycle img_start pulse, no data yet
            @(negedge clk);
            img_start = 1'b1; img_valid = 1'b0;
            @(negedge clk);
            img_start = 1'b0;
            @(negedge clk);

            // vertical blanking
            for (lb = 0; lb < vblank_lines*(TB_IN_W+hblank); lb = lb + 1) @(negedge clk);

            for (lj = 0; lj < TB_IN_H; lj = lj + 1) begin
                for (li = 0; li < TB_IN_W; li = li + 1) begin
                    img_valid = 1'b1;
                    img_data  = {li[7:0], lj[7:0], li[7:0] ^ lj[7:0]};
                    @(negedge clk);
                end
                img_valid = 1'b0;

                if (lj == arm_line) begin
                    @(negedge clk); hwpre_start = 1'b1;
                    @(negedge clk); hwpre_start = 1'b0;
                    $display("  STIM: hwpre_start pulsed while busy at line %0d (expect ignored)", lj);
                end
                if (lj == bab_line) begin
                    base_byte_addr = 32'hDEAD_BEE0;   // must not affect this frame
                    $display("  STIM: base_byte_addr rewritten mid-frame (expect ignored)");
                end
                if (lj == rst_line) begin
                    $display("  STIM: rst_n asserted mid-frame at line %0d", lj);
                    @(negedge clk); rst_n = 1'b0;
                    repeat (8) @(negedge clk);
                    rst_n = 1'b1;
                    // abandon this frame: leave the line loop entirely
                    lj = TB_IN_H;
                    li = TB_IN_W;
                end

                for (li = 0; li < hblank; li = li + 1) @(negedge clk);
            end
            img_valid = 1'b0;
        end
    endtask

    task pulse_arm;
        begin
            @(negedge clk); hwpre_start = 1'b1;
            @(negedge clk); hwpre_start = 1'b0;
            @(negedge clk);
        end
    endtask

    task snap(input [255:0] tag);
        begin
            $display("  SNAP[%0s] t=%0t state=%0d mig_fa=%0b byte_addr=0x%08x base=0x%08x beats=%0d",
                     tag, $time, dut.state, dut.u_mig.frame_active, dut.u_mig.byte_addr,
                     base_byte_addr, frame_beat_total);
        end
    endtask

    task clear_monitors;
        begin
            // Align to the negedge so no beat can be accepted in this same delta.
            // Otherwise the monitor's pending NBA updates race with these blocking
            // clears and can leave stale first_addr / last_addr / beat totals.
            @(negedge clk);
            beat_cnt         = 0;
            frame_beat_total = 0;
            first_addr       = 32'h0;
            last_addr        = 32'h0;
            prev_addr        = 32'h0;
            prev_addr_vld    = 1'b0;
            drop_cnt         = 0;
            x_cnt            = 0;
            misalign_cnt     = 0;
            cmd_err          = 0;
            back_step_cnt    = 0;
        end
    endtask

    task check_frame(input [255:0] tag, input [31:0] expect_base, input integer expect_beats);
        begin
            if (frame_beat_total != expect_beats) begin
                $display("  [%0s] ERROR beat total=%0d expect %0d", tag, frame_beat_total, expect_beats);
                errors = errors + 1;
            end else
                $display("  [%0s] OK beat total=%0d", tag, frame_beat_total);

            if (first_addr !== expect_base) begin
                $display("  [%0s] ERROR first_addr=0x%08x expect 0x%08x", tag, first_addr, expect_base);
                errors = errors + 1;
            end else
                $display("  [%0s] OK first_addr=0x%08x", tag, first_addr);

            if (last_addr !== expect_base + expect_beats*64 - 64) begin
                $display("  [%0s] ERROR last_addr=0x%08x expect 0x%08x",
                         tag, last_addr, expect_base + expect_beats*64 - 64);
                errors = errors + 1;
            end

            if (drop_cnt != 0) begin
                $display("  [%0s] ERROR I1: %0d resize pixels dropped (out_ready low while valid)", tag, drop_cnt);
                errors = errors + 1;
            end
            if (x_cnt != 0) begin
                $display("  [%0s] ERROR I2: %0d beats with X data", tag, x_cnt);
                errors = errors + 1;
            end
            if (misalign_cnt != 0) begin
                $display("  [%0s] ERROR I3: %0d misaligned addresses", tag, misalign_cnt);
                errors = errors + 1;
            end
            if (back_step_cnt != 0) begin
                $display("  [%0s] ERROR I3: %0d non-monotonic address steps", tag, back_step_cnt);
                errors = errors + 1;
            end
            if (cmd_err != 0) begin
                $display("  [%0s] ERROR %0d app_cmd != WRITE", tag, cmd_err);
                errors = errors + 1;
            end
        end
    endtask

    integer idle_i;

    //==========================================================================
    initial begin
        errors = 0;
        $display("STRESS: geometry %0dx%0d, %0d beats/frame", TB_OUT_W, TB_OUT_H, TB_BEATS);

        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        //======================================================================
        // S1: ideal timing baseline (same conditions as golden TB)
        //======================================================================
        $display("\n=== S1: ideal timing, no blanking ===");
        clear_monitors;
        base_byte_addr = 32'h3000_0000;
        snap("pre-S1");
        pulse_arm;
        fork
            drive_frame(0, 0, -1, -1, -1);
            begin
                wait (hwpre_done == 1'b1);
                repeat (20) @(posedge clk);
                check_frame("S1", 32'h3000_0000, TB_BEATS);
                check_content("S1");
            end
        join

        //======================================================================
        // S2: realistic 1080p60 timing - 45 blank lines + 280-cycle hblank
        //======================================================================
        $display("\n=== S2: realistic timing (45 vblank lines, 280-cycle hblank) ===");
        clear_monitors;
        base_byte_addr = 32'h4000_0000;
        snap("pre-S2");
        pulse_arm;
        fork
            drive_frame(45, 280, -1, -1, -1);
            begin
                wait (hwpre_done == 1'b1);
                repeat (20) @(posedge clk);
                check_frame("S2", 32'h4000_0000, TB_BEATS);
                snap("post-S2");
                check_content("S2");
            end
        join

        //======================================================================
        // S3 + S4: arm while busy, base rewrite mid-frame (both must be ignored)
        //======================================================================
        $display("\n=== S3/S4: arm-while-busy + mid-frame base rewrite ===");
        clear_monitors;
        base_byte_addr = 32'h5000_0000;
        snap("pre-S3S4");
        pulse_arm;
        fork
            drive_frame(0, 40, 500, 700, -1);
            begin
                wait (hwpre_done == 1'b1);
                repeat (20) @(posedge clk);
                check_frame("S3/S4", 32'h5000_0000, TB_BEATS);
                snap("post-S3S4");
                // full content check is unnecessary here (input stream identical to
                // S1); the address/beat-count assertions are what S3/S4 target.
            end
        join
        if (frame_beat_total != TB_BEATS) begin
            $display("  [S3/S4] ERROR arm-while-busy corrupted the frame");
            errors = errors + 1;
        end
        base_byte_addr = 32'h5000_0000;   // restore (S4 scribbled it)

        //======================================================================
        // S6: post-done silence - no stale beat may appear in a long idle window
        //======================================================================
        $display("\n=== S6: post-done silence (300k idle cycles) ===");
        clear_monitors;
        img_valid = 1'b0;
        img_start = 1'b0;
        for (idle_i = 0; idle_i < 300000; idle_i = idle_i + 1) @(posedge clk);
        if (frame_beat_total != 0) begin
            $display("  [S6] ERROR %0d spurious beats during idle window", frame_beat_total);
            errors = errors + 1;
        end else
            $display("  [S6] OK zero spurious beats in 300k idle cycles");

        //======================================================================
        // S5: mid-frame reset, then a clean frame must still run
        //======================================================================
        $display("\n=== S5: reset mid-frame, then recover ===");
        clear_monitors;
        base_byte_addr = 32'h6000_0000;
        pulse_arm;
        drive_frame(0, 40, -1, -1, 400);      // aborts at line 400 via rst_n
        repeat (50) @(posedge clk);
        $display("  [S5] recovered, busy=%0b", busy);
        if (busy) begin
            $display("  [S5] ERROR busy stuck high after mid-frame reset");
            errors = errors + 1;
        end
        clear_monitors;
        base_byte_addr = 32'h6100_0000;
        snap("pre-S5");
        pulse_arm;
        fork
            drive_frame(0, 0, -1, -1, -1);
            begin
                wait (hwpre_done == 1'b1);
                repeat (20) @(posedge clk);
                check_frame("S5", 32'h6100_0000, TB_BEATS);
                check_content("S5");
            end
        join

        //======================================================================
        $display("");
        if (errors == 0) $display("=== STRESS PASS: all scenarios OK ===");
        else             $display("=== STRESS FAIL: %0d errors ===", errors);
        $finish;
    end

    initial begin
        #400000000;
        $display("=== STRESS TIMEOUT ===");
        $finish;
    end

endmodule
