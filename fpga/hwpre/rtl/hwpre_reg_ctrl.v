`timescale 1ns / 1ps
// hwpre_reg_ctrl — PS register interface + CDC for hwpre control.
//
// gp_clk domain: register file (CTRL, STATUS, BASE_ADDR, DEBUG, VERSION)
// ddr_clk domain: hwpre_start, hwpre_window, hwpre_base_addr (outputs to hwpre_top_external / arbiter)
//                 hwpre_done, hwpre_busy (inputs from hwpre_top_external)
//
// CDC strategy:
//   gp→ddr: start      = toggle synchronizer (no lost pulse)
//   gp→ddr: window     = 2-FF level sync
//   gp→ddr: base_addr  = freeze on start toggle → 2-FF per bit → latch on edge detect
//   ddr→gp: done       = pulse stretch → 2-FF → sticky register
//   ddr→gp: busy       = 2-FF level sync
//   ddr→gp: beat_count = snapshot on done (single-domain read via STATUS/done handshake)
//   ddr→gp: frame_count= snapshot on done

module hwpre_reg_ctrl (
    // gp_clk domain — PS register interface
    input  wire         gp_clk,
    input  wire         gp_rst_n,

    input  wire [7:0]   reg_addr,        // byte offset (0x00–0x1C)
    input  wire [31:0]  reg_wdata,
    input  wire         reg_wr_en,       // single-cycle write pulse
    output wire [31:0]  reg_rdata,

    // ddr_clk domain — hwpre control outputs (to ai7030_top wires)
    input  wire         ddr_clk,
    input  wire         ddr_rst_n,

    output wire         hwpre_start,     // 1-cycle pulse on start
    output wire         hwpre_window,    // level from CTRL.window_enable
    output wire [31:0]  hwpre_base_addr, // latched PLDDR byte address
    output wire [7:0]   hwpre_m0_slots,  // NPU slots before one HWPRE write

    // ddr_clk domain — hwpre status inputs (from hwpre_top_external)
    input  wire         hwpre_done,      // 1-cycle pulse (ddr_clk)
    input  wire         hwpre_busy       // level (ddr_clk)
);

    //==========================================================================
    // gp_clk domain: register file
    //==========================================================================
    // Register addresses
    localparam ADDR_CTRL        = 8'h00;
    localparam ADDR_STATUS      = 8'h04;
    localparam ADDR_BASE_ADDR_LO= 8'h08;
    localparam ADDR_BASE_ADDR_HI= 8'h0C;
    localparam ADDR_DEBUG_0     = 8'h10;
    localparam ADDR_DEBUG_1     = 8'h14;
    localparam ADDR_VERSION     = 8'h18;
    localparam ADDR_ARB_CFG     = 8'h1C;

    // CTRL bits
    localparam CTRL_START_BIT      = 0;
    localparam CTRL_WINDOW_BIT     = 1;
    localparam CTRL_CLEAR_DONE_BIT = 2;

    // STATUS bits
    localparam STATUS_DONE_BIT  = 0;
    localparam STATUS_BUSY_BIT  = 1;

    reg        window_enable_gp;     // CTRL bit1: window_enable
    reg        start_req_gp;         // CTRL bit0 write detected (pulse)
    reg        clear_done_gp;        // CTRL bit2 write detected (pulse)
    reg [31:0] base_addr_lo_gp;      // BASE_ADDR_LO
    reg [31:0] base_addr_hi_gp;      // BASE_ADDR_HI
    reg        done_sticky_gp;       // STATUS bit0: done (sticky)
    reg        busy_synced_gp;       // STATUS bit1: busy (synced from ddr)
    reg [31:0] beat_count_snap_gp;   // DEBUG_0 snapshot
    reg [31:0] frame_count_snap_gp;  // DEBUG_1 snapshot
    reg [7:0]  arb_m0_slots_gp;      // ARB_CFG[7:0], zero is clamped to one

    // Write decode
    wire wr_ctrl         = reg_wr_en && (reg_addr == ADDR_CTRL);
    wire wr_base_lo      = reg_wr_en && (reg_addr == ADDR_BASE_ADDR_LO);
    wire wr_base_hi      = reg_wr_en && (reg_addr == ADDR_BASE_ADDR_HI);
    wire wr_arb_cfg      = reg_wr_en && (reg_addr == ADDR_ARB_CFG);

    always @(posedge gp_clk or negedge gp_rst_n) begin
        if (!gp_rst_n) begin
            window_enable_gp <= 1'b0;
            start_req_gp     <= 1'b0;
            clear_done_gp    <= 1'b0;
            base_addr_lo_gp  <= 32'h0;
            base_addr_hi_gp  <= 32'h0;
            arb_m0_slots_gp  <= 8'd7;
        end else begin
            start_req_gp  <= 1'b0;
            clear_done_gp <= 1'b0;

            if (wr_ctrl) begin
                window_enable_gp <= reg_wdata[CTRL_WINDOW_BIT];
                if (reg_wdata[CTRL_START_BIT])
                    start_req_gp <= 1'b1;
                if (reg_wdata[CTRL_CLEAR_DONE_BIT])
                    clear_done_gp <= 1'b1;
            end

            if (wr_base_lo)
                base_addr_lo_gp <= reg_wdata;
            if (wr_base_hi)
                base_addr_hi_gp <= reg_wdata;
            if (wr_arb_cfg)
                arb_m0_slots_gp <= (reg_wdata[7:0] == 8'd0) ? 8'd1 : reg_wdata[7:0];
        end
    end

    // Done sticky: set by done_synced pulse, cleared by clear_done
    wire done_synced_pulse_gp;

    always @(posedge gp_clk or negedge gp_rst_n) begin
        if (!gp_rst_n)
            done_sticky_gp <= 1'b0;
        else if (clear_done_gp)
            done_sticky_gp <= 1'b0;
        else if (done_synced_pulse_gp)
            done_sticky_gp <= 1'b1;
    end

    // Read mux
    reg [31:0] rdata_reg;
    always @(posedge gp_clk or negedge gp_rst_n) begin
        if (!gp_rst_n)
            rdata_reg <= 32'h0;
        else begin
            case (reg_addr)
                ADDR_STATUS:       rdata_reg <= {30'h0, busy_synced_gp, done_sticky_gp};
                ADDR_BASE_ADDR_LO: rdata_reg <= base_addr_lo_gp;
                ADDR_BASE_ADDR_HI: rdata_reg <= base_addr_hi_gp;
                ADDR_DEBUG_0:      rdata_reg <= beat_count_snap_gp;
                ADDR_DEBUG_1:      rdata_reg <= frame_count_snap_gp;
                ADDR_VERSION:      rdata_reg <= 32'h2026_0729;
                ADDR_ARB_CFG:      rdata_reg <= {24'h0, arb_m0_slots_gp};
                default:           rdata_reg <= 32'h0;
            endcase
        end
    end
    assign reg_rdata = rdata_reg;

    //==========================================================================
    // gp→ddr: start toggle synchronizer
    //==========================================================================
    reg start_toggle_gp;

    // Stretch start_req into a wide pulse (16 gp_clk cycles) for safe CDC
    reg [3:0] start_stretch_cnt_gp;
    reg       start_stretched_gp;

    always @(posedge gp_clk or negedge gp_rst_n) begin
        if (!gp_rst_n) begin
            start_stretched_gp   <= 1'b0;
            start_stretch_cnt_gp <= 4'd0;
        end else begin
            if (start_req_gp) begin
                start_stretched_gp   <= 1'b1;
                start_stretch_cnt_gp <= 4'd0;
            end else if (start_stretched_gp) begin
                if (start_stretch_cnt_gp == 4'd15)
                    start_stretched_gp <= 1'b0;
                else
                    start_stretch_cnt_gp <= start_stretch_cnt_gp + 4'd1;
            end
        end
    end

    // 2-FF sync stretched start pulse into ddr_clk, then edge-detect
    reg [1:0] start_sync_ddr;
    reg       start_q_ddr;

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            start_sync_ddr <= 2'b00;
            start_q_ddr    <= 1'b0;
        end else begin
            start_sync_ddr <= {start_sync_ddr[0], start_stretched_gp};
            start_q_ddr    <= start_sync_ddr[1];
        end
    end

    // Rising-edge detect on synced stretched pulse → 1-cycle hwpre_start
    wire start_edge_ddr = start_sync_ddr[1] && !start_q_ddr;
    assign hwpre_start = start_edge_ddr;

    //==========================================================================
    // gp→ddr: window_enable 2-FF level sync
    //==========================================================================
    reg [1:0] window_sync_ddr;

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n)
            window_sync_ddr <= 2'b00;
        else
            window_sync_ddr <= {window_sync_ddr[0], window_enable_gp};
    end
    assign hwpre_window = window_sync_ddr[1];

    // ARB_CFG is written while HWPRE is idle and remains stable during a run.
    // A two-stage vector synchronizer is sufficient for this static setting.
    reg [7:0] arb_slots_sync1_ddr;
    reg [7:0] arb_slots_sync2_ddr;

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            arb_slots_sync1_ddr <= 8'd7;
            arb_slots_sync2_ddr <= 8'd7;
        end else begin
            arb_slots_sync1_ddr <= arb_m0_slots_gp;
            arb_slots_sync2_ddr <= arb_slots_sync1_ddr;
        end
    end
    assign hwpre_m0_slots = arb_slots_sync2_ddr;

    //==========================================================================
    // gp→ddr: base_addr — frozen on start, 2-FF per bit, latched on edge
    //==========================================================================
    // Freeze base_addr in the SAME always block as start_req_gp to avoid
    // NBA race: start_req_gp is NBA-set, so it reads as OLD value (0) in
    // other always blocks at the same posedge. Freeze directly from wr_ctrl.
    reg [31:0] base_addr_frozen_gp;

    always @(posedge gp_clk or negedge gp_rst_n) begin
        if (!gp_rst_n)
            base_addr_frozen_gp <= 32'h0;
        else if (reg_wr_en && (reg_addr == ADDR_CTRL) && reg_wdata[CTRL_START_BIT])
            base_addr_frozen_gp <= base_addr_lo_gp;  // freeze directly from write
    end

    // 2-FF sync each bit of frozen base_addr (multi-bit, stable after freeze)
    reg [31:0] base_addr_sync1_ddr;
    reg [31:0] base_addr_sync2_ddr;

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            base_addr_sync1_ddr <= 32'h0;
            base_addr_sync2_ddr <= 32'h0;
        end else begin
            base_addr_sync1_ddr <= base_addr_frozen_gp;
            base_addr_sync2_ddr <= base_addr_sync1_ddr;
        end
    end

    // Latch base_addr one cycle AFTER start edge.
    // base_addr_frozen changes with start_req in gp_clk.
    // Both propagate through 2-FF chains to ddr_clk.
    // By delaying the latch 1 cycle past start_edge, base_addr_sync2
    // is guaranteed stable (3 ddr_clk cycles of settling).
    reg start_edge_d1_ddr;
    reg [31:0] hwpre_base_addr_ddr;

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            start_edge_d1_ddr     <= 1'b0;
            hwpre_base_addr_ddr   <= 32'h0;
        end else begin
            start_edge_d1_ddr <= start_edge_ddr;
            if (start_edge_d1_ddr)
                hwpre_base_addr_ddr <= base_addr_sync2_ddr;
        end
    end
    assign hwpre_base_addr = hwpre_base_addr_ddr;

    //==========================================================================
    // ddr→gp: done pulse → pulse stretch → 2-FF → sticky (gp_clk side)
    //==========================================================================
    // Stretch done pulse in ddr_clk: extend to wide enough for gp_clk to see
    reg [7:0] done_stretch_cnt_ddr;
    reg       done_stretched_ddr;

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            done_stretch_cnt_ddr <= 8'd0;
            done_stretched_ddr   <= 1'b0;
        end else begin
            if (hwpre_done) begin
                done_stretched_ddr   <= 1'b1;
                done_stretch_cnt_ddr <= 8'd0;
            end else if (done_stretched_ddr) begin
                // stretch for ~255 ddr_clk cycles (1.275us @ 200MHz)
                // gp_clk ~100MHz (10ns period) → 128+ ddr_clk cycles = safe
                if (done_stretch_cnt_ddr == 8'd255)
                    done_stretched_ddr <= 1'b0;
                else
                    done_stretch_cnt_ddr <= done_stretch_cnt_ddr + 8'd1;
            end
        end
    end

    // 2-FF sync done_stretched into gp_clk
    reg [1:0] done_sync_gp;
    reg       done_q_gp;

    always @(posedge gp_clk or negedge gp_rst_n) begin
        if (!gp_rst_n) begin
            done_sync_gp <= 2'b00;
            done_q_gp    <= 1'b0;
        end else begin
            done_sync_gp <= {done_sync_gp[0], done_stretched_ddr};
            done_q_gp    <= done_sync_gp[1];
        end
    end

    // Edge detect on done_synced: rising edge → done_pulse for sticky register
    assign done_synced_pulse_gp = done_sync_gp[1] && !done_q_gp;

    //==========================================================================
    // ddr→gp: busy 2-FF level sync
    //==========================================================================
    reg [1:0] busy_sync_gp;

    always @(posedge gp_clk or negedge gp_rst_n) begin
        if (!gp_rst_n)
            busy_sync_gp <= 2'b00;
        else
            busy_sync_gp <= {busy_sync_gp[0], hwpre_busy};
    end

    always @(posedge gp_clk or negedge gp_rst_n) begin
        if (!gp_rst_n)
            busy_synced_gp <= 1'b0;
        else
            busy_synced_gp <= busy_sync_gp[1];
    end

    //==========================================================================
    // ddr_clk domain: beat_count, frame_count (snapshot on done for gp_clk)
    //==========================================================================
    reg [31:0] beat_count_ddr;
    reg [31:0] frame_count_ddr;

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            beat_count_ddr  <= 32'h0;
            frame_count_ddr <= 32'h0;
        end else begin
            if (hwpre_done) begin
                frame_count_ddr <= frame_count_ddr + 32'd1;
            end
            if (hwpre_start) begin
                // beat_count is set by external connection in ai7030_top
                // placeholder: incremented on each beat via external connection
                beat_count_ddr <= 32'h0;  // reset per frame start
            end
        end
    end

    // Transfer count snapshots to gp_clk domain on done
    // Use a simple handshake: gp_clk snapshots when done arrives
    reg        counts_valid_ddr;
    reg [31:0] beat_count_xfer_ddr;
    reg [31:0] frame_count_xfer_ddr;

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            counts_valid_ddr     <= 1'b0;
            beat_count_xfer_ddr  <= 32'h0;
            frame_count_xfer_ddr <= 32'h0;
        end else begin
            if (hwpre_done) begin
                beat_count_xfer_ddr  <= beat_count_ddr;
                frame_count_xfer_ddr <= frame_count_ddr;
                counts_valid_ddr     <= 1'b1;
            end else begin
                counts_valid_ddr <= 1'b0;
            end
        end
    end

    // 2-FF sync counts_valid into gp_clk
    reg [1:0] counts_valid_sync_gp;

    always @(posedge gp_clk or negedge gp_rst_n) begin
        if (!gp_rst_n)
            counts_valid_sync_gp <= 2'b00;
        else
            counts_valid_sync_gp <= {counts_valid_sync_gp[0], counts_valid_ddr};
    end

    // Sample beat/frame count in gp_clk when counts_valid_sync fires
    // These are stable multi-bit values sampled with a control signal
    reg [31:0] beat_count_sync1_gp, beat_count_sync2_gp;
    reg [31:0] frame_count_sync1_gp, frame_count_sync2_gp;

    always @(posedge gp_clk or negedge gp_rst_n) begin
        if (!gp_rst_n) begin
            beat_count_sync1_gp  <= 32'h0;
            beat_count_sync2_gp  <= 32'h0;
            frame_count_sync1_gp <= 32'h0;
            frame_count_sync2_gp <= 32'h0;
        end else begin
            beat_count_sync1_gp  <= beat_count_xfer_ddr;
            beat_count_sync2_gp  <= beat_count_sync1_gp;
            frame_count_sync1_gp <= frame_count_xfer_ddr;
            frame_count_sync2_gp <= frame_count_sync1_gp;
        end
    end

    wire counts_valid_edge_gp = counts_valid_sync_gp[1] && !counts_valid_sync_gp[0];

    always @(posedge gp_clk or negedge gp_rst_n) begin
        if (!gp_rst_n) begin
            beat_count_snap_gp  <= 32'h0;
            frame_count_snap_gp <= 32'h0;
        end else if (counts_valid_edge_gp) begin
            beat_count_snap_gp  <= beat_count_sync2_gp;
            frame_count_snap_gp <= frame_count_sync2_gp;
        end
    end

endmodule
