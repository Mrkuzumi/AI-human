`timescale 1ns / 1ps
// hwpre_axi_lite_bridge — AXI4-Lite slave → register bridge (v2 fix)
//
// v2 fixes (vs v1):
//   Read: latch ARADDR on AR handshake; wait 1 cycle for hwpre_reg_ctrl's
//         registered rdata_reg to update; then capture reg_rdata and assert RVALID.
//         First VERSION read returns 0x20260601 correctly.
//   Write: decouples AW and W channels. AW can arrive before W, after W, or
//          simultaneously. Exactly one reg_wr_en pulse per AXI transaction.
//          BVALID held until BREADY.
//
// Supported: 32-bit full-word reads/writes only.
// Unsupported: sub-word writes (WSTRB != 4'b1111) → BRESP=SLVERR.

module hwpre_axi_lite_bridge (
    // AXI4-Lite slave (from PS M_AXI_GP1 via axi_interconnect_1/M02_AXI → auto_pc)
    input  wire         s_axi_aclk,
    input  wire         s_axi_aresetn,

    // Write address channel
    input  wire [31:0]  s_axi_awaddr,
    input  wire         s_axi_awvalid,
    output reg          s_axi_awready,

    // Write data channel
    input  wire [31:0]  s_axi_wdata,
    input  wire [3:0]   s_axi_wstrb,
    input  wire         s_axi_wvalid,
    output reg          s_axi_wready,

    // Write response channel
    output reg  [1:0]   s_axi_bresp,
    output reg          s_axi_bvalid,
    input  wire         s_axi_bready,

    // Read address channel
    input  wire [31:0]  s_axi_araddr,
    input  wire         s_axi_arvalid,
    output reg          s_axi_arready,

    // Read data channel
    output reg  [31:0]  s_axi_rdata,
    output reg  [1:0]   s_axi_rresp,
    output reg          s_axi_rvalid,
    input  wire         s_axi_rready,

    // Simple register interface (to hwpre_reg_ctrl, gp_clk domain)
    output wire [7:0]   reg_addr,
    output wire [31:0]  reg_wdata,
    output wire         reg_wr_en,
    input  wire [31:0]  reg_rdata
);

    //==========================================================================
    // Write channel (AW + W + B)
    //==========================================================================

    // AW accepted flag: set when AW handshake completes
    reg aw_done;
    reg [7:0] aw_addr_latched;  // latch awaddr during handshake

    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            aw_done          <= 1'b0;
            aw_addr_latched  <= 8'h00;
        end else if (s_axi_awvalid && s_axi_awready) begin
            aw_done          <= 1'b1;
            aw_addr_latched  <= s_axi_awaddr[7:0];
        end else if (reg_wr_en) begin
            aw_done <= 1'b0;
        end
    end

    // W accepted flag: set when W handshake completes
    reg w_done;
    reg [31:0] wdata_latched;
    reg        wstrb_ok;    // 1 = full 32-bit write (WSTRB=4'b1111)

    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            w_done        <= 1'b0;
            wdata_latched <= 32'h0;
            wstrb_ok      <= 1'b0;
        end else if (s_axi_wvalid && s_axi_wready) begin
            w_done        <= 1'b1;
            wdata_latched <= s_axi_wdata;
            wstrb_ok      <= (s_axi_wstrb == 4'b1111);
        end else if (reg_wr_en) begin
            w_done <= 1'b0;
        end
    end

    // AWREADY: accept when we're not still processing a previous write
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn)
            s_axi_awready <= 1'b1;
        else
            s_axi_awready <= !aw_done;   // accept new AW only if previous done
    end

    // WREADY: accept when AW was already done (or simultaneous), and we haven't
    // already captured W data for this transaction
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn)
            s_axi_wready <= 1'b0;
        else
            // Ready for W when: AW already accepted (or accepting simultaneously)
            s_axi_wready <= (aw_done || (s_axi_awvalid && s_axi_awready)) && !w_done;
    end

    // reg_wr_en: single-cycle pulse when both AW and W are accepted
    wire wr_complete = aw_done && w_done;
    assign reg_wr_en  = wr_complete;

    // Write response: set when write completes, hold until BREADY
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            s_axi_bvalid <= 1'b0;
            s_axi_bresp  <= 2'b00;
        end else if (wr_complete && !s_axi_bvalid) begin
            s_axi_bvalid <= 1'b1;
            s_axi_bresp  <= wstrb_ok ? 2'b00 : 2'b10;  // OKAY or SLVERR
        end else if (s_axi_bvalid && s_axi_bready) begin
            s_axi_bvalid <= 1'b0;
        end
    end

    //==========================================================================
    // Read channel (AR + R)
    //==========================================================================

    // Read pipeline stages:
    //   pv[0] = AR handshake → latch address
    //   pv[1] = wait for reg_rdata update (hwpre_reg_ctrl 1-cycle pipeline)
    //   pv[2] = capture reg_rdata + assert RVALID
    reg [2:0] r_pv;               // read pipeline valid
    reg [7:0] ar_addr_latched;    // latched ARADDR

    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            r_pv             <= 3'b000;
            ar_addr_latched  <= 8'h00;
            s_axi_arready    <= 1'b1;
            s_axi_rdata      <= 32'h0;
            s_axi_rresp      <= 2'b00;
            s_axi_rvalid     <= 1'b0;
        end else begin
            // Default: deassert RVALID on handshake
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end

            // Stage 2: rdata is stable for the latched address
            if (r_pv[1]) begin
                s_axi_rdata  <= reg_rdata;
                s_axi_rresp  <= 2'b00;
                s_axi_rvalid <= 1'b1;
            end

            // Stage 1: wait for reg_rdata to update
            r_pv[1] <= r_pv[0];

            // Stage 0: AR handshake → latch address, advance pipeline
            // ARREADY=1 in idle (no read in progress)
            if (s_axi_arvalid && s_axi_arready) begin
                ar_addr_latched <= s_axi_araddr[7:0];
                s_axi_arready   <= 1'b0;    // block new reads until current done
                r_pv[0]         <= 1'b1;
            end else begin
                r_pv[0] <= 1'b0;
            end

            // Re-arm ARREADY when read completes (RVALID→RREADY) and no new
            // AR handshake is taking place this cycle (avoids NBA race with
            // the handshake-detection block above)
            if (!r_pv[1] && !r_pv[0]
                && !(s_axi_rvalid && !s_axi_rready)
                && !(s_axi_arvalid && s_axi_arready)) begin
                s_axi_arready <= 1'b1;
            end
        end
    end

    //==========================================================================
    // Register interface outputs
    //==========================================================================
    // reg_addr: driven by latched address when read is active,
    //           driven by AWADDR during writes (level, stable after AW accepted)
    assign reg_addr  = (r_pv[0] || r_pv[1]) ? ar_addr_latched :
                       aw_done ? aw_addr_latched :
                       (s_axi_awvalid && s_axi_awready) ? s_axi_awaddr[7:0] :
                       8'h00;

    assign reg_wdata = wdata_latched;

endmodule
