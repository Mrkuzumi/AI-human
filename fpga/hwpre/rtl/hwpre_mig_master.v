`timescale 1ns / 1ps
// hwpre_mig_master v3 — direct passthrough, no internal beat buffering.
// beat_ready = app_rdy && app_wdf_rdy (combinational, like ddr_wrapper awwready).
// All MIG outputs are combinational. Address increments via counter on accept.
// done asserted when beat_last is accepted by MIG.

module hwpre_mig_master #(parameter BYTE_ADDR_WIDTH=32) (
    input  wire         clk, rst_n,
    input  wire         beat_valid,
    input  wire [511:0] beat_data,
    input  wire         beat_last,
    output wire         beat_ready,
    input  wire [31:0]  base_byte_addr,
    // MIG app outputs — ALL COMBINATIONAL (ddr_wrapper replication)
    output wire [28:0]  app_addr,
    output wire [2:0]   app_cmd,
    output wire         app_en,
    output wire [511:0] app_wdf_data,
    output wire         app_wdf_end,
    output wire [63:0]  app_wdf_mask,
    output wire         app_wdf_wren,
    input  wire         app_rdy, app_wdf_rdy,
    output wire         busy, done
);

    // Beat acceptance: when MIG is ready AND core presents a beat
    wire accept = beat_valid && app_rdy && app_wdf_rdy;

    // Address counter: increments on each accepted beat
    reg [31:0] byte_addr;
    reg        frame_active;  // set by first beat, cleared on last-beat acceptance
    reg        done_q;        // registered done pulse

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_addr <= 0;
            frame_active <= 0;
            done_q <= 0;
        end else begin
            done_q <= 0;
            if (beat_valid && !beat_ready) begin
                // core is presenting but MIG not ready — we wait
            end else if (accept) begin
                // beat accepted by MIG
                if (!frame_active) begin
                    // first beat: load base address, pre-increment for next beat
                    frame_active <= 1;
                    byte_addr <= base_byte_addr + 32'd64;
                end else begin
                    // subsequent beat: advance address
                    byte_addr <= byte_addr + 32'd64;
                end
                if (beat_last) begin
                    frame_active <= 0;
                    done_q <= 1;
                end
            end
        end
    end

    // =========================================================================
    // COMBINATIONAL MIG outputs (exact ddr_wrapper.sv replication)
    // =========================================================================
    assign beat_ready    = app_rdy && app_wdf_rdy;

    assign app_addr      = frame_active ? byte_addr[31:3] : base_byte_addr[31:3];
    assign app_cmd       = 3'b000;                       // WRITE
    assign app_wdf_data  = beat_valid ? beat_data : 512'd0;
    assign app_wdf_end   = 1'b1;                         // single-beat per command
    assign app_wdf_mask  = 64'd0;                        // full 64-byte write
    assign app_en        = beat_valid && app_wdf_rdy;    // ddr_wrapper line 190
    assign app_wdf_wren  = beat_valid && app_rdy;        // ddr_wrapper line 191

    assign busy = frame_active;
    assign done = done_q;

endmodule
