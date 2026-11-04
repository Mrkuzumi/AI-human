`timescale 1ns / 1ps
// beat_fifo_xpm — XPM sync FIFO wrapper for beat stream buffering.
// Placed between nhwc_stream_packer and hwpre_mig_master.
// Absorbs MIG backpressure bursts up to FIFO_DEPTH beats.
//
// Width: 513 bits = {beat_last, beat_data[511:0]}
// FWFT mode: !empty drives rd_valid, rd_ready pops the beat.

module beat_fifo_xpm #(
    parameter FIFO_DEPTH = 128,
    parameter FIFO_WIDTH = 513
) (
    input  wire         clk,
    input  wire         rst_n,          // active-low reset
    // Write side (from packer)
    input  wire         wr_valid,
    output wire         wr_ready,       // combinational: !full
    input  wire [511:0] wr_data,
    input  wire         wr_last,
    // Read side (to MIG master)
    output wire         rd_valid,       // combinational: !empty
    input  wire         rd_ready,       // pop beat
    output wire [511:0] rd_data,
    output wire         rd_last
);

    // Pack {last,data} into single vector for FIFO
    wire [FIFO_WIDTH-1:0] din;
    assign din = {wr_last, wr_data};

    wire [FIFO_WIDTH-1:0] dout;
    wire xpm_full, xpm_empty;
    wire xpm_rst = !rst_n;

    // =========================================================================
    // XPM_SYNC_FIFO — FWFT, block RAM
    // =========================================================================
    xpm_fifo_sync #(
        .FIFO_MEMORY_TYPE   ("block"),
        .ECC_MODE           ("no_ecc"),
        .FIFO_WRITE_DEPTH   (FIFO_DEPTH),
        .WRITE_DATA_WIDTH   (FIFO_WIDTH),
        .READ_DATA_WIDTH    (FIFO_WIDTH),
        .READ_MODE          ("fwft"),
        .FIFO_READ_LATENCY  (0),
        .USE_ADV_FEATURES   ("0000"),
        .FULL_RESET_VALUE   (0),
        .DOUT_RESET_VALUE   ("0")
    ) u_xpm_fifo (
        .sleep          (1'b0),
        .rst            (xpm_rst),
        .wr_clk         (clk),
        .wr_en          (wr_valid && !xpm_full),
        .din            (din),
        .full           (xpm_full),
        .prog_full      (),
        .wr_data_count  (),
        .overflow       (),
        .wr_rst_busy    (),
        .almost_full    (),
        .wr_ack         (),
        .rd_en          (rd_ready && !xpm_empty),
        .dout           (dout),
        .empty          (xpm_empty),
        .prog_empty     (),
        .rd_data_count  (),
        .underflow      (),
        .rd_rst_busy    (),
        .almost_empty   (),
        .data_valid     (),
        .injectsbiterr  (1'b0),
        .injectdbiterr  (1'b0),
        .sbiterr        (),
        .dbiterr        ()
    );

    // Combinational ready/valid based on FIFO flags
    assign wr_ready = !xpm_full;
    assign rd_valid = !xpm_empty;

    // Unpack dout
    assign rd_last = dout[512];
    assign rd_data = dout[511:0];

endmodule
