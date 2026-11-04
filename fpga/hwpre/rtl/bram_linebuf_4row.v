`timescale 1ns / 1ps
// bram_linebuf_4row — 4-line circular BRAM storage for bilinear resize
//
// 8 independent BRAM arrays (4 rows × 2 copies A/B).
// A copy reads at x0, B copy reads at x1 for single-cycle bilinear.
//
// Write: one pixel per cycle to both A/B copies of the selected slot.
// Read:  synchronous — address presented at cycle N, data valid at cycle N+1.
//        Read data is registered (rd_*) and available to parent module.
//
// No reset, no FSM. Pure BRAM storage + registered read data.

module bram_linebuf_4row #(
    parameter IN_W = 1920,
    parameter LINE_BUF_COUNT = 4
) (
    input  wire         clk,

    // Write port
    input  wire [1:0]   wr_slot,
    input  wire [10:0]  in_x,
    input  wire [23:0]  in_rgb,
    input  wire         in_valid,
    input  wire [10:0]  in_y,       // for bounds check only
    input  wire [10:0]  IN_H,

    // Read address ports (A=x0, B=x1 for each slot) — v19: [9:0]→[10:0] 支持 IN_W>1023
    input  wire [10:0]  ra_0A, ra_0B,
    input  wire [10:0]  ra_1A, ra_1B,
    input  wire [10:0]  ra_2A, ra_2B,
    input  wire [10:0]  ra_3A, ra_3B,

    // Read data outputs (registered, 1 cycle after address)
    output wire [23:0]  rd_0A, rd_0B,
    output wire [23:0]  rd_1A, rd_1B,
    output wire [23:0]  rd_2A, rd_2B,
    output wire [23:0]  rd_3A, rd_3B
);

    // BRAM arrays — no reset, no init
    (* ram_style = "block" *) reg [23:0] lb0_A [0:IN_W-1];
    (* ram_style = "block" *) reg [23:0] lb0_B [0:IN_W-1];
    (* ram_style = "block" *) reg [23:0] lb1_A [0:IN_W-1];
    (* ram_style = "block" *) reg [23:0] lb1_B [0:IN_W-1];
    (* ram_style = "block" *) reg [23:0] lb2_A [0:IN_W-1];
    (* ram_style = "block" *) reg [23:0] lb2_B [0:IN_W-1];
    (* ram_style = "block" *) reg [23:0] lb3_A [0:IN_W-1];
    (* ram_style = "block" *) reg [23:0] lb3_B [0:IN_W-1];

    // Read data registers
    reg [23:0] rd0_A_r, rd0_B_r;
    reg [23:0] rd1_A_r, rd1_B_r;
    reg [23:0] rd2_A_r, rd2_B_r;
    reg [23:0] rd3_A_r, rd3_B_r;

    assign rd_0A = rd0_A_r; assign rd_0B = rd0_B_r;
    assign rd_1A = rd1_A_r; assign rd_1B = rd1_B_r;
    assign rd_2A = rd2_A_r; assign rd_2B = rd2_B_r;
    assign rd_3A = rd3_A_r; assign rd_3B = rd3_B_r;

    // Slot 0: write + sync read
    always @(posedge clk) begin
        if (wr_slot == 2'd0 && in_valid && in_x < IN_W && in_y < IN_H) begin
            lb0_A[in_x] <= in_rgb;
            lb0_B[in_x] <= in_rgb;
        end
        rd0_A_r <= lb0_A[ra_0A];
        rd0_B_r <= lb0_B[ra_0B];
    end

    // Slot 1
    always @(posedge clk) begin
        if (wr_slot == 2'd1 && in_valid && in_x < IN_W && in_y < IN_H) begin
            lb1_A[in_x] <= in_rgb;
            lb1_B[in_x] <= in_rgb;
        end
        rd1_A_r <= lb1_A[ra_1A];
        rd1_B_r <= lb1_B[ra_1B];
    end

    // Slot 2
    always @(posedge clk) begin
        if (wr_slot == 2'd2 && in_valid && in_x < IN_W && in_y < IN_H) begin
            lb2_A[in_x] <= in_rgb;
            lb2_B[in_x] <= in_rgb;
        end
        rd2_A_r <= lb2_A[ra_2A];
        rd2_B_r <= lb2_B[ra_2B];
    end

    // Slot 3
    always @(posedge clk) begin
        if (wr_slot == 2'd3 && in_valid && in_x < IN_W && in_y < IN_H) begin
            lb3_A[in_x] <= in_rgb;
            lb3_B[in_x] <= in_rgb;
        end
        rd3_A_r <= lb3_A[ra_3A];
        rd3_B_r <= lb3_B[ra_3B];
    end

endmodule
