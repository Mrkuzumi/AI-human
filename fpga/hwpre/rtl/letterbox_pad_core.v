`timescale 1ns / 1ps
// Letterbox v7 — with frame_active gate to prevent inter-frame pad leakage.
//
// frame_active = 1 during FSM CAPTURE/FLUSH states, 0 otherwise.
// When frame_active=0: no output, pos frozen.
// On frame_active 0→1 edge: reset pos/cur_y for new frame.
//
// Row sequence (per row):
//   1. Pad-left:  output PAD_VALUE, in_ready=0 (PAD_LEFT cycles)
//   2. Data:      output resize pixels, in_ready=1 (IN_W cycles)
//   3. Pad-right: output PAD_VALUE, in_ready=0 (PAD_RIGHT cycles)
//   After pad-right, pos wraps to 0 for next row.

module letterbox_pad_core #(
    parameter OUT_W=960, OUT_H=512, IN_W=910, IN_H=512,
    parameter PAD_LEFT=25, PAD_RIGHT=25, PAD_TOP=0, PAD_BOTTOM=0,
    parameter [7:0] PAD_VALUE = 8'd114
) (
    input  wire         clk, rst_n,
    input  wire         frame_active,    // 1 = frame in progress, 0 = idle
    input  wire         in_valid,
    output reg          in_ready,
    input  wire [9:0]   in_x,
    input  wire [8:0]   in_y,
    input  wire [23:0]  in_rgb,
    output reg          out_valid,
    output reg  [9:0]   out_x,
    output reg  [8:0]   out_y,
    output reg  [23:0]  out_rgb
);

    reg [9:0]  pos;           // current output column (0..OUT_W-1)
    reg [8:0]  cur_y;         // current row

    // in_ready: accept input only during data (pass-through) region
    wire in_data_region = (pos >= PAD_LEFT) && (pos < PAD_LEFT + IN_W);
    always @(*) in_ready = in_data_region;

    // Detect frame_active rising edge for reset
    reg frame_active_q;
    wire frame_start = frame_active && !frame_active_q;

    // Stage 0: compute next output
    reg        s0_valid;
    reg [9:0]  s0_x;
    reg [8:0]  s0_y;
    reg [23:0] s0_rgb;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos <= 0; cur_y <= 0;
            s0_valid <= 0; s0_x <= 0; s0_y <= 0; s0_rgb <= 0;
            frame_active_q <= 0;
        end else begin
            frame_active_q <= frame_active;

            if (frame_start) begin
                // New frame: reset position
                pos <= 0; cur_y <= 0; s0_valid <= 0;
            end else if (!frame_active) begin
                // Between frames: suppress all output
                s0_valid <= 0;
            end else begin
                s0_valid <= 0;
                if (in_valid) cur_y <= in_y;

                if (pos < PAD_LEFT) begin
                    s0_valid <= 1; s0_x <= pos; s0_y <= cur_y;
                    s0_rgb <= {PAD_VALUE, PAD_VALUE, PAD_VALUE};
                    pos <= pos + 1;
                end else if (pos < PAD_LEFT + IN_W) begin
                    if (in_valid && in_ready) begin
                        s0_valid <= 1; s0_x <= pos; s0_y <= in_y; s0_rgb <= in_rgb;
                        pos <= pos + 1;
                    end
                end else begin
                    s0_valid <= 1; s0_x <= pos; s0_y <= cur_y;
                    s0_rgb <= {PAD_VALUE, PAD_VALUE, PAD_VALUE};
                    if (pos == OUT_W - 1) pos <= 0;
                    else pos <= pos + 1;
                end
            end
        end
    end

    // Stage 1: registered output (1-cycle delay for valid↔data alignment)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 0;
            out_x <= 0;
            out_y <= 0;
            out_rgb <= 0;
        end else begin
            out_valid <= s0_valid;
            out_x     <= s0_x;
            out_y     <= s0_y;
            out_rgb   <= s0_rgb;
        end
    end

endmodule
