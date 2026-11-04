`timescale 1ns / 1ps
// 1-deep skid buffer — decouples producer/consumer handshake timing.
//
// Cases:
//   1. Pass-through: buffer empty, out_valid = in_valid, in_ready = out_ready
//   2. Store:        in_valid=1, out_ready=0, buffer empty → store, in_ready=0
//   3. Drain:        out_ready=1, in_valid=0, buffer full → clear, in_ready=1
//   4. Swap:         in_valid=1, out_ready=1, buffer full → store new, in_ready=1

module skid_buffer (
    input  wire         clk, rst_n,
    input  wire         frame_active,    // 1 = CAPTURE/FLUSH, 0 = idle → clear buffer
    input  wire         in_valid,
    output wire         in_ready,
    input  wire [9:0]   in_x,
    input  wire [8:0]   in_y,
    input  wire [23:0]  in_rgb,
    output wire         out_valid,
    input  wire         out_ready,
    output wire [9:0]   out_x,
    output wire [8:0]   out_y,
    output wire [23:0]  out_rgb
);

    reg        buf_valid;
    reg [9:0]  buf_x;
    reg [8:0]  buf_y;
    reg [23:0] buf_rgb;

    // Output always valid when buffer has data OR input is valid
    assign out_valid = buf_valid || in_valid;
    assign out_x     = buf_valid ? buf_x   : in_x;
    assign out_y     = buf_valid ? buf_y   : in_y;
    assign out_rgb   = buf_valid ? buf_rgb : in_rgb;

    // Producer can send when buffer empty OR being drained this cycle
    assign in_ready  = !buf_valid || out_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buf_valid <= 0;
        end else if (!frame_active) begin
            buf_valid <= 0;
        end else begin
            if (in_valid && in_ready && !buf_valid && !out_ready) begin
                // Case 2: Store — input valid, output not consumed, buffer empty
                buf_valid <= 1;
                buf_x <= in_x;
                buf_y <= in_y;
                buf_rgb <= in_rgb;
            end else if (in_valid && in_ready && buf_valid && out_ready) begin
                // Case 4: Swap — consume from buffer, store new input
                buf_valid <= 1;
                buf_x <= in_x;
                buf_y <= in_y;
                buf_rgb <= in_rgb;
            end else if (out_valid && out_ready && buf_valid) begin
                // Case 3: Drain — consume from buffer, no new input (or pass-through)
                if (!in_valid || !in_ready) begin
                    buf_valid <= 0;
                end
            end
            // Case 1: Pass-through (buf_valid=0, in_valid=1, out_ready=1): no state change
        end
    end

endmodule
