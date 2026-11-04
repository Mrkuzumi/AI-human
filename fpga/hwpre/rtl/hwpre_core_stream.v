`timescale 1ns / 1ps
// hwpre_core_stream — thin wrapper: resize_stream -> skid -> letterbox -> packer.
// No GP1/ImageMake register assumptions. Produces 512-bit tensor beat stream.
module hwpre_core_stream #(
    parameter IN_W=1920, IN_H=1080, RESIZE_W=910, RESIZE_H=512, OUT_W=960, OUT_H=512,
    parameter PAD_LEFT=25, PAD_RIGHT=25, PAD_TOP=0, PAD_BOTTOM=0,
    parameter [7:0] PAD_VALUE = 8'd114,
    parameter LINE_BUF_COUNT=4
) (
    input  wire         clk, rst_n,
    input  wire         frame_active,    // 1 = CAPTURE/FLUSH, 0 = idle
    input  wire         in_valid,
    input  wire [10:0]  in_x, in_y,
    input  wire [23:0]  in_rgb,
    output wire         beat_valid,
    output wire [511:0] beat_data,
    output wire         beat_last,
    input  wire         beat_ready
);
    wire        resize_valid, skid_ready;
    wire [9:0]  resize_x;
    wire [9:0]  resize_y;  // v20: 10-bit to match resize out_y fix
    wire [23:0] resize_rgb;

    resize_bilinear_stream_core #(.IN_W(IN_W),.IN_H(IN_H),.OUT_W(RESIZE_W),.OUT_H(RESIZE_H),
        .FRAC_BITS(14),.LINE_BUF_COUNT(LINE_BUF_COUNT)) u_resize (
        .clk,.rst_n,.frame_active(frame_active),.in_valid,.in_x,.in_y,.in_rgb,
        .out_valid(resize_valid),.out_ready(skid_ready),.out_x(resize_x),.out_y(resize_y),.out_rgb(resize_rgb),
        .overflow()
    );

    wire skid_valid, lb_ready;
    wire [9:0] skid_x; wire [8:0] skid_y; wire [23:0] skid_rgb;
    skid_buffer u_skid (.clk,.rst_n,
        .frame_active(frame_active),
        .in_valid(resize_valid),.in_ready(skid_ready),.in_x(resize_x),.in_y(resize_y),.in_rgb(resize_rgb),
        .out_valid(skid_valid),.out_ready(lb_ready),.out_x(skid_x),.out_y(skid_y),.out_rgb(skid_rgb));

    wire lb_valid; wire [9:0] lb_x; wire [8:0] lb_y; wire [23:0] lb_rgb;
    letterbox_pad_core #(.OUT_W(OUT_W),.OUT_H(OUT_H),.IN_W(RESIZE_W),.IN_H(RESIZE_H),
        .PAD_LEFT(PAD_LEFT),.PAD_RIGHT(PAD_RIGHT),.PAD_TOP(PAD_TOP),.PAD_BOTTOM(PAD_BOTTOM),
        .PAD_VALUE(PAD_VALUE)) u_lb (.clk,.rst_n,
        .frame_active(frame_active),
        .in_valid(skid_valid),.in_ready(lb_ready),.in_x(skid_x),.in_y(skid_y),.in_rgb(skid_rgb),
        .out_valid(lb_valid),.out_x(lb_x),.out_y(lb_y),.out_rgb(lb_rgb));

    // Packer → FIFO internal connections
    wire        packer_beat_valid, packer_beat_last;
    wire [511:0] packer_beat_data;
    wire        packer_beat_ready;

    nhwc_stream_packer #(.OUT_W(OUT_W),.OUT_H(OUT_H),.BYTES_PER_PIX(3),.BEAT_BYTES(64),
        .GAP_THRESH(65535)) u_packer (.clk,.rst_n,
        .frame_active(frame_active),
        .in_valid(lb_valid),.in_ready(),.in_x(lb_x),.in_y(lb_y),.in_rgb(lb_rgb),
        .beat_ready(packer_beat_ready),.beat_valid(packer_beat_valid),.beat_data(packer_beat_data),
        .beat_last(packer_beat_last),.frame_done());

    // Beat FIFO — absorbs MIG backpressure bursts
    beat_fifo_xpm #(.FIFO_DEPTH(128)) u_fifo (
        .clk        (clk),
        .rst_n      (rst_n),
        .wr_valid   (packer_beat_valid),
        .wr_ready   (packer_beat_ready),
        .wr_data    (packer_beat_data),
        .wr_last    (packer_beat_last),
        .rd_valid   (beat_valid),
        .rd_ready   (beat_ready),
        .rd_data    (beat_data),
        .rd_last    (beat_last)
    );
endmodule
