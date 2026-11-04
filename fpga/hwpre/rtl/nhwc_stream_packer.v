`timescale 1ns / 1ps
// nhwc_stream_packer v8 — Vivado 2018.3 compatible (no unpacked array function params)
// Uses generate-for + packed wire instead of function with unpacked array args.
//
// Stage 0: pixel → bbuf + immediate beat capture on fill
// Stage 1: beat holding → handshake

module nhwc_stream_packer #(
    parameter OUT_W=960, OUT_H=512, BYTES_PER_PIX=3, BEAT_BYTES=64,
    parameter TOTAL_PIX = OUT_W * OUT_H,  // override for testing
    parameter GAP_THRESH = 65535           // gap-timer flush threshold (65535 = disabled)
) (
    input  wire         clk, rst_n,
    input  wire         in_valid,
    input  wire         frame_active,   // 1 = CAPTURE/FLUSH, 0 = idle → discard residual
    output wire         in_ready,
    input  wire [9:0]   in_x, in_y,
    input  wire [23:0]  in_rgb,
    input  wire         beat_ready,
    output reg          beat_valid,
    output reg  [511:0] beat_data,
    output reg          beat_last,
    output reg          frame_done
);

    // Unpacked byte buffer (Vivado-friendly for indexed byte access)
    reg [7:0]  bbuf [0:BEAT_BYTES-1];
    reg [5:0]  idx;

    reg        beat_holding;
    reg [511:0] hold_data;
    reg        hold_last;

    reg [31:0] pix_cnt;
    reg        all_pix_done;
    reg [15:0] gap_cnt;      // consecutive idle cycles before flush (widened for streaming)
    reg        started;       // first pixel received
    reg        beat_emitted;  // at least one full beat sent (suppress early flush)
    integer    j;

    // in_ready: always accept
    assign in_ready = 1'b1;

    // ============================================================
    // Generate: map bbuf unpacked → packed wire for easy beat building
    // Each bbuf[i] drives bbuf_packed[i*8 +: 8] continuously.
    // Since bbuf is updated with NBA in the always block, bbuf_packed
    // reflects the "pre-update" value during the same clock cycle's
    // always block evaluation — which is exactly what we want for
    // beat capture (new pixel bytes override positions explicitly).
    // ============================================================
    wire [511:0] bbuf_packed;
    genvar gi;
    generate
        for (gi = 0; gi < 64; gi = gi + 1) begin : gen_bbuf_pack
            assign bbuf_packed[gi*8 +: 8] = bbuf[gi];
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            beat_valid <= 0; beat_data <= 0; beat_last <= 0; frame_done <= 0;
            idx <= 0; pix_cnt <= 0; beat_holding <= 0;
            hold_data <= 0; hold_last <= 0;
            all_pix_done <= 0; gap_cnt <= 0; started <= 0; beat_emitted <= 0;
            for (j = 0; j < BEAT_BYTES; j = j + 1) bbuf[j] <= 0;
        end else if (!frame_active) begin
            // Between frames: discard any leaked bytes, reset state
            idx <= 0; started <= 0; beat_emitted <= 0;
            beat_holding <= 0; beat_valid <= 0;
            all_pix_done <= 0; gap_cnt <= 0; pix_cnt <= 0;
            hold_last <= 0;
            for (j = 0; j < BEAT_BYTES; j = j + 1) bbuf[j] <= 0;
        end else begin
            frame_done <= 0;

            // ======================================================
            // STAGE 1: Output handshake
            //
            // Two scenarios for handshake completion (beat_ready=1):
            //   (a) beat_valid was already 1 (stall): deassert now
            //   (b) beat_valid is still 0 (new beat): assert for 1 cycle
            // In both cases, beat_holding clears and the else branch
            // handles deassertion on the following cycle.
            // ======================================================
            if (beat_holding) begin
                beat_data   <= hold_data;
                beat_last   <= hold_last;
                if (beat_ready) begin
                    // Handshake completes this cycle
                    if (beat_valid) begin
                        // Already asserted (backpressure case) → deassert
                        beat_valid <= 0;
                    end else begin
                        // Not yet asserted (new beat) → assert for 1 cycle
                        beat_valid <= 1;
                    end
                    beat_holding <= 0;
                    if (hold_last) begin
                        frame_done <= 1;
                        all_pix_done <= 0;
                        pix_cnt <= 0;
                    end
                end else begin
                    // Waiting: keep valid asserted
                    beat_valid <= 1;
                end
            end else begin
                beat_valid <= 0;
            end

            // ======================================================
            // STAGE 0: Process input pixel, capture beat on fill
            // ======================================================
            if (in_valid && in_ready) begin
                if (!started) started <= 1;
                pix_cnt <= pix_cnt + 1;

                if (idx + 3 < BEAT_BYTES) begin
                    // Case A1: 3 bytes fit, no beat fill (idx <= 60)
                    bbuf[idx+0] <= in_rgb[23:16];  // R
                    bbuf[idx+1] <= in_rgb[15:8];   // G
                    bbuf[idx+2] <= in_rgb[7:0];    // B
                    idx <= idx + 3;

                end else if (idx + 3 == BEAT_BYTES) begin
                    // Case A2: exact fill at idx=61
                    bbuf[61] <= in_rgb[23:16];
                    bbuf[62] <= in_rgb[15:8];
                    bbuf[63] <= in_rgb[7:0];
                    hold_data <= {in_rgb[7:0], in_rgb[15:8], in_rgb[23:16], bbuf_packed[487:0]};
                    hold_last <= (pix_cnt == TOTAL_PIX - 1);
                    beat_holding <= 1;
                    idx <= 0;
                    beat_emitted <= 1;

                end else if (idx + 2 <= BEAT_BYTES) begin
                    // Case B: 2 bytes fit at idx=62, 1 overflow
                    bbuf[62] <= in_rgb[23:16];
                    bbuf[63] <= in_rgb[15:8];
                    hold_data <= {in_rgb[15:8], in_rgb[23:16], bbuf_packed[495:0]};
                    hold_last <= 1'b0;
                    beat_holding <= 1;
                    beat_emitted <= 1;
                    bbuf[0] <= in_rgb[7:0];   // B
                    idx <= 1;

                end else begin
                    // Case C: 1 byte fits at idx=63, 2 overflow
                    bbuf[63] <= in_rgb[23:16];
                    hold_data <= {in_rgb[23:16], bbuf_packed[503:0]};
                    hold_last <= 1'b0;
                    beat_holding <= 1;
                    beat_emitted <= 1;
                    bbuf[0] <= in_rgb[15:8];  // G
                    bbuf[1] <= in_rgb[7:0];   // B
                    idx <= 2;
                end

                if (pix_cnt == TOTAL_PIX - 1) all_pix_done <= 1;
            end

            // ======================================================
            // Gap counter: tracks consecutive idle cycles
            // ======================================================
            if (!in_valid) begin
                if (gap_cnt < GAP_THRESH) gap_cnt <= gap_cnt + 1;
            end else begin
                gap_cnt <= 0;
            end

            // ======================================================
            // FRAME END: flush remaining bytes in bbuf
            // Only fires after sustained idle (GAP_THRESH cycles).
            // Brief gaps during row transitions won't trigger flush.
            // ======================================================
            // Immediate flush: all expected pixels received, data remains in bbuf
            if (started && all_pix_done && !beat_holding && idx > 0) begin
                hold_data   <= bbuf_packed;
                hold_last   <= 1'b1;
                beat_holding <= 1;
                idx <= 0;
            // Gap-timer flush: sustained idle after at least one full beat emitted
            end else if (started && beat_emitted && !in_valid && !beat_holding && idx > 0 && gap_cnt >= GAP_THRESH) begin
                hold_data   <= bbuf_packed;
                hold_last   <= 1'b1;
                beat_holding <= 1;
                idx <= 0;
            end
        end
    end

endmodule
