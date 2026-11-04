`timescale 1ns / 1ps
// hwpre_mig_arbiter v3 - weighted transaction arbiter.
//
// M0 (AI/NPU) owns the MIG interface by default. While hwpre_window is high,
// M1 receives one complete single-beat write transaction after m0_slots M0
// opportunities. An M1 grant is only created when m1_request is asserted, so
// an idle HWPRE path never steals cycles from M0.
//
// Both current masters issue one MIG command per beat. M1 is write-only and
// keeps m1_request asserted until app_rdy and app_wdf_rdy accept the beat.
module hwpre_mig_arbiter (
    input  wire         clk,
    input  wire         rst_n,

    // M0: AI_Mate app_*
    input  wire [28:0]  m0_app_addr,
    input  wire [2:0]   m0_app_cmd,
    input  wire         m0_app_en,
    input  wire         m0_app_wdf_wren,
    input  wire [511:0] m0_app_wdf_data,
    input  wire         m0_app_wdf_end,
    input  wire [63:0]  m0_app_wdf_mask,
    output wire         m0_app_rdy,
    output wire         m0_app_wdf_rdy,

    // M1: HWPRE app_*
    input  wire [28:0]  m1_app_addr,
    input  wire [2:0]   m1_app_cmd,
    input  wire         m1_app_en,
    input  wire         m1_app_wdf_wren,
    input  wire [511:0] m1_app_wdf_data,
    input  wire         m1_app_wdf_end,
    input  wire [63:0]  m1_app_wdf_mask,
    output wire         m1_app_rdy,
    output wire         m1_app_wdf_rdy,
    input  wire         m1_request,

    // MIG slave side
    output wire [28:0]  app_addr,
    output wire [2:0]   app_cmd,
    output wire         app_en,
    output wire         app_wdf_wren,
    output wire [511:0] app_wdf_data,
    output wire         app_wdf_end,
    output wire [63:0]  app_wdf_mask,
    input  wire         app_rdy,
    input  wire         app_wdf_rdy,

    // Read data is always consumed by M0 outside this arbiter.
    input  wire [511:0] app_rd_data,
    input  wire         app_rd_data_end,
    input  wire         app_rd_data_valid,

    // Control
    input  wire         hwpre_window,
    input  wire [7:0]   m0_slots
);
    reg       hwpre_window_q;
    reg       grant_m1;
    reg [7:0] m0_slot_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            hwpre_window_q <= 1'b0;
        else
            hwpre_window_q <= hwpre_window;
    end

    wire [7:0] effective_m0_slots = (m0_slots == 8'd0) ? 8'd1 : m0_slots;
    wire owner_m1 = grant_m1 && hwpre_window_q;
    wire m1_accept = owner_m1 && m1_request &&
                     m1_app_en && m1_app_wdf_wren &&
                     app_rdy && app_wdf_rdy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            grant_m1      <= 1'b0;
            m0_slot_count <= 8'd0;
        end else if (!hwpre_window_q) begin
            grant_m1      <= 1'b0;
            m0_slot_count <= 8'd0;
        end else if (grant_m1) begin
            // Keep the grant until the complete single-beat write is accepted.
            if (m1_accept || !m1_request) begin
                grant_m1      <= 1'b0;
                m0_slot_count <= 8'd0;
            end
        end else if (!m1_request) begin
            // No HWPRE beat is waiting, so M0 retains the full interface.
            m0_slot_count <= 8'd0;
        end else begin
            // Count M0 opportunities only while an HWPRE beat is pending.
            if (m0_slot_count + 8'd1 >= effective_m0_slots) begin
                grant_m1      <= 1'b1;
                m0_slot_count <= 8'd0;
            end else begin
                m0_slot_count <= m0_slot_count + 8'd1;
            end
        end
    end

    // Command and write-data signals always switch together.
    assign app_addr     = owner_m1 ? m1_app_addr     : m0_app_addr;
    assign app_cmd      = owner_m1 ? m1_app_cmd      : m0_app_cmd;
    assign app_en       = owner_m1 ? m1_app_en       : m0_app_en;
    assign app_wdf_data = owner_m1 ? m1_app_wdf_data : m0_app_wdf_data;
    assign app_wdf_end  = owner_m1 ? m1_app_wdf_end  : m0_app_wdf_end;
    assign app_wdf_mask = owner_m1 ? m1_app_wdf_mask : m0_app_wdf_mask;
    assign app_wdf_wren = owner_m1 ? m1_app_wdf_wren : m0_app_wdf_wren;

    assign m0_app_rdy     = owner_m1 ? 1'b0 : app_rdy;
    assign m0_app_wdf_rdy = owner_m1 ? 1'b0 : app_wdf_rdy;
    assign m1_app_rdy     = owner_m1 ? app_rdy : 1'b0;
    assign m1_app_wdf_rdy = owner_m1 ? app_wdf_rdy : 1'b0;

    // Keep lint tools quiet; read data is intentionally not multiplexed.
    wire unused_read_data = &{1'b0, app_rd_data[0],
                              app_rd_data_end, app_rd_data_valid};

endmodule
