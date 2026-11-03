`timescale 1ns / 1ps

module test_scorer #(
    parameter integer CLK_HZ     = 50_000_000,
    parameter integer WAIT_S     = 5,
    parameter integer MEASURE_S  = 10,
    parameter [0:0]   ROLL_FLIP  = 1'b0,
    parameter [0:0]   PITCH_FLIP = 1'b0
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               test_mode,       // 实体检测模式电平
    input  wire               restart,         // 单拍：陀螺仪重新消零，检测从等待重新开始
    input  wire               calibrated,
    input  wire               attitude_valid,
    input  wire signed [15:0] roll_deg_x10,
    input  wire signed [15:0] pitch_deg_x10,
    output reg  [1:0]         phase,
    output wire [3:0]         secs,
    output wire [6:0]         score,
    output wire [1:0]         tilt_dir,
    output reg  [3:0]         tilt_tens,
    output reg  [3:0]         tilt_ones,
    output reg  [3:0]         tilt_tenths
);

localparam [1:0] P_IDLE    = 2'd0;
localparam [1:0] P_WAIT    = 2'd1;
localparam [1:0] P_MEASURE = 2'd2;
localparam [1:0] P_RESULT  = 2'd3;

localparam integer SEC_CYCLES = CLK_HZ;
localparam integer SEC_WIDTH  = $clog2(SEC_CYCLES + 1);

reg [SEC_WIDTH-1:0] tick_cnt;
reg [3:0]           elapsed;       // 本阶段已过整秒数
reg [15:0]          max_tilt;      // 检测阶段最大倾角（0.1 度）
reg                 calc;
reg [15:0]          calc_rest;
reg [6:0]           score_acc;
reg                 tilt_axis;     // 最大倾角所在轴：0 roll，1 pitch
reg                 tilt_neg;      // 该轴角度为负
reg [9:0]           bcd_rest;      // 幅度转 BCD 的余数

assign score = (phase == P_RESULT) ? score_acc : 7'd0;

wire sec_tick = (tick_cnt == SEC_CYCLES - 1);

function [15:0] abs16;
    input signed [15:0] v;
    abs16 = v[15] ? -v : v;
endfunction

wire [15:0] roll_abs  = abs16(roll_deg_x10);
wire [15:0] pitch_abs = abs16(pitch_deg_x10);
wire [15:0] tilt      = (roll_abs > pitch_abs) ? roll_abs : pitch_abs;
wire        axis_now  = !(roll_abs > pitch_abs);
wire        neg_now   = axis_now ? pitch_deg_x10[15] : roll_deg_x10[15];

assign tilt_dir = {tilt_axis, ~tilt_neg ^ (tilt_axis ? PITCH_FLIP : ROLL_FLIP)};

wire score_fin = (calc_rest < 16'd5) || (score_acc == 7'd0);
wire bcd_fin   = (bcd_rest < 10'd10);

assign secs = (phase == P_WAIT)    ? ((elapsed >= WAIT_S - 1) ? 4'd1 : WAIT_S - elapsed) :
              (phase == P_MEASURE)  ? ((elapsed >= MEASURE_S) ? 4'd1 : MEASURE_S - elapsed) : 4'd0;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        phase     <= P_IDLE;
        tick_cnt  <= {SEC_WIDTH{1'b0}};
        elapsed   <= 4'd0;
        max_tilt  <= 16'd0;
        calc      <= 1'b0;
        calc_rest <= 16'd0;
        score_acc <= 7'd0;
        tilt_axis <= 1'b0;
        tilt_neg  <= 1'b0;
        bcd_rest  <= 10'd0;
        tilt_tens   <= 4'd0;
        tilt_ones   <= 4'd0;
        tilt_tenths <= 4'd0;
    end else if (!test_mode) begin
        phase    <= P_IDLE;
        calc     <= 1'b0;
    end else if (restart || phase == P_IDLE) begin
        phase    <= P_WAIT;
        tick_cnt <= {SEC_WIDTH{1'b0}};
        elapsed  <= 4'd0;
        calc     <= 1'b0;
    end else begin
        tick_cnt <= sec_tick ? {SEC_WIDTH{1'b0}} : tick_cnt + 1'b1;
        if (sec_tick && elapsed != 4'd15)
            elapsed <= elapsed + 1'b1;

        case (phase)
            P_WAIT: begin
                if (elapsed >= WAIT_S && calibrated && attitude_valid) begin
                    phase    <= P_MEASURE;
                    tick_cnt <= {SEC_WIDTH{1'b0}};
                    elapsed  <= 4'd0;
                    max_tilt  <= tilt;
                    tilt_axis <= axis_now;
                    tilt_neg  <= neg_now;
                end
            end
            P_MEASURE: begin
                if (calc) begin
                    if (!score_fin) begin
                        calc_rest <= calc_rest - 16'd5;
                        score_acc <= score_acc - 1'b1;
                    end
                    if (bcd_rest >= 10'd100) begin
                        bcd_rest  <= bcd_rest - 10'd100;
                        tilt_tens <= tilt_tens + 1'b1;
                    end else if (!bcd_fin) begin
                        bcd_rest  <= bcd_rest - 10'd10;
                        tilt_ones <= tilt_ones + 1'b1;
                    end
                    if (score_fin && bcd_fin) begin
                        calc        <= 1'b0;
                        phase       <= P_RESULT;
                        tilt_tenths <= bcd_rest[3:0];
                    end
                end else if (elapsed == MEASURE_S) begin
                    calc      <= 1'b1;
                    calc_rest <= max_tilt;
                    score_acc <= 7'd100;
                    bcd_rest  <= (max_tilt > 16'd999) ? 10'd999 : max_tilt[9:0];
                    tilt_tens <= 4'd0;
                    tilt_ones <= 4'd0;
                end else if (attitude_valid && tilt > max_tilt) begin
                    max_tilt  <= tilt;
                    tilt_axis <= axis_now;
                    tilt_neg  <= neg_now;
                end
            end
            default: ;
        endcase
    end
end

endmodule
