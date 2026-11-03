`timescale 1ns / 1ps

module page_chime #(
    parameter integer NOTE2_DELAY   = 6000,          // 流程页 0.125s 后第 2 音
    parameter integer DONG_DELAY    = 12000,         // 0.25s 后响“咚”（流程页第 3 音）
    parameter integer TOTAL_SAMPLES = 96000,         // 2s 后结束（此时“咚”已衰减到约 -45dB）
    parameter integer DING_DECAY    = 13,
    parameter integer DONG_DECAY    = 14             // 约 0.34s
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        trig_toggle,
    input  wire [2:0]  page_async,
    input  wire        valid_in,
    input  wire [23:0] left_in,
    input  wire [23:0] right_in,
    output reg         valid_out,
    output reg  [23:0] left_out,
    output reg  [23:0] right_out
);

// 相位增量：f * 2^20 / 48000
localparam [19:0] INC_C6 = 20'd22861;   // 1046.5Hz
localparam [19:0] INC_D6 = 20'd25661;   // 1174.7Hz
localparam [19:0] INC_E6 = 20'd28803;   // 1318.5Hz
localparam [19:0] INC_F6 = 20'd30516;   // 1396.9Hz
localparam [19:0] INC_G6 = 20'd34253;   // 1568.0Hz
localparam [19:0] INC_A6 = 20'd38448;   // 1760.0Hz
localparam [19:0] INC_B6 = 20'd43156;   // 1975.5Hz
localparam [19:0] INC_C7 = 20'd45722;   // 2093.0Hz

(* ASYNC_REG = "TRUE" *) reg [1:0] trig_sync;
(* ASYNC_REG = "TRUE" *) reg [2:0] page_meta;
(* ASYNC_REG = "TRUE" *) reg [2:0] page_sync;
reg        trig_d;
reg        active;
reg [2:0]  page;                    // 本次播放对应的页
reg        first;                   // 流程页“叮”声部还在第 1 音
reg        dong_on;
reg [16:0] sample_cnt;
reg [19:0] ding_phase, dong_phase;
reg [23:0] ding_env, dong_env;
reg        valid_d1;
reg signed [24:0] chime;            // 两音之和
reg signed [23:0] left_d1, right_d1;

// 三角波：相位前半段上升、后半段下降，输出有符号 16bit
function signed [15:0] tri16;
    input [19:0] phase;
    begin
        tri16 = (phase[19] ? ~phase[18:3] : phase[18:3]) ^ 16'h8000;
    end
endfunction

function [19:0] ding_inc_of;
    input [2:0] p;
    input       f;
    begin
        case (p)
            3'd2:    ding_inc_of = f ? INC_C6 : INC_E6;   // C6 -> E6
            3'd3:    ding_inc_of = f ? INC_A6 : INC_F6;   // A6 -> F6
            3'd4:    ding_inc_of = f ? INC_E6 : INC_C7;   // E6 -> C7
            3'd5:    ding_inc_of = f ? INC_G6 : INC_E6;   // G6 -> E6
            default: ding_inc_of = INC_E6;
        endcase
    end
endfunction

function [19:0] dong_inc_of;
    input [2:0] p;
    begin
        case (p)
            3'd2:    dong_inc_of = INC_G6;   // 页 2 收在 G6
            3'd3:    dong_inc_of = INC_D6;   // 页 3 收在 D6
            3'd4:    dong_inc_of = INC_A6;   // 页 4 收在 A6
            3'd5:    dong_inc_of = INC_C7;   // 页 5 收在 C7
            default: dong_inc_of = INC_C6;
        endcase
    end
endfunction

wire [19:0]        ding_inc = ding_inc_of(page, first);
wire [19:0]        dong_inc = dong_inc_of(page);
wire               trig     = (trig_sync[1] != trig_d);
wire               new_flow = page_sync[2] ^ page_sync[1];   // 页 2~5

wire signed [15:0] ding_tri = tri16(ding_phase);
wire signed [15:0] dong_tri = tri16(dong_phase);
wire signed [24:0] ding_mul = ding_tri * $signed({1'b0, ding_env[23:16]});
wire signed [24:0] dong_mul = dong_tri * $signed({1'b0, dong_env[23:16]});

wire signed [25:0] sum_l = left_d1  + chime;
wire signed [25:0] sum_r = right_d1 + chime;

function [23:0] sat24;
    input signed [25:0] v;
    begin
        if (v[25:23] == 3'b000 || v[25:23] == 3'b111)
            sat24 = v[23:0];
        else
            sat24 = v[25] ? 24'h800000 : 24'h7FFFFF;
    end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        trig_sync  <= 2'b00;
        page_meta  <= 3'd0;
        page_sync  <= 3'd0;
        trig_d     <= 1'b0;
        active     <= 1'b0;
        page       <= 3'd0;
        first      <= 1'b0;
        dong_on    <= 1'b0;
        sample_cnt <= 17'd0;
        ding_phase <= 20'd0;
        dong_phase <= 20'd0;
        ding_env   <= 24'd0;
        dong_env   <= 24'd0;
        valid_d1   <= 1'b0;
        chime      <= 25'sd0;
        left_d1    <= 24'sd0;
        right_d1   <= 24'sd0;
        valid_out  <= 1'b0;
        left_out   <= 24'd0;
        right_out  <= 24'd0;
    end else begin
        trig_sync <= {trig_sync[0], trig_toggle};
        trig_d    <= trig_sync[1];
        page_meta <= page_async;
        page_sync <= page_meta;

        // 第 1 拍：锁存输入采样，算本采样的提示音（每音峰值约 2M）
        valid_d1 <= valid_in;
        if (valid_in) begin
            left_d1  <= left_in;
            right_d1 <= right_in;
            chime    <= active ? ((ding_mul >>> 2) + (dong_mul >>> 2)) : 25'sd0;

            if (active) begin
                ding_phase <= ding_phase + ding_inc;
                ding_env   <= ding_env - (ding_env >> DING_DECAY);
                if (dong_on) begin
                    dong_phase <= dong_phase + dong_inc;
                    dong_env   <= dong_env - (dong_env >> DONG_DECAY);
                end
                if (first && sample_cnt == NOTE2_DELAY - 1) begin
                    ding_env <= 24'hFFFFFF;     // 流程页第 2 音重新起音，相位连续
                    first    <= 1'b0;
                end
                if (sample_cnt == DONG_DELAY - 1) begin
                    dong_env <= 24'hFFFFFF;
                    dong_on  <= 1'b1;
                end
                if (sample_cnt == TOTAL_SAMPLES - 1)
                    active <= 1'b0;
                sample_cnt <= sample_cnt + 17'd1;
            end
        end

        if (trig) begin
            active     <= 1'b1;
            page       <= page_sync;
            first      <= new_flow;
            dong_on    <= 1'b0;
            sample_cnt <= 17'd0;
            ding_phase <= 20'd0;
            dong_phase <= 20'd0;
            ding_env   <= 24'hFFFFFF;
            dong_env   <= 24'd0;
        end

        // 第 2 拍：叠加并饱和
        valid_out <= valid_d1;
        if (valid_d1) begin
            left_out  <= sat24(sum_l);
            right_out <= sat24(sum_r);
        end
    end
end

endmodule
