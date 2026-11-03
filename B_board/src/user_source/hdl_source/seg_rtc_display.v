`timescale 1ns / 1ps

module seg_rtc_display #(
    parameter integer CLK_HZ       = 50_000_000,
    parameter integer DIGIT_CYCLES = CLK_HZ / 1000,   // 每位 1ms
    parameter integer BLANK_CYCLES = CLK_HZ / 50_000, // 换位消隐 20us
    parameter integer BLINK_CYCLES = CLK_HZ / 3,      // 闪烁亮 / 灭各约 0.33s
    parameter integer IDLE_CYCLES  = CLK_HZ * 10      // 旋钮 10s 不动作隐藏光标
)(
    input  wire       clk,
    input  wire       knob_cw,    // 旋钮顺时针一格
    input  wire       knob_ccw,   // 旋钮逆时针一格
    input  wire       knob_press, // 旋钮按下
    output reg  [7:0] seg_sel,    // [i] = SEL i，低有效
    output reg  [7:0] seg_data,   // {dp, g, f, e, d, c, b, a}，低有效
    output wire [19:0] time_bcd
);

// ---------------- 旋钮选位 / 调值状态 ----------------
reg [2:0]  cur_pos  = 3'd0;    // 0~5 = 时十、时个、分十、分个、秒十、秒个
reg        cur_show = 1'b0;    // 光标可见
reg        editing  = 1'b0;    // 调值中（闪烁、暂停走时）
reg [28:0] idle_cnt = 29'd0;
reg [24:0] blink_cnt = 25'd0;
reg        blink_off = 1'b0;

wire knob_step = knob_cw | knob_ccw;
wire idle_end  = (idle_cnt == IDLE_CYCLES - 1);

always @(posedge clk) begin
    if (knob_step || knob_press)
        idle_cnt <= 29'd0;
    else if (cur_show && !idle_end)
        idle_cnt <= idle_cnt + 1'b1;

    if (knob_press) begin
        cur_show <= 1'b1;
        editing  <= !editing;
    end else if (knob_step) begin
        cur_show <= 1'b1;
        if (!editing && cur_show) begin
            if (knob_cw) cur_pos <= (cur_pos == 3'd5) ? 3'd0 : cur_pos + 1'b1;
            else         cur_pos <= (cur_pos == 3'd0) ? 3'd5 : cur_pos - 1'b1;
        end
    end else if (cur_show && idle_end) begin
        cur_show <= 1'b0;
        editing  <= 1'b0;
    end

    // 进入调值或数值变化时从 "亮" 开始闪，转动时能立刻看到新值
    if (knob_step || knob_press || blink_cnt == BLINK_CYCLES - 1) begin
        blink_cnt <= 25'd0;
        blink_off <= (knob_step || knob_press) ? 1'b0 : !blink_off;
    end else
        blink_cnt <= blink_cnt + 1'b1;
end

// 单个数字位循环加 / 减 1，范围 0~vmax
function [3:0] wrap_step;
    input [3:0] v;
    input [3:0] vmax;
    input       up;
    begin
        if (up) wrap_step = (v >= vmax) ? 4'd0 : v + 1'b1;
        else    wrap_step = (v == 4'd0 || v > vmax) ? vmax : v - 1'b1;
    end
endfunction

// ---------------- 走时 ----------------
reg [25:0] div_cnt = 26'd0;
reg [3:0]  sec_u   = 4'd0;
reg [2:0]  sec_t   = 3'd0;
reg [3:0]  min_u   = 4'd0;
reg [2:0]  min_t   = 3'd0;
reg [3:0]  hour_u  = 4'd0;
reg [1:0]  hour_t  = 2'd0;

assign time_bcd = {hour_t, hour_u, min_t, min_u, sec_t, sec_u};

wire tick     = !editing && (div_cnt == CLK_HZ - 1);
wire sec_end  = (sec_t == 3'd5) && (sec_u == 4'd9);
wire min_end  = (min_t == 3'd5) && (min_u == 4'd9);
wire hour_end = (hour_t == 2'd2) && (hour_u == 4'd3);
wire [3:0] hour_u_max = (hour_t == 2'd2) ? 4'd3 : 4'd9;
wire [3:0] hour_t_nx  = wrap_step({2'b00, hour_t}, 4'd2, knob_cw);

always @(posedge clk) begin
    div_cnt <= (tick || editing) ? 26'd0 : div_cnt + 1'b1;
    if (editing && knob_step) begin
        case (cur_pos)
            3'd0: begin
                hour_t <= hour_t_nx[1:0];
                if (hour_t_nx == 4'd2 && hour_u > 4'd3)   // 时十位调到 2 时个位最大 3
                    hour_u <= 4'd3;
            end
            3'd1:    hour_u <= wrap_step(hour_u, hour_u_max, knob_cw);
            3'd2:    min_t  <= wrap_step({1'b0, min_t}, 4'd5, knob_cw);
            3'd3:    min_u  <= wrap_step(min_u, 4'd9, knob_cw);
            3'd4:    sec_t  <= wrap_step({1'b0, sec_t}, 4'd5, knob_cw);
            default: sec_u  <= wrap_step(sec_u, 4'd9, knob_cw);
        endcase
    end else if (tick) begin
        sec_u <= (sec_u == 4'd9) ? 4'd0 : sec_u + 1'b1;
        if (sec_u == 4'd9)
            sec_t <= (sec_t == 3'd5) ? 3'd0 : sec_t + 1'b1;
        if (sec_end) begin
            min_u <= (min_u == 4'd9) ? 4'd0 : min_u + 1'b1;
            if (min_u == 4'd9)
                min_t <= (min_t == 3'd5) ? 3'd0 : min_t + 1'b1;
            if (min_end) begin
                if (hour_end) begin
                    hour_t <= 2'd0;
                    hour_u <= 4'd0;
                end else if (hour_u == 4'd9) begin
                    hour_t <= hour_t + 1'b1;
                    hour_u <= 4'd0;
                end else
                    hour_u <= hour_u + 1'b1;
            end
        end
    end
end

// ---------------- 段码 ----------------
// 返回 {g, f, e, d, c, b, a}，低有效；4'hf 作为 "-"（只亮 g）
function [6:0] seg7;
    input [3:0] v;
    begin
        case (v)
            4'd0:    seg7 = 7'b100_0000;
            4'd1:    seg7 = 7'b111_1001;
            4'd2:    seg7 = 7'b010_0100;
            4'd3:    seg7 = 7'b011_0000;
            4'd4:    seg7 = 7'b001_1001;
            4'd5:    seg7 = 7'b001_0010;
            4'd6:    seg7 = 7'b000_0010;
            4'd7:    seg7 = 7'b111_1000;
            4'd8:    seg7 = 7'b000_0000;
            4'd9:    seg7 = 7'b001_0000;
            4'hf:    seg7 = 7'b011_1111;
            default: seg7 = 7'b111_1111;
        endcase
    end
endfunction

// ---------------- 扫描 ----------------
reg [15:0] scan_cnt = 16'd0;
reg [2:0]  scan_idx = 3'd0;
reg [3:0]  digit;

// 光标位 0~5 对应扫描位 0、1、3、4、6、7（跳过两个 "-"）
wire [2:0] cur_idx = cur_pos + {1'b0, cur_pos[2:1]};
wire       at_cur  = cur_show && (scan_idx == cur_idx);

always @(*) begin
    case (scan_idx)
        3'd0:    digit = {2'b00, hour_t};
        3'd1:    digit = hour_u;
        3'd3:    digit = {1'b0, min_t};
        3'd4:    digit = min_u;
        3'd6:    digit = {1'b0, sec_t};
        3'd7:    digit = sec_u;
        default: digit = 4'hf;          // 第 3、6 位为 "-"
    endcase
end

always @(posedge clk) begin
    if (scan_cnt == DIGIT_CYCLES - 1) begin
        scan_cnt <= 16'd0;
        scan_idx <= scan_idx + 1'b1;
    end else
        scan_cnt <= scan_cnt + 1'b1;

    seg_sel <= (scan_cnt < BLANK_CYCLES) ? 8'hff : ~(8'd1 << scan_idx);
    if (at_cur && editing)
        seg_data <= blink_off ? 8'hff : {1'b1, seg7(digit)};   // 调值：整位闪烁
    else
        seg_data <= {!(at_cur), seg7(digit)};                   // 选位：光标位小数点亮
end

endmodule
