`timescale 1ns / 1ps

module rotary_encoder #(
    parameter integer CLK_HZ  = 50_000_000,
    parameter integer AB_CYCLES  = CLK_HZ / 1000,   // A / B 消抖 1ms
    parameter integer SW_CYCLES  = CLK_HZ / 50,     // 按键消抖 20ms
    parameter         REVERSE = 1'b0
)(
    input  wire clk,
    input  wire rst_n,
    input  wire enc_a,
    input  wire enc_b,
    input  wire enc_sw,      // 低 = 按下
    output reg  step_cw,     // 顺时针一格
    output reg  step_ccw,    // 逆时针一格
    output reg  press        // 按下瞬间
);

// ---------------- 同步 + 消抖 ----------------
(* ASYNC_REG = "TRUE" *) reg [1:0] a_sync, b_sync, sw_sync;
reg [15:0] a_cnt, b_cnt;
reg [19:0] sw_cnt;
reg        a_st, b_st, sw_st;   // 消抖后电平（sw_st 1 = 松开）

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        a_sync  <= 2'b11;
        b_sync  <= 2'b11;
        sw_sync <= 2'b11;
        a_cnt   <= 16'd0;
        b_cnt   <= 16'd0;
        sw_cnt  <= 20'd0;
        a_st    <= 1'b1;
        b_st    <= 1'b1;
        sw_st   <= 1'b1;
    end else begin
        a_sync  <= {a_sync[0], enc_a};
        b_sync  <= {b_sync[0], enc_b};
        sw_sync <= {sw_sync[0], enc_sw};

        if (a_sync[1] == a_st)                a_cnt <= 16'd0;
        else if (a_cnt == AB_CYCLES - 1) begin a_cnt <= 16'd0; a_st <= a_sync[1]; end
        else                                  a_cnt <= a_cnt + 1'b1;

        if (b_sync[1] == b_st)                b_cnt <= 16'd0;
        else if (b_cnt == AB_CYCLES - 1) begin b_cnt <= 16'd0; b_st <= b_sync[1]; end
        else                                  b_cnt <= b_cnt + 1'b1;

        if (sw_sync[1] == sw_st)               sw_cnt <= 20'd0;
        else if (sw_cnt == SW_CYCLES - 1) begin sw_cnt <= 20'd0; sw_st <= sw_sync[1]; end
        else                                   sw_cnt <= sw_cnt + 1'b1;
    end
end

// ---------------- 正交解码 ----------------
reg [1:0]        ab_prev;
reg signed [3:0] acc;          // 自上次回到 11 以来的净跳变数（+ = cw）
wire [1:0]       ab_now = {a_st, b_st};
reg signed [3:0] delta;

always @(*) begin
    case ({ab_prev, ab_now})
        4'b11_01, 4'b01_00, 4'b00_10, 4'b10_11: delta = 4'sd1;    // A 先落
        4'b11_10, 4'b10_00, 4'b00_01, 4'b01_11: delta = -4'sd1;   // B 先落
        default:                                delta = 4'sd0;    // 不变或两路同时变（丢步）
    endcase
end

wire signed [3:0] acc_nx = acc + delta;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ab_prev  <= 2'b11;
        acc      <= 4'sd0;
        step_cw  <= 1'b0;
        step_ccw <= 1'b0;
        press    <= 1'b0;
    end else begin
        ab_prev  <= ab_now;
        step_cw  <= 1'b0;
        step_ccw <= 1'b0;
        if (ab_now != ab_prev) begin
            if (ab_now == 2'b11) begin
                acc <= 4'sd0;
                if (acc_nx >= 4'sd3)  begin step_cw  <= !REVERSE; step_ccw <= REVERSE;  end
                if (acc_nx <= -4'sd3) begin step_cw  <= REVERSE;  step_ccw <= !REVERSE; end
            end else
                acc <= acc_nx;
        end

        press <= 1'b0;
        if (sw_cnt == SW_CYCLES - 1 && sw_st)   // 即将从松开翻到按下
            press <= 1'b1;
    end
end

endmodule
