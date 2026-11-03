`timescale 1ns / 1ps

module imu_alarm_led #(
    parameter integer CLK_HZ         = 50_000_000,
    parameter integer THRESH_DEG_X10 = 300,   // 30.0 度
    parameter integer HOLD_TICKS     = 500,
    parameter integer BLINK_TICKS    = 20     // 闪烁半周期（10ms 单位），0.2s
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire               attitude_valid,
    input  wire               calibrated,
    input  wire signed [15:0] roll_deg_x10,
    input  wire signed [15:0] pitch_deg_x10,
    input  wire signed [15:0] yaw_deg_x10,
    output reg  [3:0]         led            // {LED4, LED3(Z), LED2(Y), LED1(X)}
);

localparam integer TICK_CYCLES = CLK_HZ / 100;   // 10ms

function over_th;
    input signed [15:0] v;
    begin
        over_th = (v > THRESH_DEG_X10) || (v < -THRESH_DEG_X10);
    end
endfunction

wire       judge    = attitude_valid && calibrated;
wire [2:0] over_now = {over_th(yaw_deg_x10), over_th(pitch_deg_x10), over_th(roll_deg_x10)} & {3{judge}};

reg [18:0] tick_cnt;
reg        tick;          // 每 10ms 一拍
reg [8:0]  hold [0:2];    // 各轴剩余保持时间，非 0 即该轴处于超限
reg [4:0]  blink_cnt;
reg        blink_on;      // 闪烁相位：1 = 亮

wire [2:0] hold_nz = {hold[2] != 9'd0, hold[1] != 9'd0, hold[0] != 9'd0};

integer a;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tick_cnt  <= 19'd0;
        tick      <= 1'b0;
        for (a = 0; a < 3; a = a + 1)
            hold[a] <= 9'd0;
        blink_cnt <= 5'd0;
        blink_on  <= 1'b1;
        led       <= 4'd0;
    end else begin
        tick     <= (tick_cnt == TICK_CYCLES - 1);
        tick_cnt <= (tick_cnt == TICK_CYCLES - 1) ? 19'd0 : tick_cnt + 1'b1;

        for (a = 0; a < 3; a = a + 1) begin
            if (over_now[a])
                hold[a] <= HOLD_TICKS;
            else if (tick && (hold[a] != 9'd0))
                hold[a] <= hold[a] - 1'b1;
        end

        // 没有任何轴超限时闪烁相位停在"亮"，保证新报警第一下立即亮起
        if (hold_nz == 3'b000) begin
            blink_cnt <= 5'd0;
            blink_on  <= 1'b1;
        end else if (tick) begin
            if (blink_cnt == BLINK_TICKS - 1) begin
                blink_cnt <= 5'd0;
                blink_on  <= !blink_on;
            end else
                blink_cnt <= blink_cnt + 1'b1;
        end

        led <= {1'b0, hold_nz & {3{enable && blink_on}}};
    end
end

endmodule
