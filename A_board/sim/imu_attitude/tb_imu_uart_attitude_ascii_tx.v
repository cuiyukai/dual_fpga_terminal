`timescale 1ns / 1ps

module tb_imu_uart_attitude_ascii_tx;
reg clk = 0;
reg rst_n = 0;
reg attitude_valid = 0;
reg signed [15:0] roll_input = 16'sd123;
reg signed [15:0] pitch_input = -16'sd45;
reg signed [15:0] yaw_input = 16'sd0;
wire tx;
wire busy;
integer i;
reg [7:0] actual [0:45];

always #10 clk = ~clk;

imu_uart_attitude_ascii_tx dut (
    .clk(clk), .rst_n(rst_n), .attitude_valid(attitude_valid),
    .calibrated(1'b1), .roll_deg_x10(roll_input),
    .pitch_deg_x10(pitch_input), .yaw_deg_x10(yaw_input),
    .tx(tx), .busy(busy)
);

initial begin
    #1;
    dut.roll_latched = 16'sd123;
    dut.pitch_latched = -16'sd45;
    dut.yaw_latched = 16'sd0;
    dut.roll_bcd = 16'h0123;
    dut.pitch_bcd = 16'h0045;
    dut.yaw_bcd = 16'h0000;
    for (i = 0; i < 46; i = i + 1)
        actual[i] = dut.frame_byte(i);
    for (i = 0; i < 46; i = i + 1)
        $write("%c", actual[i]);
    if (actual[13] != "g" || actual[29] != "g" || actual[43] != "g")
        $fatal(1, "missing angle unit");
    if (actual[44] != 8'h0d || actual[45] != 8'h0a)
        $fatal(1, "missing line ending");
    repeat (3) @(negedge clk);
    rst_n = 1;
    @(negedge clk);
    attitude_valid = 1;
    @(negedge clk);
    attitude_valid = 0;
    repeat (100) @(negedge clk);
    if (dut.roll_bcd != 16'h0123 || dut.pitch_bcd != 16'h0045 ||
        dut.yaw_bcd != 16'h0000)
        $fatal(1, "serial decimal conversion failed");
    $finish;
end
endmodule
