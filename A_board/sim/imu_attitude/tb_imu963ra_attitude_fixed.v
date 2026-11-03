`timescale 1ns / 1ps

module tb_imu963ra_attitude_fixed;
reg clk = 0;
reg rst_n = 0;
reg sample_valid = 0;
reg calibrated = 1;
reg signed [15:0] ax = 0;
reg signed [15:0] ay = 0;
reg signed [15:0] az = 4098;
reg signed [15:0] gx = 0;
reg signed [15:0] gy = 0;
reg signed [15:0] gz = 0;
wire signed [15:0] roll_deg_x10;
wire signed [15:0] pitch_deg_x10;
wire signed [15:0] yaw_deg_x10;
wire attitude_valid;
integer i;

always #10 clk = ~clk;

imu963ra_attitude_fixed dut (
    .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid),
    .calibrated(calibrated), .ax(ax), .ay(ay), .az(az),
    .gx(gx), .gy(gy), .gz(gz),
    .roll_deg_x10(roll_deg_x10),
    .pitch_deg_x10(pitch_deg_x10),
    .yaw_deg_x10(yaw_deg_x10),
    .attitude_valid(attitude_valid)
);

task sample;
begin
    repeat (35) @(negedge clk);
    @(negedge clk);
    sample_valid = 1;
    @(negedge clk);
    sample_valid = 0;
end
endtask

initial begin
    repeat (3) @(negedge clk);
    rst_n = 1;
    for (i = 0; i < 30; i = i + 1) sample();
    $display("level roll=%0d pitch=%0d yaw=%0d", roll_deg_x10, pitch_deg_x10, yaw_deg_x10);
    if (roll_deg_x10 < -20 || roll_deg_x10 > 20 ||
        pitch_deg_x10 < -20 || pitch_deg_x10 > 20 ||
        yaw_deg_x10 < -20 || yaw_deg_x10 > 20) $fatal(1, "level attitude failed");

    gx = 16'sd1430;
    for (i = 0; i < 10; i = i + 1) sample();
    $display("rotation roll=%0d pitch=%0d yaw=%0d", roll_deg_x10, pitch_deg_x10, yaw_deg_x10);
    if (roll_deg_x10 < 100) $fatal(1, "positive roll rotation failed");

    rst_n = 0;
    gx = 0;
    ay = 2049;
    az = 3549;
    repeat (3) @(negedge clk);
    rst_n = 1;
    for (i = 0; i < 250; i = i + 1) sample();
    $display("tilted still roll=%0d pitch=%0d yaw=%0d", roll_deg_x10, pitch_deg_x10, yaw_deg_x10);
    if (roll_deg_x10 < 220 || roll_deg_x10 > 380)
        $fatal(1, "gravity correction failed");

    rst_n = 0;
    ay = 0;
    az = 4098;
    gz = 16'sd1430;
    repeat (3) @(negedge clk);
    rst_n = 1;
    for (i = 0; i < 10; i = i + 1) sample();
    $display("yaw rotation roll=%0d pitch=%0d yaw=%0d", roll_deg_x10, pitch_deg_x10, yaw_deg_x10);
    if (yaw_deg_x10 < 300 || yaw_deg_x10 > 500)
        $fatal(1, "yaw rotation failed");
    $finish;
end
endmodule
