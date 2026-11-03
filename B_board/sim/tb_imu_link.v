`timescale 1ns / 1ps
module tb_imu_link;
reg clk = 0; always #10 clk = ~clk;
reg rst_n = 0;
reg sample_valid = 0, attitude_valid = 0, calibrated = 0, sensor_error = 0;
reg signed [15:0] gx = 0, gy = 0, gz = 0, roll = 0, pitch = 0, yaw = 0;
reg touch_valid = 0, touch_pressed = 0;
reg [7:0] touch_page = 0, touch_control = 0;
reg corrupt = 0;
wire tx;

imu_link_tx #(.HEARTBEAT_HZ(200)) u_tx (
    .clk(clk), .rst_n(rst_n), .sample_valid(sample_valid), .attitude_valid(attitude_valid),
    .calibrated(calibrated), .sensor_error(sensor_error), .gx(gx), .gy(gy), .gz(gz),
    .roll_deg_x10(roll), .pitch_deg_x10(pitch), .yaw_deg_x10(yaw),
    .touch_valid(touch_valid), .touch_pressed(touch_pressed),
    .touch_page(touch_page), .touch_control(touch_control), .tx(tx));

wire r_sv, r_av, r_cal, r_err, r_next;
wire signed [15:0] r_gx, r_gy, r_gz, r_roll, r_pitch, r_yaw;
imu_link_rx u_rx (
    .clk(clk), .rst_n(rst_n), .rx(tx ^ corrupt),
    .sample_valid(r_sv), .attitude_valid(r_av), .calibrated(r_cal), .sensor_error(r_err),
    .gx(r_gx), .gy(r_gy), .gz(r_gz), .roll_deg_x10(r_roll), .pitch_deg_x10(r_pitch),
    .yaw_deg_x10(r_yaw), .next_pulse(r_next));

integer n_sv = 0, n_av = 0, n_next = 0, n_frames_tx = 0, n_frames_rx = 0, errors = 0;
always @(posedge clk) begin
    if (r_sv) n_sv = n_sv + 1;
    if (r_av) n_av = n_av + 1;
    if (r_next) n_next = n_next + 1;
    if (u_tx.launch) begin n_frames_tx = n_frames_tx + 1; $display("  tx launch #%0d @%0t", n_frames_tx, $time); end
    if (u_rx.rx_valid && u_rx.byte_idx == 19) $display("  rx end  @%0t data=%h sum=%h flags=%h", $time, u_rx.rx_data, u_rx.checksum, u_rx.buf_flags);
    if (u_rx.rx_valid && u_rx.byte_idx == 19 && u_rx.rx_data == u_rx.checksum)
        n_frames_rx = n_frames_rx + 1;
end

task pulse_sample; begin @(posedge clk) sample_valid <= 1; @(posedge clk) sample_valid <= 0; end endtask
task pulse_att;    begin @(posedge clk) attitude_valid <= 1; @(posedge clk) attitude_valid <= 0; end endtask
task press(input [7:0] ctl); begin
    @(posedge clk) begin touch_valid <= 1; touch_pressed <= 1; touch_control <= ctl; touch_page <= 8'd3; end
    @(posedge clk) begin touch_valid <= 0; touch_pressed <= 0; end
end endtask
task check(input cond, input [8*40-1:0] msg); begin
    if (!cond) begin errors = errors + 1; $display("FAIL: %0s @%0t", msg, $time); end
    else $display("ok  : %0s", msg);
end endtask

initial begin
    #200 rst_n = 1;
    // 1. 未校准：每次采样都发帧，A 端只出 sample_valid
    gx = 16'sd100; gy = -16'sd200; gz = 16'sd300;
    pulse_sample;
    #2_500_000;
    check(n_sv == 1 && n_av == 0 && r_cal == 0, "uncal sample -> sv only");
    check(r_gx == 100 && r_gy == -200 && r_gz == 300, "gyro values");

    calibrated = 1;
    pulse_sample;
    repeat (2000) @(posedge clk);
    roll = -16'sd1234; pitch = 16'sd456; yaw = -16'sd1799;
    pulse_att;
    #2_500_000;
    check(n_sv == 2 && n_av == 1 && r_cal == 1, "cal: sv+av pulses");
    check(r_roll == -1234 && r_pitch == 456 && r_yaw == -1799, "attitude values");

    // 3. 触摸：控件 1 翻页，控件 2 不翻页
    press(8'd1);
    #2_500_000;
    check(n_next == 1, "control 1 -> next_pulse");
    press(8'd2);
    #2_500_000;
    check(n_next == 1, "control 2 -> no next");

    // 4. 传输中干扰一个位，该帧必须丢弃且后续帧能恢复
    sensor_error = 1;
    pulse_sample;
    pulse_att;                           // 已校准时由姿态完成触发发帧
    repeat (40000) @(posedge clk);       // 大约第 5 个字节处
    corrupt = 1; repeat (434) @(posedge clk); corrupt = 0;
    #2_500_000;
    check(n_frames_rx == n_frames_tx - 1, "corrupted frame dropped");
    check(n_av == 1, "no attitude pulse from bad frame");
    check(r_err == 0, "error flag not taken from bad frame");
    #6_000_000;                           // 心跳帧恢复
    check(r_err == 1, "heartbeat frame recovers state");
    check(n_av == 1, "heartbeat does not fake attitude pulse");
    check(n_frames_rx == n_frames_tx - 1, "later frames all received");

    $display("frames tx=%0d rx=%0d, errors=%0d", n_frames_tx, n_frames_rx, errors);
    if (errors == 0) $display("ALL PASS"); else $display("TEST FAILED");
    $finish;
end
endmodule
