`timescale 1ns / 1ps

`include "../user_source/hdl_source/reset_sync.v"
`include "../user_source/hdl_source/imu963ra_spi_master.v"
`include "../user_source/hdl_source/imu963ra_driver.v"
`include "../user_source/hdl_source/imu963ra_attitude_serial.v"
`include "../user_source/hdl_source/imu_uart_byte_tx.v"
`include "../user_source/hdl_source/imu_link_tx.v"
`include "../user_source/hdl_source/imu_alarm_led.v"
`include "../user_source/hdl_source/menu_ctrl.v"
`include "../user_source/hdl_source/test_scorer.v"
`include "../user_source/hdl_source/rotary_encoder.v"
`include "../user_source/hdl_source/seg_rtc_display.v"

module top #(
    parameter integer IMU_SPI_HZ   = 10_000_000,
    parameter [7:0]   IMU_GYRO_REG = 8'h22,
    parameter [7:0]   IMU_ACC_REG  = 8'h28,
    parameter integer LINK_BAUD    = 115_200
)(
    input  wire clk,            // 50MHz
    input  wire rst_n,          // 板载复位键，低有效
    input  wire imu_rst_key,    // 按键 KEY2(B2)：长按回菜单 / 唤醒，短按陀螺仪消零
    input  wire sw4,
    input  wire sw3,
    input  wire sw2,
    input  wire sw1,
    input  wire key3,           // 按键 KEY3(B1)：流程短按下一页、长按上一页
    input  wire key4,           // 按键 KEY4(C1)：流程短按开始轮播、长按退出轮播
    input  wire sleep_n,

    // IMU963RA SPI（J2 3/4/6/7 脚，与 A 板原接法相同）
    output wire imu_spi_sck,
    output wire imu_spi_mosi,
    input  wire imu_spi_miso,
    output wire imu_spi_cs,

    output wire link_tx,
    output wire usb_tx,

    output wire [3:0] led,

    input  wire enc_a,
    input  wire enc_b,
    input  wire enc_sw,

    output wire [7:0] seg_sel,
    output wire [7:0] seg_data
);

wire sys_rst_n;

reset_sync u_sys_reset (
    .clk        (clk),
    .ext_rst_n  (rst_n),
    .rst_n_sync (sys_rst_n)
);

wire       knob_cw;
wire       knob_ccw;
wire       knob_press;
wire       rtc_cw;
wire       rtc_ccw;
wire       rtc_press;
wire [3:0] flow_flags;
wire       flow_stop;
wire [2:0] knob_tgl;
wire       test_mode;
wire       sleep_on;
wire       imu_zero;
wire       stamp_tgl;      // 主菜单 KEY2 短按：A 板首页飘过时间戳
wire [19:0] rtc_time;      // RTC 当前时间（BCD），经链路发给 A 板
wire [6:0] flow_brightness;   // 0~100，上电 100
wire [6:0] flow_volume;
wire [7:0] menu_state;

menu_ctrl #(
    .CLK_HZ (50_000_000)
) u_menu_ctrl (
    .clk        (clk),
    .rst_n      (sys_rst_n),
    .key2       (imu_rst_key),
    .key3       (key3),
    .key4       (key4),
    .knob_cw    (knob_cw),
    .knob_ccw   (knob_ccw),
    .knob_press (knob_press),
    .flow_flags (flow_flags),
    .flow_stop  (flow_stop),
    .knob_tgl   (knob_tgl),
    .test_mode  (test_mode),
    .sleep_on   (sleep_on),
    .imu_zero   (imu_zero),
    .stamp_tgl  (stamp_tgl),
    .brightness (flow_brightness),
    .volume     (flow_volume),
    .menu_state (menu_state),
    .rtc_cw     (rtc_cw),
    .rtc_ccw    (rtc_ccw),
    .rtc_press  (rtc_press)
);

wire imu_rst_n;

reset_sync u_imu_reset (
    .clk        (clk),
    .ext_rst_n  (sys_rst_n && !imu_zero),
    .rst_n_sync (imu_rst_n)
);

wire att_rst_n;   // 姿态解算 / 警报灯：休眠期间也保持复位

reset_sync u_att_reset (
    .clk        (clk),
    .ext_rst_n  (sys_rst_n && !imu_zero && !sleep_on),
    .rst_n_sync (att_rst_n)
);

// ---------------- IMU ----------------
wire               imu_sample_valid;
wire               imu_calibrated;
wire               imu_sensor_error;
wire signed [15:0] imu_ax;
wire signed [15:0] imu_ay;
wire signed [15:0] imu_az;
wire signed [15:0] imu_gx;
wire signed [15:0] imu_gy;
wire signed [15:0] imu_gz;
wire signed [15:0] imu_roll_deg_x10;
wire signed [15:0] imu_pitch_deg_x10;
wire signed [15:0] imu_yaw_deg_x10;
wire               imu_attitude_valid;

imu963ra_driver #(
    .CLK_HZ               (50_000_000),
    .SPI_HZ               (IMU_SPI_HZ),
    .SAMPLE_HZ            (25),
    .STARTUP_DELAY_CYCLES (500_000),
    .CAL_SAMPLES          (128),
    .GYRO_REG             (IMU_GYRO_REG),
    .ACC_REG              (IMU_ACC_REG)
) u_imu963ra_driver (
    .clk          (clk),
    .rst_n        (imu_rst_n),
    .sleep        (sleep_on),
    .imu_spi_miso (imu_spi_miso),
    .imu_spi_sck  (imu_spi_sck),
    .imu_spi_mosi (imu_spi_mosi),
    .imu_spi_cs   (imu_spi_cs),
    .sample_valid (imu_sample_valid),
    .calibrated   (imu_calibrated),
    .sensor_error (imu_sensor_error),
    .ax           (imu_ax),
    .ay           (imu_ay),
    .az           (imu_az),
    .gx           (imu_gx),
    .gy           (imu_gy),
    .gz           (imu_gz)
);

imu963ra_attitude_serial u_imu963ra_attitude_serial (
    .clk            (clk),
    .rst_n          (att_rst_n),
    .sample_valid   (imu_sample_valid),
    .calibrated     (imu_calibrated),
    .ax             (imu_ax),
    .ay             (imu_ay),
    .az             (imu_az),
    .gx             (imu_gx),
    .gy             (imu_gy),
    .gz             (imu_gz),
    .roll_deg_x10   (imu_roll_deg_x10),
    .pitch_deg_x10  (imu_pitch_deg_x10),
    .yaw_deg_x10    (imu_yaw_deg_x10),
    .attitude_valid (imu_attitude_valid),
    .quat_w         (),
    .quat_x         (),
    .quat_y         (),
    .quat_z         ()
);

// ---------------- 实体检测计时 / 评分 ----------------
wire [1:0] test_phase;
wire [3:0] test_secs;
wire [6:0] test_score;
wire [1:0] test_tilt_dir;
wire [3:0] test_tilt_tens;
wire [3:0] test_tilt_ones;
wire [3:0] test_tilt_tenths;

test_scorer #(
    .CLK_HZ (50_000_000)
) u_test_scorer (
    .clk            (clk),
    .rst_n          (sys_rst_n),
    .test_mode      (test_mode),
    .restart        (imu_zero),
    .calibrated     (imu_calibrated),
    .attitude_valid (imu_attitude_valid),
    .roll_deg_x10   (imu_roll_deg_x10),
    .pitch_deg_x10  (imu_pitch_deg_x10),
    .phase          (test_phase),
    .secs           (test_secs),
    .score          (test_score),
    .tilt_dir       (test_tilt_dir),
    .tilt_tens      (test_tilt_tens),
    .tilt_ones      (test_tilt_ones),
    .tilt_tenths    (test_tilt_tenths)
);

// ---------------- B -> A 链路 ----------------
imu_link_tx #(
    .CLK_HZ (50_000_000),
    .BAUD   (LINK_BAUD)
) u_imu_link_tx (
    .clk            (clk),
    .rst_n          (sys_rst_n),
    .sample_valid   (imu_sample_valid),
    .attitude_valid (imu_attitude_valid),
    .calibrated     (imu_calibrated),
    .sensor_error   (imu_sensor_error),
    .gx             (imu_gx),
    .gy             (imu_gy),
    .gz             (imu_gz),
    .roll_deg_x10   (imu_roll_deg_x10),
    .pitch_deg_x10  (imu_pitch_deg_x10),
    .yaw_deg_x10    (imu_yaw_deg_x10),
    .bright_valid   (1'b1),
    .bright_value   (flow_brightness),
    .vol_valid      (1'b1),
    .vol_value      (flow_volume),
    .flow_flags     (flow_flags),
    .test_mode      (test_mode),
    .menu_state     (menu_state),
    .flow_stop      (flow_stop),
    .knob_tgl       (knob_tgl),
    .test_phase     (test_phase),
    .test_secs      (test_secs),
    .test_score     (test_score),
    .test_tilt_dir  (test_tilt_dir),
    .test_tilt_tens (test_tilt_tens),
    .test_tilt_ones (test_tilt_ones),
    .test_tilt_tenths (test_tilt_tenths),
    .stamp_tgl      (stamp_tgl),
    .time_bcd       (rtc_time),
    .tx             (link_tx)
);

assign usb_tx = link_tx;

imu_alarm_led #(
    .CLK_HZ         (50_000_000),
    .THRESH_DEG_X10 (300)
) u_imu_alarm_led (
    .clk            (clk),
    .rst_n          (att_rst_n),
    .enable         (test_mode),
    .attitude_valid (imu_attitude_valid),
    .calibrated     (imu_calibrated),
    .roll_deg_x10   (imu_roll_deg_x10),
    .pitch_deg_x10  (imu_pitch_deg_x10),
    .yaw_deg_x10    (imu_yaw_deg_x10),
    .led            (led)
);

rotary_encoder #(
    .CLK_HZ (50_000_000)
) u_rotary_encoder (
    .clk      (clk),
    .rst_n    (sys_rst_n),
    .enc_a    (enc_a),
    .enc_b    (enc_b),
    .enc_sw   (enc_sw),
    .step_cw  (knob_cw),
    .step_ccw (knob_ccw),
    .press    (knob_press)
);

seg_rtc_display #(
    .CLK_HZ (50_000_000)
) u_seg_rtc_display (
    .clk        (clk),
    .knob_cw    (rtc_cw),
    .knob_ccw   (rtc_ccw),
    .knob_press (rtc_press),
    .seg_sel    (seg_sel),
    .seg_data   (seg_data),
    .time_bcd   (rtc_time)
);

endmodule
