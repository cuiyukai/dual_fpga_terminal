`timescale 1ns / 1ps

module imu_link_rx #(
    parameter integer CLK_HZ             = 50_000_000,
    parameter integer BAUD               = 115_200,
    parameter integer GAP_TIMEOUT_CYCLES = 25_000      // 字节间隔超过 0.5ms 视为断帧
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               rx,
    output reg                sample_valid,
    output reg                attitude_valid,
    output reg                calibrated,
    output reg                sensor_error,
    output reg signed [15:0]  gx,
    output reg signed [15:0]  gy,
    output reg signed [15:0]  gz,
    output reg signed [15:0]  roll_deg_x10,
    output reg signed [15:0]  pitch_deg_x10,
    output reg signed [15:0]  yaw_deg_x10,
    output reg  [6:0]         brightness_pct,
    output reg  [6:0]         volume_pct,
    output reg                frame_valid,
    output reg                flow_mode,
    output reg  [6:0]         flow_evt_toggle,
    output reg                test_mode,
    output reg  [7:0]         menu_state,
    output reg  [1:0]         test_phase,
    output reg  [3:0]         test_secs,
    output reg  [6:0]         test_score,
    output reg  [13:0]        test_tilt,
    output reg  [19:0]        rtc_time,
    output reg                stamp_toggle
);

localparam integer GAP_WIDTH = $clog2(GAP_TIMEOUT_CYCLES + 1);
localparam [4:0]   LAST_BYTE = 5'd26;

wire [7:0] rx_data;
wire       rx_valid;

screen_uart_rx #(
    .CLK_HZ (CLK_HZ),
    .BAUD   (BAUD)
) u_link_uart_rx (
    .clk           (clk),
    .rst_n         (rst_n),
    .rx            (rx),
    .data          (rx_data),
    .valid         (rx_valid),
    .framing_error ()
);

reg [4:0]           byte_idx;
reg [7:0]           checksum;
reg [GAP_WIDTH-1:0] gap_count;

// 帧内暂存，校验通过后一次性提交
reg [7:0]  buf_flags;
reg [7:0]  buf_score;
reg [7:0]  buf_menu;
reg [7:0]  buf_test;
reg [7:0]  buf_bright;
reg [7:0]  buf_vol;
reg [15:0] buf_tilt;          // bit15~14 = 旋钮 {逆时针, 顺时针} 翻转位
reg [15:0] buf_roll, buf_pitch, buf_yaw;
reg [15:0] buf_gx, buf_gy, buf_gz;
reg [7:0]  buf_hour;          // bit7 = 时间戳事件翻转位
reg [6:0]  buf_min, buf_sec;
reg        last_stamp;

reg [6:0]  last_flow_evt;
reg        have_frame;
wire [6:0] flow_evt_now = {buf_test[4], buf_tilt[15:14], buf_test[7], buf_flags[7:5]};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        byte_idx       <= 5'd0;
        checksum       <= 8'd0;
        gap_count      <= {GAP_WIDTH{1'b0}};
        buf_flags      <= 8'd0;
        buf_score      <= 8'd0;
        buf_menu       <= 8'd0;
        buf_test       <= 8'd0;
        buf_bright     <= 8'd100;
        buf_vol        <= 8'd100;
        buf_tilt       <= 16'd0;
        buf_roll       <= 16'd0;
        buf_pitch      <= 16'd0;
        buf_yaw        <= 16'd0;
        buf_gx         <= 16'd0;
        buf_gy         <= 16'd0;
        buf_gz         <= 16'd0;
        buf_hour       <= 8'd0;
        buf_min        <= 7'd0;
        buf_sec        <= 7'd0;
        last_stamp     <= 1'b0;
        rtc_time       <= 20'd0;
        stamp_toggle   <= 1'b0;
        have_frame     <= 1'b0;
        last_flow_evt  <= 7'd0;
        flow_mode      <= 1'b0;
        flow_evt_toggle <= 7'd0;
        test_mode      <= 1'b0;
        menu_state     <= 8'd0;
        test_phase     <= 2'd0;
        test_secs      <= 4'd0;
        test_score     <= 7'd0;
        test_tilt      <= 14'd0;
        sample_valid   <= 1'b0;
        attitude_valid <= 1'b0;
        calibrated     <= 1'b0;
        sensor_error   <= 1'b0;
        gx             <= 16'sd0;
        gy             <= 16'sd0;
        gz             <= 16'sd0;
        roll_deg_x10   <= 16'sd0;
        pitch_deg_x10  <= 16'sd0;
        yaw_deg_x10    <= 16'sd0;
        brightness_pct <= 7'd100;
        volume_pct     <= 7'd100;
        frame_valid    <= 1'b0;
    end else begin
        sample_valid   <= 1'b0;
        attitude_valid <= 1'b0;
        frame_valid    <= 1'b0;

        if (rx_valid) begin
            gap_count <= {GAP_WIDTH{1'b0}};
            if (byte_idx >= 5'd2 && byte_idx < LAST_BYTE)
                checksum <= checksum + rx_data;

            case (byte_idx)
                5'd0: begin
                    checksum <= 8'd0;
                    if (rx_data == 8'hA5)
                        byte_idx <= 5'd1;
                end
                5'd1: begin
                    // 不是 5A 时，若本字节恰好是 A5 则把它当作新帧头
                    if (rx_data == 8'h5A)
                        byte_idx <= 5'd2;
                    else if (rx_data != 8'hA5)
                        byte_idx <= 5'd0;
                end
                5'd3:  buf_flags        <= rx_data;
                5'd4:  buf_score        <= rx_data;
                5'd5:  buf_menu         <= rx_data;
                5'd6:  buf_test         <= rx_data;
                5'd7:  buf_roll[7:0]    <= rx_data;
                5'd8:  buf_roll[15:8]   <= rx_data;
                5'd9:  buf_pitch[7:0]   <= rx_data;
                5'd10: buf_pitch[15:8]  <= rx_data;
                5'd11: buf_yaw[7:0]     <= rx_data;
                5'd12: buf_yaw[15:8]    <= rx_data;
                5'd13: buf_gx[7:0]      <= rx_data;
                5'd14: buf_gx[15:8]     <= rx_data;
                5'd15: buf_gy[7:0]      <= rx_data;
                5'd16: buf_gy[15:8]     <= rx_data;
                5'd17: buf_gz[7:0]      <= rx_data;
                5'd18: buf_gz[15:8]     <= rx_data;
                5'd19: buf_bright       <= rx_data;
                5'd20: buf_vol          <= rx_data;
                5'd21: buf_tilt[7:0]    <= rx_data;
                5'd22: buf_tilt[15:8]   <= rx_data;
                5'd23: buf_hour         <= rx_data;
                5'd24: buf_min          <= rx_data[6:0];
                5'd25: buf_sec          <= rx_data[6:0];
                5'd26: begin
                    byte_idx <= 5'd0;
                    if (rx_data == checksum) begin
                        frame_valid    <= 1'b1;
                        calibrated     <= buf_flags[0];
                        sensor_error   <= buf_flags[1];
                        sample_valid   <= buf_flags[2];
                        attitude_valid <= buf_flags[3];
                        roll_deg_x10   <= buf_roll;
                        pitch_deg_x10  <= buf_pitch;
                        yaw_deg_x10    <= buf_yaw;
                        gx             <= buf_gx;
                        gy             <= buf_gy;
                        gz             <= buf_gz;
                        brightness_pct <= (buf_bright[6:0] > 7'd100) ? 7'd100 : buf_bright[6:0];
                        volume_pct     <= (buf_vol > 8'd100) ? 7'd100 : buf_vol[6:0];
                        have_frame     <= 1'b1;
                        flow_mode      <= buf_flags[4];
                        test_mode      <= buf_bright[7];
                        menu_state     <= buf_menu;
                        test_phase     <= buf_test[6:5];
                        test_secs      <= buf_test[3:0];
                        test_score     <= (buf_score[6:0] > 7'd100) ? 7'd100 : buf_score[6:0];
                        test_tilt      <= buf_tilt[13:0];
                        last_flow_evt  <= flow_evt_now;
                        rtc_time       <= {buf_hour[5:0], buf_min, buf_sec};
                        last_stamp     <= buf_hour[7];
                        if (have_frame && (buf_hour[7] != last_stamp))
                            stamp_toggle <= ~stamp_toggle;
                        if (have_frame)
                            flow_evt_toggle <= flow_evt_toggle ^ (flow_evt_now ^ last_flow_evt);
                    end
                end
                default: ;
            endcase

            if (byte_idx >= 5'd2 && byte_idx < LAST_BYTE)
                byte_idx <= byte_idx + 1'b1;
        end else if (byte_idx != 5'd0) begin
            if (gap_count == GAP_TIMEOUT_CYCLES - 1) begin
                gap_count <= {GAP_WIDTH{1'b0}};
                byte_idx  <= 5'd0;
            end else begin
                gap_count <= gap_count + 1'b1;
            end
        end
    end
end

endmodule
