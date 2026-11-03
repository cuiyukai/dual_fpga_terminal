`timescale 1ns / 1ps

module imu_link_tx #(
    parameter integer CLK_HZ       = 50_000_000,
    parameter integer BAUD         = 115_200,
    parameter integer HEARTBEAT_HZ = 10
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_valid,
    input  wire               attitude_valid,
    input  wire               calibrated,
    input  wire               sensor_error,
    input  wire signed [15:0] gx,
    input  wire signed [15:0] gy,
    input  wire signed [15:0] gz,
    input  wire signed [15:0] roll_deg_x10,
    input  wire signed [15:0] pitch_deg_x10,
    input  wire signed [15:0] yaw_deg_x10,
    input  wire               bright_valid,
    input  wire [6:0]         bright_value,
    input  wire               vol_valid,
    input  wire [6:0]         vol_value,
    input  wire [3:0]         flow_flags,   // {轮播, 上一页, 下一页, 流程模式}，见 menu_ctrl.v
    input  wire               test_mode,    // 实体检测电平
    input  wire [7:0]         menu_state,   // 菜单状态，见 menu_ctrl.v
    input  wire               flow_stop,    // 退出轮播事件翻转位
    input  wire [2:0]         knob_tgl,     // 流程模式旋钮事件翻转位 {按下, 逆时针, 顺时针}
    input  wire [1:0]         test_phase,   // 实体检测阶段 / 剩余秒数 / 评分，见 test_scorer.v
    input  wire [3:0]         test_secs,
    input  wire [6:0]         test_score,
    input  wire [1:0]         test_tilt_dir,   // 最大倾角方向 / BCD 幅度，见 test_scorer.v
    input  wire [3:0]         test_tilt_tens,
    input  wire [3:0]         test_tilt_ones,
    input  wire [3:0]         test_tilt_tenths,
    input  wire               stamp_tgl,       // 时间戳事件翻转位，见 menu_ctrl.v
    input  wire [19:0]        time_bcd,        // B 板 RTC 时间，见 seg_rtc_display.v
    output wire               tx
);

localparam integer HB_CYCLES = CLK_HZ / HEARTBEAT_HZ;
localparam integer HB_WIDTH  = $clog2(HB_CYCLES + 1);
localparam [4:0]   LAST_BYTE = 5'd26;

wire imu_trig    = calibrated ? attitude_valid : sample_valid;

reg  [6:0] brightness;
wire bright_change = bright_valid && (bright_value != brightness);
reg  [6:0] volume;
wire vol_change = vol_valid && (vol_value != volume);
wire flow_change;
wire test_change;
wire menu_change;
wire [7:0] test_b4 = {1'b0, test_score};
wire [7:0] test_b6 = {flow_stop, test_phase, knob_tgl[2], test_secs};
wire [7:0] test_b21 = {test_tilt_ones, test_tilt_tenths};
wire [7:0] test_b22 = {knob_tgl[1:0], test_tilt_dir, test_tilt_tens};
wire test_info_change;
wire stamp_change;

// 两帧之间是否出现过新采样 / 新姿态（发帧时清零）
reg sample_seen;
reg attitude_seen;

reg [HB_WIDTH-1:0] hb_count;
reg send_pending;
reg sending;
wire launch = send_pending && !sending;

// 发帧瞬间的快照，保证一帧内各字段来自同一时刻
reg [7:0]  seq;
reg [7:0]  snap_flags;
reg [7:0]  snap_menu;
reg [7:0]  snap_b4, snap_b6, snap_b21, snap_b22;
reg [6:0]  snap_bright;
reg        snap_test;
reg [6:0]  snap_vol;
reg [15:0] snap_roll, snap_pitch, snap_yaw;
reg [15:0] snap_gx, snap_gy, snap_gz;
reg        snap_stamp;
reg [19:0] snap_time;

reg [4:0]  byte_idx;
reg [7:0]  checksum;
reg        uart_start;
reg [7:0]  uart_data;
wire       uart_busy;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        brightness <= 7'd100;
    else if (bright_valid)
        brightness <= bright_value;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        volume <= 7'd100;
    else if (vol_valid)
        volume <= vol_value;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sample_seen   <= 1'b0;
        attitude_seen <= 1'b0;
    end else if (launch) begin
        sample_seen   <= sample_valid;
        attitude_seen <= attitude_valid;
    end else begin
        if (sample_valid)
            sample_seen <= 1'b1;
        if (attitude_valid)
            attitude_seen <= 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        hb_count     <= {HB_WIDTH{1'b0}};
        send_pending <= 1'b0;
    end else begin
        if (launch || hb_count == HB_CYCLES - 1)
            hb_count <= {HB_WIDTH{1'b0}};
        else
            hb_count <= hb_count + 1'b1;

        if (imu_trig || bright_change || vol_change || flow_change || test_change || menu_change ||
            test_info_change || stamp_change ||
            hb_count == HB_CYCLES - 1)
            send_pending <= 1'b1;
        else if (launch)
            send_pending <= 1'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        seq        <= 8'd0;
        snap_flags <= 8'd0;
        snap_menu  <= 8'd0;
        snap_b4    <= 8'd0;
        snap_b6    <= 8'd0;
        snap_b21   <= 8'd0;
        snap_b22   <= 8'd0;
        snap_bright <= 7'd100;
        snap_test  <= 1'b0;
        snap_vol   <= 7'd100;
        snap_roll  <= 16'd0;
        snap_pitch <= 16'd0;
        snap_yaw   <= 16'd0;
        snap_gx    <= 16'd0;
        snap_gy    <= 16'd0;
        snap_gz    <= 16'd0;
        snap_stamp <= 1'b0;
        snap_time  <= 20'd0;
    end else if (launch) begin
        seq        <= seq + 1'b1;
        snap_flags <= {flow_flags, attitude_seen || attitude_valid,
                       sample_seen || sample_valid, sensor_error, calibrated};
        snap_menu  <= menu_state;
        snap_b4    <= test_b4;
        snap_b6    <= test_b6;
        snap_b21   <= test_b21;
        snap_b22   <= test_b22;
        snap_bright <= brightness;
        snap_test  <= test_mode;
        snap_vol   <= volume;
        snap_roll  <= roll_deg_x10;
        snap_pitch <= pitch_deg_x10;
        snap_yaw   <= yaw_deg_x10;
        snap_gx    <= gx;
        snap_gy    <= gy;
        snap_gz    <= gz;
        snap_stamp <= stamp_tgl;
        snap_time  <= time_bcd;
    end
end

// 流程位与上一帧不同就补发，直到发出的帧带上最新值
assign flow_change = (flow_flags != snap_flags[7:4]);
assign test_change = (test_mode != snap_test);
assign menu_change = (menu_state != snap_menu);
assign test_info_change = (test_b4 != snap_b4) || (test_b6 != snap_b6) ||
                          (test_b21 != snap_b21) || (test_b22 != snap_b22);
assign stamp_change = (stamp_tgl != snap_stamp);

reg [7:0] cur_byte;
always @(*) begin
    case (byte_idx)
        5'd0:  cur_byte = 8'hA5;
        5'd1:  cur_byte = 8'h5A;
        5'd2:  cur_byte = seq;
        5'd3:  cur_byte = snap_flags;
        5'd4:  cur_byte = snap_b4;
        5'd5:  cur_byte = snap_menu;
        5'd6:  cur_byte = snap_b6;
        5'd7:  cur_byte = snap_roll[7:0];
        5'd8:  cur_byte = snap_roll[15:8];
        5'd9:  cur_byte = snap_pitch[7:0];
        5'd10: cur_byte = snap_pitch[15:8];
        5'd11: cur_byte = snap_yaw[7:0];
        5'd12: cur_byte = snap_yaw[15:8];
        5'd13: cur_byte = snap_gx[7:0];
        5'd14: cur_byte = snap_gx[15:8];
        5'd15: cur_byte = snap_gy[7:0];
        5'd16: cur_byte = snap_gy[15:8];
        5'd17: cur_byte = snap_gz[7:0];
        5'd18: cur_byte = snap_gz[15:8];
        5'd19: cur_byte = {snap_test, snap_bright};
        5'd20: cur_byte = {1'b0, snap_vol};
        5'd21: cur_byte = snap_b21;
        5'd22: cur_byte = snap_b22;
        5'd23: cur_byte = {snap_stamp, 1'b0, snap_time[19:14]};
        5'd24: cur_byte = {1'b0, snap_time[13:7]};
        5'd25: cur_byte = {1'b0, snap_time[6:0]};
        5'd26: cur_byte = checksum;
        default: cur_byte = 8'h00;
    endcase
end

// 逐字节送入 UART：start 打一拍，等 busy 拉低后再送下一字节
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sending    <= 1'b0;
        byte_idx   <= 5'd0;
        checksum   <= 8'd0;
        uart_start <= 1'b0;
        uart_data  <= 8'hFF;
    end else begin
        uart_start <= 1'b0;
        if (launch) begin
            sending  <= 1'b1;
            byte_idx <= 5'd0;
            checksum <= 8'd0;
        end else if (sending && !uart_busy && !uart_start) begin
            uart_start <= 1'b1;
            uart_data  <= cur_byte;
            if (byte_idx >= 5'd2 && byte_idx < LAST_BYTE)
                checksum <= checksum + cur_byte;
            if (byte_idx == LAST_BYTE)
                sending <= 1'b0;
            else
                byte_idx <= byte_idx + 1'b1;
        end
    end
end

imu_uart_byte_tx #(
    .CLK_HZ (CLK_HZ),
    .BAUD   (BAUD)
) u_uart_tx (
    .clk   (clk),
    .rst_n (rst_n),
    .start (uart_start),
    .data  (uart_data),
    .busy  (uart_busy),
    .tx    (tx)
);

endmodule
