`timescale 1ns / 1ps

module imu963ra_driver #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SPI_HZ = 10000000,
    parameter integer SAMPLE_HZ = 25,
    parameter integer STARTUP_DELAY_CYCLES = 500000,
    parameter integer RESET_DELAY_CYCLES = 100000,
    parameter integer CAL_SAMPLES = 128,
    parameter [7:0] GYRO_REG = 8'h22,
    parameter [7:0] ACC_REG = 8'h28,
    parameter [7:0] ID_REG = 8'h0F,         // WHO_AM_I
    parameter [7:0] ID_VALUE = 8'h6B
)(
    input                    clk,
    input                    rst_n,
    input                    sleep,          // 1: power down accel / gyro, then stop SPI traffic
    input                    imu_spi_miso,
    output                   imu_spi_sck,
    output                   imu_spi_mosi,
    output                   imu_spi_cs,
    output reg               sample_valid,
    output reg               calibrated,
    output reg               sensor_error,
    output reg signed [15:0] ax,
    output reg signed [15:0] ay,
    output reg signed [15:0] az,
    output reg signed [15:0] gx,
    output reg signed [15:0] gy,
    output reg signed [15:0] gz
);

localparam integer SAMPLE_INTERVAL = (CLK_HZ / SAMPLE_HZ) < 2 ? 2 : (CLK_HZ / SAMPLE_HZ);
localparam integer TIMER_WIDTH = (SAMPLE_INTERVAL <= 2) ? 2 : $clog2(SAMPLE_INTERVAL);
localparam integer STARTUP_WIDTH = (STARTUP_DELAY_CYCLES <= 2) ? 2 : $clog2(STARTUP_DELAY_CYCLES + 1);
localparam integer RESET_WIDTH = (RESET_DELAY_CYCLES <= 2) ? 2 : $clog2(RESET_DELAY_CYCLES + 1);
localparam integer CAL_COUNT_WIDTH = (CAL_SAMPLES <= 2) ? 2 : $clog2(CAL_SAMPLES + 1);
localparam integer CAL_SHIFT = $clog2(CAL_SAMPLES);

localparam [4:0] ST_STARTUP       = 5'd0;
localparam [4:0] ST_CFG_PREP      = 5'd1;
localparam [4:0] ST_CFG_LAUNCH    = 5'd2;
localparam [4:0] ST_CFG_WAIT      = 5'd3;
localparam [4:0] ST_RESET_DELAY   = 5'd4;
localparam [4:0] ST_SAMPLE_WAIT   = 5'd5;
localparam [4:0] ST_GYRO_PREP     = 5'd6;
localparam [4:0] ST_GYRO_LAUNCH   = 5'd7;
localparam [4:0] ST_GYRO_WAIT     = 5'd8;
localparam [4:0] ST_ACC_PREP      = 5'd9;
localparam [4:0] ST_ACC_LAUNCH    = 5'd10;
localparam [4:0] ST_ACC_WAIT      = 5'd11;
localparam [4:0] ST_PD_PREP       = 5'd12;
localparam [4:0] ST_PD_LAUNCH     = 5'd13;
localparam [4:0] ST_PD_WAIT       = 5'd14;
localparam [4:0] ST_SLEEP         = 5'd15;
localparam [4:0] ST_ID_PREP       = 5'd16;
localparam [4:0] ST_ID_LAUNCH     = 5'd17;
localparam [4:0] ST_ID_WAIT       = 5'd18;

reg [4:0] state;
reg [STARTUP_WIDTH-1:0] startup_timer;
reg [RESET_WIDTH-1:0] reset_timer;
reg [TIMER_WIDTH-1:0] sample_timer;
reg [CAL_COUNT_WIDTH-1:0] cal_count;
reg [3:0] config_index;
reg [7:0] bus_reg;
reg [7:0] bus_write_data;
reg bus_read_mode;
reg bus_start;
wire bus_busy;
wire bus_done;
wire bus_ack_error;
wire [95:0] bus_read_data;
reg signed [15:0] gyro_x_raw;
reg signed [15:0] gyro_y_raw;
reg signed [15:0] gyro_z_raw;
reg signed [15:0] acc_x_raw;
reg signed [15:0] acc_y_raw;
reg signed [15:0] acc_z_raw;

reg signed [31:0] sum_ax;
reg signed [31:0] sum_ay;
reg signed [31:0] sum_az;
reg signed [31:0] sum_gx;
reg signed [31:0] sum_gy;
reg signed [31:0] sum_gz;
reg signed [15:0] bias_ax;
reg signed [15:0] bias_ay;
reg signed [15:0] bias_az;
reg signed [15:0] bias_gx;
reg signed [15:0] bias_gy;
reg signed [15:0] bias_gz;

wire signed [15:0] gyro_x_sample = {bus_read_data[39:32], bus_read_data[47:40]};
wire signed [15:0] gyro_y_sample = {bus_read_data[23:16], bus_read_data[31:24]};
wire signed [15:0] gyro_z_sample = {bus_read_data[7:0],   bus_read_data[15:8]};
wire signed [15:0] acc_x_sample = {bus_read_data[39:32], bus_read_data[47:40]};
wire signed [15:0] acc_y_sample = {bus_read_data[23:16], bus_read_data[31:24]};
wire signed [15:0] acc_z_sample = {bus_read_data[7:0],   bus_read_data[15:8]};
wire               id_match = (bus_read_data[47:40] == ID_VALUE);
wire signed [15:0] mapped_ax = -acc_y_raw;
wire signed [15:0] mapped_ay = acc_x_raw;
wire signed [15:0] mapped_az = acc_z_raw;
wire signed [15:0] mapped_ax_sample = -acc_y_sample;
wire signed [15:0] mapped_ay_sample = acc_x_sample;
wire signed [15:0] mapped_az_sample = acc_z_sample;
wire signed [15:0] mapped_gx = -gyro_y_raw;
wire signed [15:0] mapped_gy = gyro_x_raw;
wire signed [15:0] mapped_gz = gyro_z_raw;
wire signed [31:0] mapped_ax_ext = {{16{mapped_ax[15]}}, mapped_ax};
wire signed [31:0] mapped_ay_ext = {{16{mapped_ay[15]}}, mapped_ay};
wire signed [31:0] mapped_az_ext = {{16{mapped_az[15]}}, mapped_az};
wire signed [31:0] mapped_gx_ext = {{16{mapped_gx[15]}}, mapped_gx};
wire signed [31:0] mapped_gy_ext = {{16{mapped_gy[15]}}, mapped_gy};
wire signed [31:0] mapped_gz_ext = {{16{mapped_gz[15]}}, mapped_gz};
wire signed [31:0] mapped_ax_sample_ext = {{16{mapped_ax_sample[15]}}, mapped_ax_sample};
wire signed [31:0] mapped_ay_sample_ext = {{16{mapped_ay_sample[15]}}, mapped_ay_sample};
wire signed [31:0] mapped_az_sample_ext = {{16{mapped_az_sample[15]}}, mapped_az_sample};

function [7:0] config_reg;
    input [3:0] index;
    begin
        case (index)
            4'd0: config_reg = 8'h01;
            4'd1: config_reg = 8'h12;
            4'd2: config_reg = 8'h01;
            4'd3: config_reg = 8'h0D;
            4'd4: config_reg = 8'h10;
            4'd5: config_reg = 8'h11;
            4'd6: config_reg = 8'h12;
            4'd7: config_reg = 8'h13;
            4'd8: config_reg = 8'h14;
            4'd9: config_reg = 8'h15;
            4'd10: config_reg = 8'h16;
            4'd11: config_reg = 8'h18;
            default: config_reg = 8'h01;
        endcase
    end
endfunction

function [7:0] config_value;
    input [3:0] index;
    begin
        case (index)
            4'd0: config_value = 8'h00;
            4'd1: config_value = 8'h01;
            4'd2: config_value = 8'h00;
            4'd3: config_value = 8'h03;
            4'd4: config_value = 8'h3C;
            4'd5: config_value = 8'h5C;
            4'd6: config_value = 8'h44;
            4'd7: config_value = 8'h02;
            4'd8: config_value = 8'h00;
            4'd9: config_value = 8'h00;
            4'd10: config_value = 8'h00;
            4'd11: config_value = 8'h01;
            default: config_value = 8'h00;
        endcase
    end
endfunction

imu963ra_spi_master #(
    .CLK_HZ(CLK_HZ),
    .SPI_HZ(SPI_HZ),
    .READ_BYTES(6)
) u_imu963ra_spi_master (
    .clk        (clk),
    .rst_n      (rst_n),
    .start      (bus_start),
    .read_mode  (bus_read_mode),
    .write_data (bus_write_data),
    .reg_addr   (bus_reg),
    .busy       (bus_busy),
    .done       (bus_done),
    .ack_error  (bus_ack_error),
    .read_data  (bus_read_data),
    .miso       (imu_spi_miso),
    .sck        (imu_spi_sck),
    .mosi       (imu_spi_mosi),
    .cs_n       (imu_spi_cs)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= ST_STARTUP;
        startup_timer <= {STARTUP_WIDTH{1'b0}};
        reset_timer <= {RESET_WIDTH{1'b0}};
        sample_timer <= {TIMER_WIDTH{1'b0}};
        cal_count <= {CAL_COUNT_WIDTH{1'b0}};
        config_index <= 4'd0;
        bus_reg <= 8'd0;
        bus_write_data <= 8'd0;
        bus_read_mode <= 1'b0;
        bus_start <= 1'b0;
        gyro_x_raw <= 16'sd0;
        gyro_y_raw <= 16'sd0;
        gyro_z_raw <= 16'sd0;
        acc_x_raw <= 16'sd0;
        acc_y_raw <= 16'sd0;
        acc_z_raw <= 16'sd0;
        sum_ax <= 32'sd0;
        sum_ay <= 32'sd0;
        sum_az <= 32'sd0;
        sum_gx <= 32'sd0;
        sum_gy <= 32'sd0;
        sum_gz <= 32'sd0;
        bias_ax <= 16'sd0;
        bias_ay <= 16'sd0;
        bias_az <= 16'sd0;
        bias_gx <= 16'sd0;
        bias_gy <= 16'sd0;
        bias_gz <= 16'sd0;
        sample_valid <= 1'b0;
        calibrated <= 1'b0;
        sensor_error <= 1'b0;
        ax <= 16'sd0;
        ay <= 16'sd0;
        az <= 16'sd0;
        gx <= 16'sd0;
        gy <= 16'sd0;
        gz <= 16'sd0;
    end else begin
        bus_start <= 1'b0;
        sample_valid <= 1'b0;
        case (state)
            ST_STARTUP: begin
                if (startup_timer >= STARTUP_DELAY_CYCLES - 1) begin
                    config_index <= 4'd0;
                    state <= ST_CFG_PREP;
                end else begin
                    startup_timer <= startup_timer + 1'b1;
                end
            end
            ST_CFG_PREP: begin
                if (!bus_busy) begin
                    bus_reg <= config_reg(config_index);
                    bus_write_data <= config_value(config_index);
                    bus_read_mode <= 1'b0;
                    state <= ST_CFG_LAUNCH;
                end
            end
            ST_CFG_LAUNCH: begin
                bus_start <= 1'b1;
                state <= ST_CFG_WAIT;
            end
            ST_CFG_WAIT: begin
                if (bus_done) begin
                    if (bus_ack_error)
                        sensor_error <= 1'b1;
                    if (bus_ack_error) begin
                        state <= ST_STARTUP;
                    end else if (config_index == 4'd1) begin
                        reset_timer <= {RESET_WIDTH{1'b0}};
                        state <= ST_RESET_DELAY;
                    end else if (config_index == 4'd2) begin
                        state <= ST_ID_PREP;
                    end else if (config_index == 4'd11) begin
                        sample_timer <= {TIMER_WIDTH{1'b0}};
                        state <= ST_SAMPLE_WAIT;
                    end else begin
                        config_index <= config_index + 1'b1;
                        state <= ST_CFG_PREP;
                    end
                end
            end
            ST_RESET_DELAY: begin
                if (reset_timer >= RESET_DELAY_CYCLES - 1) begin
                    config_index <= config_index + 1'b1;
                    state <= ST_CFG_PREP;
                end else begin
                    reset_timer <= reset_timer + 1'b1;
                end
            end
            ST_ID_PREP: begin
                if (!bus_busy) begin
                    bus_reg <= ID_REG;
                    bus_read_mode <= 1'b1;
                    state <= ST_ID_LAUNCH;
                end
            end
            ST_ID_LAUNCH: begin
                bus_start <= 1'b1;
                state <= ST_ID_WAIT;
            end
            ST_ID_WAIT: begin
                if (bus_done) begin
                    if (id_match) begin
                        sensor_error <= 1'b0;
                        config_index <= config_index + 1'b1;
                        state <= ST_CFG_PREP;
                    end else begin
                        sensor_error <= 1'b1;
                        startup_timer <= {STARTUP_WIDTH{1'b0}};
                        state <= sleep ? ST_SLEEP : ST_STARTUP;
                    end
                end
            end
            ST_SAMPLE_WAIT: begin
                if (sleep) begin
                    config_index <= 4'd4;
                    state <= ST_PD_PREP;
                end else if (sample_timer >= SAMPLE_INTERVAL - 1) begin
                    sample_timer <= {TIMER_WIDTH{1'b0}};
                    state <= ST_GYRO_PREP;
                end else begin
                    sample_timer <= sample_timer + 1'b1;
                end
            end
            ST_GYRO_PREP: begin
                if (!bus_busy) begin
                    bus_reg <= GYRO_REG;
                    bus_read_mode <= 1'b1;
                    state <= ST_GYRO_LAUNCH;
                end
            end
            ST_GYRO_LAUNCH: begin
                bus_start <= 1'b1;
                state <= ST_GYRO_WAIT;
            end
            ST_GYRO_WAIT: begin
                if (bus_done) begin
                    sensor_error <= bus_ack_error;
                    if (bus_ack_error) begin
                        state <= ST_SAMPLE_WAIT;
                    end else begin
                        gyro_x_raw <= gyro_x_sample;
                        gyro_y_raw <= gyro_y_sample;
                        gyro_z_raw <= gyro_z_sample;
                        state <= ST_ACC_PREP;
                    end
                end
            end
            ST_ACC_PREP: begin
                if (!bus_busy) begin
                    bus_reg <= ACC_REG;
                    bus_read_mode <= 1'b1;
                    state <= ST_ACC_LAUNCH;
                end
            end
            ST_ACC_LAUNCH: begin
                bus_start <= 1'b1;
                state <= ST_ACC_WAIT;
            end
            ST_ACC_WAIT: begin
                if (bus_done) begin
                    sensor_error <= bus_ack_error;
                    if (bus_ack_error) begin
                        state <= ST_SAMPLE_WAIT;
                    end else begin
                        acc_x_raw <= acc_x_sample;
                        acc_y_raw <= acc_y_sample;
                        acc_z_raw <= acc_z_sample;
                        sample_valid <= 1'b1;
                        if (!calibrated) begin
                            sum_gx <= sum_gx + mapped_gx_ext;
                            sum_gy <= sum_gy + mapped_gy_ext;
                            sum_gz <= sum_gz + mapped_gz_ext;
                            if (cal_count == CAL_SAMPLES - 1) begin
                                bias_gx <= (sum_gx + mapped_gx_ext) >>> CAL_SHIFT;
                                bias_gy <= (sum_gy + mapped_gy_ext) >>> CAL_SHIFT;
                                bias_gz <= (sum_gz + mapped_gz_ext) >>> CAL_SHIFT;
                                calibrated <= 1'b1;
                                ax <= mapped_ax_sample;
                                ay <= mapped_ay_sample;
                                az <= mapped_az_sample;
                                gx <= mapped_gx - ((sum_gx + mapped_gx_ext) >>> CAL_SHIFT);
                                gy <= mapped_gy - ((sum_gy + mapped_gy_ext) >>> CAL_SHIFT);
                                gz <= mapped_gz - ((sum_gz + mapped_gz_ext) >>> CAL_SHIFT);
                            end else begin
                                cal_count <= cal_count + 1'b1;
                            end
                        end else begin
                            ax <= mapped_ax_sample;
                            ay <= mapped_ay_sample;
                            az <= mapped_az_sample;
                            gx <= mapped_gx - bias_gx;
                            gy <= mapped_gy - bias_gy;
                            gz <= mapped_gz - bias_gz;
                        end
                        state <= ST_SAMPLE_WAIT;
                    end
                end
            end
            ST_PD_PREP: begin
                if (!bus_busy) begin
                    bus_reg <= config_reg(config_index);
                    bus_write_data <= 8'h00;
                    bus_read_mode <= 1'b0;
                    state <= ST_PD_LAUNCH;
                end
            end
            ST_PD_LAUNCH: begin
                bus_start <= 1'b1;
                state <= ST_PD_WAIT;
            end
            ST_PD_WAIT: begin
                if (bus_done) begin
                    if (config_index == 4'd5 || bus_ack_error) begin
                        calibrated <= 1'b0;
                        state <= ST_SLEEP;
                    end else begin
                        config_index <= config_index + 1'b1;
                        state <= ST_PD_PREP;
                    end
                end
            end
            ST_SLEEP: begin
                if (!sleep) begin
                    startup_timer <= {STARTUP_WIDTH{1'b0}};
                    cal_count <= {CAL_COUNT_WIDTH{1'b0}};
                    sum_gx <= 32'sd0;
                    sum_gy <= 32'sd0;
                    sum_gz <= 32'sd0;
                    state <= ST_STARTUP;
                end
            end
            default: begin
                state <= ST_STARTUP;
            end
        endcase
    end
end

endmodule
