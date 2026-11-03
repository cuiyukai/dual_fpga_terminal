`timescale 1ns / 1ps

module imu_uart_attitude_ascii_tx #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200
)(
    input                    clk,
    input                    rst_n,
    input                    attitude_valid,
    input                    calibrated,
    input signed [15:0]      roll_deg_x10,
    input signed [15:0]      pitch_deg_x10,
    input signed [15:0]      yaw_deg_x10,
    output                   tx,
    output                   busy
);

localparam integer FRAME_BYTES = 46;
reg        frame_active;
reg [5:0]  frame_index;
reg signed [15:0] roll_latched;
reg signed [15:0] pitch_latched;
reg signed [15:0] yaw_latched;
reg [15:0] roll_bcd, pitch_bcd, yaw_bcd;
reg convert_active;
reg [1:0] convert_axis, digit_pos;
reg [15:0] remainder, work_bcd;
reg [15:0] place_value;
reg        byte_start;
reg [7:0]  byte_data;
reg        byte_pending;
reg [31:0] heartbeat_count;
wire       byte_busy;

assign busy = convert_active | frame_active | byte_busy;

function [15:0] magnitude16;
    input signed [15:0] value;
    begin
        magnitude16 = value[15] ? (~value + 16'd1) : value;
    end
endfunction

function [7:0] angle_byte;
    input signed [15:0] value;
    input [15:0] bcd;
    input [3:0] pos;
    begin
        case (pos)
            4'd0: angle_byte = value[15] ? 8'h2D : 8'h2B;
            4'd1: angle_byte = 8'h30 + bcd[15:12];
            4'd2: angle_byte = 8'h30 + bcd[11:8];
            4'd3: angle_byte = 8'h30 + bcd[7:4];
            4'd4: angle_byte = 8'h2E;
            4'd5: angle_byte = 8'h30 + bcd[3:0];
            4'd6: angle_byte = 8'h64; // d
            4'd7: angle_byte = 8'h65; // e
            4'd8: angle_byte = 8'h67; // g
            default: angle_byte = 8'h20;
        endcase
    end
endfunction

function [7:0] frame_byte;
    input [5:0] index;
    begin
        if (index <= 6'd13) begin
            case (index)
                6'd0: frame_byte = "R";
                6'd1: frame_byte = "O";
                6'd2: frame_byte = "L";
                6'd3: frame_byte = "L";
                6'd4: frame_byte = "=";
                default: frame_byte = angle_byte(roll_latched, roll_bcd, index - 6'd5);
            endcase
        end else if (index == 6'd14) begin
            frame_byte = " ";
        end else if (index <= 6'd29) begin
            case (index)
                6'd15: frame_byte = "P";
                6'd16: frame_byte = "I";
                6'd17: frame_byte = "T";
                6'd18: frame_byte = "C";
                6'd19: frame_byte = "H";
                6'd20: frame_byte = "=";
                default: frame_byte = angle_byte(pitch_latched, pitch_bcd, index - 6'd21);
            endcase
        end else if (index == 6'd30) begin
            frame_byte = " ";
        end else if (index <= 6'd43) begin
            case (index)
                6'd31: frame_byte = "Y";
                6'd32: frame_byte = "A";
                6'd33: frame_byte = "W";
                6'd34: frame_byte = "=";
                default: frame_byte = angle_byte(yaw_latched, yaw_bcd, index - 6'd35);
            endcase
        end else if (index == 6'd44) begin
            frame_byte = 8'h0D;
        end else if (index == 6'd45) begin
            frame_byte = 8'h0A;
        end else begin
            frame_byte = 8'h20;
        end
    end
endfunction

imu_uart_byte_tx #(
    .CLK_HZ (CLK_HZ),
    .BAUD   (BAUD)
) u_imu_uart_byte_tx (
    .clk   (clk),
    .rst_n (rst_n),
    .start (byte_start),
    .data  (byte_data),
    .busy  (byte_busy),
    .tx    (tx)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        frame_active <= 1'b0;
        frame_index <= 6'd0;
        roll_latched <= 16'sd0;
        pitch_latched <= 16'sd0;
        yaw_latched <= 16'sd0;
        roll_bcd <= 16'd0;
        pitch_bcd <= 16'd0;
        yaw_bcd <= 16'd0;
        convert_active <= 1'b0;
        convert_axis <= 2'd0;
        digit_pos <= 2'd0;
        remainder <= 16'd0;
        work_bcd <= 16'd0;
        place_value <= 16'd1000;
        byte_start <= 1'b0;
        byte_data <= 8'd0;
        byte_pending <= 1'b0;
        heartbeat_count <= 32'd0;
    end else begin
        byte_start <= 1'b0;
        if (heartbeat_count >= CLK_HZ - 1)
            heartbeat_count <= 32'd0;
        else
            heartbeat_count <= heartbeat_count + 1'b1;

        if (convert_active) begin
            if (digit_pos == 2'd3) begin
                case (convert_axis)
                    2'd0: begin
                        roll_bcd <= {work_bcd[15:4], remainder[3:0]};
                        remainder <= magnitude16(pitch_latched);
                        convert_axis <= 2'd1;
                        digit_pos <= 2'd0;
                        place_value <= 16'd1000;
                        work_bcd <= 16'd0;
                    end
                    2'd1: begin
                        pitch_bcd <= {work_bcd[15:4], remainder[3:0]};
                        remainder <= magnitude16(yaw_latched);
                        convert_axis <= 2'd2;
                        digit_pos <= 2'd0;
                        place_value <= 16'd1000;
                        work_bcd <= 16'd0;
                    end
                    default: begin
                        yaw_bcd <= {work_bcd[15:4], remainder[3:0]};
                        convert_active <= 1'b0;
                        frame_active <= 1'b1;
                        frame_index <= 6'd0;
                    end
                endcase
            end else if (remainder >= place_value) begin
                remainder <= remainder - place_value;
                case (digit_pos)
                    2'd0: work_bcd[15:12] <= work_bcd[15:12] + 1'b1;
                    2'd1: work_bcd[11:8] <= work_bcd[11:8] + 1'b1;
                    default: work_bcd[7:4] <= work_bcd[7:4] + 1'b1;
                endcase
            end else begin
                digit_pos <= digit_pos + 1'b1;
                case (digit_pos)
                    2'd0: place_value <= 16'd100;
                    2'd1: place_value <= 16'd10;
                    default: place_value <= 16'd1;
                endcase
            end
        end else if (byte_pending) begin
            byte_pending <= 1'b0;
        end else if (!frame_active && !byte_busy) begin
            if (attitude_valid && calibrated) begin
                roll_latched <= roll_deg_x10;
                pitch_latched <= pitch_deg_x10;
                yaw_latched <= yaw_deg_x10;
                remainder <= magnitude16(roll_deg_x10);
                convert_axis <= 2'd0;
                digit_pos <= 2'd0;
                place_value <= 16'd1000;
                work_bcd <= 16'd0;
                convert_active <= 1'b1;
            end else if (!calibrated && heartbeat_count >= CLK_HZ - 1) begin
                frame_active <= 1'b1;
                frame_index <= 6'd0;
                roll_latched <= 16'sd0;
                pitch_latched <= 16'sd0;
                yaw_latched <= 16'sd0;
                roll_bcd <= 16'd0;
                pitch_bcd <= 16'd0;
                yaw_bcd <= 16'd0;
            end
        end else if (frame_active && !byte_busy) begin
            byte_data <= frame_byte(frame_index);
            byte_start <= 1'b1;
            byte_pending <= 1'b1;
            if (frame_index == FRAME_BYTES - 1)
                frame_active <= 1'b0;
            else
                frame_index <= frame_index + 1'b1;
        end
    end
end

endmodule
