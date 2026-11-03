`timescale 1ns / 1ps

module imu_uart_byte_tx #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200
)(
    input       clk,
    input       rst_n,
    input       start,
    input [7:0] data,
    output reg  busy,
    output      tx
);

localparam integer BAUD_DIV = (CLK_HZ / BAUD) < 2 ? 2 : (CLK_HZ / BAUD);
localparam integer BAUD_WIDTH = (BAUD_DIV <= 2) ? 2 : $clog2(BAUD_DIV);

reg [BAUD_WIDTH-1:0] baud_count;
reg [9:0] shift_reg;
reg [3:0] bit_count;
wire baud_tick = (baud_count == BAUD_DIV - 1);

assign tx = busy ? shift_reg[0] : 1'b1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        baud_count <= {BAUD_WIDTH{1'b0}};
        shift_reg <= 10'h3ff;
        bit_count <= 4'd0;
        busy <= 1'b0;
    end else if (!busy) begin
        baud_count <= {BAUD_WIDTH{1'b0}};
        if (start) begin
            shift_reg <= {1'b1, data, 1'b0};
            bit_count <= 4'd0;
            busy <= 1'b1;
        end
    end else if (baud_tick) begin
        baud_count <= {BAUD_WIDTH{1'b0}};
        if (bit_count == 4'd9) begin
            busy <= 1'b0;
            bit_count <= 4'd0;
        end else begin
            shift_reg <= {1'b1, shift_reg[9:1]};
            bit_count <= bit_count + 1'b1;
        end
    end else begin
        baud_count <= baud_count + 1'b1;
    end
end

endmodule
