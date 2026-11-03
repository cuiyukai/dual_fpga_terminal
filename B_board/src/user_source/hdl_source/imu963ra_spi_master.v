`timescale 1ns / 1ps

module imu963ra_spi_master #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SPI_HZ = 10000000,
    parameter integer READ_BYTES = 6
)(
    input             clk,
    input             rst_n,
    input             start,
    input             read_mode,
    input      [7:0]  write_data,
    input      [7:0]  reg_addr,
    input             miso,
    output reg        busy,
    output reg        done,
    output reg        ack_error,
    output reg [95:0] read_data,
    output            sck,
    output            mosi,
    output            cs_n
);

localparam integer HALF_DIV_VALUE = ((CLK_HZ + (SPI_HZ * 2) - 1) / (SPI_HZ * 2)) < 1 ? 1 : ((CLK_HZ + (SPI_HZ * 2) - 1) / (SPI_HZ * 2));
localparam integer HALF_DIV_WIDTH = (HALF_DIV_VALUE <= 1) ? 1 : $clog2(HALF_DIV_VALUE + 1);
localparam [2:0] ST_IDLE   = 3'd0;
localparam [2:0] ST_LOW    = 3'd1;
localparam [2:0] ST_HIGH   = 3'd2;
localparam [2:0] ST_FINISH = 3'd3;

reg [2:0] state;
reg [HALF_DIV_WIDTH-1:0] half_count;
reg [7:0] tx_byte;
reg [7:0] rx_byte;
reg [3:0] bit_index;
reg [3:0] bytes_left;
reg [1:0] byte_phase;
reg active_read;
reg [7:0] latched_write_data;

wire half_tick = (half_count == HALF_DIV_VALUE - 1);

assign cs_n = (state == ST_IDLE);
assign sck = (state == ST_HIGH);
assign mosi = (state == ST_IDLE || state == ST_FINISH) ? 1'b0 : tx_byte[bit_index];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        half_count <= {HALF_DIV_WIDTH{1'b0}};
    end else if (state == ST_IDLE) begin
        half_count <= {HALF_DIV_WIDTH{1'b0}};
    end else if (half_tick) begin
        half_count <= {HALF_DIV_WIDTH{1'b0}};
    end else begin
        half_count <= half_count + 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= ST_IDLE;
        busy <= 1'b0;
        done <= 1'b0;
        ack_error <= 1'b0;
        read_data <= 96'd0;
        tx_byte <= 8'd0;
        rx_byte <= 8'd0;
        bit_index <= 4'd7;
        bytes_left <= 4'd0;
        byte_phase <= 2'd0;
        active_read <= 1'b0;
        latched_write_data <= 8'd0;
    end else begin
        done <= 1'b0;
        if (state == ST_IDLE) begin
            if (start && !busy) begin
                busy <= 1'b1;
                read_data <= 96'd0;
                active_read <= read_mode;
                latched_write_data <= write_data;
                tx_byte <= read_mode ? (reg_addr | 8'h80) : (reg_addr & 8'h7f);
                bit_index <= 4'd7;
                byte_phase <= 2'd0;
                state <= ST_LOW;
            end
        end else if (half_tick) begin
            case (state)
                ST_LOW: begin
                    // SPI mode 0 samples on the rising edge.  ST_LOW is
                    // the low-clock half immediately before SCK rises.
                    if (active_read && byte_phase == 2'd1) begin
                        rx_byte <= {rx_byte[6:0], miso};
                    end
                    state <= ST_HIGH;
                end
                ST_HIGH: begin
                    if (bit_index != 0) begin
                        bit_index <= bit_index - 1'b1;
                        state <= ST_LOW;
                    end else if (byte_phase == 2'd0) begin
                        bit_index <= 4'd7;
                        if (active_read) begin
                            tx_byte <= 8'h00;
                            rx_byte <= 8'd0;
                            bytes_left <= READ_BYTES[3:0];
                            byte_phase <= 2'd1;
                            state <= ST_LOW;
                        end else begin
                            tx_byte <= latched_write_data;
                            byte_phase <= 2'd2;
                            state <= ST_LOW;
                        end
                    end else if (byte_phase == 2'd1) begin
                        read_data <= {read_data[87:0], rx_byte};
                        if (bytes_left == 1) begin
                            state <= ST_FINISH;
                        end else begin
                            bytes_left <= bytes_left - 1'b1;
                            bit_index <= 4'd7;
                            tx_byte <= 8'h00;
                            rx_byte <= 8'd0;
                            state <= ST_LOW;
                        end
                    end else begin
                        state <= ST_FINISH;
                    end
                end
                ST_FINISH: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= ST_IDLE;
                end
                default: begin
                    busy <= 1'b0;
                    state <= ST_IDLE;
                end
            endcase
        end
    end
end

endmodule
