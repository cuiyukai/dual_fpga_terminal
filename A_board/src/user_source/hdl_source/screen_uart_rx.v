`timescale 1ns/1ps

module screen_uart_rx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 9_600
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg [7:0]  data,
    output reg        valid,
    output reg        framing_error
);
    localparam integer BIT_TICKS  = (CLK_HZ + BAUD/2) / BAUD;
    localparam integer HALF_TICKS = BIT_TICKS / 2;
    localparam integer TICK_WIDTH = (BIT_TICKS < 2) ? 2 : $clog2(BIT_TICKS + 1);
    reg rx_meta, rx_sync;
    reg active;
    reg [3:0] bit_index;
    reg [TICK_WIDTH-1:0] tick_count;
    reg [7:0] shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
            active <= 1'b0;
            bit_index <= 3'd0;
            tick_count <= {TICK_WIDTH{1'b0}};
            shift <= 8'd0;
            data <= 8'd0;
            valid <= 1'b0;
            framing_error <= 1'b0;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
            valid <= 1'b0;
            framing_error <= 1'b0;
            if (!active) begin
                if (!rx_sync) begin
                    active <= 1'b1;
                    bit_index <= 3'd0;
                    tick_count <= HALF_TICKS[TICK_WIDTH-1:0];
                end
            end else if (tick_count != 0) begin
                tick_count <= tick_count - 1'b1;
            end else if (bit_index == 0) begin
                if (!rx_sync) begin
                    bit_index <= 3'd1;
                    tick_count <= BIT_TICKS[TICK_WIDTH-1:0];
                end else begin
                    active <= 1'b0;
                    framing_error <= 1'b1;
                end
            end else if (bit_index <= 4'd8) begin
                shift[bit_index-1] <= rx_sync;
                bit_index <= bit_index + 1'b1;
                tick_count <= BIT_TICKS[TICK_WIDTH-1:0];
            end else begin
                active <= 1'b0;
                if (rx_sync) begin
                    data <= shift;
                    valid <= 1'b1;
                end else begin
                    framing_error <= 1'b1;
                end
            end
        end
    end
endmodule
