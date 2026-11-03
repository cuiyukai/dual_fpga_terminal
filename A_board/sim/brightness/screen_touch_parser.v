`timescale 1ns/1ps

module screen_touch_parser (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    input  wire       valid,
    output reg        touch_valid,
    output reg [7:0]  page_id,
    output reg [7:0]  control_id,
    output reg        pressed,
    output reg        screen_ready
);
    localparam [3:0] WAIT_HEAD=0, PAGE=1, CONTROL=2, EVENT=3,
                     END1=4, END2=5, END3=6,
                     ALT_CODE=7, ALT_VALUE=8, ALT_END1=9,
                     ALT_END2=10, ALT_END3=11;
    reg [3:0] state;
    reg ready_frame;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= WAIT_HEAD;
            touch_valid <= 1'b0;
            page_id <= 8'd0;
            control_id <= 8'd0;
            pressed <= 1'b0;
            screen_ready <= 1'b0;
            ready_frame <= 1'b0;
        end else begin
            touch_valid <= 1'b0;
            screen_ready <= 1'b0;
            if (valid) begin
                case (state)
                    WAIT_HEAD: begin
                        if (data == 8'h65) begin ready_frame <= 1'b0; state <= PAGE; end
                        else if (data == 8'h88) begin ready_frame <= 1'b1; state <= END1; end
                        else if (data == 8'h80) state <= ALT_CODE;
                    end
                    PAGE: begin page_id <= data; state <= CONTROL; end
                    CONTROL: begin control_id <= data; state <= EVENT; end
                    EVENT: begin pressed <= data[0]; state <= END1; end
                    END1: begin
                        if (data == 8'hff) state <= END2;
                        else state <= WAIT_HEAD;
                    end
                    END2: begin
                        if (data == 8'hff) state <= END3;
                        else state <= WAIT_HEAD;
                    end
                    END3: begin
                        if (data == 8'hff) begin
                            if (!ready_frame)
                                touch_valid <= 1'b1;
                            if (ready_frame) screen_ready <= 1'b1;
                        end
                        state <= WAIT_HEAD;
                    end
                    ALT_CODE: begin
                        if (data == 8'h3f || data == 8'h80) state <= ALT_VALUE;
                        else state <= WAIT_HEAD;
                    end
                    ALT_VALUE: begin
                        if (data == 8'he0) begin
                            page_id <= 8'd0;
                            control_id <= 8'd1;
                            pressed <= 1'b1;
                            state <= ALT_END1;
                        end else if (data == 8'hfe) begin
                            page_id <= 8'd0;
                            control_id <= 8'd1;
                            pressed <= 1'b0;
                            state <= ALT_END2;
                        end else state <= WAIT_HEAD;
                    end
                    ALT_END1: begin
                        if (data == 8'hfe) state <= ALT_END2;
                        else state <= WAIT_HEAD;
                    end
                    ALT_END2: begin
                        if (data == 8'hfe) state <= ALT_END3;
                        else state <= WAIT_HEAD;
                    end
                    ALT_END3: begin
                        if (data == 8'hfe) touch_valid <= 1'b1;
                        state <= WAIT_HEAD;
                    end
                    default: state <= WAIT_HEAD;
                endcase
            end
        end
    end
endmodule
