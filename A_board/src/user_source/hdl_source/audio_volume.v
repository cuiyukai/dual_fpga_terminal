`timescale 1ns/1ps

module audio_volume (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [6:0]  pct_async,             // 0~100，clk 域寄存器输出
    input  wire        valid_in,
    input  wire [23:0] left_in,
    input  wire [23:0] right_in,
    output reg         valid_out,
    output reg  [23:0] left_out,
    output reg  [23:0] right_out
);
    wire [8:0] gain;
    pct_sync u_pct_sync (
        .clk       (clk),
        .rst_n     (rst_n),
        .pct_async (pct_async),
        .pct       (),
        .gain      (gain)
    );

    wire signed [33:0] mul_l = $signed(left_in)  * $signed({1'b0, gain});
    wire signed [33:0] mul_r = $signed(right_in) * $signed({1'b0, gain});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            left_out  <= 24'd0;
            right_out <= 24'd0;
        end else begin
            valid_out <= valid_in;
            left_out  <= gain[8] ? left_in  : mul_l[31:8];
            right_out <= gain[8] ? right_in : mul_r[31:8];
        end
    end
endmodule
