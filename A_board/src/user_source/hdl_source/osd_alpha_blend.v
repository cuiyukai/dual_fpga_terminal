`timescale 1ns/1ps

module osd_alpha_blend #(
    parameter USE_DSP = 0
) (
    input  wire [23:0] bg,
    input  wire [23:0] fg,
    input  wire [3:0]  alpha,
    output wire [23:0] out
);
    // alpha=15 时 w=16，墨色完全覆盖背景，直接输出 fg
    wire [4:0] w = {1'b0, alpha} + {4'd0, alpha[3]};

    function [7:0] mix;
        input [7:0] f;
        input [7:0] b;
        input [4:0] wt;
        reg  [11:0] d;
        reg  [11:0] p;
        integer k;
        begin
            d = {4'd0, f} - {4'd0, b};
            p = 12'd0;
            for (k = 0; k < 4; k = k + 1)
                if (wt[k])
                    p = p + (d << k);
            mix = wt[4] ? f : b + p[11:4];
        end
    endfunction

    function [7:0] mix_dsp;
        input [7:0] f;
        input [7:0] b;
        input [4:0] wt;
        reg signed [8:0]  d;
        reg signed [14:0] p;
        begin
            d       = $signed({1'b0, f}) - $signed({1'b0, b});
            p       = d * $signed({1'b0, wt[3:0]});   // wt = 16 时不经乘法
            mix_dsp = wt[4] ? f : b + p[11:4];
        end
    endfunction

    generate
        if (USE_DSP) begin : g_dsp
            assign out = {mix_dsp(fg[23:16], bg[23:16], w),
                          mix_dsp(fg[15:8],  bg[15:8],  w),
                          mix_dsp(fg[7:0],   bg[7:0],   w)};
        end else begin : g_lut
            assign out = {mix(fg[23:16], bg[23:16], w),
                          mix(fg[15:8],  bg[15:8],  w),
                          mix(fg[7:0],   bg[7:0],   w)};
        end
    endgenerate
endmodule
