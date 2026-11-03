`timescale 1ns/1ps

module pct_sync (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [6:0] pct_async,
    output reg  [6:0] pct,
    output reg  [8:0] gain
);
    (* ASYNC_REG = "TRUE" *) reg [6:0] pct_meta, pct_sync_r;
    reg [6:0] pct_last;
    wire [15:0] gain_mul = pct * 10'd656;   // 656/256 ≈ 256/100，pct<100 时不溢出

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pct_meta   <= 7'd100;
            pct_sync_r <= 7'd100;
            pct_last   <= 7'd100;
            pct        <= 7'd100;
            gain       <= 9'd256;
        end else begin
            pct_meta   <= pct_async;
            pct_sync_r <= pct_meta;
            pct_last   <= pct_sync_r;
            if (pct_sync_r == pct_last)
                pct <= (pct_sync_r > 7'd100) ? 7'd100 : pct_sync_r;
            gain <= (pct >= 7'd100) ? 9'd256 : gain_mul[15:8];
        end
    end
endmodule
