`timescale 1ns/1ps
module tb_audio_volume;
    reg clk = 0, rst_n = 0;
    always #20 clk = ~clk;
    reg  [6:0]  pct = 100;
    reg         vin = 0;
    reg  [23:0] lin = 0, rin = 0;
    wire        vout;
    wire [23:0] lout, rout;
    audio_volume dut (.clk(clk), .rst_n(rst_n), .pct_async(pct), .valid_in(vin),
        .left_in(lin), .right_in(rin), .valid_out(vout), .left_out(lout), .right_out(rout));

    integer gain, n, errs = 0, total = 0, seed = 7;
    reg signed [47:0] el, er;
    reg [23:0] lq, rq;
    task run(input [6:0] p);
        begin
            pct = p; repeat (10) @(posedge clk);
            gain = (p >= 100) ? 256 : (p * 656) >> 8;
            for (n = 0; n < 2000; n = n + 1) begin
                @(negedge clk);
                vin = 1; lin = $random(seed); rin = (n < 4) ? {1'b1, 23'd0} + n : $random(seed);
                lq = lin; rq = rin;
                @(negedge clk); vin = 0;
                el = ($signed(lq) * gain) >>> 8;
                er = ($signed(rq) * gain) >>> 8;
                total = total + 1;
                if (!vout || lout != el[23:0] || rout != er[23:0]) begin
                    errs = errs + 1;
                    if (errs < 5) $display("ERR pct=%0d in %h/%h out %h/%h exp %h/%h v=%b",
                                           p, lq, rq, lout, rout, el[23:0], er[23:0], vout);
                end
            end
            $display("pct %3d gain %3d done, errs so far %0d", p, gain, errs);
        end
    endtask
    initial begin
        #100 rst_n = 1;
        run(100); run(99); run(64); run(50); run(37); run(1); run(0);
        $display("total %0d errors %0d", total, errs);
        $finish;
    end
endmodule
