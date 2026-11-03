`timescale 1ns/1ps
module tb_video;
    reg clk = 0, rst_n = 0;
    always #20 clk = ~clk;
    reg [23:0] image [0:307199];
    initial $readmemh("pic2.hex", image);

    reg [9:0] hc = 0, vc = 0;
    always @(posedge clk) begin
        if (hc == 799) begin hc <= 0; vc <= (vc == 524) ? 10'd0 : vc + 1'b1; end
        else hc <= hc + 1'b1;
    end
    wire de = (hc < 640) && (vc < 480);
    wire vs = !(vc == 490 || vc == 491);
    wire [23:0] rgb_in = de ? image[vc*640 + hc] : 24'h0;

    reg [6:0] pct = 100, vol = 100; reg show = 0;
    wire vso, deo; wire [23:0] rgbo;
    screen_brightness_osd dut (.pix_clk(clk), .pix_rst_n(rst_n), .pct_async(pct),
        .vol_pct_async(vol), .show_async(show), .vsync(vs), .de(de), .rgb_in(rgb_in),
        .vsync_out(vso), .de_out(deo), .rgb_out(rgbo));

    integer fd = 0, dumping = 0, de_cnt = 0;
    always @(posedge clk) begin
        if (deo && dumping) begin
            $fwrite(fd, "%06x\n", rgbo);
            de_cnt = de_cnt + 1;
        end
    end

    task wait_frame;  // 等到输入回到第 0 行第 0 列
        begin
            @(posedge clk); while (!(hc == 0 && vc == 0)) @(posedge clk);
        end
    endtask
    task shot(input integer n);
        reg [8*16-1:0] name;
        begin
            wait_frame; wait_frame;
            $sformat(name, "shot_%0d.hex", n);
            fd = $fopen(name, "w"); dumping = 1; de_cnt = 0;
            wait_frame; repeat (8) @(posedge clk);   // 输出晚 4 拍，多等几拍收完最后一点
            dumping = 0; $fclose(fd);
            $display("shot %0d pct=%0d vol=%0d show=%0d de_count=%0d", n, pct, vol, show, de_cnt);
        end
    endtask

    initial begin
        #200 rst_n = 1;
        show = 0; pct = 100; vol = 100; shot(0);   // 应与原图逐位相同
        show = 1; pct = 100; vol = 100; shot(1);
        pct = 37;  vol = 45;            shot(2);
        pct = 5;   vol = 7;             shot(3);
        pct = 0;   vol = 0;             shot(4);
        show = 0; pct = 60; vol = 88;   shot(5);
        show = 1; pct = 100; vol = 10;  shot(6);
        $finish;
    end
endmodule
