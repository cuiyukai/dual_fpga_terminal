`timescale 1ns/1ps

module tb_audio_spectrum_osd;
    localparam integer MUSIC_FRAMES = 40;
    localparam integer TOTAL_FRAMES = 52;
    localparam integer CHECK_FRAME  = 6;
    localparam integer SHOT_A = 12, SHOT_B = 30, SHOT_C = 51;

    reg pix_clk = 0;
    reg rst_n = 0;
    always #20 pix_clk = ~pix_clk;     // 25 MHz

    reg [23:0] image [0:307199];
    reg [15:0] music [0:57599];
    initial $readmemh("pic5.hex", image);
    initial $readmemh("music.hex", music);

    reg [9:0] hc = 0, vc = 0;
    wire de = (hc < 640) && (vc < 480);
    wire [23:0] rgb_in = de ? image[vc*640 + hc] : 24'd0;
    wire [23:0] rgb_out;
    integer frame_no = 0;

    always @(posedge pix_clk) begin
        if (hc == 799) begin
            hc <= 0;
            if (vc == 524) begin vc <= 0; frame_no <= frame_no + 1; end
            else vc <= vc + 1;
        end else hc <= hc + 1;
    end

    reg audio_valid = 0;
    reg signed [23:0] left = 0, right = 0;
    integer div = 0, n = 0;
    always @(posedge pix_clk) begin
        audio_valid <= 1'b0;
        left  <= 24'sd0;
        right <= 24'sd0;
        if (div == 520) begin
            div <= 0;
            if (frame_no < MUSIC_FRAMES) begin
                left  <= {music[n], 8'd0};
                right <= {music[n], 8'd0};
                audio_valid <= 1'b1;
                n = n + 1;
            end
        end else div <= div + 1;
    end

    audio_spectrum_osd dut (
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .de(de), .display_enable(1'b1),
        .audio_valid(audio_valid), .audio_left(left), .audio_right(right),
        .rgb_in(rgb_in), .rgb_out(rgb_out)
    );

    integer fd_bars, fd_in, fd_out, fd_shot, k;
    reg [1:0] eng_q = 0;
    always @(posedge pix_clk) begin
        eng_q <= dut.eng;
        // LOAD 开始：记录窗前 1024 点
        if (frame_no == CHECK_FRAME && eng_q == 2'd0 && dut.eng == 2'd1) begin
            fd_in = $fopen("fft_in.txt", "w");
            for (k = 0; k < 1024; k = k + 1)
                $fdisplay(fd_in, "%0d", $signed(dut.ring[(dut.ld_base + k) % 1024]));
            $fdisplay(fd_in, "silent %0d", dut.silent);
            $fclose(fd_in);
        end
        // FFT 结束：记录频点 0..511
        if (frame_no == CHECK_FRAME && eng_q == 2'd2 && dut.eng == 2'd3) begin
            fd_out = $fopen("fft_out.txt", "w");
            for (k = 0; k < 512; k = k + 1)
                $fdisplay(fd_out, "%0d %0d", $signed(dut.fmem[k][35:18]), $signed(dut.fmem[k][17:0]));
            $fclose(fd_out);
        end
        // POST 结束：记录本帧柱高
        if (eng_q == 2'd3 && dut.eng == 2'd0) begin
            $fwrite(fd_bars, "%0d", frame_no);
            for (k = 0; k < 144; k = k + 1)
                $fwrite(fd_bars, " %0d:%0d", dut.st_ram[k][9:3], dut.st_ram[k][18:12]);
            $fwrite(fd_bars, "\n");
        end
    end

    // 每个像素在 negedge 按当前帧号写入对应文件
    always @(negedge pix_clk) begin
        if (de && (frame_no == SHOT_A || frame_no == SHOT_B || frame_no == SHOT_C)) begin
            if (hc == 0 && vc == 0)
                fd_shot = $fopen($sformatf("frame_%0d.hex", frame_no), "w");
            $fdisplay(fd_shot, "%06h", rgb_out);
            if (hc == 639 && vc == 479)
                $fclose(fd_shot);
        end
    end

    initial begin
        fd_bars = $fopen("bars.txt", "w");
        #200 rst_n = 1;
        wait (frame_no == TOTAL_FRAMES);
        $fclose(fd_bars);
        $display("samples=%0d", n);
        $finish;
    end
endmodule
