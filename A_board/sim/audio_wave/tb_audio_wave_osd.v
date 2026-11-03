`timescale 1ns/1ps

module tb_audio_wave_osd;
    reg pix_clk = 0;
    reg rst_n = 0;
    always #20 pix_clk = ~pix_clk;     // 25 MHz

    reg [23:0] image [0:307199];
    initial $readmemh("pic5.hex", image);

    reg [9:0] hc = 0, vc = 0;
    wire de = (hc < 640) && (vc < 480);
    wire [23:0] rgb_in = de ? image[vc*640 + hc] : 24'd0;
    wire [23:0] rgb_out;
    reg  [1:0] frame_no = 0;

    always @(posedge pix_clk) begin
        if (hc == 799) begin
            hc <= 0;
            if (vc == 524) begin vc <= 0; frame_no <= frame_no + 1; end
            else vc <= vc + 1;
        end else hc <= hc + 1;
    end

    // 25MHz / 521 ≈ 48kHz；左声道 440Hz + 1320Hz，右声道 440Hz
    reg audio_valid = 0;
    reg signed [23:0] left = 0, right = 0;
    integer div = 0, n = 0;
    real t;
    always @(posedge pix_clk) begin
        audio_valid <= 1'b0;
        left  <= 24'sd0;
        right <= 24'sd0;
        if (div == 520) begin
            div <= 0;
            if (frame_no < 2) begin
                t = n / 48000.0;
                left  <= $rtoi(2500000.0 * $sin(6.2831853 * 440.0 * t) + 1200000.0 * $sin(6.2831853 * 1320.0 * t));
                right <= $rtoi(2500000.0 * $sin(6.2831853 * 440.0 * t));
                audio_valid <= 1'b1;
                n = n + 1;
            end
        end else div <= div + 1;
    end

    audio_wave_osd dut (
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .de(de), .display_enable(1'b1),
        .audio_valid(audio_valid), .audio_left(left), .audio_right(right),
        .rgb_in(rgb_in), .rgb_out(rgb_out)
    );

    // 每个像素在 negedge 按当前帧号写入对应文件，帧边界不会错位
    integer fd_on, fd_off;
    always @(negedge pix_clk) begin
        if (de && frame_no == 2) $fdisplay(fd_on,  "%06h", rgb_out);
        if (de && frame_no == 3) $fdisplay(fd_off, "%06h", rgb_out);
    end

    initial begin
        fd_on  = $fopen("frame_on.hex", "w");
        fd_off = $fopen("frame_off.hex", "w");
        #200 rst_n = 1;
        wait (frame_no == 3);
        wait (frame_no == 0);
        $fclose(fd_on);
        $fclose(fd_off);
        $display("samples=%0d disp_valid=%0d peak=%0d", n, dut.disp_valid, dut.peak_disp);
        $finish;
    end
endmodule
