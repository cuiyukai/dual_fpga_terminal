`timescale 1ns/1ps

module tb_imu_gyro_wave_osd;
    reg ctrl_clk = 0, pix_clk = 0;
    reg rst_n = 0;
    always #10 ctrl_clk = ~ctrl_clk;   // 50 MHz
    always #20 pix_clk  = ~pix_clk;    // 25 MHz

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

    reg sample_valid = 0;
    reg signed [15:0] gx = 0, gy = 0, gz = 0;
    integer n = 0;
    real t;
    // 激励含正弦、快速晃动段和超量程钳位段。
    always begin
        if (frame_no < 2) #40000; else #5000000;
        @(posedge ctrl_clk);
        t = n;
        gx = $rtoi(5000.0 * $sin(t * 0.06));
        gy = $rtoi(3000.0 * $sin(t * 0.15 + 1.0) * ((n % 200) > 100 ? 1.0 : 0.3));
        gz = (n > 600 && n < 640) ? ((n % 6) < 3 ? 16'sd9000 : -16'sd9000)
                                  : $rtoi(1500.0 * $sin(t * 0.03));
        sample_valid = 1;
        @(posedge ctrl_clk);
        sample_valid = 0;
        n = n + 1;
    end

    imu_gyro_wave_osd dut (
        .ctrl_clk(ctrl_clk), .ctrl_rst_n(rst_n), .sample_valid(sample_valid), .calibrated(1'b1),
        .gx(gx), .gy(gy), .gz(gz),
        .pix_clk(pix_clk), .pix_rst_n(rst_n), .de(de), .display_enable(1'b1),
        .rgb_in(rgb_in), .rgb_out(rgb_out)
    );

    integer fd;
    initial begin
        #200 rst_n = 1;
        fd = $fopen("frame_out.hex", "w");
        wait (frame_no == 2);
        while (frame_no == 2) begin
            @(negedge pix_clk);
            if (de) $fdisplay(fd, "%06h", rgb_out);
        end
        $fclose(fd);
        $display("samples=%0d", n);
        $finish;
    end
endmodule
