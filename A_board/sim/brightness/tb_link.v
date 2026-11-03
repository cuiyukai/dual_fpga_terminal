`timescale 1ns/1ps
module tb_link;
    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;   // 50 MHz
    reg scr = 1;
    localparam integer BIT_NS = 104167;

    task send_byte(input [7:0] b);
        integer k;
        begin
            scr = 0; #(BIT_NS);
            for (k = 0; k < 8; k = k + 1) begin scr = b[k]; #(BIT_NS); end
            scr = 1; #(BIT_NS);
        end
    endtask
    task send_num(input [7:0] head, input [31:0] v);
        begin
            send_byte(head); send_byte(v[7:0]); send_byte(v[15:8]);
            send_byte(v[23:16]); send_byte(v[31:24]);
            send_byte(8'hFF); send_byte(8'hFF); send_byte(8'hFF);
        end
    endtask

    wire [7:0] d; wire v, tv, tp, nv, nvol; wire [7:0] pg, ct; wire [6:0] nval;
    screen_uart_rx #(.CLK_HZ(50_000_000), .BAUD(9600)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(scr), .data(d), .valid(v), .framing_error());
    screen_touch_parser u_p (.clk(clk), .rst_n(rst_n), .data(d), .valid(v),
        .touch_valid(tv), .page_id(pg), .control_id(ct), .pressed(tp), .screen_ready(),
        .number_valid(nv), .number_value(nval), .number_volume(nvol));
    wire link;
    imu_link_tx #(.CLK_HZ(50_000_000), .BAUD(115200)) u_tx (
        .clk(clk), .rst_n(rst_n), .sample_valid(1'b0), .attitude_valid(1'b0),
        .calibrated(1'b0), .sensor_error(1'b0), .gx(16'sd0), .gy(16'sd0), .gz(16'sd0),
        .roll_deg_x10(16'sd123), .pitch_deg_x10(-16'sd45), .yaw_deg_x10(16'sd0),
        .touch_valid(tv), .touch_pressed(tp), .touch_page(pg), .touch_control(ct),
        .bright_valid(nv && !nvol), .bright_value(nval),
        .vol_valid(nv && nvol), .vol_value(nval), .tx(link));
    wire np; wire [6:0] pct, vol; wire signed [15:0] roll, pitch;
    imu_link_rx #(.CLK_HZ(50_000_000), .BAUD(115200)) u_arx (
        .clk(clk), .rst_n(rst_n), .rx(link), .sample_valid(), .attitude_valid(),
        .calibrated(), .sensor_error(), .gx(), .gy(), .gz(),
        .roll_deg_x10(roll), .pitch_deg_x10(pitch), .yaw_deg_x10(),
        .next_pulse(np), .brightness_pct(pct), .volume_pct(vol));

    integer nps = 0, frames = 0, fails = 0;
    always @(posedge clk) if (np) nps = nps + 1;
    always @(posedge clk) if (u_arx.byte_idx == 5'd21 && u_arx.rx_valid) frames = frames + 1;
    always @(posedge clk) if (nv) $display("%0t parser number %0d volume=%0d", $time, nval, nvol);

    task expect(input [6:0] eb, input [6:0] ev);
        begin
            #3_000_000;   // 等链路帧送达（22 字节 @115200 ≈ 1.9ms）
            if (pct != eb || vol != ev || roll != 123 || pitch != -45) fails = fails + 1;
            $display("%0t A bright=%0d(exp %0d) vol=%0d(exp %0d) %s roll=%0d pitch=%0d", $time,
                     pct, eb, vol, ev,
                     (pct == eb && vol == ev && roll == 123 && pitch == -45) ? "OK" : "FAIL",
                     roll, pitch);
        end
    endtask

    initial begin
        #1000 rst_n = 1;
        #200_000;
        $display("reset bright=%0d vol=%0d", pct, vol);
        send_num(8'h71, 50);  expect(50, 100);
        send_num(8'h72, 30);  expect(50, 30);
        send_num(8'h71, 7);   expect(7, 30);
        send_num(8'h72, 0);   expect(7, 0);
        send_num(8'h71, 0);   expect(0, 0);
        send_num(8'h72, 200); expect(0, 100);
        send_num(8'h72, 32'h00010032); expect(0, 100);   // 高字节非零 -> 100
        send_num(8'h71, 200); expect(100, 100);
        send_num(8'h71, 73);  expect(73, 100);
        send_num(8'h72, 64);  expect(73, 64);
        // 原有触摸帧：控件 1 按下 -> next_pulse
        send_byte(8'h65); send_byte(8'h00); send_byte(8'h01); send_byte(8'h01);
        send_byte(8'hFF); send_byte(8'hFF); send_byte(8'hFF);
        expect(73, 64);
        // 损坏的数值帧（结束符错误）不得改变数值
        send_byte(8'h72); send_byte(8'd20); send_byte(0); send_byte(0); send_byte(0);
        send_byte(8'hFF); send_byte(8'hFE); send_byte(8'hFF);
        expect(73, 64);
        $display("next_pulse count %0d (expect 1), A good frames %0d, fails %0d", nps, frames, fails);
        $finish;
    end
endmodule
