`timescale 1ns/1ps

module alarm_sparks (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               frame_start,   // 单拍：场同步起点
    input  wire               run,           // 特效播放中（帧计数只在播放时走）
    input  wire signed [10:0] xp,            // 当前像素列号 - 320
    input  wire signed [9:0]  yp,            // 当前像素行号 - 240
    output reg  [1:0]         lvl            // 晚两拍：亮度档 0~3
);
    reg [9:0] cnt;                           // 帧计数：上升偏移 / 闪烁相位

    function [11:0] hash12;
        input [11:0] v0;
        reg   [11:0] v1, v2, v3, v4;
        begin
            v1 = v0 ^ {v0[6:0], 5'd0};
            v2 = v1 + 12'h9B7;
            v3 = v2 ^ {4'd0, v2[11:4]};
            v4 = v3 ^ {v3[8:0], 3'd0};
            hash12 = v4 ^ {7'd0, v4[11:7] & v4[6:2]};
        end
    endfunction

    // 平方表（0~3 -> 0 / 1 / 4 / 9），不用乘法器
    function [3:0] sq2;
        input [1:0] a;
        sq2 = (a == 2'd0) ? 4'd0 : (a == 2'd1) ? 4'd1 : (a == 2'd2) ? 4'd4 : 4'd9;
    endfunction
    function [1:0] spot;
        input [1:0] s;
        input [1:0] ax;
        input [1:0] ay;
        reg   [4:0] d2;
        begin
            d2 = {1'b0, sq2(ax)} + {1'b0, sq2(ay)};
            case (s)
                2'd0:    spot = (d2 == 5'd0) ? 2'd3 : (d2 <= 5'd1) ? 2'd1 : 2'd0;
                2'd1:    spot = (d2 == 5'd0) ? 2'd3 : (d2 <= 5'd1) ? 2'd2 : (d2 <= 5'd2) ? 2'd1 : 2'd0;
                2'd2:    spot = (d2 <= 5'd1) ? 2'd3 : (d2 <= 5'd2) ? 2'd2 : (d2 <= 5'd4) ? 2'd1 : 2'd0;
                default: spot = (d2 <= 5'd1) ? 2'd2 : (d2 <= 5'd4) ? 2'd1 : 2'd0;
            endcase
        end
    endfunction

    wire [4:0] cxi  = xp[8:4];
    wire       fast = cxi[0] ^ cxi[2];
    wire [9:0] yy   = yp + (fast ? cnt : {1'b0, cnt[9:1]});
    wire       win  = (xp >= -11'sd224) && (xp < 11'sd224) && (yp >= 10'sd48) && (yp < 10'sd216);

    reg        win1, fd2_1, fd1_1;
    reg [4:0]  cx1;
    reg [5:0]  cy1;
    reg [3:0]  px1, py1;

    // ---------------- 第 2 拍：哈希、亮度 ----------------
    wire [11:0] h     = hash12({cy1, 1'b0, cx1} ^ {cx1, 7'd0});
    wire        ex    = (h[2:0] == 3'd0) || ((h[2:0] == 3'd1) && h[11]);
    wire [4:0]  dx    = {1'b0, px1} - ({2'b0, h[5:3]} + 5'd4);   // px - (4 + h[5:3])
    wire [4:0]  dy    = {1'b0, py1} - ({2'b0, h[8:6]} + 5'd4);
    wire [4:0]  adx   = dx[4] ? 5'd0 - dx : dx;
    wire [4:0]  ady   = dy[4] ? 5'd0 - dy : dy;
    wire        near  = (adx <= 5'd3) && (ady <= 5'd3);
    wire [1:0]  base  = spot(h[10:9], adx[1:0], ady[1:0]);
    wire [3:0]  ph    = h[11:8] + cnt[7:4];
    wire [1:0]  l_tw  = (ph <= 4'd1 && base != 2'd0) ? base - 2'd1 : base;
    wire [1:0]  l_fd  = fd2_1 ? ((l_tw >= 2'd2) ? l_tw - 2'd2 : 2'd0) :
                        fd1_1 ? ((l_tw != 2'd0) ? l_tw - 2'd1 : 2'd0) : l_tw;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt   <= 10'd0;
            win1  <= 1'b0;
            fd2_1 <= 1'b0;
            fd1_1 <= 1'b0;
            cx1   <= 5'd0;
            cy1   <= 6'd0;
            px1   <= 4'd0;
            py1   <= 4'd0;
            lvl   <= 2'd0;
        end else begin
            if (frame_start && run)
                cnt <= cnt + 1'b1;
            win1  <= win;
            fd2_1 <= (yp < 10'sd64);
            fd1_1 <= (yp < 10'sd80);
            cx1   <= cxi;
            cy1   <= yy[9:4];
            px1   <= xp[3:0];
            py1   <= yy[3:0];
            lvl   <= (win1 && ex && near) ? l_fd : 2'd0;
        end
    end
endmodule
