`timescale 1ns/1ps

module menu_corner_osd #(
    parameter [23:0] CORE = 24'hFFFFFF,   // 线色
    parameter [23:0] GLOW = 24'hA0DCFF,   // 光晕：与背景 1:1 平均
    parameter [23:0] TICK = 24'h22C55E    // 复选框对勾
)(
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        show_async,      // 当前为第一页且 B 板处于菜单 / 调节时显示
    input  wire [2:0]  cursor_async,
    input  wire        blink_async,
    input  wire        chk_show_async,   // 流程第 1 / 2 步页面：画复选项光标和勾
    input  wire        chk_step2_async,  // 0 第 1 步 / 1 第 2 步
    input  wire [1:0]  chk_cursor_async,
    input  wire [3:0]  chk_mask_async,   // 已勾的复选项
    input  wire        vsync,
    input  wire        de,
    input  wire [23:0] rgb_in,
    output wire        vsync_out,
    output wire        de_out,
    output wire [23:0] rgb_out
);
    localparam integer LATENCY = 2;

    // ---------------- 参数同步与帧边界锁存 ----------------
    (* ASYNC_REG = "TRUE" *) reg [12:0] ctl_meta, ctl_sync;
    reg [8:0] ctl_last;               // 勾（bit12~9）不参与稳定判断
    reg       vs_q;
    reg       frame_show;
    reg       frame_big;              // 臂长 24（否则 12）
    reg [9:0] gx0, gy0, gx1, gy1;     // 光晕外框：左上角 / 右下角
    reg [4:0] frame_cnt;
    wire      frame_start = vs_q && !vsync;

    wire [2:0] cur      = ctl_sync[2:0];
    wire       ctl_hold = (ctl_sync[8:0] == ctl_last);
    wire       blink_on = ctl_sync[4] && frame_cnt[4];
    wire       chk      = ctl_sync[5];
    // 查表序号：0~5 菜单项，8~11 第 1 步卡片，12~14 第 2 步参数格
    wire [3:0] sel      = chk ? {1'b1, ctl_sync[6], ctl_sync[8:7]} : {1'b0, cur};

    // 按钮外框（x0, y0, x1, y1）外扩 3 像素再外扩 1 像素光晕
    reg [9:0] bx0, by0, bx1, by1;
    reg       big;
    always @* begin
        case (sel)
            4'd0:    begin bx0 = 10'd431; by0 = 10'd17;  bx1 = 10'd517; by1 = 10'd54;  big = 1'b0; end  // 声音
            4'd1:    begin bx0 = 10'd528; by0 = 10'd17;  bx1 = 10'd614; by1 = 10'd54;  big = 1'b0; end  // 亮度
            4'd2:    begin bx0 = 10'd24;  by0 = 10'd157; bx1 = 10'd212; by1 = 10'd424; big = 1'b1; end  // 操作流程
            4'd3:    begin bx0 = 10'd225; by0 = 10'd157; bx1 = 10'd413; by1 = 10'd424; big = 1'b1; end  // 实体检测
            4'd4:    begin bx0 = 10'd426; by0 = 10'd157; bx1 = 10'd614; by1 = 10'd424; big = 1'b1; end  // 摄像头显示
            4'd8:    begin bx0 = 10'd554; by0 = 10'd125; bx1 = 10'd620; by1 = 10'd160; big = 1'b0; end  // 第 1 步：工作台
            4'd9:    begin bx0 = 10'd554; by0 = 10'd171; bx1 = 10'd620; by1 = 10'd205; big = 1'b0; end  // 刀具
            4'd10:   begin bx0 = 10'd554; by0 = 10'd217; bx1 = 10'd620; by1 = 10'd251; big = 1'b0; end  // 陀螺仪
            4'd11:   begin bx0 = 10'd554; by0 = 10'd261; bx1 = 10'd620; by1 = 10'd295; big = 1'b0; end  // 影像系统
            4'd12:   begin bx0 = 10'd14;  by0 = 10'd390; bx1 = 10'd180; by1 = 10'd456; big = 1'b0; end  // 第 2 步：参数 1
            4'd13:   begin bx0 = 10'd190; by0 = 10'd390; bx1 = 10'd346; by1 = 10'd456; big = 1'b0; end  // 参数 2
            4'd14:   begin bx0 = 10'd356; by0 = 10'd390; bx1 = 10'd626; by1 = 10'd432; big = 1'b0; end  // 参数 3（避开下方按钮）
            default: begin bx0 = 10'd521; by0 = 10'd439; bx1 = 10'd614; by1 = 10'd472; big = 1'b0; end  // 休眠模式
        endcase
    end

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            ctl_meta   <= 13'd0;
            ctl_sync   <= 13'd0;
            ctl_last   <= 9'd0;
            vs_q       <= 1'b1;
            frame_show <= 1'b0;
            frame_big  <= 1'b0;
            gx0        <= 10'd0;
            gy0        <= 10'd0;
            gx1        <= 10'd0;
            gy1        <= 10'd0;
            frame_cnt  <= 5'd0;
        end else begin
            ctl_meta <= {chk_mask_async, chk_cursor_async, chk_step2_async, chk_show_async,
                         blink_async, show_async, cursor_async};
            ctl_sync <= ctl_meta;
            ctl_last <= ctl_sync[8:0];
            vs_q     <= vsync;
            if (frame_start) begin
                frame_cnt <= frame_cnt + 1'b1;
                // 光标不稳定的那一帧沿用上一帧位置
                if (ctl_hold) begin
                    frame_show <= chk || (ctl_sync[3] && (cur <= 3'd5) && !blink_on);
                    frame_big  <= big;
                    gx0        <= bx0 - 10'd4;
                    gy0        <= by0 - 10'd4;
                    gx1        <= bx1 + 10'd4;
                    gy1        <= by1 + 10'd4;
                end
            end
        end
    end

    // ---------------- 像素坐标 ----------------
    reg [9:0] pixel_x, pixel_y;
    reg       de_q;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            pixel_x <= 10'd0;
            pixel_y <= 10'd0;
            de_q    <= 1'b0;
        end else begin
            de_q    <= de;
            pixel_x <= de ? pixel_x + 10'd1 : 10'd0;
            if (!vsync)
                pixel_y <= 10'd0;
            else if (de_q && !de)
                pixel_y <= pixel_y + 1'b1;
        end
    end

    wire [9:0] tl_x = pixel_x - gx0;
    wire [9:0] tl_y = pixel_y - gy0;
    wire [9:0] br_x = gx1 - pixel_x;
    wire [9:0] br_y = gy1 - pixel_y;
    function [4:0] sat5;
        input [9:0] x;
        sat5 = (x[9:5] != 5'd0) ? 5'd31 : x[4:0];
    endfunction
    function le_arm;                      // u <= 臂长（a = 24 / 12）
        input [4:0] u;
        input       big;
        le_arm = big ? (u <= 5'd24) : (u <= 5'd12);
    endfunction
    function le_arm1;                     // u <= 臂长 + 1
        input [4:0] u;
        input       big;
        le_arm1 = big ? (u <= 5'd25) : (u <= 5'd13);
    endfunction

    function corner_glow;
        input [4:0] u, v;
        input       big;
        corner_glow = (v < 5'd4 && le_arm1(u, big)) || (u < 5'd4 && le_arm1(v, big));
    endfunction

    function corner_core;
        input [4:0] u, v;
        input       big;
        corner_core = ((v == 5'd1 || v == 5'd2) && u != 5'd0 && le_arm(u, big)) ||
                      ((u == 5'd1 || u == 5'd2) && v != 5'd0 && le_arm(v, big));
    endfunction

    wire [4:0] tlx5 = sat5(tl_x), tly5 = sat5(tl_y), brx5 = sat5(br_x), bry5 = sat5(br_y);
    wire hit_glow = corner_glow(tlx5, tly5, frame_big) || corner_glow(brx5, bry5, frame_big);
    wire hit_core = corner_core(tlx5, tly5, frame_big) || corner_core(brx5, bry5, frame_big);
    wire show_px  = de && frame_show;

    // 窗口内坐标 = 低 4 位相减（窗口宽 16，模 16 结果即为偏移）。
    localparam [9:0] X1   = 10'd553;
    localparam [9:0] Y1_0 = 10'd129, Y1_1 = 10'd173, Y1_2 = 10'd217, Y1_3 = 10'd261;
    localparam [9:0] Y2   = 10'd406;
    localparam [9:0] X2_0 = 10'd20,  X2_1 = 10'd196, X2_2 = 10'd362;

    function win_nx;
        input       cur;
        input [9:0] p, o;
        win_nx = (p == o - 10'd1) ? 1'b1 : (p == o + 10'd15) ? 1'b0 : cur;
    endfunction

    reg       x1_on;
    reg [2:0] x2_on;
    reg [3:0] y1_on;
    reg       y2_on;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            x1_on <= 1'b0;
            x2_on <= 3'd0;
            y1_on <= 4'd0;
            y2_on <= 1'b0;
        end else begin
            if (!de) begin
                x1_on <= 1'b0;
                x2_on <= 3'd0;
            end else begin
                x1_on    <= win_nx(x1_on,    pixel_x, X1);
                x2_on[0] <= win_nx(x2_on[0], pixel_x, X2_0);
                x2_on[1] <= win_nx(x2_on[1], pixel_x, X2_1);
                x2_on[2] <= win_nx(x2_on[2], pixel_x, X2_2);
            end
            if (!vsync) begin
                y1_on <= 4'd0;
                y2_on <= 1'b0;
            end else if (de_q && !de) begin
                y1_on[0] <= win_nx(y1_on[0], pixel_y, Y1_0);
                y1_on[1] <= win_nx(y1_on[1], pixel_y, Y1_1);
                y1_on[2] <= win_nx(y1_on[2], pixel_y, Y1_2);
                y1_on[3] <= win_nx(y1_on[3], pixel_y, Y1_3);
                y2_on    <= win_nx(y2_on,    pixel_y, Y2);
            end
        end
    end

    // 短笔画：u 2~6 且 v-u 4~6；长笔画：u 6~13 且 u+v 16~18
    function tick;
        input [3:0] u, v;
        reg   [4:0] d, s;
        begin
            d    = {1'b0, v} - {1'b0, u};
            s    = {1'b0, u} + {1'b0, v};
            tick = (u >= 4'd2 && u <= 4'd6  && d >= 5'd4  && d <= 5'd6) ||
                   (u >= 4'd6 && u <= 4'd13 && s >= 5'd16 && s <= 5'd18);
        end
    endfunction

    wire [3:0] chk_mask = ctl_sync[12:9];
    wire       chk_step2 = ctl_sync[6];
    wire [3:0] row1 = y1_on;
    wire [2:0] col2 = x2_on;
    wire       box1 = x1_on && |(row1 & chk_mask);
    wire       box2 = y2_on && |(col2 & chk_mask[2:0]);
    // 先选窗口内坐标，再共用一个 tick 判断
    wire [3:0] tu   = pixel_x[3:0] - (!chk_step2 ? X1[3:0] :
                                      col2[0] ? X2_0[3:0] : col2[1] ? X2_1[3:0] : X2_2[3:0]);
    wire [3:0] tv   = pixel_y[3:0] - ( chk_step2 ? Y2[3:0] :
                                      row1[0] ? Y1_0[3:0] : row1[1] ? Y1_1[3:0] :
                                      row1[2] ? Y1_2[3:0] : Y1_3[3:0]);
    wire       hit_tick = chk && (chk_step2 ? box2 : box1) && tick(tu, tv);

    // ---------------- 第 1 拍：混色 ----------------
    reg [23:0] rgb_d1, rgb_d2;
    reg        core_d1, glow_d1, tick_d1;
    reg [LATENCY-1:0] de_pipe, vs_pipe;

    wire [8:0] avg_r = rgb_d1[23:16] + GLOW[23:16];
    wire [8:0] avg_g = rgb_d1[15:8]  + GLOW[15:8];
    wire [8:0] avg_b = rgb_d1[7:0]   + GLOW[7:0];

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            rgb_d1  <= 24'd0;
            rgb_d2  <= 24'd0;
            core_d1 <= 1'b0;
            glow_d1 <= 1'b0;
            tick_d1 <= 1'b0;
            de_pipe <= {LATENCY{1'b0}};
            vs_pipe <= {LATENCY{1'b1}};
        end else begin
            de_pipe <= {de_pipe[LATENCY-2:0], de};
            vs_pipe <= {vs_pipe[LATENCY-2:0], vsync};

            rgb_d1  <= rgb_in;
            core_d1 <= show_px && hit_core;
            glow_d1 <= show_px && hit_glow;
            tick_d1 <= de && hit_tick;

            rgb_d2 <= core_d1 ? CORE :
                      glow_d1 ? {avg_r[8:1], avg_g[8:1], avg_b[8:1]} :
                      tick_d1 ? TICK : rgb_d1;
        end
    end

    assign rgb_out   = rgb_d2;
    assign de_out    = de_pipe[LATENCY-1];
    assign vsync_out = vs_pipe[LATENCY-1];
endmodule
