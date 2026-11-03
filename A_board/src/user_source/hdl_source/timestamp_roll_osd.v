`timescale 1ns/1ps

module timestamp_roll_osd #(
    parameter integer SHOW_FRAMES = 480   // 首页显示时长（帧），60Hz 下 8s，<= 511
)(
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        sleep_async,
    input  wire        show_async,
    input  wire        trig_async,        // B 板主菜单 KEY2 短按事件翻转位（clk 域）
    input  wire [19:0] time_async,
    input  wire        vsync,
    input  wire        de,
    input  wire [23:0] rgb_in,
    output wire        vsync_out,
    output wire        de_out,
    output wire [23:0] rgb_out
);
    localparam integer LATENCY = 3;
    localparam integer X0      = 152;     // 文字左沿（336 像素宽，水平居中）
    localparam integer WY0     = 204;     // 窗口顶行（主数字区 72 行中线 240，下接 36 行倒影）
    localparam integer WH      = 72;      // 数字带每格行数 = 滚动距离
    localparam integer WT      = 108;     // 窗口总行数（主数字区 72 + 倒影 36）
    localparam integer RT      = 4;       // 字形 64 行在窗口内的起始行
    localparam [13:0]  COLON_BASE = 14'd15360;
    localparam [23:0]  WHITE   = 24'hFFFFFF;   // 首页数字
    localparam [23:0]  ICE_HI  = 24'hF4FAFF;   // 休眠数字上半
    localparam [23:0]  ICE_LO  = 24'hC8E8FF;   // 休眠数字下半

    // ---------------- 工具函数 ----------------
    function [3:0] dmax;
        input [2:0] k;
        dmax = (k == 3'd0) ? 4'd2 : (k == 3'd2 || k == 3'd4) ? 4'd5 : 4'd9;
    endfunction
    function [3:0] incm;
        input [3:0] v;
        input [2:0] k;
        incm = (v >= dmax(k)) ? 4'd0 : v + 1'b1;
    endfunction
    // 滚动 ease-out：72 x (1 - (1 - (i+1)/16)^3)
    function [6:0] ease;
        input [3:0] i;
        begin
            case (i)
                4'd0:  ease = 7'd13;  4'd1:  ease = 7'd24;  4'd2:  ease = 7'd33;  4'd3:  ease = 7'd42;
                4'd4:  ease = 7'd49;  4'd5:  ease = 7'd54;  4'd6:  ease = 7'd59;  4'd7:  ease = 7'd63;
                4'd8:  ease = 7'd66;  4'd9:  ease = 7'd68;  4'd10: ease = 7'd70;  4'd11: ease = 7'd71;
                default: ease = 7'd72;
            endcase
        end
    endfunction

    // ---------------- 同步与帧级状态 ----------------
    (* ASYNC_REG = "TRUE" *) reg [22:0] st_meta, st_sync;   // {sleep, trig, show, time}
    reg [19:0] time_last;
    reg [19:0] time_f;
    reg        trig_d;
    reg        vs_q;
    reg        on;                        // 显示中（未开始淡出）
    reg        slp_f;
    reg [8:0]  hold;                      // 剩余显示帧数
    reg [4:0]  lvl;                       // 压暗 / 数字淡入程度 0~16
    reg [5:0]  fcnt;
    reg [23:0] v_old, v_new;              // 6 个数字位 {d0..d5}，d0（时十）在高位
    reg [5:0]  chg;                       // 本次滚动中发生变化的位，bit 5 - k 对应 dk
    reg        rolling;
    reg [3:0]  ri;                        // 滚动帧号
    wire       frame_start = vs_q && !vsync;
    wire [23:0] tnow = {2'b00, time_f[19:18], time_f[17:14], 1'b0, time_f[13:11], time_f[10:7],
                        1'b0, time_f[6:4], time_f[3:0]};
    wire [5:0] diff = {v_old[23:20] != tnow[23:20], v_old[19:16] != tnow[19:16], v_old[15:12] != tnow[15:12],
                       v_old[11:8]  != tnow[11:8],  v_old[7:4]   != tnow[7:4],   v_old[3:0]   != tnow[3:0]};

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            st_meta   <= 23'd0;
            st_sync   <= 23'd0;
            time_last <= 20'd0;
            time_f    <= 20'd0;
            trig_d    <= 1'b0;
            vs_q      <= 1'b1;
            on        <= 1'b0;
            slp_f     <= 1'b0;
            hold      <= 9'd0;
            lvl       <= 5'd0;
            fcnt      <= 6'd0;
            v_old     <= 24'd0;
            v_new     <= 24'd0;
            chg       <= 6'd0;
            rolling   <= 1'b0;
            ri        <= 4'd0;
        end else begin
            st_meta   <= {sleep_async, trig_async, show_async, time_async};
            st_sync   <= st_meta;
            time_last <= st_sync[19:0];
            vs_q      <= vsync;
            if (frame_start) begin
                trig_d <= st_sync[21];
                fcnt   <= fcnt + 1'b1;
                if (st_sync[19:0] == time_last)
                    time_f <= st_sync[19:0];

                if (st_sync[22]) begin
                    on    <= 1'b1;
                    hold  <= 9'd0;
                    slp_f <= 1'b1;
                end else if (!st_sync[20]) begin
                    on <= 1'b0;
                end else if (st_sync[21] != trig_d) begin
                    on    <= 1'b1;
                    hold  <= SHOW_FRAMES;
                    slp_f <= 1'b0;
                end else if (on) begin
                    if (hold == 9'd0)
                        on <= 1'b0;
                    else
                        hold <= hold - 1'b1;
                end
                if (on)
                    lvl <= lvl[4] ? lvl : lvl + 1'b1;
                else
                    lvl <= (lvl == 5'd0) ? lvl : lvl - 1'b1;

                if (lvl == 5'd0) begin
                    v_old   <= tnow;
                    rolling <= 1'b0;
                    chg     <= 6'd0;
                end else if (rolling) begin
                    ri <= ri + 1'b1;
                    if (ri == 4'd15) begin
                        rolling <= 1'b0;
                        v_old   <= v_new;
                        chg     <= 6'd0;
                    end
                end else if (diff != 6'd0) begin
                    rolling <= 1'b1;
                    ri      <= 4'd0;
                    v_new   <= tnow;
                    chg     <= diff;
                end
            end
        end
    end

    // ---------------- 像素坐标、窗口与格内列号 ----------------
    reg [9:0] pixel_x, pixel_y;
    reg       de_q;
    reg       x_on, y_on;
    reg [6:0] wy;                         // 窗口内行号 0~107
    reg [2:0] cidx;                       // 格号：时十 时个 : 分十 分个 : 秒十 秒个
    reg [5:0] cc;                         // 格内列号（数字 0~47，冒号 0~23）
    wire      colon = (cidx == 3'd2) || (cidx == 3'd5);

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            pixel_x <= 10'd0;
            pixel_y <= 10'd0;
            de_q    <= 1'b0;
            x_on    <= 1'b0;
            y_on    <= 1'b0;
            wy      <= 7'd0;
            cidx    <= 3'd0;
            cc      <= 6'd0;
        end else begin
            de_q    <= de;
            pixel_x <= de ? pixel_x + 10'd1 : 10'd0;
            if (!de) begin
                x_on <= 1'b0;
            end else if (pixel_x == X0 - 1) begin
                x_on <= 1'b1;
                cidx <= 3'd0;
                cc   <= 6'd0;
            end else if (pixel_x == X0 + 335) begin
                x_on <= 1'b0;
            end else if (x_on) begin
                if (cc == (colon ? 6'd23 : 6'd47)) begin
                    cc   <= 6'd0;
                    cidx <= cidx + 1'b1;
                end else begin
                    cc <= cc + 1'b1;
                end
            end
            if (!vsync) begin
                pixel_y <= 10'd0;
                y_on    <= 1'b0;
            end else if (de_q && !de) begin
                pixel_y <= pixel_y + 1'b1;
                if (pixel_y == WY0 - 1) begin
                    y_on <= 1'b1;
                    wy   <= 7'd0;
                end else if (pixel_y == WY0 + WT - 1) begin
                    y_on <= 1'b0;
                end else if (y_on) begin
                    wy <= wy + 1'b1;
                end
            end
        end
    end

    wire [7:0] tc   = {1'b0, wy} - RT;                        // 冒号 / 格 0 字形行
    wire       sels = (wy >= WH);
    wire [7:0] ts   = sels ? ({1'b0, wy} - (WH + RT)) : tc;
    wire [7:0] s    = {1'b0, wy} + {1'b0, ease(ri)};
    wire [1:0] cr   = (s >= 2 * WH) ? 2'd2 : (s >= WH) ? 2'd1 : 2'd0;
    wire [7:0] tr   = s - ((cr == 2'd2) ? (2 * WH + RT) : (cr == 2'd1) ? (WH + RT) : RT);
    reg  [10:0] off_s, off_r;             // 数字行偏移 row * 24
    reg  [9:0]  off_c;                    // 冒号行偏移 row * 12
    reg         ok_s, ok_r, ok_c, cel_s, lo_s, lo_r;
    reg  [1:0]  cel_r;
    reg  [1:0]  fsh_l;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            off_s <= 11'd0;
            off_r <= 11'd0;
            off_c <= 10'd0;
            ok_s  <= 1'b0;
            ok_r  <= 1'b0;
            ok_c  <= 1'b0;
            cel_s <= 1'b0;
            cel_r <= 2'd0;
            lo_s  <= 1'b0;
            lo_r  <= 1'b0;
            fsh_l <= 2'd0;
        end else begin
            off_s <= {ts[5:0], 4'b0} + {ts[5:0], 3'b0};
            off_r <= {tr[5:0], 4'b0} + {tr[5:0], 3'b0};
            off_c <= {tc[5:0], 3'b0} + {tc[5:0], 2'b0};
            ok_s  <= (ts[7:6] == 2'd0);
            ok_r  <= (tr[7:6] == 2'd0);
            ok_c  <= (tc[7:6] == 2'd0);
            cel_s <= sels;
            cel_r <= cr;
            lo_s  <= ts[5];
            lo_r  <= tr[5];
            fsh_l <= (wy < WH) ? 2'd0 : (wy < WH + 8) ? 2'd1 : (wy < WH + 20) ? 2'd2 : 2'd3;
        end
    end

    // ---------------- 第 0 拍：选数字与行偏移 ----------------
    reg  [2:0] k;                                     // 数字位序号 0~5
    always @* begin
        case (cidx)
            3'd0:    k = 3'd0;
            3'd1:    k = 3'd1;
            3'd3:    k = 3'd2;
            3'd4:    k = 3'd3;
            3'd6:    k = 3'd4;
            default: k = 3'd5;
        endcase
    end
    wire [3:0]  vo   = v_old[(5 - k) * 4 +: 4];
    wire [3:0]  vn   = v_new[(5 - k) * 4 +: 4];
    wire        ch   = rolling && chg[5 - k] && !colon;
    wire [3:0]  vn1  = incm(vn, k);
    wire [3:0]  vo1  = incm(vo, k);
    reg  [3:0]  code;
    always @* begin
        if (ch)
            code = (cel_r == 2'd0) ? vo : (cel_r == 2'd1) ? vn : vn1;
        else
            code = cel_s ? vo1 : vo;
    end
    wire [10:0] off  = colon ? {1'b0, off_c} : ch ? off_r : off_s;
    wire        ok   = colon ? ok_c : ch ? ok_r : ok_s;
    wire [10:0] offb = off + cc[5:1];                 // 行偏移 + 行内字节（< 1536）

    // 4x4 Bayer 抖动阈值
    function [3:0] bayer;
        input [1:0] x, y;
        begin
            case ({y, x})
                4'd0:  bayer = 4'd0;   4'd1:  bayer = 4'd8;   4'd2:  bayer = 4'd2;   4'd3:  bayer = 4'd10;
                4'd4:  bayer = 4'd12;  4'd5:  bayer = 4'd4;   4'd6:  bayer = 4'd14;  4'd7:  bayer = 4'd6;
                4'd8:  bayer = 4'd3;   4'd9:  bayer = 4'd11;  4'd10: bayer = 4'd1;   4'd11: bayer = 4'd9;
                4'd12: bayer = 4'd15;  4'd13: bayer = 4'd7;   4'd14: bayer = 4'd13;  default: bayer = 4'd5;
            endcase
        end
    endfunction

    // ---------------- 第 1 拍：字库寻址 ----------------
    reg [23:0] rgb_d1, rgb_d2;
    reg [3:0]  code_d1;
    reg [10:0] offb_d1;
    reg        col_d1;
    reg        txt_d1, txt_d2;            // 窗口内且该行有字形
    reg        odd_d1, odd_d2;
    reg        lo_d1,  lo_d2;             // 字形下半（休眠冰蓝）
    reg [1:0]  fsh_d1, fsh_d2;            // 亮度档（倒影渐隐 / 冒号呼吸）
    reg        vis_d1, vis_d2;            // 本像素已淡入（抖动），首页时同时压暗背景

    // 数字基址 code * 1536 = (code * 3) << 9
    wire [4:0]  c3   = {1'b0, code_d1} + {code_d1, 1'b0};
    wire [13:0] addr = col_d1 ? (COLON_BASE + {3'b0, offb_d1}) : ({c3, 9'b0} + {3'b0, offb_d1});

    wire [7:0] font_q;
    stamp_font_rom u_font (
        .clk  (pix_clk),
        .addr (addr),
        .q    (font_q)
    );

    // ---------------- 第 2 拍：混色 ----------------
    function [23:0] shr;                  // 每通道右移 n 位
        input [23:0] c;
        input [1:0]  n;
        begin
            case (n)
                2'd0:    shr = c;
                2'd1:    shr = {1'b0, c[23:17], 1'b0, c[15:9], 1'b0, c[7:1]};
                2'd2:    shr = {2'b0, c[23:18], 2'b0, c[15:10], 2'b0, c[7:2]};
                default: shr = {3'b0, c[23:19], 3'b0, c[15:11], 3'b0, c[7:3]};
            endcase
        end
    endfunction

    wire [3:0]  nib    = odd_d2 ? font_q[3:0] : font_q[7:4];
    wire [1:0]  cov    = (txt_d2 && vis_d2) ? nib[1:0] : 2'd0;
    wire [23:0] bg4    = {2'b0, rgb_d2[23:18], 2'b0, rgb_d2[15:10], 2'b0, rgb_d2[7:2]};
    wire [23:0] bg8    = {3'b0, rgb_d2[23:19], 3'b0, rgb_d2[15:11], 3'b0, rgb_d2[7:3]};
    wire [23:0] dimmed = {bg4[23:16] + bg8[23:16], bg4[15:8] + bg8[15:8], bg4[7:0] + bg8[7:0]};
    wire [23:0] base   = (vis_d2 && !slp_f) ? dimmed : rgb_d2;
    wire [23:0] col0   = !slp_f ? WHITE : lo_d2 ? ICE_LO : ICE_HI;
    function [23:0] sat_add;              // 每通道饱和相加
        input [23:0] a, b;
        reg   [8:0]  r, g, bl;
        begin
            r  = a[23:16] + b[23:16];
            g  = a[15:8]  + b[15:8];
            bl = a[7:0]   + b[7:0];
            sat_add = {r[8] ? 8'hFF : r[7:0], g[8] ? 8'hFF : g[7:0], bl[8] ? 8'hFF : bl[7:0]};
        end
    endfunction
    wire [23:0] bg_g   = base;
    wire [1:0]  fsh_e  = (cov == 2'd1 && fsh_d2 != 2'd3) ? fsh_d2 + 2'd1 : fsh_d2;
    wire [23:0] mixed  = (cov == 2'd0) ? bg_g :
                         (cov == 2'd2 && fsh_d2 == 2'd0) ? col0 : sat_add(bg_g, shr(col0, fsh_e));

    // ---------------- 第 3 拍：输出 ----------------
    reg [23:0] rgb_d3;
    reg [LATENCY-1:0] de_pipe, vs_pipe;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            rgb_d1  <= 24'd0;
            code_d1 <= 4'd0;
            offb_d1 <= 11'd0;
            col_d1  <= 1'b0;
            txt_d1  <= 1'b0;
            odd_d1  <= 1'b0;
            lo_d1   <= 1'b0;
            fsh_d1  <= 2'd0;
            vis_d1  <= 1'b0;
            rgb_d2  <= 24'd0;
            txt_d2  <= 1'b0;
            odd_d2  <= 1'b0;
            lo_d2   <= 1'b0;
            fsh_d2  <= 2'd0;
            vis_d2  <= 1'b0;
            rgb_d3  <= 24'd0;
            de_pipe <= {LATENCY{1'b0}};
            vs_pipe <= {LATENCY{1'b1}};
        end else begin
            de_pipe <= {de_pipe[LATENCY-2:0], de};
            vs_pipe <= {vs_pipe[LATENCY-2:0], vsync};

            rgb_d1  <= rgb_in;
            code_d1 <= code;
            offb_d1 <= offb;
            col_d1  <= colon;
            txt_d1  <= x_on && y_on && ok;
            odd_d1  <= cc[0];
            lo_d1   <= ch ? lo_r : lo_s;
            // 倒影按行渐隐；冒号呼吸：约 0.5s 一次在原亮 / 1/2 之间切换
            fsh_d1  <= colon ? {1'b0, fcnt[5]} : fsh_l;
            vis_d1  <= de && (lvl[4] || (bayer(pixel_x[1:0], pixel_y[1:0]) < lvl[3:0]));

            rgb_d2  <= rgb_d1;
            txt_d2  <= txt_d1;
            odd_d2  <= odd_d1;
            lo_d2   <= lo_d1;
            fsh_d2  <= fsh_d1;
            vis_d2  <= vis_d1;

            rgb_d3  <= mixed;
        end
    end

    assign rgb_out   = rgb_d3;
    assign de_out    = de_pipe[LATENCY-1];
    assign vsync_out = vs_pipe[LATENCY-1];
endmodule
