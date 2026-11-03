`timescale 1ns/1ps

module gyro_storm_osd #(
    parameter integer THRESH_DEG_X10  = 300,     // 30.0 度
    parameter integer MS_CYCLES       = 50_000,  // ctrl_clk 50MHz 下 1ms
    parameter integer HOLD_MS         = 50,
    parameter integer HOLD_FRAMES     = 300
)(
    // ---------------- ctrl_clk 域：超限判断 ----------------
    input  wire               ctrl_clk,
    input  wire               ctrl_rst_n,
    input  wire               attitude_valid,
    input  wire               calibrated,
    input  wire signed [15:0] roll_deg_x10,   // X 轴
    input  wire signed [15:0] pitch_deg_x10,  // Y 轴
    input  wire signed [15:0] yaw_deg_x10,    // Z 轴
    // ---------------- 像素域 ----------------
    input  wire               pix_clk,
    input  wire               pix_rst_n,
    input  wire               show_async,     // 当前为第五页且转场结束
    input  wire               release_async,
    input  wire               vsync,
    input  wire               de,
    input  wire [23:0]        rgb_in,
    input  wire [59:0]        vis_band,
    output wire               vsync_out,
    output wire               de_out,
    output wire [23:0]        rgb_out,
    output reg                alarm_active    // 像素域电平：警报有效（第五页且有带在屏幕上）
);
    localparam integer LATENCY = 4;

    localparam integer TAPE_L  = 768;
    localparam [63:0] BAND_T  = 64'h00FF_FFFF_FFFF_FE00;   // kb 9 ~ 55
    localparam [63:0] SIDE_T  = 64'hFFF0_0000_0000_1FFF;   // kb < 13 或 kb > 51（|k| > 19）
    localparam integer BOX     = 128;
    localparam integer BAR_R0  = 168;
    localparam integer BEAT_FRAMES = 48;       // 心跳周期（帧），约 0.8s

    localparam [23:0] TAPE_YELLOW = 24'hFFC400;
    localparam [23:0] INK         = 24'h101010;
    localparam [23:0] HALO_COLOR  = 24'hFF1E10;
    localparam [23:0] SYM_COLOR   = 24'hFF2418;
    localparam [23:0] BAR_MID     = 24'h876A08;   // 警示条斜纹分界像素（黄黑各半）

    function over_th;
        input signed [15:0] v;
        begin
            over_th = (v > THRESH_DEG_X10) || (v < -THRESH_DEG_X10);
        end
    endfunction
    wire [2:0]  over_now = {over_th(yaw_deg_x10), over_th(pitch_deg_x10), over_th(roll_deg_x10)};
    reg  [15:0] ms_cnt;
    reg         ms_tick;
    reg  [2:0]  over_axes;        // {z, y, x}
    wire [2:0]  hold_nz;

    always @(posedge ctrl_clk or negedge ctrl_rst_n) begin
        if (!ctrl_rst_n) begin
            ms_cnt    <= 16'd0;
            ms_tick   <= 1'b0;
            over_axes <= 3'b000;
        end else begin
            ms_tick   <= (ms_cnt == MS_CYCLES - 1);
            ms_cnt    <= (ms_cnt == MS_CYCLES - 1) ? 16'd0 : ms_cnt + 1'b1;
            over_axes <= hold_nz;
        end
    end

    genvar a;
    generate
        for (a = 0; a < 3; a = a + 1) begin : g_hold
            reg [5:0] hold_ms;

            always @(posedge ctrl_clk or negedge ctrl_rst_n) begin
                if (!ctrl_rst_n)
                    hold_ms <= 6'd0;
                else if (attitude_valid && calibrated && over_now[a])
                    hold_ms <= HOLD_MS;
                else if (ms_tick && (hold_ms != 6'd0))
                    hold_ms <= hold_ms - 1'b1;
            end

            assign hold_nz[a] = (hold_ms != 6'd0);
        end
    endgenerate

    function [3:0] sat5;
        input signed [12:0] v;
        begin
            sat5 = v[12] ? 4'd0 : (v[11:5] != 7'd0) ? 4'd15 : v[4:1];
        end
    endfunction

    // 1/4 周期正弦表：round(32 * sin(b * 3.75 度))，b = 0 ~ 24
    function [5:0] qsin32;
        input [4:0] b;
        begin
            case (b)
                5'd0:    qsin32 = 6'd0;
                5'd1:    qsin32 = 6'd2;
                5'd2:    qsin32 = 6'd4;
                5'd3:    qsin32 = 6'd6;
                5'd4:    qsin32 = 6'd8;
                5'd5:    qsin32 = 6'd10;
                5'd6:    qsin32 = 6'd12;
                5'd7:    qsin32 = 6'd14;
                5'd8:    qsin32 = 6'd16;
                5'd9:    qsin32 = 6'd18;
                5'd10:   qsin32 = 6'd19;
                5'd11:   qsin32 = 6'd21;
                5'd12:   qsin32 = 6'd23;
                5'd13:   qsin32 = 6'd24;
                5'd14:   qsin32 = 6'd25;
                5'd15:   qsin32 = 6'd27;
                5'd16:   qsin32 = 6'd28;
                5'd17:   qsin32 = 6'd29;
                5'd18:   qsin32 = 6'd30;
                5'd19:   qsin32 = 6'd30;
                5'd20:   qsin32 = 6'd31;
                5'd21:   qsin32 = 6'd31;
                5'd22:   qsin32 = 6'd32;
                5'd23:   qsin32 = 6'd32;
                default: qsin32 = 6'd32;
            endcase
        end
    endfunction

    function [3:0] cov_dn;
        input       lt;
        input       eq;
        input [3:0] lo;
        begin
            cov_dn = lt ? 4'd15 : eq ? ~lo : 4'd0;
        end
    endfunction

    function [3:0] min4;
        input [3:0] p;
        input [3:0] q;
        begin
            min4 = (p < q) ? p : q;
        end
    endfunction

    function [3:0] beat_env;
        input [4:0] i;
        begin
            case (i)
                5'd0:    beat_env = 4'd15;
                5'd1:    beat_env = 4'd14;
                5'd2:    beat_env = 4'd11;
                5'd3:    beat_env = 4'd8;
                5'd4:    beat_env = 4'd6;
                5'd5:    beat_env = 4'd5;
                5'd6:    beat_env = 4'd12;
                5'd7:    beat_env = 4'd11;
                5'd8:    beat_env = 4'd8;
                5'd9:    beat_env = 4'd6;
                5'd10:   beat_env = 4'd4;
                5'd11:   beat_env = 4'd3;
                5'd12:   beat_env = 4'd2;
                5'd13:   beat_env = 4'd1;
                default: beat_env = 4'd0;
            endcase
        end
    endfunction

    function [3:0] halo_scale;
        input [3:0] c;
        input [1:0] e;
        begin
            case (e)
                2'd3:    halo_scale = c - {3'd0, c[3]};
                2'd2:    halo_scale = c - {2'd0, c[3:2]};
                2'd1:    halo_scale = {1'b0, c[3:1]} + {3'd0, c[3]};
                default: halo_scale = {1'b0, c[3:1]};
            endcase
        end
    endfunction

    // 轴色边条与 "边条 / 黄底" 分界像素颜色
    function [23:0] strip_rgb;
        input [1:0] ax;
        input       mid;
        begin
            case (ax)
                2'd1:    strip_rgb = mid ? 24'hA1B223 : 24'h43A047;
                2'd2:    strip_rgb = mid ? 24'h8EA672 : 24'h1E88E5;
                default: strip_rgb = mid ? 24'hF27E1A : 24'hE53935;
            endcase
        end
    endfunction

    function [23:0] tape_rgb;
        input [1:0] cls;
        input [1:0] ax;
        input [1:0] g;
        begin
            if (cls != 2'd2)
                tape_rgb = strip_rgb(ax, cls[0]);
            else
                case (g)
                    2'd0:    tape_rgb = TAPE_YELLOW;
                    2'd1:    tape_rgb = 24'hAF8805;
                    2'd2:    tape_rgb = 24'h604C0B;
                    default: tape_rgb = INK;
                endcase
        end
    endfunction

    (* ASYNC_REG = "TRUE" *) reg [4:0] st_meta, st_sync;
    reg        vs_q;
    wire       frame_start = vs_q && !vsync;
    wire       show_s      = st_sync[3];
    wire       rel_s       = st_sync[4];

    wire [2:0] want;              // 本轴带处于保持期
    wire [2:0] idle;              // 本轴带不在屏幕上
    wire [2:0] start = want & idle;                             // 需要插入的带
    wire [2:0] go    = start & ~{start[1:0], 1'b0} & ~{start[0], 2'b00};   // 每帧只插入一条（序号小的优先）
    wire       active = (idle != 3'b111);

    reg        show_f;            // 本帧是否在第五页
    reg [3:0]  lvl;
    reg [5:0]  beat;              // 心跳帧号
    reg [6:0]  scr;               // 带上文字流动偏移，每帧 2 像素
    reg [4:0]  bscr;              // 警示条斜纹滚动偏移，每帧 1 像素
    reg [4:0]  rot;               // 辐射标志旋转相位，每帧 +1（3.75 度）
    reg [1:0]  ord0, ord1, ord2;  // 带的上下层次：ord0 最上（最近插入）
    reg [2:0]  go_r;              // 帧起点锁存的插入请求
    reg        step_v;            // 正在依次更新各带帧状态
    reg [1:0]  step_i;            // 本拍更新的带

    wire       fx_on = show_f && (active || (lvl != 4'd0));
    wire [1:0] go_i  = go[1] ? 2'd1 : go[2] ? 2'd2 : 2'd0;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n)
            alarm_active <= 1'b0;
        else
            alarm_active <= show_f && active;
    end

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            st_meta <= 5'b00000;
            st_sync <= 5'b00000;
            vs_q    <= 1'b1;
            show_f  <= 1'b0;
            lvl     <= 4'd0;
            beat    <= 6'd0;
            scr     <= 7'd0;
            bscr    <= 5'd0;
            rot     <= 5'd0;
            ord0    <= 2'd0;
            ord1    <= 2'd1;
            ord2    <= 2'd2;
            go_r    <= 3'b000;
            step_v  <= 1'b0;
            step_i  <= 2'd0;
        end else begin
            st_meta <= {release_async, show_async, over_axes};
            st_sync <= st_meta;
            vs_q    <= vsync;
            if (frame_start) begin
                go_r   <= go;
                step_v <= 1'b1;
                step_i <= 2'd0;
            end else if (step_v) begin
                step_v <= (step_i != 2'd2);
                step_i <= step_i + 1'b1;
            end
            if (frame_start) begin
                show_f <= show_s;
                scr    <= scr + 7'd2;
                bscr   <= bscr + 5'd1;
                rot    <= rot + 5'd1;
                if (!show_s)
                    lvl <= 4'd0;
                else if (active)
                    lvl <= (lvl == 4'd15) ? 4'd15 : lvl + 1'b1;
                else
                    lvl <= (lvl == 4'd0) ? 4'd0 : lvl - 1'b1;
                beat <= (lvl == 4'd0 || beat == BEAT_FRAMES - 1) ? 6'd0 : beat + 1'b1;
                // 新插入的带移到最上层
                if (show_s && (go != 3'b000)) begin
                    if (go_i == ord1) begin
                        ord0 <= ord1;
                        ord1 <= ord0;
                    end else if (go_i == ord2) begin
                        ord0 <= ord2;
                        ord1 <= ord0;
                        ord2 <= ord1;
                    end
                end
            end
        end
    end

    wire [29:0] h_bus, t_bus;
    wire [26:0] hold_bus;
    wire [9:0]  s_h    = h_bus[10*step_i +: 10];
    wire [9:0]  s_t    = t_bus[10*step_i +: 10];
    wire [8:0]  s_hold = hold_bus[9*step_i +: 9];
    wire        s_want = (s_hold != 9'd0);
    wire        s_idle = (s_h == 10'd0);
    wire [10:0] h_up   = {1'b0, s_h} + {5'd0, 6'd48 - s_h[9:4]} + 11'd3;   // 插入：先快后慢（剩余距离 / 16 + 3）
    wire [10:0] t_up   = {1'b0, s_t} + {5'd0, s_t[9:4]} + 11'd3;           // 抽出：先慢后快
    wire [9:0]  t_dd   = {4'd0, s_t[9:4]} + 10'd3;                         // 倒回
    wire [9:0]  h_sat  = (h_up >= TAPE_L) ? TAPE_L : h_up[9:0];
    reg  [9:0]  h_nx, t_nx;
    reg  [8:0]  hold_nx;

    always @* begin
        h_nx    = s_h;
        t_nx    = s_t;
        hold_nx = st_sync[step_i] ? HOLD_FRAMES : s_want ? s_hold - 1'b1 : 9'd0;
        if (rel_s)
            hold_nx = 9'd0;
        if (!show_s) begin
            h_nx    = 10'd0;
            t_nx    = 10'd0;
            hold_nx = 9'd0;
        end else if (s_want) begin
            if (!s_idle || go_r[step_i])
                h_nx = h_sat;
            t_nx = (s_t <= t_dd) ? 10'd0 : s_t - t_dd;
        end else if (!s_idle) begin
            if (t_up >= TAPE_L) begin
                h_nx = 10'd0;
                t_nx = 10'd0;
            end else begin
                h_nx = h_sat;
                t_nx = t_up[9:0];
            end
        end
    end

    reg  signed [10:0] xp;        // 列号 - 320
    reg  signed [9:0]  yp;        // 行号 - 240
    reg         de_q;
    reg  [15:0] dysq;             // (yp^2 + BOX^2)，行号 112~367 内逐行递推
    reg  [15:0] rr;               // xp^2 + yp^2，列号 192~447 内逐列递推

    wire               in_bx = (xp[10:7] == 4'b0000) || (xp[10:7] == 4'b1111);   // -128 <= xp < 128
    wire               in_by = (yp[9:7] == 3'b000) || (yp[9:7] == 3'b111);
    wire [10:0]        dx2p1 = {xp[9:0], 1'b1};                 // 2 * dx + 1
    wire [10:0]        dy2p1 = {yp[9:0], 1'b1};
    wire [4:0]         row5  = {~yp[4], yp[3:0]};               // 屏幕行号低 5 位

    reg  [1:0]         nj;
    reg  signed [6:0]  na0, nb0, na1, nb1;
    reg  signed [12:0] d0, d1;
    wire [6:0]         ang   = {2'd0, rot} + ((nj == 2'd0) ? 7'd32 : (nj == 2'd1) ? 7'd8 : (nj == 2'd2) ? 7'd64 : 7'd40);
    wire               ang_h = (ang >= 7'd48);                                  // 后半周：取负
    wire [5:0]         ang_m = ang_h ? ang[5:0] - 6'd48 : ang[5:0];            // 0 ~ 47
    wire [4:0]         ang_q = (ang_m > 6'd24) ? 5'd16 - ang_m[4:0] : ang_m[4:0];   // 48 - m（模 32），0 ~ 24
    wire [5:0]         sv_m  = qsin32(ang_q);
    wire signed [6:0]  sv    = ang_h ? -$signed({1'b0, sv_m}) : $signed({1'b0, sv_m});
    wire [5:0]         nab   = (nj[1] ? na1[5:0] : na0[5:0]) + sv[5:0];          // na + nb（模 64）
    wire [12:0]        d_ld  = 13'd0 - {nab, 7'd0};                              // -128 * (na + nb)
    // 行尾修正量：-256 * na（+ nb）
    wire [12:0]        dr0   = {5'd0 - na0[4:0], 8'd0} + (in_by ? {{6{nb0[6]}}, nb0} : 13'd0);
    wire [12:0]        dr1   = {5'd0 - na1[4:0], 8'd0} + (in_by ? {{6{nb1[6]}}, nb1} : 13'd0);

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            xp   <= -11'sd320;
            yp   <= -10'sd240;
            de_q <= 1'b0;
            dysq <= 16'd0;
            rr   <= 16'd0;
            nj   <= 2'd0;
            na0  <= 7'sd0;
            nb0  <= 7'sd0;
            na1  <= 7'sd0;
            nb1  <= 7'sd0;
            d0   <= 13'sd0;
            d1   <= 13'sd0;
        end else begin
            de_q <= de;
            if (!vsync) begin
                xp   <= -11'sd320;
                yp   <= -10'sd240;
                dysq <= 2 * BOX * BOX;
                nj   <= nj + 1'b1;
                case (nj)
                    2'd0: na0 <= sv;
                    2'd1: begin
                        nb0 <= sv;
                        d0  <= d_ld;
                    end
                    2'd2: na1 <= sv;
                    default: begin
                        nb1 <= sv;
                        d1  <= d_ld;
                    end
                endcase
            end else if (fx_on) begin
                xp <= de ? xp + 11'sd1 : -11'sd320;
                if (de_q && !de) begin
                    yp <= yp + 1'b1;
                    if (in_by)
                        dysq <= dysq + {{5{dy2p1[10]}}, dy2p1};
                end
                rr <= !de   ? dysq :
                      in_bx ? rr + {{5{dx2p1[10]}}, dx2p1} : rr;
                d0 <= d0 + ((de_q && !de) ? dr0 : in_bx ? {{6{na0[6]}}, na0} : 13'd0);
                d1 <= d1 + ((de_q && !de) ? dr1 : in_bx ? {{6{na1[6]}}, na1} : 13'd0);
            end
        end
    end

    wire signed [12:0] x3   = xp + (xp <<< 1);                  // 3x
    wire [10:0]        xb   = {xp[10:7] + 4'd3, xp[6:0]};       // x + 384（Y 带用 383 - x）
    wire [10:0]        xnb  = {~xp[10:7] + 4'd3, ~xp[6:0]};
    reg  signed [12:0] y3;                                      // 3y（行参数）

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n)
            y3 <= 13'sd0;
        else if (fx_on && !de)
            y3 <= yp + (yp <<< 1);
    end

    // 心跳：光晕半径与亮度随包络脉动
    wire [3:0]  env   = beat_env(beat[5:1]);
    // 圆周覆盖率：过渡段宽度约 1 像素（r^2 段宽 / 2R）
    wire [8:0]  r7    = rr[15:7];
    wire [3:0]  c_out = cov_dn(r7 < 9'd33, r7 == 9'd33, rr[6:3]);              // R 约 65.5
    wire [3:0]  c_rim = cov_dn(r7 < 9'd41, r7 == 9'd41, rr[6:3]);              // R 约 72.9（光晕内沿）
    wire [3:0]  c_dot = cov_dn(rr[15:4] < 12'd10, rr[15:4] == 12'd10, rr[3:0]); // R 约 13
    wire [3:0]  c_in  = ~cov_dn(rr[15:5] < 11'd12, rr[15:5] == 11'd12, rr[4:1]); // R 约 20（外侧覆盖）
    wire [7:0]  hd    = {3'd0, {1'b0, env} + 5'd15} - {1'b0, rr[15:9]};
    wire [3:0]  c_hal = hd[7] ? 4'd0 : (hd[6:4] != 3'd0) ? 4'd15 : hd[3:0];
    wire signed [12:0] d2   = ~(d0 + d1);                       // -(d0 + d1)，差 1/32 像素

    wire signed [12:0] tb8    = $signed({yp, 3'b100}) + xp;
    wire [11:0]        tv     = tb8[11:0] ^ {12{tb8[12]}};
    wire [8:0]         tb     = tv[11:3] - BAR_R0;
    wire [2:0]         tf     = tv[2:0];
    wire               bar_on = (tb < 9'd24) && tb8[12];               // 只保留上方警示条（8y + x < 0 一侧）
    wire               bar_ln = (tb[4:1] == 4'd0) || (tb[4:1] == 4'd11);
    wire [3:0]         bar_a  = !bar_on      ? 4'd0 :
                                (tb == 9'd0)  ? {tf, tf[2]} :
                                (tb == 9'd23) ? {~tf, 1'b1} : 4'd15;
    wire [4:0]         bd     = row5 - bscr;
    wire [4:0]         bp     = yp[9] ? xp[4:0] + bd : xp[4:0] - bd;
    wire [1:0]         bar_c  = bar_ln              ? 2'd2 :          // 1 黄 / 2 黑 / 3 分界
                                (bp[3:0] == 4'd0)   ? 2'd3 :
                                bp[4]               ? 2'd2 : 2'd1;
    // 4x4 有序抖动阈值：压暗与标志按强度溶解出现 / 消失
    wire [3:0] bay    = {xp[0] ^ yp[0], yp[0], xp[1] ^ yp[1], yp[1]};

    wire [1:0] rib_lvl;
    alarm_ribbon3d #(
        .RY0   (120),
        .WIN_H (96)
    ) u_ribbon (
        .clk         (pix_clk),
        .rst_n       (pix_rst_n),
        .frame_start (frame_start),
        .de          (de),
        .de_q        (de_q),
        .xp          (xp),
        .yp          (yp),
        .vis_band    (vis_band),
        .lvl         (rib_lvl)
    );

    // 红色上升粒子：同样按 xp / yp 现算，sp_lvl 与第 2 拍对齐
    wire [1:0] sp_lvl;
    alarm_sparks u_sparks (
        .clk         (pix_clk),
        .rst_n       (pix_rst_n),
        .frame_start (frame_start),
        .run         (fx_on),
        .xp          (xp),
        .yp          (yp),
        .lvl         (sp_lvl)
    );

    reg [23:0] rgb_d1;
    reg        act_d1, box_d1;
    reg [3:0]  c_out_d1, c_in_d1, c_dot_d1, c_rim_d1, c_halo_d1, w0_d1, w1_d1, w2_d1;
    reg [1:0]  bar_d1;
    reg [3:0]  bar_a_d1;
    reg        dpass_d1;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            rgb_d1    <= 24'd0;
            act_d1    <= 1'b0;
            box_d1    <= 1'b0;
            c_out_d1  <= 4'd0;
            c_in_d1   <= 4'd0;
            c_dot_d1  <= 4'd0;
            c_rim_d1  <= 4'd0;
            c_halo_d1 <= 4'd0;
            w0_d1     <= 4'd0;
            w1_d1     <= 4'd0;
            w2_d1     <= 4'd0;
            bar_d1    <= 2'd0;
            bar_a_d1  <= 4'd0;
            dpass_d1  <= 1'b0;
        end else begin
            rgb_d1    <= rgb_in;
            act_d1    <= fx_on && de;
            box_d1    <= fx_on && de && in_bx && in_by;
            c_out_d1  <= c_out;
            c_in_d1   <= c_in;
            c_dot_d1  <= c_dot;
            c_rim_d1  <= c_rim;
            c_halo_d1 <= c_hal;
            w0_d1     <= sat5(d0);
            w1_d1     <= sat5(d1);
            w2_d1     <= sat5(d2);
            bar_d1    <= bar_c;
            bar_a_d1  <= bar_a;
            dpass_d1  <= (lvl == 4'd15) || (bay < lvl);
        end
    end

    wire [2:0]  hit_v, full_v, txt_v;
    wire [11:0] alpha_v;
    wire [5:0]  cls_v;
    wire [20:0] w_v;              // 长度坐标 W[9:3]
    wire [14:0] k_v;              // 横向坐标 k[4:0]

    generate
        for (a = 0; a < 3; a = a + 1) begin : g_tape
            reg  [8:0]  hold_f;
            reg  [9:0]  h, t;

            assign want[a]           = (hold_f != 9'd0);
            assign idle[a]           = (h == 10'd0);
            assign h_bus[10*a +: 10] = h;
            assign t_bus[10*a +: 10] = t;
            assign hold_bus[9*a +: 9] = hold_f;

            always @(posedge pix_clk or negedge pix_rst_n) begin
                if (!pix_rst_n) begin
                    hold_f <= 9'd0;
                    h      <= 10'd0;
                    t      <= 10'd0;
                end else if (step_v && (step_i == a)) begin
                    hold_f <= hold_nx;
                    h      <= h_nx;
                    t      <= t_nx;
                end
            end

            wire signed [12:0] ac8_nx = (a == 0) ? $signed({yp, 3'b100}) - x3 :
                                        (a == 1) ? $signed({yp, 3'b100}) + x3 : $signed({yp, 3'b100}) + xp;
            wire signed [13:0] w_nx   = (a == 0) ? $signed({1'b0, xb, 3'b100}) + y3 :
                                        (a == 1) ? $signed({1'b0, xnb, 3'b011}) + y3 : $signed({1'b0, xb, 3'b100}) - yp;
            reg  signed [12:0] ac8;
            reg         [10:0] wq;    // W[13:3]

            always @(posedge pix_clk or negedge pix_rst_n) begin
                if (!pix_rst_n) begin
                    ac8 <= 13'sd0;
                    wq  <= 11'd0;
                end else begin
                    ac8 <= ac8_nx;
                    wq  <= w_nx[13:3];
                end
            end

            wire [9:0] k    = ac8[12:3];
            wire [2:0] f    = ac8[2:0];
            wire       kin  = (k[9:5] == 5'b00000) || (k[9:5] == 5'b11111);    // -32 <= k < 32
            wire [5:0] kb   = {~k[5], k[4:0]};                                  // k + 32
            wire       vis  = !wq[10] && (wq[9:0] >= t) && (wq[9:0] < h);
            wire       band = kin && BAND_T[kb];
            wire [3:0] alp  = (kb == 6'd9) ? {f, f[2]} : (kb == 6'd55) ? {~f, 1'b1} : 4'd15;
            // 颜色类别：0 边条 / 1 分界 / 2 黄底
            wire [1:0] cls  = SIDE_T[kb]    ? 2'd0 :
                              (kb == 6'd13) ? ((f <= 3'd2) ? 2'd0 : (f >= 3'd6) ? 2'd2 : 2'd1) :
                              (kb == 6'd51) ? ((f <= 3'd2) ? 2'd2 : (f >= 3'd6) ? 2'd0 : 2'd1) : 2'd2;

            assign hit_v[a]          = act_d1 && vis && band;
            assign full_v[a]         = hit_v[a] && (alp == 4'd15);
            assign txt_v[a]          = (k[9:4] == 6'b000000) || (k[9:4] == 6'b111111);   // -16 <= k < 16
            assign alpha_v[4*a +: 4] = alp;
            assign cls_v[2*a +: 2]   = cls;
            assign w_v[7*a +: 7]     = wq[6:0];
            assign k_v[5*a +: 5]     = k[4:0];
        end
    endgenerate

    wire       hA    = hit_v[ord0];
    wire       hB    = hit_v[ord1];
    wire       hC    = hit_v[ord2];
    wire [1:0] top_i = hA ? ord0 : hB ? ord1 : ord2;
    wire       top_v = hA || hB || hC;
    wire [1:0] low_i = (hA && hB) ? ord1 : ord2;
    wire       low_v = hA ? (hB || hC) : (hB && hC);

    wire [6:0]  u_q   = ~w_v[7*top_i +: 7] + scr;
    wire [6:0]  u     = (top_i == 2'd1) ? u_q : ~u_q;
    wire [4:0]  k_top = k_v[5*top_i +: 5];
    wire [2:0]  gid   = (u[6:5] == 2'd0) ? {1'b0, top_i} : {1'b0, u[6:5]} + 3'd2;
    wire [10:0] warn_addr = {gid, ~k_top[4], k_top[3:0], u[4:2]};      // 字形行 = k + 16
    wire [7:0]  warn_q;

    warn_sprite_rom u_warn_rom (
        .clk  (pix_clk),
        .addr (warn_addr),
        .q    (warn_q)
    );

    function [3:0] med3;
        input [3:0] p;
        input [3:0] q;
        input [3:0] r;
        begin
            med3 = (p < q) ? ((q < r) ? q : (p < r) ? r : p) : ((p < r) ? p : (q < r) ? r : q);
        end
    endfunction

    wire [3:0] blade = min4(min4(med3(w0_d1, w1_d1, w2_d1), c_out_d1), c_in_d1);

    reg [23:0] rgb_d2;
    reg        act_d2, box_d2, dpass_d2;
    reg [3:0]  sym_d2, halo_d2;
    reg [1:0]  bar_d2;
    reg [3:0]  bar_a_d2;
    reg        top_v_d2, top_full_d2, top_txt_d2, low_full_d2;
    reg [3:0]  top_a_d2;
    reg [1:0]  top_cls_d2, top_ax_d2, top_csel_d2, low_cls_d2, low_ax_d2;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            rgb_d2      <= 24'd0;
            act_d2      <= 1'b0;
            box_d2      <= 1'b0;
            dpass_d2    <= 1'b0;
            sym_d2      <= 4'd0;
            halo_d2     <= 4'd0;
            bar_d2      <= 2'd0;
            bar_a_d2    <= 4'd0;
            top_v_d2    <= 1'b0;
            top_full_d2 <= 1'b0;
            top_txt_d2  <= 1'b0;
            low_full_d2 <= 1'b0;
            top_a_d2    <= 4'd0;
            top_cls_d2  <= 2'd0;
            top_ax_d2   <= 2'd0;
            top_csel_d2 <= 2'd0;
            low_cls_d2  <= 2'd0;
            low_ax_d2   <= 2'd0;
        end else begin
            rgb_d2      <= rgb_d1;
            act_d2      <= act_d1;
            box_d2      <= box_d1;
            dpass_d2    <= dpass_d1;
            sym_d2      <= (blade > c_dot_d1) ? blade : c_dot_d1;
            halo_d2     <= halo_scale(min4(c_halo_d1, ~c_rim_d1), env[3:2]);
            bar_d2      <= bar_d1;
            bar_a_d2    <= act_d1 ? bar_a_d1 : 4'd0;
            top_v_d2    <= top_v;
            top_full_d2 <= full_v[top_i];
            top_txt_d2  <= txt_v[top_i];
            low_full_d2 <= low_v && full_v[low_i];
            top_a_d2    <= alpha_v[{top_i, 2'b00} +: 4];
            top_cls_d2  <= cls_v[{top_i, 1'b0} +: 2];
            top_ax_d2   <= top_i;
            top_csel_d2 <= u[1:0];
            low_cls_d2  <= cls_v[{low_i, 1'b0} +: 2];
            low_ax_d2   <= low_i;
        end
    end

    reg  [1:0]  gv;
    always @* begin
        case (top_csel_d2)
            2'd0:    gv = warn_q[1:0];
            2'd1:    gv = warn_q[3:2];
            2'd2:    gv = warn_q[5:4];
            default: gv = warn_q[7:6];
        endcase
    end

    wire [23:0] top_rgb  = tape_rgb(top_cls_d2, top_ax_d2, top_txt_d2 ? gv : 2'd0);
    wire [23:0] low_rgb  = tape_rgb(low_cls_d2, low_ax_d2, 2'd0);
    wire        sym_on   = box_d2 && dpass_d2 && (sym_d2 != 4'd0);
    wire [23:0] bar_rgb  = (bar_d2 == 2'd1) ? TAPE_YELLOW : (bar_d2 == 2'd2) ? INK : BAR_MID;
    // 压暗 50%，渐入 / 渐隐期间按有序抖动逐步铺满
    wire        fx_px    = act_d2 && dpass_d2;
    wire [23:0] dim0     = fx_px ? {1'b0, rgb_d2[23:17], 1'b0, rgb_d2[15:9], 1'b0, rgb_d2[7:1]} : rgb_d2;
    wire [23:0] rib_rgb  = (rib_lvl == 2'd3) ? 24'hFFC8C8 : (rib_lvl == 2'd2) ? 24'hFF3040 :
                           (rib_lvl == 2'd1) ? 24'hB01828 : 24'h000000;
    wire [23:0] sp_rgb   = (sp_lvl == 2'd3) ? 24'hFFD0D0 : (sp_lvl == 2'd2) ? 24'hFF3040 : 24'hB01828;
    wire        sp_v     = (sp_lvl != 2'd0) && (sp_lvl >= rib_lvl);
    wire [23:0] dim      = fx_px ? (sp_v ? sp_rgb : (dim0 | rib_rgb)) : dim0;
    wire [3:0]  halo_a   = box_d2 ? min4(halo_d2, lvl) : 4'd0;
    wire        bar_v    = (bar_a_d2 != 4'd0);
    wire [23:0] under    = bar_a_d2[3] ? bar_rgb : halo_a[3] ? HALO_COLOR : dim;

    reg [23:0] fg_d3, bg_d3;
    reg [3:0]  a_d3;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            fg_d3 <= 24'd0;
            bg_d3 <= 24'd0;
            a_d3  <= 4'd0;
        end else if (sym_on) begin
            fg_d3 <= SYM_COLOR;
            bg_d3 <= top_full_d2 ? top_rgb : under;
            a_d3  <= sym_d2;
        end else if (top_v_d2) begin
            fg_d3 <= top_rgb;
            bg_d3 <= low_full_d2 ? low_rgb : under;
            a_d3  <= top_a_d2;
        end else begin
            fg_d3 <= bar_v ? bar_rgb : HALO_COLOR;
            bg_d3 <= dim;
            a_d3  <= bar_v ? min4(bar_a_d2, lvl) : halo_a;
        end
    end

    wire [23:0] mixed;
    osd_alpha_blend #(
        .USE_DSP (1)
    ) u_blend (
        .bg    (bg_d3),
        .fg    (fg_d3),
        .alpha (a_d3),
        .out   (mixed)
    );

    reg [23:0] rgb_d4;
    reg [LATENCY-1:0] de_pipe, vs_pipe;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            rgb_d4  <= 24'd0;
            de_pipe <= {LATENCY{1'b0}};
            vs_pipe <= {LATENCY{1'b1}};
        end else begin
            rgb_d4  <= mixed;
            de_pipe <= {de_pipe[LATENCY-2:0], de};
            vs_pipe <= {vs_pipe[LATENCY-2:0], vsync};
        end
    end

    assign rgb_out   = rgb_d4;
    assign de_out    = de_pipe[LATENCY-1];
    assign vsync_out = vs_pipe[LATENCY-1];
endmodule
