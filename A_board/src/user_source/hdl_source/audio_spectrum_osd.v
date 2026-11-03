`timescale 1ns/1ps

module audio_spectrum_osd #(
    parameter integer LEVEL_FLOOR   = 24,   // 8*log2 幅度低于此值视为无信号（柱高 0，均值冻结）
    parameter integer REL_BIAS      = 4,    // 均衡偏置（级）：电平等于长期平均时柱高 2*REL_BIAS
    parameter integer ABS_KNEE      = 70,   // 电平超过该值后按 0.5 像素/级叠加响度加成
    parameter integer VBLANK_CYCLES = 1024  // de 连续低电平超过该值视为场消隐（需大于行消隐）
)(
    input  wire               pix_clk,
    input  wire               pix_rst_n,
    input  wire               de,
    input  wire               audio_valid,    // 每个立体声采样一拍
    input  wire signed [23:0] audio_left,
    input  wire signed [23:0] audio_right,
    output reg  [59:0]        vis_band
);
    localparam integer NUM_BARS    = 48;
    localparam integer H_MAX       = 84;
    localparam [3:0]   FS_NOSCALE  = 4'd7;
    localparam integer EDGE_BASE   = 320;               // ROM 中柱频点范围表的起始地址

    localparam [2:0] E_IDLE = 3'd0, E_LOAD = 3'd1, E_FFT = 3'd2, E_POST = 3'd3;

    // ---------------- 工具函数 ----------------
    function [9:0] bitrev10;
        input [9:0] v;
        integer k;
        begin
            for (k = 0; k < 10; k = k + 1)
                bitrev10[k] = v[9 - k];
        end
    endfunction

    function [9:0] cos_sel;
        input [9:0] t;
        begin
            // t <= 256（t 最大 512）：高两位为 0，或恰为 256
            if ((t[9:8] == 2'b00) || (t == 10'd256)) cos_sel = {1'b0, 9'd256 - t[8:0]};
            else                                     cos_sel = {1'b1, t[8:0] - 9'd256};
        end
    endfunction

    function [17:0] sat18;
        input signed [20:0] v;
        begin
            if ((v[20:17] == 4'b0000) || (v[20:17] == 4'b1111)) sat18 = v[17:0];
            else                                                 sat18 = v[20] ? 18'h20000 : 18'h1FFFF;
        end
    endfunction

    function [17:0] abs18;
        input [17:0] v;
        begin
            abs18 = v[17] ? (18'd0 - v) : v;
        end
    endfunction

    // log2 近似：{最高位 1 的位置, 其后 3 位尾数}，即 8*log2(v)
    function [7:0] log2q;
        input [17:0] v;
        integer k;
        begin
            log2q = 8'd0;
            for (k = 3; k < 18; k = k + 1)
                if (v[k]) log2q = {k[4:0], v[k-1 -: 3]};
        end
    endfunction

    // ---------------- 场消隐检测 ----------------
    reg [10:0] blank_cnt;
    wire frame_start = !de && (blank_cnt == VBLANK_CYCLES - 1);

    // ---------------- 采样环形缓冲 ----------------
    wire signed [24:0] mix   = audio_left + audio_right;
    wire signed [24:0] mix_s = mix >>> 7;
    wire signed [17:0] smp18 = (mix_s >  25'sd131071) ?  18'sd131071 :
                               (mix_s < -25'sd131071) ? -18'sd131071 : mix_s[17:0];
    reg  [17:0] ring [0:1023];
    reg  [17:0] ring_q;
    reg  [9:0]  ring_wp;
    reg         smp_seen;    // 本帧收到过采样
    reg         silent;      // 上一帧没有采样：FFT 输入置 0，柱子自然回落

    // ---------------- 计算引擎寄存器 ----------------
    reg [2:0]  eng;
    // LOAD：环形缓冲 x Hann 窗 -> FFT 存储器（位反序地址）
    reg [10:0] ld_cnt;
    reg [9:0]  ld_base;
    reg        ld_v1, ld_v2, ld_neg1;
    reg [9:0]  ld_n1, ld_n2;
    reg signed [35:0] ld_prod;
    reg [3:0]  fs;
    reg [8:0]  bf;
    reg [2:0]  cyc;
    reg signed [17:0] a_re, a_im, w_c;
    reg signed [35:0] p_rc, p_is, p_ic, p_rs;
    reg [35:0] y_word;
    reg [7:0]  pb;
    reg [7:0]  pk, pk_hi;
    reg [2:0]  pst;
    reg        pv;
    reg [17:0] acc;
    reg [7:0]  r_prev, r_cur;   // 第 pb-2、pb-1 根柱的原始电平
    reg [7:0]  s_lv;            // 第 pb-1 根柱平滑后的电平
    reg [6:0]  tgt;
    reg [9:0]  avg_n;
    reg [6:0]  vb_acc;          // 当前段已写回柱的最高柱高
    reg        st_valid;

    // ---------------- 存储器 ----------------
    reg  [35:0] fmem [0:1023];     // {re, im}
    reg  [35:0] fq;
    reg  [19:0] st_ram [0:63];     // {avg[9:0] 8.2, h[9:0] 7.3}
    reg  [19:0] st_q;
    integer     m;

    initial begin
        for (m = 0; m < 1024; m = m + 1) ring[m] = 18'd0;
        for (m = 0; m < 64;   m = m + 1) st_ram[m] = 20'd0;
    end

    reg  [8:0]  rom_addr;
    wire [16:0] rom_q;
    audio_spectrum_rom u_rom (.clk(pix_clk), .addr(rom_addr), .q(rom_q));

    // ---------------- 组合地址与数据 ----------------
    wire [9:0] ld_t   = ld_cnt[9] ? (10'd0 - ld_cnt[9:0]) : ld_cnt[9:0]; // cos 偶对称折到 0..512
    wire [9:0] cs_ld  = cos_sel(ld_t);
    wire [17:0] ld_w18 = ld_neg1 ? (18'd65536 + {1'b0, rom_q}) : (18'd65536 - {1'b0, rom_q});
    wire [16:0] ld_w  = ld_w18[17:1];                       // (1 - cos)/2，Q16
    wire signed [17:0] ld_x = silent ? 18'sd0 : $signed(ring_q);
    wire [17:0] ld_re = ld_prod[33:16];

    wire [7:0] pk_inc  = pk + 8'd1;
    wire [8:0] lo_mask = (9'd1 << fs) - 9'd1;
    wire [8:0] bf_lo   = bf & lo_mask;
    wire [8:0] bf_hi   = bf & ~lo_mask;
    wire [9:0] fa      = {bf_hi, 1'b0} | {1'b0, bf_lo};
    wire [9:0] fb      = fa | (10'd1 << fs);
    wire [8:0] tw      = bf_lo << (4'd9 - fs);
    wire [9:0] cs_tw   = cos_sel({1'b0, tw});
    wire [8:0] sin_adr = (!tw[8] || (tw[7:0] == 8'd0)) ? tw : (9'd0 - tw);   // tw <= 256

    wire signed [17:0] b_re = fq[35:18];
    wire signed [17:0] b_im = fq[17:0];
    wire signed [17:0] w_s  = $signed({1'b0, rom_q});
    wire signed [36:0] t_re_w = p_rc + p_is + 37'sd32768;
    wire signed [36:0] t_im_w = p_ic - p_rs + 37'sd32768;
    wire signed [19:0] t_re = t_re_w[35:16];
    wire signed [19:0] t_im = t_im_w[35:16];
    wire signed [20:0] xs_re = a_re + t_re;
    wire signed [20:0] xs_im = a_im + t_im;
    wire signed [20:0] ys_re = a_re - t_re;
    wire signed [20:0] ys_im = a_im - t_im;
    wire               no_scale = (fs >= FS_NOSCALE);
    wire [35:0] x_word = no_scale ? {sat18(xs_re), sat18(xs_im)}
                                  : {sat18(xs_re >>> 1), sat18(xs_im >>> 1)};
    wire [35:0] y_calc = no_scale ? {sat18(ys_re), sat18(ys_im)}
                                  : {sat18(ys_re >>> 1), sat18(ys_im >>> 1)};

    reg  [9:0]  fm_raddr, fm_waddr;
    reg         fm_we;
    reg  [35:0] fm_wdata;
    always @* begin
        fm_raddr = 10'd0;
        fm_we    = 1'b0;
        fm_waddr = 10'd0;
        fm_wdata = 36'd0;
        rom_addr = 9'd0;
        case (eng)
            E_LOAD: begin
                rom_addr = cs_ld[8:0];
                fm_we    = ld_v2;
                fm_waddr = bitrev10(ld_n2);
                fm_wdata = {ld_re, 18'd0};
            end
            E_FFT: begin
                rom_addr = (cyc == 3'd0) ? cs_tw[8:0] : sin_adr;
                fm_raddr = (cyc == 3'd0) ? fa : fb;
                fm_we    = (cyc == 3'd3) || (cyc == 3'd4);
                fm_waddr = (cyc == 3'd3) ? fa : fb;
                fm_wdata = (cyc == 3'd3) ? x_word : y_word;
            end
            E_POST: begin
                rom_addr = EDGE_BASE + pb;
                fm_raddr = {2'b00, pk};
            end
            default: ;
        endcase
    end

    // ---------------- 频点幅度 ----------------
    wire [17:0] q_re_a = abs18(fq[35:18]);
    wire [17:0] q_im_a = abs18(fq[17:0]);
    wire [17:0] q_max  = (q_re_a > q_im_a) ? q_re_a : q_im_a;
    wire [17:0] q_min  = (q_re_a > q_im_a) ? q_im_a : q_re_a;
    wire [17:0] mag    = q_max + (q_min >> 2) + (q_min >> 3);  // |z| ≈ max + 3/8 min

    // ---------------- 空间平滑 ----------------
    wire [7:0] r_new  = (pb == NUM_BARS) ? r_cur : log2q(acc);   // 右边界复制
    wire [9:0] s_sum  = {2'b00, r_prev} + {1'b0, r_cur, 1'b0} + {2'b00, r_new};

    // ---------------- 自适应均衡与目标柱高 ----------------
    wire [19:0] st_use = st_valid ? st_q : 20'd0;
    wire [9:0]  s_h    = st_use[9:0];
    wire [9:0]  s_avg  = st_use[19:10];

    wire        gated  = (s_lv < LEVEL_FLOOR);
    wire signed [11:0] av_d   = $signed({2'b00, s_lv, 2'b00}) - $signed({2'b00, s_avg});
    wire        [10:0] av_abs = av_d[11] ? (11'd0 - av_d[10:0]) : av_d[10:0];
    wire        [5:0]  av_stp = (av_abs[10:5] == 6'd0) ? 6'd1 : av_abs[10:5];
    wire        [9:0]  avg_w  = gated             ? s_avg :
                                (s_avg == 10'd0)  ? {s_lv, 2'b00} :
                                (av_abs == 11'd0) ? s_avg :
                                av_d[11] ? (s_avg - {4'd0, av_stp}) : (s_avg + {4'd0, av_stp});
    // 柱高 = 2*(电平 - 更新后平均 + 偏置) + 响度加成
    wire signed [9:0]  rel    = $signed({2'b00, s_lv}) - $signed({2'b00, avg_w[9:2]}) + REL_BIAS;
    wire        [9:0]  t_rel  = rel[9] ? 10'd0 : {rel[8:0], 1'b0};
    wire        [7:0]  ab_d   = s_lv - ABS_KNEE;
    wire        [9:0]  t_abs  = (s_lv > ABS_KNEE) ? {3'd0, ab_d[7:1]} : 10'd0;
    wire        [9:0]  t_sum  = t_rel + t_abs;
    wire        [6:0]  tgt_w  = gated ? 7'd0 : (t_sum > H_MAX) ? H_MAX[6:0] : t_sum[6:0];

    // ---------------- 柱高动态 ----------------
    wire [9:0]  t8     = {tgt, 3'b000};
    wire        h_up   = (t8 > s_h);
    wire [9:0]  h_dd   = h_up ? (t8 - s_h) : (s_h - t8);
    wire [9:0]  n_h    = h_up            ? (s_h + {1'b0, h_dd[9:1]} + 10'd1) :
                         (h_dd == 10'd0) ? s_h : (s_h - {3'd0, h_dd[9:3]} - 10'd1);
    wire       post_upd = (eng == E_POST) && (pst == 3'd6);
    wire [7:0] pu       = pb - 8'd1;                     // 正在更新状态的柱
    wire [6:0] vb_cur   = (pu[1:0] == 2'd0 || n_h[9:3] > vb_acc) ? n_h[9:3] : vb_acc;
    wire [4:0] vb_q     = vb_cur[6] ? 5'd16 : {1'b0, vb_cur[5:2]};

    always @(posedge pix_clk) begin
        if (audio_valid)
            ring[ring_wp] <= smp18;
        ring_q <= ring[ld_base + ld_cnt[9:0]];

        if (fm_we)
            fmem[fm_waddr] <= fm_wdata;
        fq <= fmem[fm_raddr];

        if (post_upd)
            st_ram[pu[5:0]] <= {avg_n, n_h};
        st_q <= st_ram[pu[5:0]];

        ld_prod <= ld_x * $signed({1'b0, ld_w});
        if (eng == E_FFT && cyc == 3'd1) begin
            a_re <= fq[35:18];
            a_im <= fq[17:0];
            w_c  <= cs_tw[9] ? -$signed({1'b0, rom_q}) : $signed({1'b0, rom_q});
        end
        if (eng == E_FFT && cyc == 3'd2) begin
            p_rc <= b_re * w_c;
            p_is <= b_im * w_s;
            p_ic <= b_im * w_c;
            p_rs <= b_re * w_s;
        end
        if (eng == E_FFT && cyc == 3'd3)
            y_word <= y_calc;
    end

    // ---------------- 控制 ----------------
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            blank_cnt    <= 11'd0;
            ring_wp      <= 10'd0;
            smp_seen     <= 1'b0;
            silent       <= 1'b1;
            eng          <= E_IDLE;
            ld_cnt       <= 11'd0;
            ld_base      <= 10'd0;
            ld_v1        <= 1'b0;
            ld_v2        <= 1'b0;
            ld_neg1      <= 1'b0;
            ld_n1        <= 10'd0;
            ld_n2        <= 10'd0;
            fs           <= 4'd0;
            bf           <= 9'd0;
            cyc          <= 3'd0;
            pb           <= 8'd0;
            pk           <= 8'd0;
            pk_hi        <= 8'd0;
            pst          <= 3'd0;
            pv           <= 1'b0;
            acc          <= 18'd0;
            r_prev       <= 8'd0;
            r_cur        <= 8'd0;
            s_lv         <= 8'd0;
            tgt          <= 7'd0;
            avg_n        <= 10'd0;
            vb_acc       <= 7'd0;
            vis_band     <= 60'd0;
            st_valid     <= 1'b0;
        end else begin
            if (de)
                blank_cnt <= 11'd0;
            else if (blank_cnt != VBLANK_CYCLES)
                blank_cnt <= blank_cnt + 11'd1;

            if (audio_valid) begin
                ring_wp  <= ring_wp + 10'd1;
                smp_seen <= 1'b1;
            end

            if (frame_start) begin
                silent   <= !smp_seen;
                smp_seen <= 1'b0;
            end

            pv <= (eng == E_POST) && (pst == 3'd2);
            if (eng == E_POST && pst == 3'd1)
                acc <= 18'd0;
            else if (pv && mag > acc)
                acc <= mag;

            case (eng)
                E_IDLE: begin
                    if (frame_start) begin
                        eng     <= E_LOAD;
                        ld_base <= ring_wp;          // 环形缓冲已满：写指针处即最旧采样
                        ld_cnt  <= 11'd0;
                        ld_v1   <= 1'b0;
                        ld_v2   <= 1'b0;
                    end
                end
                E_LOAD: begin
                    ld_cnt  <= ld_cnt + 11'd1;
                    ld_v1   <= !ld_cnt[10];                 // ld_cnt < 1024（ld_cnt 最大 1025）
                    ld_n1   <= ld_cnt[9:0];
                    ld_neg1 <= cs_ld[9];
                    ld_v2   <= ld_v1;
                    ld_n2   <= ld_n1;
                    if (ld_cnt == 11'd1025) begin    // 第 1023 点在本拍写入
                        eng <= E_FFT;
                        fs  <= 4'd0;
                        bf  <= 9'd0;
                        cyc <= 3'd0;
                    end
                end
                E_FFT: begin
                    if (cyc == 3'd4) begin
                        cyc <= 3'd0;
                        if (bf == 9'd511) begin
                            bf <= 9'd0;
                            if (fs == 4'd9) begin
                                eng      <= E_POST;
                                pb       <= 8'd0;
                                pst      <= 3'd0;
                            end else begin
                                fs <= fs + 4'd1;
                            end
                        end else begin
                            bf <= bf + 9'd1;
                        end
                    end else begin
                        cyc <= cyc + 3'd1;
                    end
                end
                E_POST: begin
                    case (pst)
                        3'd0: pst <= 3'd1;                               // 发本柱频点范围 ROM 地址
                        3'd1: begin
                            pk    <= rom_q[7:0];
                            pk_hi <= rom_q[15:8];
                            pst   <= (pb == NUM_BARS) ? 3'd4 : 3'd2;
                        end
                        3'd2: begin                                      // 每拍读一个频点
                            pk <= pk_inc;
                            if (pk_inc == pk_hi)
                                pst <= 3'd3;
                        end
                        3'd3: pst <= 3'd4;                               // 等最后一个频点进入 acc
                        3'd4: begin                                      // 1-2-1 平滑出第 pb-1 根柱
                            s_lv   <= s_sum[9:2];
                            r_prev <= (pb == 8'd0) ? r_new : r_cur;      // 左边界复制
                            r_cur  <= r_new;
                            if (pb == 8'd0) begin
                                pb  <= 8'd1;
                                pst <= 3'd0;
                            end else begin
                                pst <= 3'd5;
                            end
                        end
                        3'd5: begin
                            tgt   <= tgt_w;
                            avg_n <= avg_w;
                            pst   <= 3'd6;
                        end
                        default: begin                                   // 写回柱状态
                            // 律动波形：每 4 根柱一段，段末移入 vis_band
                            vb_acc <= vb_cur;
                            if (pu[1:0] == 2'd3)
                                vis_band <= {vb_q, vis_band[59:5]};
                            if (pb == NUM_BARS) begin
                                eng      <= E_IDLE;
                                st_valid <= 1'b1;
                            end else begin
                                pb  <= pb + 8'd1;
                                pst <= 3'd0;
                            end
                        end
                    endcase
                end
                default: eng <= E_IDLE;
            endcase
        end
    end

endmodule
