`timescale 1ns/1ps

module wake_streak_fx #(
    parameter integer H_ACTIVE  = 640,
    parameter integer DIM_GAIN  = 8,          // 休眠亮度 = DIM_GAIN/32（8 约 25%）
    parameter integer SPEED     = 10,         // 光头每帧前进像素数（60Hz 下约 1.5s 扫完）
    parameter integer CURVE_Y0  = 280,        // 曲线在 x=0 处的行号
    parameter integer CURVE_AMP = 90,         // 正弦振幅（像素）
    parameter integer LOGO_X0   = 16,
    parameter integer LOGO_Y0   = 12,
    parameter integer LOGO_W    = 224,
    parameter integer LOGO_H    = 78
)(
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        sleep_async,           // 1 = 休眠，clk 域寄存器输出
    input  wire        vsync,
    input  wire        de,
    input  wire [23:0] rgb_in,
    output wire        vsync_out,
    output wire        de_out,
    output wire [23:0] rgb_out
);
    localparam integer LATENCY    = 3;
    localparam integer EDGE       = 32;       // 恢复亮度的软边宽度（必须为 32，对应 dx[4:0]）
    localparam integer TRAIL      = 256;      // 拖尾长度（必须为 256，对应 dx[7:0]）
    localparam integer HEAD_START = -32;      // 光头从屏幕左侧外起步，光晕逐渐进屏
    localparam integer HEAD_END   = H_ACTIVE + TRAIL;
    localparam integer LOGO_FADE  = 512;

    (* ASYNC_REG = "TRUE" *) reg sleep_meta, sleep_sync;
    reg               vs_q;
    wire              frame_start = vs_q && !vsync;
    reg               sleep_f;                // 上一帧的休眠状态
    reg               wake_on;                // 唤醒扫屏进行中
    reg signed [11:0] head_x;                 // 光头列号，可为负（尚未进屏）
    reg        [5:0]  base_gain;              // 未被扫过区域的增益 0~32

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            sleep_meta <= 1'b0;
            sleep_sync <= 1'b0;
            vs_q       <= 1'b1;
            sleep_f    <= 1'b0;
            wake_on    <= 1'b0;
            head_x     <= HEAD_START;
            base_gain  <= 6'd32;
        end else begin
            sleep_meta <= sleep_async;
            sleep_sync <= sleep_meta;
            vs_q       <= vsync;
            if (frame_start) begin
                sleep_f <= sleep_sync;
                if (sleep_sync) begin
                    // 进入 / 保持休眠：中途再次休眠也直接停掉扫屏
                    wake_on <= 1'b0;
                    if (base_gain != DIM_GAIN)
                        base_gain <= base_gain - 1'b1;
                end else if (sleep_f) begin
                    wake_on <= 1'b1;
                    head_x  <= HEAD_START;
                end else if (wake_on) begin
                    if (head_x >= HEAD_END) begin
                        wake_on   <= 1'b0;
                        base_gain <= 6'd32;
                    end else begin
                        head_x <= head_x + SPEED;
                    end
                end
            end
        end
    end

    // ---------------- 像素坐标与曲线 ----------------
    reg [9:0] pixel_x, pixel_y;               // 当前输入像素的列/行号
    reg       de_q;
    reg  signed [17:0] osc_s, osc_c;
    wire signed [17:0] osc_c_next = osc_c - ((osc_s >>> 7) + (osc_s >>> 9));
    wire signed [17:0] osc_s_next = osc_s + ((osc_c_next >>> 7) + (osc_c_next >>> 9));
    reg  [19:0] lfsr;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            pixel_x <= 10'd0;
            pixel_y <= 10'd0;
            de_q    <= 1'b0;
            osc_s   <= 18'sd0;
            osc_c   <= CURVE_AMP * 256;
            lfsr    <= 20'hA5A5B;
        end else begin
            de_q    <= de;
            pixel_x <= de ? pixel_x + 10'd1 : 10'd0;
            if (!vsync)
                pixel_y <= 10'd0;
            else if (de_q && !de)
                pixel_y <= pixel_y + 1'b1;
            if (de) begin
                osc_c <= osc_c_next;
                osc_s <= osc_s_next;
            end else begin
                osc_c <= CURVE_AMP * 256;
                osc_s <= 18'sd0;
            end
            lfsr <= {lfsr[18:0], lfsr[19] ^ lfsr[16]};
        end
    end

    wire signed [11:0] curve_y = CURVE_Y0 - $signed({5'd0, pixel_x[9:3]}) - (osc_s >>> 8);
    wire signed [11:0] dy      = $signed({2'b00, pixel_y}) - curve_y;
    wire        [11:0] dy_abs  = dy[11] ? -dy : dy;
    wire signed [11:0] dx      = head_x - $signed({2'b00, pixel_x});   // >0：像素在光头后方

    reg         logo_x_on, logo_y_on;
    wire        logo_in0 = de && logo_x_on && logo_y_on;
    reg  [13:0] logo_addr;
    wire [7:0]  logo_q;
    reg  [1:0]  logo_a_d2, logo_s_d2;         // 第 2 拍：logo 覆盖率（0 = 不显示）与色阶
    wire [23:0] logo_fg;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            logo_addr <= 14'd0;
            logo_x_on <= 1'b0;
            logo_y_on <= 1'b0;
        end else begin
            if (!vsync)
                logo_addr <= 14'd0;
            else if (logo_in0 && pixel_x[0])
                logo_addr <= logo_addr + 1'b1;

            if (!de)
                logo_x_on <= 1'b0;
            else if (pixel_x == LOGO_X0 - 1)
                logo_x_on <= 1'b1;
            else if (pixel_x == LOGO_X0 + LOGO_W - 1)
                logo_x_on <= 1'b0;

            if (!vsync)
                logo_y_on <= 1'b0;
            else if (de_q && !de) begin
                if (pixel_y == LOGO_Y0 - 1)
                    logo_y_on <= 1'b1;
                else if (pixel_y == LOGO_Y0 + LOGO_H - 1)
                    logo_y_on <= 1'b0;
            end
        end
    end

    sleep_logo_rom u_logo_rom (
        .clk     (pix_clk),
        .addr    (logo_addr),
        .q       (logo_q),
        .pal_idx ({logo_a_d2, logo_s_d2}),
        .pal_rgb (logo_fg)
    );

    // ---------------- 第 1 拍寄存 ----------------
    reg  [23:0]        rgb_d1;
    reg  signed [11:0] dx_d1;
    reg  [5:0]         ady_d1;                // |dy| 截到 63
    reg  [16:0]        rnd_d1;
    reg                logo_in_d1;
    reg                logo_odd_d1;           // 1 = 右像素，取低半字节

    // ---------------- 第 2 拍：增益与光效强度 ----------------
    wire        d_pos  = !dx_d1[11];
    wire [11:0] d_neg  = -dx_d1;
    wire [11:0] d_abs  = d_pos ? dx_d1 : d_neg;

    // 软边：光头后 0~31 像素从 base_gain 线性升到 32
    wire [5:0]  gain_room = 6'd32 - base_gain;
    (* dsp_style = "yes" *) wire [10:0] edge_mul = gain_room * dx_d1[4:0];
    reg  [5:0]  pix_gain;
    always @* begin
        if (!wake_on || !d_pos)
            pix_gain = base_gain;
        else if (dx_d1[10:5] != 6'd0)
            pix_gain = 6'd32;
        else
            pix_gain = base_gain + edge_mul[10:5];
    end

    // 光晕衰减表：255 * 0.87^i
    function [7:0] glow_tab;
        input [4:0] i;
        begin
            case (i)
                5'd0:  glow_tab = 8'd255;  5'd1:  glow_tab = 8'd222;
                5'd2:  glow_tab = 8'd193;  5'd3:  glow_tab = 8'd168;
                5'd4:  glow_tab = 8'd146;  5'd5:  glow_tab = 8'd127;
                5'd6:  glow_tab = 8'd111;  5'd7:  glow_tab = 8'd96;
                5'd8:  glow_tab = 8'd84;   5'd9:  glow_tab = 8'd73;
                5'd10: glow_tab = 8'd63;   5'd11: glow_tab = 8'd55;
                5'd12: glow_tab = 8'd48;   5'd13: glow_tab = 8'd42;
                5'd14: glow_tab = 8'd36;   5'd15: glow_tab = 8'd32;
                5'd16: glow_tab = 8'd27;   5'd17: glow_tab = 8'd24;
                5'd18: glow_tab = 8'd21;   5'd19: glow_tab = 8'd18;
                5'd20: glow_tab = 8'd16;   5'd21: glow_tab = 8'd14;
                5'd22: glow_tab = 8'd12;   5'd23: glow_tab = 8'd10;
                5'd24: glow_tab = 8'd9;    5'd25: glow_tab = 8'd8;
                5'd26: glow_tab = 8'd7;    5'd27: glow_tab = 8'd6;
                5'd28: glow_tab = 8'd5;    5'd29: glow_tab = 8'd4;
                default: glow_tab = 8'd0;
            endcase
        end
    endfunction

    // 拖尾光晕：离光头越远，衰减表索引越大，光带越暗越细
    wire       in_trail  = wake_on && d_pos && (dx_d1[10:8] == 3'd0);
    wire [6:0] trail_idx = ady_d1 + dx_d1[7:4];
    wire [7:0] trail_i   = (in_trail && trail_idx[6:5] == 2'd0) ? glow_tab(trail_idx[4:0]) : 8'd0;

    // 白色芯线：曲线上下 1 像素，拖尾前 192 像素
    wire       in_core   = wake_on && d_pos && (dx_d1[10:8] == 3'd0) && (dx_d1[7:6] != 2'b11) && (ady_d1[5:1] == 5'd0);
    wire [7:0] core_i    = in_core ? ((8'd255 - dx_d1[7:0]) >> ady_d1[0]) : 8'd0;

    // 光头光斑：近似圆距离 max + min/2，半径 32 像素
    wire       near_head = wake_on && (d_abs[11:5] == 7'd0) && !ady_d1[5];
    wire [4:0] f_a       = d_abs[4:0];
    wire [4:0] f_b       = ady_d1[4:0];
    wire [5:0] f_r       = ((f_a > f_b) ? f_a : f_b) + (((f_a > f_b) ? f_b : f_a) >> 1);
    wire [7:0] flare_i   = (near_head && !f_r[5]) ? {~f_r[4:0], 3'b000} : 8'd0;

    // 粒子：光头附近与拖尾两侧随机点亮，越靠近光头 / 曲线越密
    wire       in_spark  = wake_on && !(ady_d1[5] && ady_d1[4]) && (d_pos ? (dx_d1[10:8] == 3'd0) : (d_abs[11:4] == 8'd0));
    wire [5:0] spark_far = d_pos ? {1'b0, dx_d1[7:3]} : {1'b0, d_abs[3:0], 1'b0};
    wire signed [7:0] spark_th = 8'sd48 - $signed({2'b00, ady_d1}) - $signed({2'b00, spark_far});
    wire       spark_hit = in_spark && !spark_th[7] && (spark_th != 8'sd0) && (rnd_d1[9:0] < {2'b00, spark_th});
    wire [7:0] spark_i   = spark_hit ? {1'b1, rnd_d1[16:10]} : 8'd0;

    wire [3:0] logo_nib    = logo_odd_d1 ? logo_q[3:0] : logo_q[7:4];
    wire       logo_behind = wake_on && d_pos;
    wire [6:0] logo_in_th  = (base_gain - DIM_GAIN) << 1;
    wire [6:0] logo_th     = logo_behind ? {1'b0, dx_d1[8:3]} : logo_in_th;
    wire       logo_gone   = logo_behind && (dx_d1[10:9] != 2'd0);
    wire       logo_vis    = (sleep_f || wake_on) && logo_in_d1 && (logo_nib[3:2] != 2'd0)
                           && !logo_gone && ({1'b0, rnd_d1[15:10]} >= logo_th);
    wire [1:0] logo_shade  = (logo_behind && (dx_d1[8:5] == 4'd0)) ? 2'd3 : logo_nib[1:0];

    // 蓝白配色：光晕偏青蓝，芯线 / 光斑 / 粒子偏白
    wire [9:0] glow_r_sum = (trail_i >> 2) + core_i + (flare_i >> 1) + (spark_i >> 1);
    wire [9:0] glow_g_sum = (trail_i >> 1) + (trail_i >> 2) + core_i
                          + (flare_i >> 1) + (flare_i >> 2) + (spark_i >> 1) + (spark_i >> 2);
    wire [9:0] glow_b_sum = trail_i + core_i + flare_i + spark_i;

    reg  [23:0] rgb_d2;
    reg  [5:0]  gain_d2;
    reg  [7:0]  glow_r_d2, glow_g_d2, glow_b_d2;

    (* dsp_style = "yes" *) wire [13:0] mul_r = rgb_d2[23:16] * gain_d2;
    (* dsp_style = "yes" *) wire [13:0] mul_g = rgb_d2[15:8]  * gain_d2;
    (* dsp_style = "yes" *) wire [13:0] mul_b = rgb_d2[7:0]   * gain_d2;
    wire [8:0]  sum_r = mul_r[12:5] + glow_r_d2;
    wire [8:0]  sum_g = mul_g[12:5] + glow_g_d2;
    wire [8:0]  sum_b = mul_b[12:5] + glow_b_d2;
    wire [7:0]  bg_r  = sum_r[8] ? 8'd255 : sum_r[7:0];
    wire [7:0]  bg_g  = sum_g[8] ? 8'd255 : sum_g[7:0];
    wire [7:0]  bg_b  = sum_b[8] ? 8'd255 : sum_b[7:0];

    function [7:0] logo_mix;
        input [7:0] bg;
        input [7:0] pfg;
        input [1:0] a;
        reg   [7:0] bg_part;
        begin
            case (a)
                2'd1:    bg_part = {1'b0, bg[7:1]};
                2'd2:    bg_part = {2'b00, bg[7:2]};
                default: bg_part = 8'd0;
            endcase
            logo_mix = (a == 2'd0) ? bg : (bg_part + pfg);   // 先选操作数再相加，只综合出一个加法器
        end
    endfunction

    reg  [23:0] rgb_d3;
    reg  [LATENCY-1:0] de_pipe, vs_pipe;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            rgb_d1    <= 24'd0;
            dx_d1     <= 12'sd0;
            ady_d1    <= 6'd0;
            rnd_d1    <= 17'd0;
            logo_in_d1  <= 1'b0;
            logo_odd_d1 <= 1'b0;
            logo_a_d2   <= 2'd0;
            logo_s_d2   <= 2'd0;
            rgb_d2    <= 24'd0;
            gain_d2   <= 6'd32;
            glow_r_d2 <= 8'd0;
            glow_g_d2 <= 8'd0;
            glow_b_d2 <= 8'd0;
            rgb_d3    <= 24'd0;
            de_pipe   <= {LATENCY{1'b0}};
            vs_pipe   <= {LATENCY{1'b1}};
        end else begin
            de_pipe <= {de_pipe[LATENCY-2:0], de};
            vs_pipe <= {vs_pipe[LATENCY-2:0], vsync};

            rgb_d1 <= rgb_in;
            dx_d1  <= dx;
            ady_d1 <= (dy_abs[11:6] != 6'd0) ? 6'd63 : dy_abs[5:0];
            rnd_d1 <= lfsr[16:0];
            logo_in_d1  <= logo_in0;
            logo_odd_d1 <= pixel_x[0];

            logo_a_d2 <= logo_vis ? logo_nib[3:2] : 2'd0;
            logo_s_d2 <= logo_shade;
            rgb_d2    <= rgb_d1;
            gain_d2   <= pix_gain;
            glow_r_d2 <= glow_r_sum[9:8] != 2'b00 ? 8'd255 : glow_r_sum[7:0];
            glow_g_d2 <= glow_g_sum[9:8] != 2'b00 ? 8'd255 : glow_g_sum[7:0];
            glow_b_d2 <= glow_b_sum[9:8] != 2'b00 ? 8'd255 : glow_b_sum[7:0];

            rgb_d3 <= {logo_mix(bg_r, logo_fg[23:16], logo_a_d2),
                       logo_mix(bg_g, logo_fg[15:8],  logo_a_d2),
                       logo_mix(bg_b, logo_fg[7:0],   logo_a_d2)};
        end
    end

    assign rgb_out   = rgb_d3;
    assign de_out    = de_pipe[LATENCY-1];
    assign vsync_out = vs_pipe[LATENCY-1];
endmodule
