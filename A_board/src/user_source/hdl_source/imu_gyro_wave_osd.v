`timescale 1ns/1ps

module imu_gyro_wave_osd #(
    parameter integer PLOT_X0       = 66,    // 绘图区左上角（屏幕坐标），左侧需 >= 36 像素刻度栏
    parameter integer PLOT_Y0       = 93,
    parameter integer PLOT_W        = 544,   // 必须为偶数且 <= 1000
    parameter integer PLOT_H        = 201,   // 必须 <= 255，且 >= 2 * 86
    parameter integer PEAK_X0       = 25,    // 读数卡片左上角（屏幕坐标）
    parameter integer PEAK_Y0       = 305,
    parameter integer PEAK_W        = 589,   // 需 >= 452（左 / 中栏各 >= 150，右栏 >= 96）
    parameter integer PEAK_H        = 104,   // 需 >= 96（三行，每行 32）
    parameter integer CD_X0         = 166,   // 倒计时左边（"已就绪" 止于 145，留 20 像素）
    parameter integer CD_Y0         = 441,   // 与第二页 "陀螺仪：" 状态同一单元顶行
    parameter integer VBLANK_CYCLES = 1024   // de 连续低电平超过该值视为场消隐（需大于行消隐）
)(
    input  wire               ctrl_clk,
    input  wire               ctrl_rst_n,
    input  wire               attitude_valid,
    input  wire               calibrated,
    input  wire signed [15:0] roll_deg_x10,
    input  wire signed [15:0] pitch_deg_x10,
    input  wire signed [15:0] yaw_deg_x10,
    input  wire               pix_clk,
    input  wire               pix_rst_n,
    input  wire               de,
    input  wire               display_enable, // pix_clk 域电平，在帧起始处锁存
    input  wire               test_show_async,  // 实体检测显示使能（ctrl 域电平，以下同）
    input  wire [1:0]         test_phase_async, // 0 未检测 / 1 等待 / 2 检测中 / 3 结果
    input  wire [3:0]         test_secs_async,  // 本阶段剩余秒数
    input  wire [6:0]         test_score_async, // 评分 0~100
    input  wire [13:0]        test_tilt_async,
    input  wire [23:0]        rgb_in,
    output wire [23:0]        rgb_out
);
    localparam integer CENTER    = PLOT_H / 2;
    localparam [7:0]   CENTER_8  = CENTER;
    localparam [9:0]   PLOT_W_10 = PLOT_W;
    localparam [7:0]   PLOT_Y0_V = PLOT_Y0;
    localparam integer G1 = (450 * 3) / 64;
    localparam integer G2 = (900 * 3) / 64;
    localparam integer G3 = (1350 * 3) / 64;
    localparam integer G4 = (1800 * 3) / 64;
    localparam [23:0]  COLOR_X    = 24'hE53935;
    localparam [23:0]  COLOR_Y    = 24'h43A047;
    localparam [23:0]  COLOR_Z    = 24'h1E88E5;
    localparam [23:0]  COLOR_AXIS = 24'hA0A0A0;
    localparam [23:0]  COLOR_GRID = 24'hE4E4E4;
    localparam [23:0]  COLOR_TEXT = 24'h64748B;
    localparam [23:0]  COLOR_PEAK = 24'hCBD5E1;  // 峰值读数
    localparam [23:0]  COLOR_DIV  = 24'h2A3B66;  // 读数卡片分栏线
    localparam [23:0]  COLOR_WAIT = 24'hFFB020;  // 倒计时 "等待开始"
    localparam [23:0]  COLOR_MEAS = 24'h3CC8FF;  // 倒计时 "检测中"
    localparam [23:0]  COLOR_EXC  = 24'h36D67A;  // 优秀
    localparam [23:0]  COLOR_GOOD = 24'h3CC8FF;  // 良好
    localparam [23:0]  COLOR_PASS = 24'hFFB020;  // 合格
    localparam [23:0]  COLOR_FAIL = 24'hFF4D4F;  // 不合格

    function [7:0] map_angle;
        input signed [15:0] angle;
        reg   signed [17:0] a3;
        integer pos;
        begin
            a3  = {{2{angle[15]}}, angle} + {angle[15], angle, 1'b0};
            pos = CENTER - (a3 >>> 6);
            if (pos < 1)
                map_angle = 8'd1;
            else if (pos > PLOT_H - 2)
                map_angle = PLOT_H - 2;
            else
                map_angle = pos[7:0];
        end
    endfunction

    function [7:0] average;
        input [7:0] a;
        input [7:0] b;
        reg [8:0] sum;
        begin
            sum = {1'b0, a} + {1'b0, b};
            average = sum[8:1];
        end
    endfunction

    function [15:0] sign_mag;
        input [15:0] v;
        reg   [15:0] a;
        begin
            a        = v[15] ? 16'd0 - v : v;
            sign_mag = {v[15], (a > 16'd9999) ? 15'd9999 : a[14:0]};
        end
    endfunction

    reg [23:0] sample_hold;
    reg [47:0] raw_hold;     // 原始角度 {roll, pitch, yaw}，供读数与峰值统计
    reg        sample_toggle;

    always @(posedge ctrl_clk or negedge ctrl_rst_n) begin
        if (!ctrl_rst_n) begin
            sample_hold   <= {3{CENTER_8}};
            raw_hold      <= 48'd0;
            sample_toggle <= 1'b0;
        end else if (attitude_valid && calibrated) begin
            sample_hold   <= {map_angle(roll_deg_x10), map_angle(pitch_deg_x10), map_angle(yaw_deg_x10)};
            raw_hold      <= {roll_deg_x10, pitch_deg_x10, yaw_deg_x10};
            sample_toggle <= ~sample_toggle;
        end
    end

    // ---------------- pix_clk 域：环形缓冲写入 ----------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] toggle_sync;
    reg        toggle_seen;
    reg [1:0]  wr_state;     // 0 空闲，1 写中点列，2 写当前列
    reg [23:0] cur_val, prev_val;
    reg [9:0]  wr_ptr;       // 下一个待写列；[wr_ptr-fill, wr_ptr) 为完整数据
    reg [9:0]  fill;

    wire new_sample = toggle_sync[1] ^ toggle_seen;
    wire [23:0] mid_val = {average(prev_val[23:16], cur_val[23:16]),
                           average(prev_val[15:8],  cur_val[15:8]),
                           average(prev_val[7:0],   cur_val[7:0])};

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            toggle_sync <= 2'b00;
            toggle_seen <= 1'b0;
            wr_state    <= 2'd0;
            cur_val     <= {3{CENTER_8}};
            prev_val    <= {3{CENTER_8}};
            wr_ptr      <= 10'd0;
            fill        <= 10'd0;
        end else begin
            toggle_sync <= {toggle_sync[0], sample_toggle};
            case (wr_state)
                2'd0: if (new_sample) begin
                    toggle_seen <= toggle_sync[1];
                    cur_val     <= sample_hold;
                    wr_state    <= 2'd1;
                end
                2'd1: wr_state <= 2'd2;
                default: begin
                    prev_val <= cur_val;
                    wr_ptr   <= wr_ptr + 10'd2;
                    fill     <= (fill >= PLOT_W - 2) ? PLOT_W_10 : fill + 10'd2;
                    wr_state <= 2'd0;
                end
            endcase
        end
    end

    // ---------------- 行场计数与帧锁存 ----------------
    reg [10:0] x_cnt;        // de 有效期间等于当前像素列
    reg [9:0]  y_cnt;
    reg        de_q;
    reg [10:0] blank_cnt;
    reg [9:0]  base_ptr;     // 本帧显示窗口右端（不含）
    reg [9:0]  fill_frame;
    reg        enable_frame;

    wire frame_start = !de && (blank_cnt == VBLANK_CYCLES - 1);

    reg  [9:0] yw;
    reg        plot_x;
    reg        lbl_x, unit_x, ltr_x, cur_x, pkv_x, tri_x;   // x_next 窗口（tri_x 为 x_cnt 窗口）
    function ywin_nx;
        input       cur;
        input [9:0] y, y0, h;
        ywin_nx = (y == y0 - 10'd1) ? 1'b1 : (y == y0 + h - 10'd1) ? 1'b0 : cur;
    endfunction
    function xwin_nx;
        input        cur;
        input [10:0] x, x0, w;
        xwin_nx = (x == x0 - 11'd1) ? 1'b1 : (x == x0 + w - 11'd1) ? 1'b0 : cur;
    endfunction

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            x_cnt        <= 11'd0;
            y_cnt        <= 10'd0;
            de_q         <= 1'b0;
            blank_cnt    <= 11'd0;
            base_ptr     <= 10'd0;
            fill_frame   <= 10'd0;
            enable_frame <= 1'b0;
            yw           <= 10'd0;
            plot_x       <= 1'b0;
            lbl_x        <= 1'b0;
            unit_x       <= 1'b0;
            ltr_x        <= 1'b0;
            cur_x        <= 1'b0;
            pkv_x        <= 1'b0;
            tri_x        <= 1'b0;
        end else begin
            de_q  <= de;
            x_cnt <= de ? x_cnt + 11'd1 : 11'd0;
            if (!de) begin
                plot_x <= 1'b0;
                lbl_x  <= 1'b0;
                unit_x <= 1'b0;
                ltr_x  <= 1'b0;
                cur_x  <= 1'b0;
                pkv_x  <= 1'b0;
                tri_x  <= 1'b0;
            end else begin
                plot_x <= xwin_nx(plot_x, x_cnt, PLOT_X0, PLOT_W);
                lbl_x  <= xwin_nx(lbl_x,  x_cnt, LABEL_X0 - 1, 32);
                unit_x <= xwin_nx(unit_x, x_cnt, UNIT_X0 - 1, 8);
                ltr_x  <= xwin_nx(ltr_x,  x_cnt, LTR_X - 1, 20);
                cur_x  <= xwin_nx(cur_x,  x_cnt, CUR_X - 1, FIELD_W);
                pkv_x  <= xwin_nx(pkv_x,  x_cnt, PKV_X - 1, FIELD_W);
                tri_x  <= xwin_nx(tri_x,  x_cnt, TRI_X, 11);
            end

            if (de)
                blank_cnt <= 11'd0;
            else if (blank_cnt != VBLANK_CYCLES)
                blank_cnt <= blank_cnt + 11'd1;

            if (frame_start) begin
                // 一帧内窗口固定，写入只发生在窗口右侧，画面不会撕裂。
                y_cnt        <= 10'd0;
                yw           <= 10'd0;
                base_ptr     <= wr_ptr;
                fill_frame   <= fill;
                enable_frame <= display_enable;
            end else if (de_q && !de) begin
                y_cnt <= y_cnt + 10'd1;
                yw[1] <= ywin_nx(yw[1], y_cnt, PK_ROW0, 96);
                yw[2] <= ywin_nx(yw[2], y_cnt, PLOT_Y0, PLOT_H);
                yw[3] <= ywin_nx(yw[3], y_cnt, LBL_Y0, 16);
                yw[4] <= ywin_nx(yw[4], y_cnt, LBL_Y1, 16);
                yw[5] <= ywin_nx(yw[5], y_cnt, LBL_Y2, 16);
                yw[6] <= ywin_nx(yw[6], y_cnt, LBL_Y3, 16);
                yw[7] <= ywin_nx(yw[7], y_cnt, LBL_Y4, 16);
                yw[8] <= ywin_nx(yw[8], y_cnt, UNIT_Y, 16);
                yw[9] <= ywin_nx(yw[9], y_cnt, CD_Y0, 16);
            end
        end
    end

    (* ASYNC_REG = "TRUE" *) reg [13:0] ctl_meta, ctl_sync;   // {show, phase, secs, score}
    reg  [13:0] ctl_last;
    wire        ctl_hold = (ctl_sync == ctl_last);
    wire        c_show   = ctl_sync[13];
    wire [1:0]  c_phase  = ctl_sync[12:11];

    reg         f_cd;                // 本帧画倒计时
    reg         f_wait;              // 倒计时为 "等待开始"（否则 "检测中"）
    reg         f_meas;              // 本帧处于检测阶段
    reg         f_res;               // 本帧显示评分 / 等级
    reg  [3:0]  f_secs;
    reg  [6:0]  f_score;
    reg  [13:0] f_tilt;

    // 进入检测阶段的首帧
    wire        meas_start = frame_start && c_show && ctl_hold && (c_phase == 2'd2) && !f_meas;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            ctl_meta <= 14'd0;
            ctl_sync <= 14'd0;
            ctl_last <= 14'd0;
            f_cd     <= 1'b0;
            f_wait   <= 1'b0;
            f_meas   <= 1'b0;
            f_res    <= 1'b0;
            f_secs   <= 4'd0;
            f_score  <= 7'd0;
            f_tilt   <= 14'd0;
        end else begin
            ctl_meta <= {test_show_async, test_phase_async, test_secs_async, test_score_async};
            ctl_sync <= ctl_meta;
            ctl_last <= ctl_sync;
            if (frame_start) begin
                if (!c_show) begin
                    f_cd   <= 1'b0;
                    f_meas <= 1'b0;
                    f_res  <= 1'b0;
                end else if (ctl_hold) begin
                    // 不稳定的那一帧沿用上一帧
                    f_cd    <= (c_phase == 2'd1 || c_phase == 2'd2) && (ctl_sync[10:7] != 4'd0);
                    f_wait  <= (c_phase == 2'd1);
                    f_meas  <= (c_phase == 2'd2);
                    f_res   <= (c_phase == 2'd3);
                    f_secs  <= ctl_sync[10:7];
                    f_score <= ctl_sync[6:0];
                    if (c_phase == 2'd3)
                        f_tilt <= test_tilt_async;
                end
            end
        end
    end

    // ---------------- 当前角度与三轴峰值 ----------------
    reg  [15:0] ang_x, ang_y, ang_z;
    reg  [15:0] pk_x, pk_y, pk_z;
    wire        peak_clr = (frame_start && display_enable && !enable_frame) || meas_start;
    wire [15:0] sm_x = sign_mag(raw_hold[47:32]);
    wire [15:0] sm_y = sign_mag(raw_hold[31:16]);
    wire [15:0] sm_z = sign_mag(raw_hold[15:0]);

    localparam integer PK_ROW0  = PEAK_Y0 + (PEAK_H - 96) / 2;
    localparam [6:0]   PK_ROW0_V = PK_ROW0;
    localparam integer FIELD_W  = 122;
    localparam integer SEC_W    = (PEAK_W - 2) / 3;                // 左 / 中栏宽
    localparam integer DIV0_X   = PEAK_X0 + SEC_W;                 // 两条分隔线，各宽 1
    localparam integer DIV1_X   = DIV0_X + 1 + SEC_W;
    localparam integer S2_X     = DIV1_X + 1;                      // 右栏起点与宽度
    localparam integer S2_W     = PEAK_X0 + PEAK_W - S2_X;
    localparam integer LTR_X    = PEAK_X0 + (SEC_W - 150) / 2;     // 轴名字母，宽 20
    localparam integer CUR_X    = LTR_X + 28;                      // 当前值读数区，宽 122
    localparam integer TRI_X    = DIV0_X + 1 + (SEC_W - 142) / 2;  // 峰值标记 ▲，宽 11
    localparam integer PKV_X    = TRI_X + 20;                      // 最大值读数区，宽 122
    localparam [6:0]   LTR_X_V  = LTR_X;
    localparam [6:0]   CUR_X_V  = CUR_X;
    localparam [6:0]   PKV_X_V  = PKV_X;
    localparam [10:0]  TTL_X    = S2_X + (S2_W - 56) / 2;
    localparam [10:0]  R1N_X    = S2_X + (S2_W - 40) / 2 - 48;
    localparam [10:0]  R1H_X    = S2_X + (S2_W - 164) / 2 - 28;
    localparam [10:0]  R1G_X    = S2_X + (S2_W - 144) / 2 - 48;
    localparam [10:0]  R1F_X    = S2_X + (S2_W - 176) / 2 - 48;
    localparam [10:0]  R1S_X    = S2_X + (S2_W - 156) / 2 - 68;
    localparam [10:0]  D1_X     = S2_X + (S2_W - 152) / 2;
    localparam [10:0]  D2_X     = S2_X + (S2_W - 160) / 2;

    // 只用到低 7 位（行号 0~95），按模 128 相减即可
    wire [6:0]  pk_ry    = y_cnt[6:0] - PK_ROW0_V[6:0];
    wire        pk_rows  = yw[1];
    wire [1:0]  pk_row   = pk_ry[6:5];                          // 0 X / 1 Y / 2 Z
    wire [4:0]  pk_cr    = pk_ry[4:0];                          // 行内第几行
    wire [15:0] pk_sel   = (pk_row == 2'd0) ? pk_x  : (pk_row == 2'd1) ? pk_y  : pk_z;
    wire [15:0] ang_sel  = (pk_row == 2'd0) ? ang_x : (pk_row == 2'd1) ? ang_y : ang_z;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            ang_x <= 16'd0;
            ang_y <= 16'd0;
            ang_z <= 16'd0;
            pk_x  <= 16'd0;
            pk_y  <= 16'd0;
            pk_z  <= 16'd0;
        end else begin
            if (wr_state == 2'd0 && new_sample) begin
                // 与 cur_val <= sample_hold 同拍取 raw_hold，同样已稳定
                ang_x <= sm_x;
                ang_y <= sm_y;
                ang_z <= sm_z;
            end
            if (peak_clr) begin
                pk_x <= 16'd0;
                pk_y <= 16'd0;
                pk_z <= 16'd0;
            end else if (wr_state == 2'd0 && new_sample) begin
                if (sm_x[14:0] > pk_x[14:0]) pk_x <= sm_x;
                if (sm_y[14:0] > pk_y[14:0]) pk_y <= sm_y;
                if (sm_z[14:0] > pk_z[14:0]) pk_z <= sm_z;
            end
        end
    end

    function [15:0] bcd_adj;
        input [15:0] v;
        integer k;
        begin
            for (k = 0; k < 16; k = k + 4)
                bcd_adj[k +: 4] = (v[k +: 4] >= 4'd5) ? v[k +: 4] + 4'd3 : v[k +: 4];
        end
    endfunction

    reg         cv_busy;
    reg  [3:0]  cv_cnt;
    reg  [14:0] cv_nc, cv_np;            // 待移出的当前值 / 峰值
    reg  [15:0] dig_c, dig_p;            // 4 位 BCD，转换完成后即为本行读数
    reg         neg_c, neg_p;            // 本行读数符号
    // y_cnt 在上一行结束后一拍已指向下一行，行消隐第 2 拍起转换
    wire        cv_start = (blank_cnt == 11'd2) && pk_rows && (pk_cr == 5'd0);
    wire [15:0] adj_c    = bcd_adj(dig_c);
    wire [15:0] adj_p    = bcd_adj(dig_p);

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            cv_busy <= 1'b0;
            cv_cnt  <= 4'd0;
            cv_nc   <= 15'd0;
            cv_np   <= 15'd0;
            dig_c   <= 16'd0;
            dig_p   <= 16'd0;
            neg_c   <= 1'b0;
            neg_p   <= 1'b0;
        end else if (cv_start) begin
            cv_busy <= 1'b1;
            cv_cnt  <= 4'd0;
            cv_nc   <= ang_sel[14:0];
            cv_np   <= pk_sel[14:0];
            dig_c   <= 16'd0;
            dig_p   <= 16'd0;
            neg_c   <= ang_sel[15];
            neg_p   <= pk_sel[15];
        end else if (cv_busy) begin
            cv_cnt <= cv_cnt + 4'd1;
            cv_nc  <= {cv_nc[13:0], 1'b0};
            cv_np  <= {cv_np[13:0], 1'b0};
            dig_c  <= {adj_c[14:0], cv_nc[14]};
            dig_p  <= {adj_p[14:0], cv_np[14]};
            if (cv_cnt == 4'd14)
                cv_busy <= 1'b0;
        end
    end

    reg  [23:0] wave_ram [0:1023];
    reg  [23:0] rd_data, rd_prev;
    wire [10:0] x_next   = de ? x_cnt + 11'd1 : 11'd0;
    wire [10:0] rd_full  = {1'b0, base_ptr} + x_next - (PLOT_X0 + PLOT_W);
    wire [9:0]  rd_addr  = rd_full[9:0];
    wire        ram_we   = (wr_state != 2'd0);
    wire [9:0]  wr_addr  = (wr_state == 2'd1) ? wr_ptr : wr_ptr + 10'd1;
    wire [23:0] wr_data  = (wr_state == 2'd1) ? mid_val : cur_val;

    always @(posedge pix_clk) begin
        if (ram_we)
            wave_ram[wr_addr] <= wr_data;
        rd_data <= wave_ram[rd_addr];
        rd_prev <= rd_data;  // 与 rd_data 对齐时恰为上一列
    end

    // ---------------- 像素着色 ----------------
    wire        in_plot  = plot_x && yw[2];
    wire [10:0] plot_col = x_cnt - PLOT_X0;
    wire [7:0]  plot_row = y_cnt[7:0] - PLOT_Y0_V[7:0];       // PLOT_H <= 255，只用低 8 位
    wire [10:0] first_col = PLOT_W - fill_frame;
    wire        col_valid = (plot_col >= first_col);
    wire        col_first = (plot_col == first_col);

    // 在相邻两列之间画竖直线段并上下各加粗 1 像素，曲线因此连续。
    function hit_trace;
        input [7:0] row;
        input [7:0] cur;
        input [7:0] prv;
        reg   [8:0] lo, hi;
        begin
            lo = (cur < prv) ? cur : prv;
            hi = (cur < prv) ? prv : cur;
            hit_trace = ({1'b0, row} + 9'd1 >= lo) && ({1'b0, row} <= hi + 9'd1);
        end
    endfunction

    wire [23:0] prev_sel = col_first ? rd_data : rd_prev;
    wire hit_x = col_valid && hit_trace(plot_row, rd_data[23:16], prev_sel[23:16]);
    wire hit_y = col_valid && hit_trace(plot_row, rd_data[15:8],  prev_sel[15:8]);
    wire hit_z = col_valid && hit_trace(plot_row, rd_data[7:0],   prev_sel[7:0]);
    wire on_axis = (plot_row == CENTER);
    wire on_grid = (plot_col[4:0] == 5'd0) ||
                   (plot_row == CENTER - G1) || (plot_row == CENTER + G1) ||
                   (plot_row == CENTER - G2) || (plot_row == CENTER + G2) ||
                   (plot_row == CENTER - G3) || (plot_row == CENTER + G3) ||
                   (plot_row == CENTER - G4) || (plot_row == CENTER + G4);

    localparam [4:0] G_PLUS = 5'd10, G_MINUS = 5'd11, G_DOT = 5'd12,
                     G_DEG  = 5'd13, G_BLANK = 5'd16;
    localparam [4:0] G_LX   = 5'd14, G_LY = 5'd15, G_LZ = 5'd12;   // 仅大字库：轴名字母
    localparam integer LABEL_X0 = PLOT_X0 - 36;
    localparam integer UNIT_X0  = PLOT_X0 + 4;
    localparam integer LBL_Y0   = PLOT_Y0 + CENTER - G4 - 7;   // +180
    localparam integer LBL_Y1   = PLOT_Y0 + CENTER - G2 - 7;   // +90
    localparam integer LBL_Y2   = PLOT_Y0 + CENTER - 7;        // 0
    localparam integer LBL_Y3   = PLOT_Y0 + CENTER + G2 - 7;   // -90
    localparam integer LBL_Y4   = PLOT_Y0 + CENTER + G4 - 7;   // -180
    localparam integer UNIT_Y   = PLOT_Y0 + CENTER - G4 + 1;   // 位于 +180 与 +135 网格之间
    localparam [4:0]   LABEL_X0_V = LABEL_X0;
    localparam [4:0]   UNIT_X0_V  = UNIT_X0;
    localparam [3:0]   LBL_Y0_V   = LBL_Y0;
    localparam [3:0]   LBL_Y1_V   = LBL_Y1;
    localparam [3:0]   LBL_Y2_V   = LBL_Y2;
    localparam [3:0]   LBL_Y3_V   = LBL_Y3;
    localparam [3:0]   LBL_Y4_V   = LBL_Y4;
    localparam [3:0]   UNIT_Y_V   = UNIT_Y;

    reg [4:0]  text_rel_x;                  // 只用低 5 位（字格 32 列）
    reg [3:0]  text_rel_y;                  // 只用低 4 位（字高 16 行）
    reg [4:0]  text_glyph;

    always @* begin
        text_glyph = G_BLANK;
        text_rel_y = 4'd0;
        text_rel_x = x_next[4:0] - LABEL_X0_V[4:0];
        if (lbl_x) begin
            // 刻度值：±180 / ±90 / 0，右对齐
            if (yw[3]) begin
                text_rel_y = y_cnt[3:0] - LBL_Y0_V[3:0];
                case (text_rel_x[4:3])
                    2'd0: text_glyph = G_PLUS;
                    2'd1: text_glyph = 5'd1;
                    2'd2: text_glyph = 5'd8;
                    default: text_glyph = 5'd0;
                endcase
            end else if (yw[4]) begin
                text_rel_y = y_cnt[3:0] - LBL_Y1_V[3:0];
                case (text_rel_x[4:3])
                    2'd0: text_glyph = G_BLANK;
                    2'd1: text_glyph = G_PLUS;
                    2'd2: text_glyph = 5'd9;
                    default: text_glyph = 5'd0;
                endcase
            end else if (yw[5]) begin
                text_rel_y = y_cnt[3:0] - LBL_Y2_V[3:0];
                text_glyph = (text_rel_x[4:3] == 2'd3) ? 5'd0 : G_BLANK;
            end else if (yw[6]) begin
                text_rel_y = y_cnt[3:0] - LBL_Y3_V[3:0];
                case (text_rel_x[4:3])
                    2'd0: text_glyph = G_BLANK;
                    2'd1: text_glyph = G_MINUS;
                    2'd2: text_glyph = 5'd9;
                    default: text_glyph = 5'd0;
                endcase
            end else if (yw[7]) begin
                text_rel_y = y_cnt[3:0] - LBL_Y4_V[3:0];
                case (text_rel_x[4:3])
                    2'd0: text_glyph = G_MINUS;
                    2'd1: text_glyph = 5'd1;
                    2'd2: text_glyph = 5'd8;
                    default: text_glyph = 5'd0;
                endcase
            end
        end else if (unit_x && yw[8]) begin
            text_rel_x = x_next[4:0] - UNIT_X0_V[4:0];
            text_rel_y = y_cnt[3:0] - UNIT_Y_V[3:0];
            text_glyph = G_DEG;
        end
    end

    localparam [4:0]  C_DIGIT  = 5'd9;
    localparam [4:0]  C_S      = 5'd19;
    localparam [4:0]  C_DOT    = 5'd30;
    localparam [4:0]  C_DEG    = 5'd31;
    localparam [10:0] CD_X0_V  = CD_X0;
    localparam [9:0]  CD_Y0_V  = CD_Y0;

    wire        s100     = (f_score >= 7'd100);
    wire        s2d      = (f_score >= 7'd10);
    wire [3:0]  s_ten    = s100                ? 4'd0 :
                           (f_score >= 7'd90)  ? 4'd9 : (f_score >= 7'd80) ? 4'd8 :
                           (f_score >= 7'd70)  ? 4'd7 : (f_score >= 7'd60) ? 4'd6 :
                           (f_score >= 7'd50)  ? 4'd5 : (f_score >= 7'd40) ? 4'd4 :
                           (f_score >= 7'd30)  ? 4'd3 : (f_score >= 7'd20) ? 4'd2 :
                           (f_score >= 7'd10)  ? 4'd1 : 4'd0;
    wire [6:0]  s_ten10  = {s_ten, 3'b000} + {2'b00, s_ten, 1'b0};
    wire [6:0]  s_rem    = f_score - s_ten10;
    wire [3:0]  s_one    = s100 ? 4'd0 : s_rem[3:0];
    wire [1:0]  grade    = (f_score >= 7'd90) ? 2'd0 : (f_score >= 7'd80) ? 2'd1 :
                           (f_score >= 7'd60) ? 2'd2 : 2'd3;           // 优秀 / 良好 / 合格 / 不合格
    wire        grade3   = (grade == 2'd3);
    wire [23:0] grade_ink = (grade == 2'd0) ? COLOR_EXC  : (grade == 2'd1) ? COLOR_GOOD :
                            (grade == 2'd2) ? COLOR_PASS : COLOR_FAIL;

    wire        cd_rows  = yw[9];
    wire        t_two    = (f_tilt[11:8] != 4'd0);                 // 幅度为两位整数
    wire [10:0] r1_x0    = !f_res ? R1N_X : s100 ? R1H_X : !s2d ? R1S_X : grade3 ? R1F_X : R1G_X;
    wire [10:0] tx0      = cd_rows ? CD_X0_V : (pk_row == 2'd0) ? TTL_X :
                           (pk_row == 2'd1) ? r1_x0 : (t_two ? D2_X : D1_X);
    wire [10:0] trel     = x_next - tx0;

    wire [1:0]  s_k      = (trel < 11'd14) ? 2'd0 : (trel < 11'd28) ? 2'd1 : (trel < 11'd42) ? 2'd2 : 2'd3;
    wire [3:0]  s_col    = trel[3:0] - ((s_k == 2'd0) ? 4'd0  : (s_k == 2'd1) ? 4'd14 :
                                        (s_k == 2'd2) ? 4'd12 : 4'd10);

    //   数字段相对文字起点 60 / 46 开始，各格起点模 8 分别为 4 / 6
    wire [3:0]  cd_row   = y_cnt[3:0] - CD_Y0_V[3:0];
    wire        sec2     = (f_secs >= 4'd10);
    wire [3:0]  sec_one  = sec2 ? f_secs - 4'd10 : f_secs;
    wire [4:0]  num_c0   = sec2 ? C_DIGIT + 5'd1 : C_DIGIT + sec_one;
    wire [4:0]  num_c1   = sec2 ? C_DIGIT + sec_one : C_S;
    wire [6:0]  n_r0     = f_wait ? 7'd60 : 7'd46;
    wire        tr_lt54  = (trel < 11'd54),  tr_lt62 = (trel < 11'd62),  tr_lt68 = (trel < 11'd68),  tr_lt76 = (trel < 11'd76);
    wire        tr_ge46  = (trel >= 11'd46), tr_ge60 = (trel >= 11'd60), tr_lt70 = (trel < 11'd70),  tr_lt84 = (trel < 11'd84);
    wire [1:0]  n_j      = (f_wait ? tr_lt68 : tr_lt54) ? 2'd0 : (f_wait ? tr_lt76 : tr_lt62) ? 2'd1 : 2'd2;
    wire [2:0]  n_col    = trel[2:0] - (f_wait ? 3'd4 : 3'd6);
    wire [4:0]  n_code   = (n_j == 2'd0) ? num_c0 : (n_j == 2'd1) ? num_c1 : C_S;
    wire        cd_txt   = cd_rows && f_cd && (f_wait ? (trel < 11'd56) : (trel < 11'd42));
    wire        cd_num   = cd_rows && f_cd && (f_wait ? tr_ge60 : tr_ge46) &&
                           (f_wait ? (sec2 ? tr_lt84 : tr_lt76) : (sec2 ? tr_lt70 : tr_lt62));
    wire [4:0]  cd_code  = f_wait ? {3'd0, s_k} : 5'd4 + s_k;

    // 右栏第 0 行：标题 "检测结果"(4,5,7,8)，占行内第 8~23 行
    wire        ttl_hit  = pk_rows && (pk_row == 2'd0) && (pk_cr >= 5'd8) && (pk_cr < 5'd24) && (trel < 11'd56);
    wire [3:0]  ttl_row  = pk_cr[3:0] - 4'd8;
    wire [4:0]  ttl_code = (s_k == 2'd0) ? 5'd4 : (s_k == 2'd1) ? 5'd5 : (s_k == 2'd2) ? 5'd7 : 5'd8;

    wire        res_row1 = pk_rows && (pk_row == 2'd1);
    wire [1:0]  r_k      = (trel < 11'd48) ? 2'd0 : (trel < 11'd68) ? 2'd1 : 2'd2;
    wire        res_dig  = res_row1 && (trel >= 11'd28) && (trel < 11'd88) &&
                           ((r_k == 2'd0) ? (f_res && s100) : (r_k == 2'd1) ? (!f_res || s2d) : 1'b1);
    wire [4:0]  res_col  = trel[4:0] - ((r_k == 2'd0) ? 5'd28 : (r_k == 2'd1) ? 5'd16 : 5'd4);
    wire [3:0]  r_dig    = (r_k == 2'd0) ? 4'd1 : (r_k == 2'd1) ? s_ten : s_one;
    wire        res_fen  = res_row1 && f_res && (trel >= 11'd88) && (trel < 11'd120);
    wire [4:0]  fen_col  = trel[4:0] - 5'd24;

    wire        res_grd  = res_row1 && f_res && (trel >= 11'd128) && (grade3 ? (trel < 11'd224) : (trel < 11'd192));
    wire [2:0]  g_code   = (trel[6:5] == 2'd0) ? (grade3 ? 3'd7 : {grade, 1'b1}) :
                           (trel[6:5] == 2'd1) ? (grade3 ? 3'd5 : {grade + 2'd1, 1'b0}) : 3'd6;

    wire        det_rows = pk_rows && (pk_row == 2'd2) && (pk_cr >= 5'd8) && (pk_cr < 5'd24) && f_res;
    wire        det_cjk  = det_rows && (trel < 11'd112);
    wire        det_num  = det_rows && (trel >= 11'd120) && (t_two ? (trel < 11'd160) : (trel < 11'd152));
    wire [2:0]  d_m      = trel[5:3] - (t_two ? 3'd7 : 3'd6);
    wire [4:0]  dn_code  = (d_m == 3'd0) ? C_DIGIT + f_tilt[11:8] : (d_m == 3'd1) ? C_DIGIT + f_tilt[7:4] :
                           (d_m == 3'd2) ? C_DOT : (d_m == 3'd3) ? C_DIGIT + f_tilt[3:0] : C_DEG;
    wire [4:0]  dc_code  = (trel[6:4] == 3'd0) ? {3'b101, f_tilt[13:12]} : 5'd23 + trel[6:4];
    wire        det_hit  = det_cjk || det_num;

    wire        cd_hit   = cd_txt || cd_num;
    wire        res_ink  = f_res && (res_dig || res_fen || res_grd);   // 用等级色
    wire        tst_hit  = cd_hit || ttl_hit || det_hit || res_fen || res_grd;
    reg  [12:0] tst_addr;
    reg         tst_nib;

    always @* begin
        if (cd_txt || ttl_hit) begin
            tst_addr = {1'b0, (cd_txt ? cd_code : ttl_code), (cd_txt ? cd_row : ttl_row), s_col[3:1]};
            tst_nib  = s_col[0];
        end else if (cd_num) begin
            tst_addr = {1'b0, n_code, cd_row, 1'b0, n_col[2:1]};
            tst_nib  = n_col[0];
        end else if (det_hit) begin
            tst_addr = {1'b0, (det_num ? dn_code : dc_code), ttl_row, (!det_num && trel[3]), trel[2:1]};
            tst_nib  = trel[0];
        end else if (res_fen) begin
            tst_addr = {1'b1, 3'd0, pk_cr, fen_col[4:1]};
            tst_nib  = fen_col[0];
        end else begin
            tst_addr = {1'b1, g_code, pk_cr, trel[4:1]};
            tst_nib  = trel[0];
        end
    end

    reg [15:0] field_bcd;
    reg        field_neg;
    reg        field_hit;
    reg        field_peak;
    reg        field_ltr;
    reg [6:0]  rel_x;
    reg [4:0]  glyph, sign_glyph;
    reg [4:0]  glyph_col;

    always @* begin
        field_hit  = pk_rows;
        field_peak = 1'b0;
        field_ltr  = 1'b0;
        field_bcd  = dig_c;
        field_neg  = neg_c;
        rel_x      = 7'd0;
        if (res_dig)
            rel_x = 7'd0;                   // 右栏分数，字形与列号见下
        else if (ltr_x) begin
            rel_x     = x_next[6:0] - LTR_X_V[6:0];
            field_ltr = 1'b1;
        end else if (cur_x)
            rel_x = x_next[6:0] - CUR_X_V[6:0];
        else if (pkv_x) begin
            rel_x      = x_next[6:0] - PKV_X_V[6:0];
            field_peak = 1'b1;
            field_bcd  = dig_p;
            field_neg  = neg_p;
        end else
            field_hit = 1'b0;

        // 0.0 不显示负号
        sign_glyph = (field_neg && field_bcd != 16'd0) ? G_MINUS : G_PLUS;

        if (res_dig) begin
            glyph_col = res_col;
            glyph = f_res ? {1'b0, r_dig} : G_MINUS;
        end else if (field_ltr) begin
            glyph_col = (pk_row == 2'd2) ? rel_x[4:0] + 5'd12 : rel_x[4:0];
            glyph = (pk_row == 2'd0) ? G_LX : (pk_row == 2'd1) ? G_LY : G_LZ;
        end else if (rel_x < 7'd20) begin
            glyph_col = rel_x[4:0];
            glyph = (field_bcd[15:12] != 4'd0) ? sign_glyph : G_BLANK;
        end else if (rel_x < 7'd40) begin
            glyph_col = rel_x - 7'd20;
            if (field_bcd[15:12] != 4'd0)
                glyph = {1'b0, field_bcd[15:12]};
            else
                glyph = (field_bcd[11:8] != 4'd0) ? sign_glyph : G_BLANK;
        end else if (rel_x < 7'd60) begin
            glyph_col = rel_x - 7'd40;
            if (field_bcd[15:8] != 8'd0)
                glyph = {1'b0, field_bcd[11:8]};
            else
                glyph = sign_glyph;
        end else if (rel_x < 7'd80) begin
            glyph_col = rel_x - 7'd60;
            glyph = {1'b0, field_bcd[7:4]};
        end else if (rel_x < 7'd89) begin
            glyph_col = rel_x - 7'd80;
            glyph = G_DOT;
        end else if (rel_x < 7'd109) begin
            glyph_col = rel_x - 7'd89;
            glyph = {1'b0, field_bcd[3:0]};
        end else begin
            glyph_col = rel_x - 7'd109;
            glyph = G_DEG;
        end
    end

    wire [3:0]  small_alpha, big_alpha, tst_alpha;
    wire [3:0]  text_alpha = small_alpha | big_alpha | tst_alpha;
    wire [23:0] text_color;
    reg  [2:0]  text_fg_q;
    reg         tst_hit_q, tst_nib_q;
    wire [7:0]  tst_q;

    always @(posedge pix_clk) begin
        text_fg_q <= cd_hit     ? (f_wait ? 3'd4 : 3'd5) :
                     res_ink    ? 3'd3 :
                     (ttl_hit || det_hit) ? 3'd2 :
                     !field_hit ? 3'd0 : (field_peak || res_dig) ? 3'd2 : 3'd1;
        tst_hit_q <= de && tst_hit;
        tst_nib_q <= tst_nib;
    end

    assign tst_alpha = !tst_hit_q ? 4'd0 : tst_nib_q ? tst_q[3:0] : tst_q[7:4];

    test_font_rom u_test_font (
        .clk   (pix_clk),
        .addr  (tst_addr),
        .q     (tst_q)
    );

    osd_font_small_rom u_font (
        .clk   (pix_clk),
        .en    (de && !text_glyph[4]),
        .addr  ({text_glyph[3:0], text_rel_y[3:0], text_rel_x[2:0]}),
        .alpha (small_alpha)
    );

    osd_font_big_rom u_font_big (
        .clk   (pix_clk),
        .en    (de && field_hit && !glyph[4]),
        .addr  ({glyph[3:0], pk_cr, glyph_col}),
        .alpha (big_alpha)
    );

    wire        pk_band  = pk_rows && (pk_cr >= 5'd4) && (pk_cr < 5'd28);
    wire        on_div   = pk_band && (x_cnt == DIV0_X || x_cnt == DIV1_X);
    wire [10:0] tri_dx   = x_cnt - TRI_X;
    wire [4:0]  tri_r    = pk_cr - 5'd8;
    wire [3:0]  tri_d    = (tri_dx[3:0] >= 4'd5) ? tri_dx[3:0] - 4'd5 : 4'd5 - tri_dx[3:0];
    wire        on_tri   = pk_rows && (pk_cr >= 5'd8) && (pk_cr < 5'd20) &&
                           tri_x && (tri_d <= {1'b0, tri_r[3:1]});
    wire [23:0] c_axis   = (pk_row == 2'd0) ? COLOR_X : (pk_row == 2'd1) ? COLOR_Y : COLOR_Z;
    reg  [23:0] c_fg;
    always @* begin
        case (text_fg_q)
            3'd1:    c_fg = c_axis;
            3'd2:    c_fg = COLOR_PEAK;
            3'd3:    c_fg = grade_ink;
            3'd4:    c_fg = COLOR_WAIT;
            3'd5:    c_fg = COLOR_MEAS;
            default: c_fg = COLOR_TEXT;
        endcase
    end

    // 图层由下到上：背景/网格/分栏线 -> 文字 -> 曲线
    reg [23:0] base_color;
    always @* begin
        if (in_plot && on_axis)      base_color = COLOR_AXIS;
        else if (in_plot && on_grid) base_color = COLOR_GRID;
        else if (on_tri)             base_color = c_axis;
        else if (on_div)             base_color = COLOR_DIV;
        else                         base_color = rgb_in;
    end

    osd_alpha_blend #(
        .USE_DSP (1)
    ) u_blend (
        .bg    (base_color),
        .fg    (c_fg),
        .alpha (text_alpha),
        .out   (text_color)
    );

    reg [23:0] pixel_color;
    always @* begin
        if (in_plot && hit_x)      pixel_color = COLOR_X;
        else if (in_plot && hit_y) pixel_color = COLOR_Y;
        else if (in_plot && hit_z) pixel_color = COLOR_Z;
        else                       pixel_color = text_color;
    end

    assign rgb_out = (enable_frame && de) ? pixel_color : rgb_in;
endmodule
