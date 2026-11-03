`timescale 1ns/1ps

module audio_wave_osd #(
    parameter integer PANEL_X0      = 27,   // 面板左上角（屏幕坐标）
    parameter integer PANEL_Y0      = 305,
    parameter integer PANEL_W       = 588,
    parameter integer PANEL_H       = 104,
    parameter integer WAVE_X0       = 39,   // 波形区左边界
    parameter integer SAMPLE_COUNT  = 128,  // 每帧采样点数，必须为 2 的幂且 <= 128
    parameter integer SAMPLE_PX     = 4,    // 每个采样点占用的像素列数，必须为 2 的幂
    parameter integer DECIMATE      = 6,
    parameter integer SAMPLE_SHIFT  = 15,   // 24 位采样右移后饱和到 ±127（相对高 8 位放大 2 倍）
    parameter integer PEAK_X0       = 566,  // 峰值电平条左边界
    parameter integer PEAK_W        = 16,
    parameter integer VBLANK_CYCLES = 1024  // de 连续低电平超过该值视为场消隐（需大于行消隐）
)(
    input  wire               pix_clk,
    input  wire               pix_rst_n,
    input  wire               de,
    input  wire               display_enable, // pix_clk 域电平，在帧起始处锁存
    input  wire               audio_valid,    // 每个立体声采样一拍
    input  wire signed [23:0] audio_left,
    input  wire signed [23:0] audio_right,
    input  wire [23:0]        rgb_in,
    output wire [23:0]        rgb_out
);
    localparam integer WAVE_W    = SAMPLE_COUNT * SAMPLE_PX;
    localparam integer HALF_H    = 40;                 // 波形区上下各 40 行，振幅最大 ±32 行
    localparam integer WAVE_H    = HALF_H * 2 + 1;
    localparam integer WAVE_Y0   = PANEL_Y0 + PANEL_H / 2 - HALF_H;
    localparam integer IDX_W     = $clog2(SAMPLE_COUNT);
    localparam integer PX_SHIFT  = $clog2(SAMPLE_PX);
    localparam [23:0]  COLOR_WAVE   = 24'h20FF90;
    localparam [23:0]  COLOR_PEAK   = 24'hFFB020;
    localparam [23:0]  COLOR_AXIS   = 24'h406060;
    localparam [23:0]  COLOR_BORDER = 24'h3A5070;

    wire signed [24:0] mix     = audio_left + audio_right;           // 25 位防溢出
    wire signed [24:0] mix_sh  = mix >>> (SAMPLE_SHIFT + 1);         // 多移 1 位即除以 2
    wire signed [7:0]  smp8    = (mix_sh >  25'sd127) ?  8'sd127 :
                                 (mix_sh < -25'sd127) ? -8'sd127 : mix_sh[7:0];
    wire        [6:0]  smp_abs = smp8[7] ? -smp8[6:0] : smp8[6:0];
    // 正采样向上：行号 = HALF_H - smp8/4，范围 8..72
    wire        [6:0]  smp_row = HALF_H - (smp8 >>> 2);

    // ---------------- 行场计数与帧切换 ----------------
    reg [10:0] x_cnt;        // de 有效期间等于当前像素列
    reg [9:0]  y_cnt;
    reg        de_q;
    reg [10:0] blank_cnt;
    reg        enable_frame;

    reg             wr_bank;       // 正在写入的 bank，显示另一块
    reg [IDX_W:0]   cap_cnt;
    reg [2:0]       dec_cnt;
    reg [6:0]       peak_cap;
    reg [6:0]       peak_disp;
    reg             disp_valid;

    wire frame_start = !de && (blank_cnt == VBLANK_CYCLES - 1);
    wire cap_full    = (cap_cnt == SAMPLE_COUNT);
    wire cap_we      = audio_valid && !cap_full && (dec_cnt == 3'd0) && !frame_start;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            x_cnt        <= 11'd0;
            y_cnt        <= 10'd0;
            de_q         <= 1'b0;
            blank_cnt    <= 11'd0;
            enable_frame <= 1'b0;
            wr_bank      <= 1'b0;
            cap_cnt      <= {(IDX_W+1){1'b0}};
            dec_cnt      <= 3'd0;
            peak_cap     <= 7'd0;
            peak_disp    <= 7'd0;
            disp_valid   <= 1'b0;
        end else begin
            de_q  <= de;
            x_cnt <= de ? x_cnt + 11'd1 : 11'd0;

            if (de)
                blank_cnt <= 11'd0;
            else if (blank_cnt != VBLANK_CYCLES)
                blank_cnt <= blank_cnt + 11'd1;

            if (frame_start) begin
                y_cnt        <= 10'd0;
                enable_frame <= display_enable;
                // 采满才切换显示 bank；否则（音频关闭）只显示中心轴
                disp_valid   <= cap_full;
                if (cap_full) begin
                    wr_bank   <= ~wr_bank;
                    peak_disp <= peak_cap;
                end else begin
                    peak_disp <= 7'd0;
                end
                cap_cnt  <= {(IDX_W+1){1'b0}};
                dec_cnt  <= 3'd0;
                peak_cap <= 7'd0;
            end else begin
                if (de_q && !de)
                    y_cnt <= y_cnt + 10'd1;

                if (audio_valid && !cap_full) begin
                    dec_cnt <= (dec_cnt == 3'd0) ? DECIMATE - 1 : dec_cnt - 3'd1;
                    if (dec_cnt == 3'd0) begin
                        cap_cnt <= cap_cnt + 1'b1;
                        if (smp_abs > peak_cap)
                            peak_cap <= smp_abs;
                    end
                end
            end
        end
    end

    reg  [6:0] wave_ram [0:2*SAMPLE_COUNT-1];
    reg  [6:0] rd_row, rd_prev;
    wire [10:0] x_next   = de ? x_cnt + 11'd1 : 11'd0;
    wire [10:0] col_next = x_next - WAVE_X0;
    wire [IDX_W-1:0] rd_idx = col_next[PX_SHIFT +: IDX_W];

    always @(posedge pix_clk) begin
        if (cap_we)
            wave_ram[{wr_bank, cap_cnt[IDX_W-1:0]}] <= smp_row;
        rd_row  <= wave_ram[{~wr_bank, rd_idx}];
        rd_prev <= rd_row;   // 与 rd_row 对齐时恰为上一列
    end

    // ---------------- 像素着色 ----------------
    wire in_panel = (x_cnt >= PANEL_X0) && (x_cnt < PANEL_X0 + PANEL_W) &&
                    (y_cnt >= PANEL_Y0) && (y_cnt < PANEL_Y0 + PANEL_H);
    wire on_border = in_panel &&
                     ((x_cnt == PANEL_X0) || (x_cnt == PANEL_X0 + PANEL_W - 1) ||
                      (y_cnt == PANEL_Y0) || (y_cnt == PANEL_Y0 + PANEL_H - 1));
    wire in_wave_y = (y_cnt >= WAVE_Y0) && (y_cnt < WAVE_Y0 + WAVE_H);
    wire in_wave   = in_wave_y && (x_cnt >= WAVE_X0) && (x_cnt < WAVE_X0 + WAVE_W);
    wire [9:0]  row_full  = y_cnt - WAVE_Y0;
    wire [6:0]  wave_row  = row_full[6:0];
    wire [10:0] wave_col  = x_cnt - WAVE_X0;
    wire        col_first = (wave_col[PX_SHIFT-1:0] == 0);
    wire [6:0]  prev_sel  = (col_first && wave_col != 0) ? rd_prev : rd_row;
    wire [6:0]  seg_lo    = (rd_row < prev_sel) ? rd_row : prev_sel;
    wire [6:0]  seg_hi    = (rd_row < prev_sel) ? prev_sel : rd_row;
    wire hit_wave = in_wave && disp_valid &&
                    ({1'b0, wave_row} + 8'd1 >= {1'b0, seg_lo}) &&
                    (wave_row <= seg_hi + 7'd1);
    wire on_axis  = in_wave && (wave_row == HALF_H);
    // 峰值条以中心轴对称，半高与波形振幅同比例（peak/4）
    wire [6:0]  peak_half = {2'b00, peak_disp[6:2]};
    wire hit_peak = in_wave_y && (x_cnt >= PEAK_X0) && (x_cnt < PEAK_X0 + PEAK_W) &&
                    (peak_half != 0) &&
                    (wave_row + peak_half >= HALF_H) && (wave_row <= HALF_H + peak_half);

    reg [23:0] pixel_color;
    always @* begin
        if (hit_wave)       pixel_color = COLOR_WAVE;
        else if (hit_peak)  pixel_color = COLOR_PEAK;
        else if (on_border) pixel_color = COLOR_BORDER;
        else if (on_axis)   pixel_color = COLOR_AXIS;
        else                pixel_color = {1'b0, rgb_in[23:17], 1'b0, rgb_in[15:9], 1'b0, rgb_in[7:1]}; // 背景亮度减半
    end

    assign rgb_out = (enable_frame && de && in_panel) ? pixel_color : rgb_in;
endmodule
