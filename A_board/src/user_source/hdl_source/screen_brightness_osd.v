`timescale 1ns/1ps

module screen_brightness_osd #(
    parameter integer BRIGHT_X0 = 560,
    parameter integer VOLUME_X0 = 464,
    parameter integer TEXT_Y0   = 30,         // 字形单元顶行，数字笔画（31~41）与 "%"（31~41）同高
    parameter [23:0]  INK       = 24'hF2FFFF
)(
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire [6:0]  pct_async,             // 亮度 0~100，clk 域寄存器输出
    input  wire [6:0]  vol_pct_async,         // 音量 0~100，只用于显示数值
    input  wire        show_async,            // 当前为第一页时显示数值
    input  wire        vsync,
    input  wire        de,
    input  wire [23:0] rgb_in,
    output wire        vsync_out,
    output wire        de_out,
    output wire [23:0] rgb_out
);
    localparam integer LATENCY = 4;
    localparam integer DIGIT_W = 9;           // 与 brightness_font_rom 的单元列数一致
    localparam integer CELL_H  = 14;

    // ---------------- 参数同步与帧边界锁存 ----------------
    (* ASYNC_REG = "TRUE" *) reg show_meta, show_sync;
    reg       frame_show;
    reg       vs_q;
    wire      frame_start = vs_q && !vsync;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            show_meta  <= 1'b0;
            show_sync  <= 1'b0;
            frame_show <= 1'b0;
            vs_q       <= 1'b1;
        end else begin
            show_meta <= show_async;
            show_sync <= show_meta;
            vs_q      <= vsync;
            if (frame_start)
                frame_show <= show_sync;
        end
    end

    wire       bri_hundred, vol_hundred;
    wire [3:0] bri_tens, bri_ones, vol_tens, vol_ones;
    wire [8:0] frame_gain;

    pct_frame_digits u_bright_digits (
        .clk           (pix_clk),
        .rst_n         (pix_rst_n),
        .pct_async     (pct_async),
        .frame_start   (frame_start),
        .frame_hundred (bri_hundred),
        .frame_tens    (bri_tens),
        .frame_ones    (bri_ones),
        .frame_gain    (frame_gain)
    );

    pct_frame_digits u_volume_digits (
        .clk           (pix_clk),
        .rst_n         (pix_rst_n),
        .pct_async     (vol_pct_async),
        .frame_start   (frame_start),
        .frame_hundred (vol_hundred),
        .frame_tens    (vol_tens),
        .frame_ones    (vol_ones),
        .frame_gain    ()
    );

    // ---------------- 像素坐标 ----------------
    reg [9:0] pixel_x, pixel_y;   // 当前输入像素的列/行号
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

    // ---------------- 第 0 拍：选字、取址 ----------------
    // 数字右对齐：百位只在 100 时显示，十位在 >=10 时显示
    wire       in_rows  = (pixel_y >= TEXT_Y0) && (pixel_y < TEXT_Y0 + CELL_H);
    wire       in_vol   = (pixel_x >= VOLUME_X0) && (pixel_x < VOLUME_X0 + 3 * DIGIT_W);
    wire       in_bri   = (pixel_x >= BRIGHT_X0) && (pixel_x < BRIGHT_X0 + 3 * DIGIT_W);
    wire       in_box   = de && frame_show && in_rows && (in_vol || in_bri);
    wire [9:0] text_row = pixel_y - TEXT_Y0;

    wire       num_hundred = in_vol ? vol_hundred : bri_hundred;
    wire [3:0] num_tens    = in_vol ? vol_tens    : bri_tens;
    wire [3:0] num_ones    = in_vol ? vol_ones    : bri_ones;

    reg [3:0] gcol;
    reg [1:0] gslot;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            gcol  <= 4'd0;
            gslot <= 2'd0;
        end else if (de) begin
            if (pixel_x == VOLUME_X0 - 1 || pixel_x == BRIGHT_X0 - 1) begin
                gcol  <= 4'd0;
                gslot <= 2'd0;
            end else if (gcol == DIGIT_W - 1) begin
                gcol  <= 4'd0;
                gslot <= gslot + 1'b1;
            end else begin
                gcol <= gcol + 1'b1;
            end
        end
    end

    reg [3:0] glyph;
    wire [3:0] glyph_col = gcol;
    reg       glyph_on;
    always @* begin
        if (gslot == 2'd0) begin
            glyph     = 4'd1;
            glyph_on  = num_hundred;
        end else if (gslot == 2'd1) begin
            glyph     = num_tens;
            glyph_on  = num_hundred || (num_tens != 4'd0);
        end else begin
            glyph     = num_ones;
            glyph_on  = 1'b1;
        end
    end

    wire [DIGIT_W*4-1:0] row_bits;
    brightness_font_rom u_font (
        .clk      (pix_clk),
        .addr     ({glyph, text_row[3:0]}),
        .row_bits (row_bits)
    );

    reg [23:0] rgb_d1, rgb_d2, rgb_d3, rgb_d4;
    reg [3:0]  col_d1;
    reg        hit_d1;
    reg [3:0]  alpha_d2;
    reg [LATENCY-1:0] de_pipe, vs_pipe;

    wire [23:0] blended;
    osd_alpha_blend #(
        .USE_DSP (1)
    ) u_blend (
        .bg    (rgb_d2),
        .fg    (INK),
        .alpha (alpha_d2),
        .out   (blended)
    );

    wire [16:0] mul_r = rgb_d3[23:16] * frame_gain;
    wire [16:0] mul_g = rgb_d3[15:8]  * frame_gain;
    wire [16:0] mul_b = rgb_d3[7:0]   * frame_gain;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            rgb_d1   <= 24'd0;
            rgb_d2   <= 24'd0;
            rgb_d3   <= 24'd0;
            rgb_d4   <= 24'd0;
            col_d1   <= 4'd0;
            hit_d1   <= 1'b0;
            alpha_d2 <= 4'd0;
            de_pipe  <= {LATENCY{1'b0}};
            vs_pipe  <= {LATENCY{1'b1}};
        end else begin
            de_pipe <= {de_pipe[LATENCY-2:0], de};
            vs_pipe <= {vs_pipe[LATENCY-2:0], vsync};

            rgb_d1 <= rgb_in;
            col_d1 <= glyph_col;
            hit_d1 <= in_box && glyph_on;

            rgb_d2   <= rgb_d1;
            alpha_d2 <= hit_d1 ? row_bits[col_d1 * 4 +: 4] : 4'd0;

            rgb_d3 <= (alpha_d2 != 4'd0) ? blended : rgb_d2;

            rgb_d4 <= frame_gain[8] ? rgb_d3 :
                      {mul_r[15:8], mul_g[15:8], mul_b[15:8]};
        end
    end

    assign rgb_out   = rgb_d4;
    assign de_out    = de_pipe[LATENCY-1];
    assign vsync_out = vs_pipe[LATENCY-1];
endmodule

module pct_frame_digits (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [6:0] pct_async,
    input  wire       frame_start,
    output reg        frame_hundred,
    output reg  [3:0] frame_tens,
    output reg  [3:0] frame_ones,
    output reg  [8:0] frame_gain
);
    wire [6:0] pct;
    wire [8:0] gain;
    pct_sync u_pct_sync (
        .clk       (clk),
        .rst_n     (rst_n),
        .pct_async (pct_async),
        .pct       (pct),
        .gain      (gain)
    );

    // 十位用 v*205>>11 近似 v/10，对 0~127 都精确
    wire [14:0] tens_mul = pct * 8'd205;
    wire [3:0]  tens_w   = tens_mul[14:11];
    reg  [3:0]  tens_calc, ones_calc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tens_calc     <= 4'd0;
            ones_calc     <= 4'd0;
            frame_hundred <= 1'b1;
            frame_tens    <= 4'd0;
            frame_ones    <= 4'd0;
            frame_gain    <= 9'd256;
        end else begin
            // pct 变化后两拍内算完，远早于下一帧起点
            tens_calc <= tens_w;
            ones_calc <= pct - tens_w * 4'd10;
            if (frame_start) begin
                frame_hundred <= (pct >= 7'd100);
                frame_tens    <= (pct >= 7'd100) ? 4'd0 : tens_calc;
                frame_ones    <= (pct >= 7'd100) ? 4'd0 : ones_calc;
                frame_gain    <= gain;
            end
        end
    end
endmodule
