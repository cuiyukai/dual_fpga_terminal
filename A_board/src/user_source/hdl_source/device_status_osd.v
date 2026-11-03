`timescale 1ns/1ps

module device_status_osd #(
    parameter integer GYRO_X0  = 104,         // "陀螺仪：" 冒号止于 94，留 10 像素间隔
    parameter integer CAM_X0   = 262,         // "摄像头：" 冒号止于 252，与陀螺仪间距相同（+158）
    parameter integer TEXT_Y0  = 449,         // 字形单元顶行，与标签同一基线（标签墨迹 y 450~462）
    parameter integer GYRO2_Y0 = 441,
    parameter [23:0]  OK_INK   = 24'h36D67A,
    parameter [23:0]  FAIL_INK = 24'hFF4D4F
)(
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        show_async,            // 当前为第一 / 二页且转场结束
    input  wire        gyro_only_async,       // 当前为第二页：只显示陀螺仪
    input  wire        gyro_ok_async,         // 陀螺仪就绪（clk 域电平）
    input  wire        cam_ok_async,          // 摄像头自检通过（clk 域电平）
    input  wire        vsync,
    input  wire        de,
    input  wire [23:0] rgb_in,
    output wire        vsync_out,
    output wire        de_out,
    output wire [23:0] rgb_out
);
    localparam integer LATENCY = 3;
    localparam integer CHAR_W  = 14;          // 与 status_font_rom 的单元列数一致
    localparam integer CELL_H  = 16;

    // ---------------- 状态同步与帧边界锁存 ----------------
    (* ASYNC_REG = "TRUE" *) reg [3:0] st_meta, st_sync;   // {gyro_only, show, gyro_ok, cam_ok}
    reg       frame_show, frame_gyro, frame_cam, frame_gyro_only;
    reg [9:0] frame_y0;
    reg       vs_q;
    wire      frame_start = vs_q && !vsync;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            st_meta    <= 4'b0000;
            st_sync    <= 4'b0000;
            frame_show <= 1'b0;
            frame_gyro <= 1'b0;
            frame_cam  <= 1'b0;
            frame_gyro_only <= 1'b0;
            frame_y0   <= TEXT_Y0;
            vs_q       <= 1'b1;
        end else begin
            st_meta <= {gyro_only_async, show_async, gyro_ok_async, cam_ok_async};
            st_sync <= st_meta;
            vs_q    <= vsync;
            if (frame_start) begin
                frame_show <= st_sync[2];
                frame_gyro <= st_sync[1];
                frame_cam  <= st_sync[0];
                frame_gyro_only <= st_sync[3];
                frame_y0   <= st_sync[3] ? GYRO2_Y0 : TEXT_Y0;
            end
        end
    end

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

    reg        in_rows;
    reg  [3:0] text_row;
    reg        in_gyro, in_cam;
    reg  [1:0] char_idx;
    reg  [3:0] char_col;
    wire       in_box   = de && frame_show && in_rows && (in_gyro || (in_cam && !frame_gyro_only));
    wire       ok       = in_gyro ? frame_gyro : frame_cam;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            in_rows  <= 1'b0;
            text_row <= 4'd0;
            in_gyro  <= 1'b0;
            in_cam   <= 1'b0;
            char_idx <= 2'd0;
            char_col <= 4'd0;
        end else begin
            if (!vsync) begin
                in_rows  <= 1'b0;
                text_row <= 4'd0;
            end else if (de_q && !de) begin
                if (pixel_y == frame_y0 - 10'd1) begin
                    in_rows  <= 1'b1;
                    text_row <= 4'd0;
                end else if (in_rows) begin
                    text_row <= text_row + 1'b1;
                    if (text_row == CELL_H - 1)
                        in_rows <= 1'b0;
                end
            end
            if (!de) begin
                in_gyro  <= 1'b0;
                in_cam   <= 1'b0;
                char_idx <= 2'd0;
                char_col <= 4'd0;
            end else begin
                if (pixel_x == GYRO_X0 - 1)
                    in_gyro <= 1'b1;
                else if (pixel_x == GYRO_X0 + 3 * CHAR_W - 1)
                    in_gyro <= 1'b0;
                if (pixel_x == CAM_X0 - 1)
                    in_cam <= 1'b1;
                else if (pixel_x == CAM_X0 + 3 * CHAR_W - 1)
                    in_cam <= 1'b0;
                if (pixel_x == GYRO_X0 - 1 || pixel_x == CAM_X0 - 1) begin
                    char_idx <= 2'd0;
                    char_col <= 4'd0;
                end else if (char_col == CHAR_W - 1) begin
                    char_idx <= char_idx + 1'b1;
                    char_col <= 4'd0;
                end else begin
                    char_col <= char_col + 1'b1;
                end
            end
        end
    end

    // 编码 0~2 为 "已就绪"，3~5 为 "未通过"
    wire [2:0] code = (ok ? 3'd0 : 3'd3) + {1'b0, char_idx};

    wire [CHAR_W*4-1:0] row_bits;
    status_font_rom u_font (
        .clk      (pix_clk),
        .addr     ({code, text_row}),
        .row_bits (row_bits)
    );

    // ---------------- 第 1~3 拍：取列、混合 ----------------
    reg [23:0] rgb_d1, rgb_d2, rgb_d3;
    reg [3:0]  col_d1;
    reg        hit_d1, ok_d1;
    reg [3:0]  alpha_d2;
    reg [23:0] ink_d2;
    reg [LATENCY-1:0] de_pipe, vs_pipe;

    wire [23:0] blended;
    osd_alpha_blend #(
        .USE_DSP (1)
    ) u_blend (
        .bg    (rgb_d2),
        .fg    (ink_d2),
        .alpha (alpha_d2),
        .out   (blended)
    );

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            rgb_d1   <= 24'd0;
            rgb_d2   <= 24'd0;
            rgb_d3   <= 24'd0;
            col_d1   <= 4'd0;
            hit_d1   <= 1'b0;
            ok_d1    <= 1'b0;
            alpha_d2 <= 4'd0;
            ink_d2   <= 24'd0;
            de_pipe  <= {LATENCY{1'b0}};
            vs_pipe  <= {LATENCY{1'b1}};
        end else begin
            de_pipe <= {de_pipe[LATENCY-2:0], de};
            vs_pipe <= {vs_pipe[LATENCY-2:0], vsync};

            rgb_d1 <= rgb_in;
            col_d1 <= char_col;
            hit_d1 <= in_box;
            ok_d1  <= ok;

            rgb_d2   <= rgb_d1;
            alpha_d2 <= hit_d1 ? row_bits[col_d1 * 4 +: 4] : 4'd0;
            ink_d2   <= ok_d1 ? OK_INK : FAIL_INK;

            rgb_d3 <= (alpha_d2 != 4'd0) ? blended : rgb_d2;
        end
    end

    assign rgb_out   = rgb_d3;
    assign de_out    = de_pipe[LATENCY-1];
    assign vsync_out = vs_pipe[LATENCY-1];
endmodule
