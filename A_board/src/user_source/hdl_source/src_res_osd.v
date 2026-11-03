`timescale 1ns/1ps

module src_res_osd #(
    parameter integer TX = 10,          // 文字左沿
    parameter integer TY = 8            // 文字顶行（16 行高）
)(
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        show_async,
    input  wire [10:0] w_async,
    input  wire [10:0] h_async,
    input  wire        vsync,
    input  wire        de,
    input  wire [23:0] rgb_in,
    output wire        vsync_out,
    output wire        de_out,
    output wire [23:0] rgb_out
);
    localparam integer LATENCY = 2;
    localparam [23:0]  YELLOW  = 24'hFFD21E;
    localparam [3:0]   CH_X    = 4'd10;
    localparam [3:0]   CH_NONE = 4'd15;

    // ---------------- 同步与帧级参数 ----------------
    (* ASYNC_REG = "TRUE" *) reg [22:0] s_meta, s_sync;   // {show, w, h}
    reg [22:0] s_last;
    reg        vs_q;
    reg        show_f;
    reg [10:0] w_f, h_f;
    wire       frame_start = vs_q && !vsync;

    // 移位加 3：11 位二进制 -> 4 位 BCD
    reg [10:0] bw, bh;
    reg [15:0] dw, dh;
    reg [3:0]  bcnt;
    reg        bbusy;
    function [15:0] add3;
        input [15:0] d;
        integer k;
        begin
            add3 = d;
            for (k = 0; k < 4; k = k + 1)
                if (add3[k*4 +: 4] >= 4'd5) add3[k*4 +: 4] = add3[k*4 +: 4] + 4'd3;
        end
    endfunction
    wire [15:0] dw_a = add3(dw);
    wire [15:0] dh_a = add3(dh);

    // 字符串：W 的有效位 + 'x' + H 的有效位
    reg  [3:0] c0, c1, c2, c3, c4, c5, c6, c7, c8;
    reg  [3:0] nch;                     // 字符个数

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            s_meta <= 23'd0; s_sync <= 23'd0; s_last <= 23'd0; vs_q <= 1'b1;
            show_f <= 1'b0; w_f <= 11'd0; h_f <= 11'd0;
            bw <= 11'd0; bh <= 11'd0; dw <= 16'd0; dh <= 16'd0; bcnt <= 4'd0; bbusy <= 1'b0;
            {c0, c1, c2, c3, c4, c5, c6, c7, c8} <= {9{CH_NONE}};
            nch <= 4'd0;
        end else begin
            s_meta <= {show_async, w_async, h_async};
            s_sync <= s_meta;
            s_last <= s_sync;
            vs_q   <= vsync;
            if (frame_start) begin
                if (s_sync == s_last) begin
                    show_f <= s_sync[22];
                    w_f    <= s_sync[21:11];
                    h_f    <= s_sync[10:0];
                    bw     <= s_sync[21:11];
                    bh     <= s_sync[10:0];
                end else begin
                    bw <= w_f;
                    bh <= h_f;
                end
                dw <= 16'd0; dh <= 16'd0; bcnt <= 4'd0; bbusy <= 1'b1;
            end else if (bbusy) begin
                dw <= {dw_a[14:0], bw[10]};
                dh <= {dh_a[14:0], bh[10]};
                bw <= {bw[9:0], 1'b0};
                bh <= {bh[9:0], 1'b0};
                bcnt <= bcnt + 4'd1;
                if (bcnt == 4'd10) bbusy <= 1'b0;
            end else if (bcnt == 4'd11) begin
                // 组字符串（场消隐内完成，显示期间不变）
                bcnt <= 4'd12;
                if (dw[15:12] != 4'd0) begin
                    {c0, c1, c2, c3, c4} <= {dw[15:12], dw[11:8], dw[7:4], dw[3:0], CH_X};
                    if (dh[15:12] != 4'd0) begin
                        {c5, c6, c7, c8} <= {dh[15:12], dh[11:8], dh[7:4], dh[3:0]}; nch <= 4'd9;
                    end else begin
                        {c5, c6, c7, c8} <= {dh[11:8], dh[7:4], dh[3:0], CH_NONE};   nch <= 4'd8;
                    end
                end else begin
                    {c0, c1, c2, c3} <= {dw[11:8], dw[7:4], dw[3:0], CH_X};
                    if (dh[15:12] != 4'd0) begin
                        {c4, c5, c6, c7, c8} <= {dh[15:12], dh[11:8], dh[7:4], dh[3:0], CH_NONE}; nch <= 4'd8;
                    end else begin
                        {c4, c5, c6, c7, c8} <= {dh[11:8], dh[7:4], dh[3:0], CH_NONE, CH_NONE};   nch <= 4'd7;
                    end
                end
            end
        end
    end

    // ---------------- 像素坐标与窗口 ----------------
    reg [9:0] px, py;
    reg       de_q;
    reg       ty_on, bx_on, by_on;     // 文字行 / 底框列 / 底框行
    reg [3:0] trow;                    // 文字行内行号
    reg       tx_on;
    reg [2:0] tcol;                    // 字内列 0~7
    reg [3:0] tci;                     // 字符序号
    wire [9:0] box_x1 = TX + {nch, 3'd0} + 10'd3;   // 底框右沿（含 3 像素边）

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            px <= 10'd0; py <= 10'd0; de_q <= 1'b0;
            ty_on <= 1'b0; bx_on <= 1'b0; by_on <= 1'b0; trow <= 4'd0;
            tx_on <= 1'b0; tcol <= 3'd0; tci <= 4'd0;
        end else begin
            de_q <= de;
            px   <= de ? px + 10'd1 : 10'd0;
            if (!de) begin
                tx_on <= 1'b0; bx_on <= 1'b0;
            end else begin
                if (px == TX - 4) bx_on <= 1'b1;
                else if (px == box_x1) bx_on <= 1'b0;
                if (px == TX - 1) begin
                    tx_on <= 1'b1; tcol <= 3'd0; tci <= 4'd0;
                end else if (tx_on) begin
                    tcol <= tcol + 3'd1;
                    if (tcol == 3'd7) begin
                        tci <= tci + 4'd1;
                        if (tci == nch - 4'd1) tx_on <= 1'b0;
                    end
                end
            end
            if (!vsync) begin
                py <= 10'd0; ty_on <= 1'b0; by_on <= 1'b0;
            end else if (de_q && !de) begin
                py <= py + 10'd1;
                if (py == TY - 3) by_on <= 1'b1;
                else if (py == TY + 17) by_on <= 1'b0;
                if (py == TY - 1) begin
                    ty_on <= 1'b1; trow <= 4'd0;
                end else if (ty_on) begin
                    trow <= trow + 4'd1;
                    if (trow == 4'd15) ty_on <= 1'b0;
                end
            end
        end
    end

    // ---------------- 第 0 拍：取字形 ----------------
    reg [3:0] code;
    always @* begin
        case (tci)
            4'd0: code = c0; 4'd1: code = c1; 4'd2: code = c2; 4'd3: code = c3; 4'd4: code = c4;
            4'd5: code = c5; 4'd6: code = c6; 4'd7: code = c7; default: code = c8;
        endcase
    end
    wire [15:0] font_q;
    res_font_rom u_font (.clk(pix_clk), .addr({code, trow}), .q(font_q));

    // ---------------- 第 1 拍：混色 ----------------
    reg [23:0] rgb_d1, rgb_d2;
    reg        txt_d1, box_d1;
    reg [2:0]  col_d1;
    reg [LATENCY-1:0] de_pipe, vs_pipe;
    wire [1:0]  a    = txt_d1 ? font_q[col_d1*2 +: 2] : 2'd0;
    wire [23:0] bgd  = box_d1 ? {1'b0, rgb_d1[23:17], 1'b0, rgb_d1[15:9], 1'b0, rgb_d1[7:1]} : rgb_d1;   // 底框压暗一半
    wire [23:0] half = {1'b0, bgd[23:17], 1'b0, bgd[15:9], 1'b0, bgd[7:1]};
    wire [23:0] y2   = {1'b0, YELLOW[23:17], 1'b0, YELLOW[15:9], 1'b0, YELLOW[7:1]};
    wire [23:0] mix  = (a == 2'd3) ? YELLOW :
                       (a != 2'd0) ? (half + y2) :              // 边缘 1/2 混合
                                     bgd;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            rgb_d1 <= 24'd0; rgb_d2 <= 24'd0; txt_d1 <= 1'b0; box_d1 <= 1'b0; col_d1 <= 3'd0;
            de_pipe <= {LATENCY{1'b0}}; vs_pipe <= {LATENCY{1'b1}};
        end else begin
            de_pipe <= {de_pipe[LATENCY-2:0], de};
            vs_pipe <= {vs_pipe[LATENCY-2:0], vsync};
            rgb_d1  <= rgb_in;
            txt_d1  <= show_f && de && tx_on && ty_on;
            box_d1  <= show_f && de && bx_on && by_on;
            col_d1  <= tcol;
            rgb_d2  <= mix;
        end
    end

    assign rgb_out   = rgb_d2;
    assign de_out    = de_pipe[LATENCY-1];
    assign vsync_out = vs_pipe[LATENCY-1];
endmodule
