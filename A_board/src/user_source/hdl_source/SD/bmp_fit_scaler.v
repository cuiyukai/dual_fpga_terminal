`timescale 1ns/1ps

module bmp_fit_scaler (
    input  wire        clk,
    input  wire        rst,
    input  wire        cfg_start,
    input  wire [10:0] cfg_w,
    input  wire [10:0] cfg_h,
    output wire        cfg_busy,
    output reg         cfg_ok,
    input  wire        abort,
    input  wire        run,            // 写通道已应答，开始输出
    input  wire        src_valid,
    input  wire [23:0] src_pix,
    // 输出到写 FIFO
    input  wire        out_full,
    output wire        out_en,
    output wire [23:0] out_pix,
    output wire        src_rows_done,  // 已收齐 H 行
    output reg         done,           // 640 x 480 全部输出
    output reg         err
);
    // ---------------- 配置与几何参数 ----------------
    localparam [3:0] C_IDLE = 4'd0, C_MUL = 4'd1, C_DIV2 = 4'd4,
                     C_DIV3 = 4'd5, C_CHK = 4'd6, C_DONE = 4'd7, C_DIVW = 4'd8;
    localparam [9:0] OW = 10'd640;
    localparam [8:0] OH = 9'd480;
    reg  [3:0]  cst, cst_ret;
    reg  [10:0] W, H;
    reg         up;                    // 1 = 放大（双线性），0 = 缩小 / 等大（区域平均）
    reg  [10:0] DW, DH;                // W - OW、H - OH（缩小时用）
    reg  [3:0]  QX, QY;                // floor(8W/OW)、floor(8H/OH)（放大时用，<= 8）
    reg  [9:0]  RX;                    // 8W mod OW
    reg  [8:0]  RY;                    // 8H mod OH
    reg  [10:0] B1, B2;                // 行缓存第 1 / 2 行起始地址 W、2W

    // 移位除法器：21 位被除数 / 11 位除数，21 拍
    reg  [20:0] dv_q;
    reg  [11:0] dv_r;
    reg  [10:0] dv_d;
    reg  [4:0]  dv_n;
    wire [11:0] dv_sh  = {dv_r[10:0], dv_q[20]};
    wire [12:0] dv_sub = {1'b0, dv_sh} - {2'b0, dv_d};

    assign cfg_busy = (cst != C_IDLE) && (cst != C_DONE);
    wire   cfg_good = up ? ((W <= 11'd640) && (H <= 11'd480) && (W >= 11'd320) && (H >= 11'd240))
                         : ((W <= 11'd1280) && (H <= 11'd960));

    // ---------------- 公共状态 ----------------
    reg         act;                   // 配置成功且未中止
    reg  [10:0] sx;                    // 源列
    reg  [10:0] sy;                    // 已收源行数（= 当前源行号）
    assign src_rows_done = act && (sy == H);
    wire        src_take = act && src_valid && (sy != H);
    wire        src_eol  = (sx == W - 11'd1);

    (* ram_style = "bram_32k" *) reg [29:0] ram [0:2047];
    reg  [10:0] ram_ra;
    reg  [29:0] ram_q;
    reg         ram_we;
    reg  [10:0] ram_wa;
    reg  [29:0] ram_wd;
    always @(posedge clk) begin
        if (ram_we)
            ram[ram_wa] <= ram_wd;
        ram_q <= ram[ram_ra];
    end

    reg  [23:0] fq0, fq1, fq2, fq3;
    reg  [1:0]  f_wp, f_rp;
    reg  [2:0]  f_cnt;
    reg         f_push;
    reg  [23:0] f_din;
    wire        f_pop;
    wire [23:0] f_head = (f_rp == 2'd0) ? fq0 : (f_rp == 2'd1) ? fq1 : (f_rp == 2'd2) ? fq2 : fq3;

    // ---------------- 输出侧：按 640 x 480 顺序 ----------------
    reg  [9:0]  ox;
    reg  [8:0]  oy;
    assign      out_en  = act && run && !done && !out_full && (f_cnt != 3'd0);
    assign      out_pix = f_head;
    assign      f_pop   = out_en;

    // ---------------- 缩小：区域平均 ----------------
    reg  [10:0] rx, ry;                // 列 / 行 DDA 余数
    reg  [9:0]  dc;                    // 当前输出列
    reg         hs;                    // 本组已有 1 列
    reg  [8:0]  rs_r, rs_g, rs_b;      // 组内列和
    reg         vf, vl;                // 当前源行是本行组的第一 / 最后一行
    // 第 1 级（src_take 当拍）组合量
    wire        gend   = (rx >= DW);
    wire [8:0]  sum_r  = (hs ? rs_r : 9'd0) + {1'b0, src_pix[23:16]};
    wire [8:0]  sum_g  = (hs ? rs_g : 9'd0) + {1'b0, src_pix[15:8]};
    wire [8:0]  sum_b  = (hs ? rs_b : 9'd0) + {1'b0, src_pix[7:0]};
    // 第 2 级寄存
    reg         d_v, d_hs2, d_vf, d_vl;
    reg  [9:0]  d_c;
    reg  [8:0]  d_r, d_g, d_b;
    wire [9:0]  t_r = {1'b0, d_r} + (d_vf ? 10'd0 : ram_q[29:20]);
    wire [9:0]  t_g = {1'b0, d_g} + (d_vf ? 10'd0 : ram_q[19:10]);
    wire [9:0]  t_b = {1'b0, d_b} + (d_vf ? 10'd0 : ram_q[9:0]);
    wire [1:0]  d_sh = {1'b0, d_hs2} + {1'b0, !d_vf};
    function [7:0] shr10;
        input [9:0] v;
        input [1:0] n;
        shr10 = (n == 2'd0) ? v[7:0] : (n == 2'd1) ? v[8:1] : v[9:2];
    endfunction

    reg  [1:0]  in_slot;               // 当前写入行所在槽 0~2
    reg  [10:0] in_base;
    reg  [8:0]  pv;                    // 输出行
    reg  [9:0]  pu;                    // 输出列
    reg  [13:0] qy8;                   // floor(v*8H/OH)
    reg  [8:0]  ryq;
    reg  [12:0] qx8;                   // floor(u*8W/OW)
    reg  [9:0]  rxq;
    reg  [1:0]  sy0;                   // y0 所在槽
    reg         p_done;
    wire [10:0] y0 = qy8[13:3];
    wire        y_last = (y0 == H - 11'd1);
    wire [10:0] y1 = y_last ? y0 : y0 + 11'd1;
    wire [1:0]  sy0n = (sy0 == 2'd2) ? 2'd0 : sy0 + 2'd1;
    wire [1:0]  sy1 = y_last ? sy0 : sy0n;
    wire [10:0] x0 = {1'b0, qx8[12:3]};
    function [10:0] slot_base;
        input [1:0]  s;
        input [10:0] b1, b2;
        slot_base = (s == 2'd0) ? 11'd0 : (s == 2'd1) ? b1 : b2;
    endfunction
    wire        rd_lo  = (pst == P_R2) || (pst == P_R3);                 // 读下行 y1
    wire        rd_rt  = (pst == P_R1) || (pst == P_R3);                 // 读右邻 x1
    wire [10:0] rd_bas = slot_base(rd_lo ? sy1 : sy0, B1, B2);
    wire        x_last = (x0 == W - 11'd1);
    wire [10:0] rd_adr = rd_bas + x0 + {10'd0, rd_rt && !x_last};
    // 行列 DDA：余数先加一次，进位再进到商（带进位的一次加法）
    wire [10:0] rx_sum = {1'b0, rxq} + {1'b0, RX};       // 最大 639 + 639，需 11 位
    wire        rx_c   = (rx_sum >= {1'b0, OW});
    wire [12:0] qx_nx  = qx8 + {9'd0, QX} + {12'd0, rx_c};
    wire [9:0]  ry_sum = {1'b0, ryq} + {1'b0, RY};
    wire        ry_c   = (ry_sum >= {1'b0, OH});
    wire [13:0] qy_nx  = qy8 + {10'd0, QY} + {13'd0, ry_c};
    // 生产者状态
    localparam [3:0] P_WAIT = 4'd0, P_R0 = 4'd1, P_R1 = 4'd2, P_R2 = 4'd3, P_R3 = 4'd4, P_CAP = 4'd5,
                     P_CAPD = 4'd10, P_CALC = 4'd6, P_PUSH = 4'd7, P_NEXT = 4'd8, P_IDLE = 4'd9;
    reg  [3:0]  pst;
    reg  [23:0] pa, pb, pc, pd;        // 左上 / 右上 / 左下 / 右下
    reg  [1:0]  kidx;                  // 计算序号 0~2：上行水平 / 下行水平 / 垂直
    reg         kph;
    reg  [23:0] op_a, op_b;
    reg  [2:0]  op_w;
    reg  [23:0] lt, lb;                // 上 / 下行水平插值结果
    reg  [23:0] po;
    reg  [2:0]  wx, wy;
    // 单通道插值器
    function [7:0] lerp8;
        input [7:0] a, b;
        input [2:0] w;
        reg   [10:0] s;
        begin
            s = {3'b0, a} + {3'b0, (w[0] ? b : a)} + {2'b0, (w[1] ? b : a), 1'b0} + {1'b0, (w[2] ? b : a), 2'b0};
            lerp8 = s[10:3];
        end
    endfunction
    wire [23:0] la  = (kidx == 2'd0) ? pa : (kidx == 2'd1) ? pc : lt;
    wire [23:0] lbv = (kidx == 2'd0) ? pb : (kidx == 2'd1) ? pd : lb;
    wire [23:0] lo  = {lerp8(op_a[23:16], op_b[23:16], op_w),
                       lerp8(op_a[15:8],  op_b[15:8],  op_w),
                       lerp8(op_a[7:0],   op_b[7:0],   op_w)};

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cst <= C_IDLE; cst_ret <= C_IDLE; cfg_ok <= 1'b0; act <= 1'b0;
            W <= 11'd640; H <= 11'd480;
            up <= 1'b0; DW <= 11'd0; DH <= 11'd0; QX <= 4'd0; QY <= 4'd0; RX <= 10'd0; RY <= 9'd0; B1 <= 11'd0; B2 <= 11'd0;
            dv_q <= 21'd0; dv_r <= 12'd0; dv_d <= 11'd1; dv_n <= 5'd0;
            sx <= 11'd0; sy <= 11'd0;
            ram_ra <= 11'd0; ram_we <= 1'b0; ram_wa <= 11'd0; ram_wd <= 30'd0;
            fq0 <= 24'd0; fq1 <= 24'd0; fq2 <= 24'd0; fq3 <= 24'd0; f_wp <= 2'd0; f_rp <= 2'd0; f_cnt <= 3'd0;
            f_push <= 1'b0; f_din <= 24'd0;
            ox <= 10'd0; oy <= 9'd0; done <= 1'b0; err <= 1'b0;
            rx <= 11'd0; ry <= 11'd0; dc <= 10'd0; hs <= 1'b0; rs_r <= 9'd0; rs_g <= 9'd0; rs_b <= 9'd0; vf <= 1'b1; vl <= 1'b0;
            d_v <= 1'b0; d_hs2 <= 1'b0; d_vf <= 1'b0; d_vl <= 1'b0; d_c <= 10'd0; d_r <= 9'd0; d_g <= 9'd0; d_b <= 9'd0;
            in_slot <= 2'd0; in_base <= 11'd0; pv <= 9'd0; pu <= 10'd0; qy8 <= 14'd0; ryq <= 9'd0; qx8 <= 13'd0; rxq <= 10'd0;
            sy0 <= 2'd0; p_done <= 1'b0; pst <= P_IDLE; pa <= 24'd0; pb <= 24'd0; pc <= 24'd0; pd <= 24'd0;
            kidx <= 2'd0; kph <= 1'b0; op_a <= 24'd0; op_b <= 24'd0; op_w <= 3'd0; lt <= 24'd0; lb <= 24'd0; po <= 24'd0; wx <= 3'd0; wy <= 3'd0;
        end else begin
            ram_we <= 1'b0;
            f_push <= 1'b0;

            // ======== 配置 ========
            case (cst)
                C_IDLE, C_DONE: begin
                    if (cfg_start) begin
                        W    <= cfg_w;
                        H    <= cfg_h;
                        act  <= 1'b0;
                        cfg_ok <= 1'b0;
                        cst  <= C_MUL;
                    end
                end
                C_MUL: begin                        // 放大：先算 8W/640
                    up   <= (W < 11'd640) || (H < 11'd480);
                    DW   <= W - 11'd640;
                    DH   <= H - 11'd480;
                    B1   <= W;
                    B2   <= {W[9:0], 1'b0};
                    dv_q <= {7'd0, W, 3'd0};
                    dv_d <= 11'd640;
                    dv_r <= 12'd0; dv_n <= 5'd0;
                    cst_ret <= C_DIV2;
                    cst  <= C_DIVW;
                end
                C_DIVW: begin                       // 21 拍移位除法
                    dv_r <= dv_sub[12] ? dv_sh : dv_sub[11:0];
                    dv_q <= {dv_q[19:0], !dv_sub[12]};
                    dv_n <= dv_n + 5'd1;
                    if (dv_n == 5'd20)
                        cst <= cst_ret;
                end
                C_DIV2: begin
                    QX <= dv_q[3:0]; RX <= dv_r[9:0];
                    dv_q <= {7'd0, H, 3'd0};
                    dv_d <= {2'd0, OH};
                    dv_r <= 12'd0; dv_n <= 5'd0;
                    cst_ret <= C_DIV3;
                    cst  <= C_DIVW;
                end
                C_DIV3: begin
                    QY <= dv_q[3:0]; RY <= dv_r[8:0];
                    cst <= C_CHK;
                end
                C_CHK: begin
                    cfg_ok <= cfg_good;
                    act    <= cfg_good;
                    cst    <= C_DONE;
                end
                default: cst <= C_IDLE;
            endcase

            // 新一帧开始（配置成功的那一拍）复位数据通路
            if (cst == C_CHK) begin
                sx <= 11'd0; sy <= 11'd0;
                f_wp <= 2'd0; f_rp <= 2'd0; f_cnt <= 3'd0;
                ox <= 10'd0; oy <= 9'd0; done <= 1'b0; err <= 1'b0;
                rx <= 11'd0; ry <= 11'd0; dc <= 10'd0; hs <= 1'b0; vf <= 1'b1; vl <= (DH == 11'd0);
                d_v <= 1'b0;
                in_slot <= 2'd0; in_base <= 11'd0;
                pv <= 9'd0; pu <= 10'd0; qy8 <= 14'd0; ryq <= 9'd0; qx8 <= 13'd0; rxq <= 10'd0;
                sy0 <= 2'd0; p_done <= 1'b0;
                pst <= up ? P_WAIT : P_IDLE;
            end
            if (abort) begin
                act <= 1'b0;
                cst <= C_IDLE;
            end

            // ======== 源像素输入 ========
            if (src_take) begin
                sx <= src_eol ? 11'd0 : sx + 11'd1;
                if (src_eol)
                    sy <= sy + 11'd1;
                if (up) begin
                    ram_we <= 1'b1;
                    ram_wa <= in_base + sx;
                    ram_wd <= {6'd0, src_pix};
                    if (src_eol) begin
                        in_slot <= (in_slot == 2'd2) ? 2'd0 : in_slot + 2'd1;
                        in_base <= (in_slot == 2'd2) ? 11'd0 : (in_slot == 2'd0) ? B1 : B2;
                        if (!p_done && (sy + 11'd1 >= y0 + 11'd3) && (sy + 11'd1 != H))
                            err <= 1'b1;
                    end
                end else begin
                    // 缩小：第 1 级
                    rx <= gend ? (rx - DW) : (rx + {1'b0, OW});
                    hs <= !gend;
                    rs_r <= sum_r; rs_g <= sum_g; rs_b <= sum_b;
                    d_v  <= gend;
                    d_hs2 <= hs;
                    d_vf <= vf; d_vl <= vl;
                    d_c  <= dc;
                    d_r  <= sum_r; d_g <= sum_g; d_b <= sum_b;
                    if (gend)
                        dc <= dc + 10'd1;
                    if (src_eol) begin
                        rx <= 11'd0; dc <= 10'd0; hs <= 1'b0;
                        // 行 DDA：本行是行组最后一行则余数减 DH，否则加 OH
                        vf <= vl;
                        if (vl) begin
                            ry <= ry - DH;
                            vl <= ((ry - DH) >= DH);
                        end else begin
                            ry <= ry + {2'd0, OH};
                            vl <= ((ry + {2'd0, OH}) >= DH);
                        end
                    end
                end
            end else begin
                d_v <= 1'b0;
            end
            // 缩小读地址：始终指向当前输出列（第 2 级用 ram_q）
            if (!up)
                ram_ra <= dc;

            if (!up && d_v) begin
                if (d_vl) begin
                    f_push <= 1'b1;
                    f_din  <= {shr10(t_r, d_sh), shr10(t_g, d_sh), shr10(t_b, d_sh)};
                end else begin
                    ram_we <= 1'b1;
                    ram_wa <= {1'b0, d_c};
                    ram_wd <= {1'b0, d_r, 1'b0, d_g, 1'b0, d_b};
                end
            end

            // ======== 放大：双线性生产者 ========
            if (up && act) begin
                case (pst)
                    P_WAIT: begin
                        if (!p_done && (sy > y1 || sy == H)) begin
                            wy  <= qy8[2:0];
                            pst <= P_R0;
                        end
                    end
                    P_R0: begin ram_ra <= rd_adr; wx <= qx8[2:0]; pst <= P_R1; end
                    P_R1: begin ram_ra <= rd_adr; pst <= P_R2; end
                    P_R2: begin ram_ra <= rd_adr; pa <= ram_q[23:0]; pst <= P_R3; end
                    P_R3: begin ram_ra <= rd_adr; pb <= ram_q[23:0]; pst <= P_CAP; end
                    P_CAP:  begin pc <= ram_q[23:0]; pst <= P_CAPD; end
                    P_CAPD: begin pd <= ram_q[23:0]; pst <= P_CALC; kidx <= 2'd0; kph <= 1'b0; end
                    P_CALC: begin
                        if (!kph) begin
                            op_a <= la;
                            op_b <= lbv;
                            op_w <= (kidx == 2'd2) ? wy : wx;
                            kph  <= 1'b1;
                        end else begin
                            case (kidx)
                                2'd0:    lt <= lo;
                                2'd1:    lb <= lo;
                                default: po <= lo;
                            endcase
                            kph  <= 1'b0;
                            kidx <= kidx + 2'd1;
                            if (kidx == 2'd2) pst <= P_PUSH;
                        end
                    end
                    P_PUSH: begin
                        if (f_cnt < 3'd3) begin
                            f_push <= 1'b1;
                            f_din  <= po;
                            pst    <= P_NEXT;
                        end
                    end
                    P_NEXT: begin
                        if (pu == OW - 10'd1) begin
                            pu <= 10'd0; qx8 <= 13'd0; rxq <= 10'd0;
                            pv <= pv + 9'd1;
                            // 行 DDA：qy8 += QY (+1)，y0 进位时槽号轮转
                            ryq <= ry_c ? (ry_sum[8:0] - OH) : ry_sum[8:0];
                            qy8 <= qy_nx;
                            if (qy_nx[13:3] != y0) sy0 <= sy0n;
                            if (pv == OH - 9'd1) begin
                                p_done <= 1'b1;
                                pst    <= P_IDLE;
                            end else
                                pst <= P_WAIT;
                        end else begin
                            pu <= pu + 10'd1;
                            rxq <= rx_c ? (rx_sum[9:0] - OW) : rx_sum[9:0];
                            qx8 <= qx_nx;
                            pst <= P_R0;
                        end
                    end
                    default: ;
                endcase
            end

            // ======== FIFO ========
            if (f_push && f_cnt == 3'd4 && !f_pop)
                err <= 1'b1;
            if (f_push && (f_cnt != 3'd4 || f_pop)) begin
                case (f_wp)
                    2'd0: fq0 <= f_din;
                    2'd1: fq1 <= f_din;
                    2'd2: fq2 <= f_din;
                    default: fq3 <= f_din;
                endcase
                f_wp <= f_wp + 2'd1;
            end
            if (f_pop)
                f_rp <= f_rp + 2'd1;
            case ({f_push && (f_cnt != 3'd4 || f_pop), f_pop})
                2'b10: f_cnt <= f_cnt + 3'd1;
                2'b01: f_cnt <= f_cnt - 3'd1;
                default: ;
            endcase

            // ======== 输出侧位置 ========
            if (out_en) begin
                if (ox == 10'd639) begin
                    ox <= 10'd0;
                    if (oy == 9'd479)
                        done <= 1'b1;
                    else
                        oy <= oy + 9'd1;
                end else
                    ox <= ox + 10'd1;
            end
        end
    end
endmodule
