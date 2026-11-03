`timescale 1ns/1ps

module alarm_ribbon3d #(
    parameter integer RY0   = 96,            // 窗口顶行（yp，相对屏幕中心），屏幕行 240 + RY0 起
    parameter integer WIN_H = 96,            // 窗口行数（4 的倍数，<= 128）
    parameter integer N     = 16,            // 丝线根数（<= 16）
    parameter [11:0]  K     = 12'd22,
    parameter [11:0]  DELTA = 12'd128,       // 相邻丝相位差（16 根共约 170 度）
    parameter [11:0]  K2    = 12'd9,         // 整体摆动每列增量
    parameter [11:0]  TW    = 12'd24,        // 丝带每帧流动（相位减量）
    parameter [11:0]  TW2   = 12'd7,         // 整体摆动每帧变化
    parameter integer FADE  = 24             // 两端降档列数
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               frame_start,   // 单拍：场同步起点（引擎随即画一帧，约 42 行内画完）
    input  wire               de,
    input  wire               de_q,
    input  wire signed [10:0] xp,            // 当前像素列号 - 320
    input  wire signed [9:0]  yp,            // 当前像素行号 - 240
    input  wire [59:0]        vis_band,
    output reg  [1:0]         lvl            // 晚两拍：亮度档 0~3
);
    localparam integer BPC   = WIN_H / 4;    // 每列字节数（<= 32）
    localparam integer DEPTH = 320 * 32;     // 地址 {列号[8:0], 字节号[4:0]}
    localparam [6:0]   YC    = WIN_H / 2;

    // ---------------- 查表函数 ----------------
    // 段值 0~16 -> 半高 4 + 13v/8
    function [5:0] kmap;
        input [4:0] v;
        begin
            case (v)
                5'd0:  kmap = 6'd4;   5'd1:  kmap = 6'd5;   5'd2:  kmap = 6'd7;   5'd3:  kmap = 6'd8;
                5'd4:  kmap = 6'd10;  5'd5:  kmap = 6'd12;  5'd6:  kmap = 6'd13;  5'd7:  kmap = 6'd15;
                5'd8:  kmap = 6'd17;  5'd9:  kmap = 6'd18;  5'd10: kmap = 6'd20;  5'd11: kmap = 6'd21;
                5'd12: kmap = 6'd23;  5'd13: kmap = 6'd25;  5'd14: kmap = 6'd26;  5'd15: kmap = 6'd28;
                default: kmap = 6'd30;
            endcase
        end
    endfunction

    // 1/4 周期正弦表：round(127 * sin((i + 0.5) / 64 * 90 度))
    function [6:0] qsin;
        input [5:0] i;
        begin
            case (i)
                6'd0:  qsin = 7'd2;   6'd1:  qsin = 7'd5;   6'd2:  qsin = 7'd8;   6'd3:  qsin = 7'd11;
                6'd4:  qsin = 7'd14;  6'd5:  qsin = 7'd17;  6'd6:  qsin = 7'd20;  6'd7:  qsin = 7'd23;
                6'd8:  qsin = 7'd26;  6'd9:  qsin = 7'd29;  6'd10: qsin = 7'd32;  6'd11: qsin = 7'd35;
                6'd12: qsin = 7'd38;  6'd13: qsin = 7'd41;  6'd14: qsin = 7'd44;  6'd15: qsin = 7'd47;
                6'd16: qsin = 7'd50;  6'd17: qsin = 7'd53;  6'd18: qsin = 7'd56;  6'd19: qsin = 7'd58;
                6'd20: qsin = 7'd61;  6'd21: qsin = 7'd64;  6'd22: qsin = 7'd67;  6'd23: qsin = 7'd69;
                6'd24: qsin = 7'd72;  6'd25: qsin = 7'd74;  6'd26: qsin = 7'd77;  6'd27: qsin = 7'd79;
                6'd28: qsin = 7'd82;  6'd29: qsin = 7'd84;  6'd30: qsin = 7'd86;  6'd31: qsin = 7'd89;
                6'd32: qsin = 7'd91;  6'd33: qsin = 7'd93;  6'd34: qsin = 7'd95;  6'd35: qsin = 7'd97;
                6'd36: qsin = 7'd99;  6'd37: qsin = 7'd101; 6'd38: qsin = 7'd103; 6'd39: qsin = 7'd105;
                6'd40: qsin = 7'd106; 6'd41: qsin = 7'd108; 6'd42: qsin = 7'd110; 6'd43: qsin = 7'd111;
                6'd44: qsin = 7'd113; 6'd45: qsin = 7'd114; 6'd46: qsin = 7'd115; 6'd47: qsin = 7'd117;
                6'd48: qsin = 7'd118; 6'd49: qsin = 7'd119; 6'd50: qsin = 7'd120; 6'd51: qsin = 7'd121;
                6'd52: qsin = 7'd122; 6'd53: qsin = 7'd123; 6'd54: qsin = 7'd124; 6'd55: qsin = 7'd124;
                6'd56: qsin = 7'd125; 6'd57: qsin = 7'd125; 6'd58: qsin = 7'd126; 6'd59: qsin = 7'd126;
                default: qsin = 7'd127;
            endcase
        end
    endfunction

    // 相位 12 bit -> 有符号 8 bit 正弦（负半周取反码）
    function signed [7:0] sin12;
        input [11:0] ph;
        reg   [6:0]  m;
        begin
            m     = qsin(ph[10] ? ~ph[9:4] : ph[9:4]);
            sin12 = ph[11] ? ~{1'b0, m} : {1'b0, m};
        end
    endfunction

    function [1:0] depth_lvl;
        input [4:0] h;
        begin
            case (h)
                5'd0, 5'd1, 5'd2, 5'd3, 5'd4,
                5'd27, 5'd28, 5'd29, 5'd30, 5'd31: depth_lvl = 2'd3;
                5'd5, 5'd6, 5'd7, 5'd8, 5'd9,
                5'd22, 5'd23, 5'd24, 5'd25, 5'd26: depth_lvl = 2'd2;
                default:                           depth_lvl = 2'd1;
            endcase
        end
    endfunction

    wire [5:0] nd1 = kmap(vis_band[4:0]);
    wire [5:0] nd2 = kmap(vis_band[14:10]);
    wire [5:0] nd3 = kmap(vis_band[24:20]);
    wire [5:0] nd4 = kmap(vis_band[39:35]);
    reg  [5:0] nl1, nl2, nl3, nl4;
    function [5:0] node;
        input [2:0] j;
        begin
            case (j)
                3'd1:    node = nl1;
                3'd2:    node = nl2;
                3'd3:    node = nl3;
                3'd4:    node = nl4;
                default: node = 6'd0;
            endcase
        end
    endfunction

    // ---------------- 引擎 ----------------
    localparam [1:0] E_IDLE = 2'd0, E_CLR = 2'd1, E_STR = 2'd2;
    reg  [1:0]  est;
    reg  [8:0]  col;
    reg  [4:0]  bidx;                  // 清零字节号 0~BPC-1
    reg  [3:0]  sidx;                  // 丝号 0~N-1
    reg  [2:0]  stp;                   // 丝内拍号 0~4
    reg  [11:0] th0, th20;             // 帧相位
    reg  [11:0] thc, th2c;             // 本列相位
    reg  [11:0] ph;                    // 当前丝相位
    reg  [11:0] env;                   // 包络 A x 64
    reg  signed [6:0] slope;
    reg  [2:0]  seg;
    reg         edge_f;                // 本列在两端降档区
    reg  [13:0] amp;                   // 当前丝半高 x 256
    reg  [8:0]  amp_dec;               // 每根丝半高减量 A x 6
    wire [5:0]  A = env[11] ? 6'd0 : env[11:6];

    reg  [5:0]  m_a;
    reg  signed [7:0]  m_b;
    reg  [1:0]  m_lv;
    reg  signed [14:0] prod;
    (* dsp_style = "yes" *) wire signed [14:0] prod_w = $signed({1'b0, m_a}) * m_b;
    reg  signed [7:0]  cswing;
    wire signed [8:0]  y_full = $signed({2'b00, YC}) + cswing + $signed(prod[14:7]);
    wire               y_ok   = !y_full[8] && (y_full[7:0] < WIN_H);

    // 帧缓存与读-改-写
    (* ram_style = "bram_32k" *) reg [7:0] fb [0:DEPTH-1];
    reg  [7:0]  rq;
    reg  [7:0]  wbyte;
    reg         we;
    reg  [13:0] we_addr;
    reg  [13:0] e_raddr;               // 引擎读地址
    reg  [1:0]  e_sel;                 // 字节内像素号
    reg         e_ok;
    wire        busy   = (est != E_IDLE);
    wire [1:0]  old_px = rq[2*e_sel +: 2];
    wire [1:0]  new_px = (m_lv > old_px) ? m_lv : old_px;
    reg  [7:0]  rmw_byte;
    always @* begin
        rmw_byte = rq;
        rmw_byte[2*e_sel +: 2] = new_px;
    end

    wire [4:0]  ph_cos_h = ph[11:7] + 5'd8;
    wire [1:0]  lv_raw = depth_lvl(ph_cos_h);
    wire [1:0]  lv_use = (edge_f && lv_raw != 2'd1) ? lv_raw - 1'b1 : lv_raw;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            est     <= E_IDLE;
            col     <= 9'd0;
            bidx    <= 5'd0;
            sidx    <= 4'd0;
            stp     <= 3'd0;
            th0     <= 12'd0;
            th20    <= 12'd0;
            thc     <= 12'd0;
            th2c    <= 12'd0;
            ph      <= 12'd0;
            env     <= 12'd0;
            slope   <= 7'sd0;
            seg     <= 3'd0;
            edge_f  <= 1'b1;
            amp     <= 14'd0;
            amp_dec <= 9'd0;
            nl1     <= 6'd0;
            nl2     <= 6'd0;
            nl3     <= 6'd0;
            nl4     <= 6'd0;
            m_a     <= 6'd0;
            m_b     <= 8'sd0;
            m_lv    <= 2'd0;
            prod    <= 15'sd0;
            cswing  <= 8'sd0;
            we      <= 1'b0;
            we_addr <= 14'd0;
            wbyte   <= 8'd0;
            e_raddr <= 14'd0;
            e_sel   <= 2'd0;
            e_ok    <= 1'b0;
        end else begin
            we   <= 1'b0;
            prod <= prod_w;                           // 发出后下一拍得到乘积

            case (est)
                E_IDLE: begin
                    if (frame_start) begin
                        th0    <= th0 - TW;
                        th20   <= th20 + TW2;
                        thc    <= th0 - TW;
                        th2c   <= th20 + TW2;
                        col    <= 9'd0;
                        env    <= 12'd0;
                        seg    <= 3'd0;
                        slope  <= $signed({1'b0, nd1});
                        nl1    <= nd1;
                        nl2    <= nd2;
                        nl3    <= nd3;
                        nl4    <= nd4;
                        edge_f <= 1'b1;
                        bidx   <= 5'd0;
                        est    <= E_CLR;
                    end
                end
                E_CLR: begin
                    we      <= 1'b1;
                    we_addr <= {col, bidx};
                    wbyte   <= 8'd0;
                    if (bidx == 5'd0) begin
                        m_a     <= {2'b00, A[5:2]};
                        m_b     <= sin12(th2c);
                        ph      <= thc;
                        amp     <= {A, 8'd0};
                        amp_dec <= {1'b0, A, 2'b00} + {2'b00, A, 1'b0};
                    end
                    if (bidx == 5'd2)
                        cswing <= prod[14:7];
                    if (bidx == BPC - 1) begin
                        est  <= E_STR;
                        sidx <= 4'd0;
                        stp  <= 3'd0;
                    end else begin
                        bidx <= bidx + 1'b1;
                    end
                end
                default: begin
                    case (stp)
                        3'd0: begin
                            m_a  <= amp[13:8];
                            m_b  <= sin12(ph);
                            m_lv <= lv_use;
                            ph   <= ph + DELTA;
                            amp  <= amp - {5'd0, amp_dec};
                        end
                        3'd2: begin
                            e_raddr <= {col, y_full[6:2]};
                            e_sel   <= y_full[1:0];
                            e_ok    <= y_ok;
                        end
                        3'd4: begin
                            we      <= e_ok;
                            we_addr <= e_raddr;
                            wbyte   <= rmw_byte;
                        end
                        default: ;
                    endcase
                    if (stp != 3'd4) begin
                        stp <= stp + 1'b1;
                    end else begin
                        stp <= 3'd0;
                        if (sidx != N - 1) begin
                            sidx <= sidx + 1'b1;
                        end else if (col == 9'd319) begin
                            est <= E_IDLE;
                        end else begin
                            // 下一列
                            col   <= col + 1'b1;
                            thc   <= thc + K;
                            th2c  <= th2c + K2;
                            env   <= env + {{5{slope[6]}}, slope};
                            if (col[5:0] == 6'd63) begin
                                seg   <= seg + 1'b1;
                                slope <= $signed({1'b0, node(seg + 3'd2)}) - $signed({1'b0, node(seg + 3'd1)});
                            end
                            if (col == FADE - 1)
                                edge_f <= 1'b0;
                            else if (col == 319 - FADE)
                                edge_f <= 1'b1;
                            bidx <= 5'd0;
                            est  <= E_CLR;
                        end
                    end
                end
            endcase
        end
    end

    reg         x_on, y_on;
    reg  [6:0]  wr;                    // 窗口内行号
    reg  [8:0]  dcol;                  // 窗口内列号
    reg         rv_d1;
    reg  [1:0]  rs_d1;

    always @(posedge clk) begin
        if (we)
            fb[we_addr] <= wbyte;
        rq <= fb[busy ? e_raddr : {dcol, wr[6:2]}];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_on  <= 1'b0;
            y_on  <= 1'b0;
            wr    <= 7'd0;
            dcol  <= 9'd0;
            rv_d1 <= 1'b0;
            rs_d1 <= 2'd0;
            lvl   <= 2'd0;
        end else begin
            if (frame_start) begin
                y_on <= 1'b0;
            end else if (de_q && !de) begin
                if (yp == RY0 - 1) begin
                    y_on <= 1'b1;
                    wr   <= 7'd0;
                end else if (yp == RY0 + WIN_H - 1) begin
                    y_on <= 1'b0;
                end else if (y_on) begin
                    wr <= wr + 1'b1;
                end
            end
            if (!de) begin
                x_on <= 1'b0;
                dcol <= 9'd0;
            end else begin
                if (xp == -11'sd161)
                    x_on <= 1'b1;
                else if (xp == 11'sd159)
                    x_on <= 1'b0;
                if (x_on)
                    dcol <= dcol + 1'b1;
            end
            rv_d1 <= de && x_on && y_on && !busy;
            rs_d1 <= wr[1:0];
            lvl   <= rv_d1 ? rq[2*rs_d1 +: 2] : 2'd0;
        end
    end
endmodule
