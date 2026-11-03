`timescale 1ns / 1ps

module tf_audio_player #(
    parameter integer ADDR_WIDTH = 12,      // 4096 x 16bit = 85ms @ 48kHz
    parameter integer PREFILL    = 2048,    // 预填充样本数，攒够再开始放
    parameter integer HOLD_LEVEL = 3584
)(
    // 写侧：sd_card_clk 域
    input  wire        wr_clk,
    input  wire        wr_rst,
    input  wire        wr_play,             // 已同步到 wr_clk 的播放使能
    input  wire        byte_valid,
    input  wire [7:0]  byte_data,
    input  wire        byte_hi,             // 1=高字节，0=低字节（小端）
    output wire        hold,

    // 读侧：audio_mclk 域
    input  wire        I_mclk,
    input  wire        I_rst,
    input  wire        rd_play,             // 已同步到 I_mclk 的播放使能
    output reg         O_i2s_BCLK,
    output reg         O_i2s_LRCK,
    output reg         O_i2s_DOUT
);

reg [15:0] buffer [0:(1<<ADDR_WIDTH)-1];

function [ADDR_WIDTH-1:0] to_gray;
    input [ADDR_WIDTH-1:0] b;
    begin
        to_gray = b ^ (b >> 1);
    end
endfunction

function [ADDR_WIDTH-1:0] from_gray;
    input [ADDR_WIDTH-1:0] g;
    integer i;
    begin
        from_gray[ADDR_WIDTH-1] = g[ADDR_WIDTH-1];
        for (i = ADDR_WIDTH-2; i >= 0; i = i - 1)
            from_gray[i] = from_gray[i+1] ^ g[i];
    end
endfunction

// ===================== 写侧 =====================
reg  [ADDR_WIDTH-1:0] wr_ptr;
reg  [ADDR_WIDTH-1:0] wr_gray;
(* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH-1:0] rd_gray_meta;
(* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH-1:0] rd_gray_sync;
reg  [7:0]            lo_byte;
reg                   byte_valid_q;
reg  [7:0]            byte_data_q;
reg                   byte_hi_q;
reg  [ADDR_WIDTH-1:0] rd_ptr_wr;
reg                   hold_q;
wire                  wr_en = wr_play && byte_valid_q && byte_hi_q;
wire [ADDR_WIDTH-1:0] level_wr = wr_ptr - rd_ptr_wr;

assign hold = hold_q;

always @(posedge wr_clk) begin
    if (wr_en)
        buffer[wr_ptr] <= {byte_data_q, lo_byte};
end

always @(posedge wr_clk or posedge wr_rst) begin
    if (wr_rst) begin
        wr_ptr       <= {ADDR_WIDTH{1'b0}};
        wr_gray      <= {ADDR_WIDTH{1'b0}};
        rd_gray_meta <= {ADDR_WIDTH{1'b0}};
        rd_gray_sync <= {ADDR_WIDTH{1'b0}};
        rd_ptr_wr    <= {ADDR_WIDTH{1'b0}};
        hold_q       <= 1'b0;
        lo_byte      <= 8'd0;
        byte_valid_q <= 1'b0;
        byte_data_q  <= 8'd0;
        byte_hi_q    <= 1'b0;
    end else begin
        rd_gray_meta <= rd_gray;
        rd_gray_sync <= rd_gray_meta;
        rd_ptr_wr    <= from_gray(rd_gray_sync);
        hold_q       <= (level_wr >= HOLD_LEVEL);
        wr_gray      <= to_gray(wr_ptr);
        byte_valid_q <= byte_valid;
        byte_data_q  <= byte_data;
        byte_hi_q    <= byte_hi;

        if (!wr_play) begin
            wr_ptr <= {ADDR_WIDTH{1'b0}};
        end else if (byte_valid_q) begin
            if (!byte_hi_q)
                lo_byte <= byte_data_q;
            else
                wr_ptr <= wr_ptr + 1'b1;
        end
    end
end

// ===================== 读侧 =====================
reg  [ADDR_WIDTH-1:0] rd_ptr;
reg  [ADDR_WIDTH-1:0] rd_gray;
(* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH-1:0] wr_gray_meta;
(* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH-1:0] wr_gray_sync;
reg  [15:0]           ram_q;
reg                   started;
reg  [5:0]            S_bit_cnt;
reg  [63:0]           S_shift_reg;
reg  [15:0]           S_sample;
wire [ADDR_WIDTH-1:0] level_rd = from_gray(wr_gray_sync) - rd_ptr;
wire                  frame_end = O_i2s_BCLK && (S_bit_cnt == 6'd63) && O_i2s_LRCK;
// 缓冲空（下溢或播完）时输出静音
wire                  sample_avail = rd_play && started && (level_rd != 0);
wire [15:0]           next_sample  = sample_avail ? ram_q : 16'd0;

always @(posedge I_mclk) begin
    ram_q <= buffer[rd_ptr];
end

always @(posedge I_mclk or posedge I_rst) begin
    if (I_rst) begin
        rd_ptr       <= {ADDR_WIDTH{1'b0}};
        rd_gray      <= {ADDR_WIDTH{1'b0}};
        wr_gray_meta <= {ADDR_WIDTH{1'b0}};
        wr_gray_sync <= {ADDR_WIDTH{1'b0}};
        started      <= 1'b0;
        S_sample     <= 16'd0;
    end else begin
        wr_gray_meta <= wr_gray;
        wr_gray_sync <= wr_gray_meta;
        rd_gray      <= to_gray(rd_ptr);

        if (!rd_play) begin
            rd_ptr   <= {ADDR_WIDTH{1'b0}};
            started  <= 1'b0;
            S_sample <= 16'd0;
        end else begin
            if (level_rd >= PREFILL)
                started <= 1'b1;

            // 每帧结束时取下一帧样本
            if (frame_end) begin
                S_sample <= next_sample;
                if (sample_avail)
                    rd_ptr <= rd_ptr + 1'b1;
            end
        end
    end
end

always @(posedge I_mclk or posedge I_rst) begin
    if (I_rst) begin
        O_i2s_BCLK  <= 1'b0;
        O_i2s_LRCK  <= 1'b0;
        O_i2s_DOUT  <= 1'b0;
        S_bit_cnt   <= 6'd0;
        S_shift_reg <= 64'd0;
    end else begin
        O_i2s_BCLK <= ~O_i2s_BCLK;

        if (O_i2s_BCLK == 1'b1) begin
            O_i2s_DOUT  <= S_shift_reg[63];
            S_shift_reg <= {S_shift_reg[62:0], 1'b0};

            if (S_bit_cnt == 6'd63) begin
                S_bit_cnt <= 6'd0;

                if (O_i2s_LRCK == 1'b0) begin
                    // 左声道发完，右声道复用同一个样本
                    O_i2s_LRCK  <= 1'b1;
                    S_shift_reg <= {S_sample, 8'd0, 40'd0};
                end else begin
                    // 右声道发完，S_sample 同拍更新，下一帧发新样本
                    O_i2s_LRCK  <= 1'b0;
                    S_shift_reg <= {next_sample, 8'd0, 40'd0};
                end
            end else begin
                S_bit_cnt <= S_bit_cnt + 6'd1;
            end
        end
    end
end

endmodule
