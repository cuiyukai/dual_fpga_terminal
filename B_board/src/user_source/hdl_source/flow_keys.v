`timescale 1ns / 1ps

module flow_keys #(
    parameter integer CLK_HZ        = 50_000_000,
    parameter integer DEBOUNCE_MS   = 20,
    parameter integer LONG_MS       = 1000,
    parameter integer ADJ_DELAY_MS  = 500,     // 调节：按住多久开始连续变化
    parameter integer ADJ_REPEAT_MS = 50,      // 调节：连续变化时每步间隔
    parameter         SW_ON_LEVEL   = 1'b0,
    parameter         ADJ_ON_LEVEL  = 1'b1     // SW2 / SW1 处于该电平时为打开（手册：拨上为 1）
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       sw4,       // 拨码开关 SW4(C5)
    input  wire       sw3,       // 拨码开关 SW3(C6)
    input  wire       sw2,       // 拨码开关 SW2(C7)：音量调节
    input  wire       sw1,       // 拨码开关 SW1(C8)：亮度调节
    input  wire       key3,      // 按键 KEY3(B1)，低有效
    input  wire       key4,      // 按键 KEY4(C1)，低有效
    output wire [3:0] flow_flags,
    output wire       test_mode, // SW3 实体测试电平
    output wire [6:0] brightness,
    output wire [6:0] volume
);

localparam integer LONG_CYCLES = CLK_HZ / 1000 * LONG_MS;
localparam integer LONG_WIDTH  = $clog2(LONG_CYCLES + 1);
localparam integer DB_CYCLES   = CLK_HZ / 1000 * DEBOUNCE_MS;

wire sw4_low;
wire sw3_low;
wire sw2_low;
wire sw1_low;
wire key3_down;
wire key4_down;

flow_key_debounce #(.CYCLES(DB_CYCLES)) u_sw4_db (
    .clk (clk), .rst_n (rst_n), .in (sw4),  .low (sw4_low));
flow_key_debounce #(.CYCLES(DB_CYCLES)) u_sw3_db (
    .clk (clk), .rst_n (rst_n), .in (sw3),  .low (sw3_low));
flow_key_debounce #(.CYCLES(DB_CYCLES)) u_sw2_db (
    .clk (clk), .rst_n (rst_n), .in (sw2),  .low (sw2_low));
flow_key_debounce #(.CYCLES(DB_CYCLES)) u_sw1_db (
    .clk (clk), .rst_n (rst_n), .in (sw1),  .low (sw1_low));
flow_key_debounce #(.CYCLES(DB_CYCLES)) u_key3_db (
    .clk (clk), .rst_n (rst_n), .in (key3), .low (key3_down));
flow_key_debounce #(.CYCLES(DB_CYCLES)) u_key4_db (
    .clk (clk), .rst_n (rst_n), .in (key4), .low (key4_down));

wire flow_mode = (sw4_low == !SW_ON_LEVEL);
assign test_mode = (sw3_low == !SW_ON_LEVEL);

wire vol_adj    = (sw2_low == !ADJ_ON_LEVEL);
wire bright_adj = (sw1_low == !ADJ_ON_LEVEL);
wire adj_mode   = vol_adj || bright_adj;
wire flow_en    = flow_mode && !adj_mode;

// ---------------- 音量 / 亮度调节 ----------------
level_adjust #(
    .DELAY_CYCLES  (CLK_HZ / 1000 * ADJ_DELAY_MS),
    .REPEAT_CYCLES (CLK_HZ / 1000 * ADJ_REPEAT_MS)
) u_vol_adj (
    .clk (clk), .rst_n (rst_n), .inc (key4_down && vol_adj), .dec (key3_down && vol_adj), .value (volume));

level_adjust #(
    .DELAY_CYCLES  (CLK_HZ / 1000 * ADJ_DELAY_MS),
    .REPEAT_CYCLES (CLK_HZ / 1000 * ADJ_REPEAT_MS)
) u_bright_adj (
    .clk (clk), .rst_n (rst_n), .inc (key4_down && bright_adj), .dec (key3_down && bright_adj), .value (brightness));

// ---------------- 流程按键 ----------------
reg                  key3_down_d;
reg [LONG_WIDTH-1:0] key3_cnt;
reg                  key3_long;
reg [LONG_WIDTH-1:0] key4_cnt;
reg                  key4_long;
reg                  next_tgl;
reg                  prev_tgl;
reg                  play_tgl;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        key3_down_d <= 1'b0;
        key3_cnt    <= {LONG_WIDTH{1'b0}};
        key3_long   <= 1'b0;
        key4_cnt    <= {LONG_WIDTH{1'b0}};
        key4_long   <= 1'b0;
        next_tgl    <= 1'b0;
        prev_tgl    <= 1'b0;
        play_tgl    <= 1'b0;
    end else begin
        key3_down_d <= key3_down;

        // KEY3：按住满 LONG_MS 触发上一页；未到长按就松开触发下一页
        if (key3_down) begin
            if (adj_mode)
                key3_long <= 1'b1;
            if (key3_cnt != LONG_CYCLES - 1)
                key3_cnt <= key3_cnt + 1'b1;
            else if (!key3_long) begin
                key3_long <= 1'b1;
                if (flow_en)
                    prev_tgl <= ~prev_tgl;
            end
        end else begin
            key3_cnt  <= {LONG_WIDTH{1'b0}};
            key3_long <= 1'b0;
            if (key3_down_d && !key3_long && flow_en)
                next_tgl <= ~next_tgl;
        end

        // KEY4：按住满 LONG_MS 触发开始轮播
        if (key4_down) begin
            if (adj_mode)
                key4_long <= 1'b1;
            if (key4_cnt != LONG_CYCLES - 1)
                key4_cnt <= key4_cnt + 1'b1;
            else if (!key4_long) begin
                key4_long <= 1'b1;
                if (flow_en)
                    play_tgl <= ~play_tgl;
            end
        end else begin
            key4_cnt  <= {LONG_WIDTH{1'b0}};
            key4_long <= 1'b0;
        end
    end
end

assign flow_flags = {play_tgl, prev_tgl, next_tgl, flow_mode};

endmodule

module level_adjust #(
    parameter integer DELAY_CYCLES  = 25_000_000,
    parameter integer REPEAT_CYCLES = 2_500_000,
    parameter [6:0]   MAX           = 7'd100,
    parameter [6:0]   INIT          = 7'd100
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       inc,
    input  wire       dec,
    output reg  [6:0] value
);

localparam integer CNT_MAX   = (DELAY_CYCLES > REPEAT_CYCLES) ? DELAY_CYCLES : REPEAT_CYCLES;
localparam integer CNT_WIDTH = $clog2(CNT_MAX + 1);

reg [1:0]           key_d;
reg [CNT_WIDTH-1:0] cnt;

wire hold  = inc ^ dec;
wire start = hold && ({inc, dec} != key_d);
wire step  = start || (hold && (cnt == {CNT_WIDTH{1'b0}}));

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        key_d <= 2'b00;
        cnt   <= {CNT_WIDTH{1'b0}};
        value <= INIT;
    end else begin
        key_d <= {inc, dec};

        if (start)
            cnt <= DELAY_CYCLES - 1;
        else if (hold)
            cnt <= (cnt == {CNT_WIDTH{1'b0}}) ? REPEAT_CYCLES - 1 : cnt - 1'b1;

        if (step) begin
            if (inc && (value < MAX))
                value <= value + 1'b1;
            else if (dec && (value != 7'd0))
                value <= value - 1'b1;
        end
    end
end

endmodule

module flow_key_debounce #(
    parameter integer CYCLES = 1_000_000
)(
    input  wire clk,
    input  wire rst_n,
    input  wire in,
    output reg  low
);

localparam integer CNT_WIDTH = $clog2(CYCLES + 1);

(* ASYNC_REG = "TRUE" *) reg [1:0] in_sync;
reg [CNT_WIDTH-1:0] cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        in_sync <= 2'b11;
        cnt     <= {CNT_WIDTH{1'b0}};
        low     <= 1'b0;
    end else begin
        in_sync <= {in_sync[0], in};
        if (!in_sync[1] == low)
            cnt <= {CNT_WIDTH{1'b0}};
        else if (cnt == CYCLES - 1) begin
            cnt <= {CNT_WIDTH{1'b0}};
            low <= !low;
        end else
            cnt <= cnt + 1'b1;
    end
end

endmodule
