`timescale 1ns / 1ps

module menu_ctrl #(
    parameter integer CLK_HZ        = 50_000_000,
    parameter integer DEBOUNCE_MS   = 20,
    parameter integer LONG_MS       = 1000,
    parameter [6:0]   ADJ_STEP      = 7'd5     // 音量 / 亮度：旋钮每格步进
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       key2,        // 按键 KEY2(B2)，低有效
    input  wire       key3,        // 按键 KEY3(B1)，低有效
    input  wire       key4,        // 按键 KEY4(C1)，低有效
    input  wire       knob_cw,     // 旋钮单拍脉冲（rotary_encoder 输出）
    input  wire       knob_ccw,
    input  wire       knob_press,
    output wire [3:0] flow_flags,
    output reg        flow_stop,   // 退出轮播事件翻转位
    output reg  [2:0] knob_tgl,    // 流程模式旋钮事件翻转位 {按下, 逆时针, 顺时针}
    output wire       test_mode,   // 实体检测模式电平
    output wire       sleep_on,    // 休眠模式电平
    output reg        imu_zero,    // 单拍：陀螺仪重新复位 + 消零
    output reg        stamp_tgl,   // 主菜单下短按 KEY2 翻转一次：A 板首页飘过时间戳
    output reg  [6:0] brightness,
    output reg  [6:0] volume,
    output wire [7:0] menu_state,
    output wire       rtc_cw,      // 休眠时转发给 RTC 的旋钮脉冲
    output wire       rtc_ccw,
    output wire       rtc_press
);

localparam integer LONG_CYCLES = CLK_HZ / 1000 * LONG_MS;
localparam integer LONG_WIDTH  = $clog2(LONG_CYCLES + 1);
localparam integer DB_CYCLES   = CLK_HZ / 1000 * DEBOUNCE_MS;

localparam [2:0] M_MENU   = 3'd0;
localparam [2:0] M_VOL    = 3'd1;   // 进入模式 = 光标 + 1
localparam [2:0] M_BRIGHT = 3'd2;
localparam [2:0] M_FLOW   = 3'd3;
localparam [2:0] M_TEST   = 3'd4;
localparam [2:0] M_CAM    = 3'd5;
localparam [2:0] M_SLEEP  = 3'd6;

wire key2_down;
wire key3_down;
wire key4_down;

flow_key_debounce #(.CYCLES(DB_CYCLES)) u_key2_db (
    .clk (clk), .rst_n (rst_n), .in (key2), .low (key2_down));
flow_key_debounce #(.CYCLES(DB_CYCLES)) u_key3_db (
    .clk (clk), .rst_n (rst_n), .in (key3), .low (key3_down));
flow_key_debounce #(.CYCLES(DB_CYCLES)) u_key4_db (
    .clk (clk), .rst_n (rst_n), .in (key4), .low (key4_down));

reg [2:0] mode;
reg [2:0] cursor;

wire in_menu   = (mode == M_MENU);
wire in_vol    = (mode == M_VOL);
wire in_bright = (mode == M_BRIGHT);
wire in_flow   = (mode == M_FLOW);

assign test_mode = (mode == M_TEST);
assign sleep_on  = (mode == M_SLEEP);

// 休眠时旋钮只给 RTC 用
assign rtc_cw    = knob_cw    && sleep_on;
assign rtc_ccw   = knob_ccw   && sleep_on;
assign rtc_press = knob_press && sleep_on;

// 音量 / 亮度旋钮步进：结果限制在 0~100
function [6:0] knob_step;
    input [6:0] v;
    input       up;
    begin
        if (up)
            knob_step = (v >= 7'd100 - ADJ_STEP) ? 7'd100 : v + ADJ_STEP;
        else
            knob_step = (v <= ADJ_STEP) ? 7'd0 : v - ADJ_STEP;
    end
endfunction

// ---------------- 模式切换 / 按键事件 ----------------
reg                  key2_down_d;
reg [LONG_WIDTH-1:0] key2_cnt;
reg                  key2_long;
reg                  key3_down_d;
reg [LONG_WIDTH-1:0] key3_cnt;
reg                  key3_long;
reg                  key4_down_d;
reg [LONG_WIDTH-1:0] key4_cnt;
reg                  key4_long;
reg                  next_tgl;
reg                  prev_tgl;
reg                  play_tgl;

wire key2_long_hit = key2_down && !key2_long && (key2_cnt == LONG_CYCLES - 1);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mode        <= M_MENU;
        cursor      <= 3'd0;
        imu_zero    <= 1'b0;
        stamp_tgl   <= 1'b0;
        key2_down_d <= 1'b0;
        key2_cnt    <= {LONG_WIDTH{1'b0}};
        key2_long   <= 1'b0;
        key3_down_d <= 1'b0;
        key3_cnt    <= {LONG_WIDTH{1'b0}};
        key3_long   <= 1'b0;
        key4_down_d <= 1'b0;
        key4_cnt    <= {LONG_WIDTH{1'b0}};
        key4_long   <= 1'b0;
        next_tgl    <= 1'b0;
        prev_tgl    <= 1'b0;
        play_tgl    <= 1'b0;
        flow_stop   <= 1'b0;
        knob_tgl    <= 3'd0;
        volume      <= 7'd100;
        brightness  <= 7'd100;
    end else begin
        key2_down_d <= key2_down;
        key3_down_d <= key3_down;
        key4_down_d <= key4_down;
        imu_zero    <= 1'b0;

        // KEY2：长按回菜单；短按在主菜单下发时间戳，其他模式陀螺仪消零
        if (key2_down) begin
            if (key2_cnt != LONG_CYCLES - 1)
                key2_cnt <= key2_cnt + 1'b1;
            else
                key2_long <= 1'b1;
        end else begin
            key2_cnt  <= {LONG_WIDTH{1'b0}};
            key2_long <= 1'b0;
            if (key2_down_d && !key2_long) begin
                if (in_menu)
                    stamp_tgl <= ~stamp_tgl;
                else if (!sleep_on)
                    imu_zero <= 1'b1;
            end
        end

        if (key2_long_hit)
            mode <= M_MENU;
        else if (in_menu) begin
            if (knob_press) begin
                mode <= cursor + 1'b1;
                if (cursor + 1'b1 == M_TEST)
                    imu_zero <= 1'b1;
            end else if (knob_cw)
                cursor <= (cursor == 3'd5) ? 3'd0 : cursor + 1'b1;
            else if (knob_ccw)
                cursor <= (cursor == 3'd0) ? 3'd5 : cursor - 1'b1;
        end else if (in_vol) begin
            if (knob_cw || knob_ccw)
                volume <= knob_step(volume, knob_cw);
        end else if (in_bright) begin
            if (knob_cw || knob_ccw)
                brightness <= knob_step(brightness, knob_cw);
        end else if (test_mode) begin
            if (knob_press)
                imu_zero <= 1'b1;   // 重新开始检测
        end else if (in_flow) begin
            knob_tgl <= knob_tgl ^ {knob_press, knob_ccw, knob_cw};
        end

        // KEY3（流程）：按住满 LONG_MS 上一页；未到长按就松开下一页
        if (key3_down) begin
            if (key3_cnt != LONG_CYCLES - 1)
                key3_cnt <= key3_cnt + 1'b1;
            else if (!key3_long) begin
                key3_long <= 1'b1;
                if (in_flow)
                    prev_tgl <= ~prev_tgl;
            end
        end else begin
            key3_cnt  <= {LONG_WIDTH{1'b0}};
            key3_long <= 1'b0;
            if (key3_down_d && !key3_long && in_flow)
                next_tgl <= ~next_tgl;
        end

        if (key4_down) begin
            if (key4_cnt != LONG_CYCLES - 1)
                key4_cnt <= key4_cnt + 1'b1;
            else if (!key4_long) begin
                key4_long <= 1'b1;
                if (in_flow)
                    flow_stop <= ~flow_stop;
            end
        end else begin
            key4_cnt  <= {LONG_WIDTH{1'b0}};
            key4_long <= 1'b0;
            if (key4_down_d && !key4_long && in_flow)
                play_tgl <= ~play_tgl;
        end
    end
end

assign flow_flags = {play_tgl, prev_tgl, next_tgl, in_flow};
assign menu_state = {1'b0, sleep_on, (mode == M_CAM), (in_vol || in_bright),
                     (in_menu || in_vol || in_bright), cursor};

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
