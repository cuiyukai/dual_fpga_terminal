`timescale 1ns/1ps

module flow_checklist (
    input  wire       clk,
    input  wire       rst,
    input  wire       flow_mode,     // 流程模式电平（已同步）
    input  wire       page_ready,    // 页面显示中且无转场 / 摄像头 / 休眠
    input  wire [2:0] page,          // 当前显示页 disp_buf_idx
    input  wire       knob_cw,       // 单拍
    input  wire       knob_ccw,
    input  wire       knob_press,
    output wire [1:0] lock,          // {第 2 步未勾全, 第 1 步未勾全}
    output wire       osd_show,      // 第 1 / 2 步页面可显示
    output wire       osd_step2,     // 0 第 1 步 / 1 第 2 步
    output wire [1:0] osd_cursor,
    output wire [3:0] osd_mask       // 当前页的勾，第 2 步只用低 3 位
);
    reg [3:0] mask1;
    reg [2:0] mask2;
    reg [1:0] cursor;

    wire       on_step = flow_mode && page_ready && (page[2:1] == 2'b01);
    wire       step2   = page[0];
    wire [1:0] last    = step2 ? 2'd2 : 2'd3;
    wire [3:0] sel     = 4'b0001 << cursor;

    assign lock       = {mask2 != 3'b111, mask1 != 4'b1111};
    assign osd_show   = on_step;
    assign osd_step2  = step2;
    assign osd_cursor = cursor;
    assign osd_mask   = step2 ? {1'b0, mask2} : mask1;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mask1  <= 4'd0;
            mask2  <= 3'd0;
            cursor <= 2'd0;
        end else begin
            if (!flow_mode) begin
                mask1  <= 4'd0;
                mask2  <= 3'd0;
                cursor <= 2'd0;
            end else if (!on_step) begin
                cursor <= 2'd0;
            end else begin
                if (knob_press) begin
                    if (step2)
                        mask2 <= mask2 ^ sel[2:0];
                    else
                        mask1 <= mask1 ^ sel;
                end else if (knob_cw)
                    cursor <= (cursor == last) ? 2'd0 : cursor + 2'd1;
                else if (knob_ccw)
                    cursor <= (cursor == 2'd0) ? last : cursor - 2'd1;
            end
        end
    end
endmodule
