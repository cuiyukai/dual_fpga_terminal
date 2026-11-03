`timescale 1ns/1ps

module ov5640_camera #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer WIDTH  = 320,
    parameter integer HEIGHT = 240
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        probe,
    input  wire        sleep,          // board sleep: PWDN high, SCCB held in reset
    input  wire        capture_allowed,
    input  wire        cam_pclk,
    input  wire        cam_vsync,
    input  wire        cam_href,
    input  wire [7:0]  cam_data,
    output wire        cam_pwdn,
    output wire        cam_reset_n,
    output wire        cam_scl,
    inout  wire        cam_sda,
    output wire        init_done,
    output reg  [3:0]  readback_code,
    output reg         pixel_valid,
    output reg  [15:0] pixel_rgb565,
    output reg  [18:0] pixel_addr,
    output reg         frame_start_toggle,
    output reg         frame_done_toggle
);
    localparam integer FRAME_PIXELS = WIDTH * HEIGHT;

    wire       config_reset_n = rst_n && !sleep && (enable || probe);

    wire        sccb_exec;
    wire [23:0] sccb_data;
    wire        sccb_config_read;
    wire        sccb_done;
    wire        sccb_drive_clk;
    wire [7:0]  sccb_read_data;
    reg  [2:0]  readback_index;
    reg  [1:0]  readback_state;
    reg  [16:0] readback_timeout;
    localparam [1:0] RB_WAIT = 2'd0, RB_START = 2'd1,
                     RB_BUSY = 2'd2, RB_FINISHED = 2'd3;
    wire readback_exec = (readback_state == RB_START);
    wire [15:0] readback_addr =
        (readback_index == 3'd0) ? 16'h300a :
        (readback_index == 3'd1) ? 16'h300b :
        (readback_index == 3'd2) ? 16'h3017 :
        (readback_index == 3'd3) ? 16'h301d : 16'h4740;

    reg  [1:0] enable_pclk_sync;
    reg  [1:0] init_pclk_sync;
    reg  [1:0] capture_allowed_sync;
    reg        cam_vsync_d0;
    reg        cam_vsync_d1;
    wire       pos_vsync = (~cam_vsync_d1) & cam_vsync_d0;
    reg        byte_phase;
    reg  [7:0] first_byte;
    reg  [18:0] frame_pixel_count;
    reg         capture_armed;
    reg         frame_accept;
    reg  [3:0]  settle_frames;

    // board sleep mode (sensor standby).
    assign cam_pwdn    = sleep;
    assign cam_reset_n = 1'b1;

    ov5640_rgb565_config #(
        .CMOS_H_PIXEL(WIDTH),
        .CMOS_V_PIXEL(HEIGHT)
    ) u_config (
        .clk      (sccb_drive_clk),
        .rst_n    (config_reset_n),
        .i2c_done (sccb_done),
        .i2c_exec (sccb_exec),
        .i2c_data (sccb_data),
        .i2c_rh_wl(sccb_config_read),
        .init_done(init_done)
    );

    ov5640_sccb_master #(
        .SLAVE_ADDR(7'h3c),
        .CLK_FREQ  (CLK_HZ),
        .I2C_FREQ  (250_000)
    ) u_sccb (
        .clk       (clk),
        .rst_n     (config_reset_n),
        .i2c_exec  (init_done ? readback_exec : sccb_exec),
        .bit_ctrl  (1'b1),
        .i2c_rh_wl (init_done ? 1'b1 : sccb_config_read),
        .i2c_addr  (init_done ? readback_addr : sccb_data[23:8]),
        .i2c_data_w(sccb_data[7:0]),
        .i2c_data_r(sccb_read_data),
        .i2c_done  (sccb_done),
        .scl       (cam_scl),
        .sda       (cam_sda),
        .dri_clk   (sccb_drive_clk)
    );

    always @(posedge sccb_drive_clk or negedge config_reset_n) begin
        if (!config_reset_n) begin
            readback_index   <= 3'd0;
            readback_state   <= RB_WAIT;
            readback_timeout <= 17'd0;
            readback_code    <= 4'd0;
        end else if (readback_state != RB_FINISHED && init_done) begin
            if (readback_timeout == 17'd100000) begin
                readback_code  <= 4'd6;
                readback_state <= RB_FINISHED;
            end else begin
                readback_timeout <= readback_timeout + 1'b1;
                case (readback_state)
                    RB_WAIT: if (!sccb_done) readback_state <= RB_START;
                    RB_START: readback_state <= RB_BUSY;
                    RB_BUSY: if (sccb_done) begin
                        if ((readback_index == 3'd0 && sccb_read_data != 8'h56) ||
                            (readback_index == 3'd1 && sccb_read_data != 8'h40)) begin
                            readback_code  <= 4'd1;
                            readback_state <= RB_FINISHED;
                        end else if (readback_index == 3'd2 &&
                                     sccb_read_data[6:4] != 3'b111) begin
                            readback_code  <= 4'd2;
                            readback_state <= RB_FINISHED;
                        end else if (readback_index == 3'd3 &&
                                     sccb_read_data[6:4] != 3'b000) begin
                            readback_code  <= 4'd3;
                            readback_state <= RB_FINISHED;
                        end else if (readback_index == 3'd4) begin
                            readback_code  <= (sccb_read_data == 8'h20) ? 4'd5 : 4'd4;
                            readback_state <= RB_FINISHED;
                        end else begin
                            readback_index <= readback_index + 1'b1;
                            readback_state <= RB_WAIT;
                        end
                    end
                    default: readback_state <= RB_WAIT;
                endcase
            end
        end
    end

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            enable_pclk_sync <= 2'b00;
            init_pclk_sync   <= 2'b00;
            capture_allowed_sync <= 2'b00;
            cam_vsync_d0     <= 1'b0;
            cam_vsync_d1     <= 1'b0;
            byte_phase       <= 1'b0;
            first_byte       <= 8'd0;
            frame_pixel_count <= 19'd0;
            pixel_addr       <= 19'd0;
            pixel_valid      <= 1'b0;
            pixel_rgb565     <= 16'd0;
            frame_start_toggle <= 1'b0;
            frame_done_toggle <= 1'b0;
            capture_armed    <= 1'b0;
            frame_accept     <= 1'b0;
            settle_frames    <= 4'd0;
        end else begin
            enable_pclk_sync <= {enable_pclk_sync[0], enable};
            init_pclk_sync   <= {init_pclk_sync[0], init_done};
            capture_allowed_sync <= {capture_allowed_sync[0], capture_allowed};
            cam_vsync_d0     <= cam_vsync;
            cam_vsync_d1     <= cam_vsync_d0;
            pixel_valid      <= 1'b0;

            if (!enable_pclk_sync[1] || !init_pclk_sync[1]) begin
                byte_phase        <= 1'b0;
                frame_pixel_count <= 19'd0;
                pixel_addr        <= 19'd0;
                frame_start_toggle <= 1'b0;
                frame_done_toggle <= 1'b0;
                capture_armed     <= 1'b0;
                frame_accept      <= 1'b0;
                settle_frames     <= 4'd0;
            end else begin
                if (pos_vsync) begin
                    byte_phase <= 1'b0;
                    if (settle_frames < 10)
                        settle_frames <= settle_frames + 1'b1;
                    else
                        capture_armed <= 1'b1;

                    // Publish only after every address in the frame was written.
                    if (frame_accept && frame_pixel_count == FRAME_PIXELS)
                        frame_done_toggle <= ~frame_done_toggle;
                    frame_accept <= capture_armed && capture_allowed_sync[1];
                    if (capture_armed && capture_allowed_sync[1])
                        frame_start_toggle <= ~frame_start_toggle;
                    frame_pixel_count <= 19'd0;
                    pixel_addr        <= 19'd0;
                end else if (frame_accept && cam_href) begin
                    if (!byte_phase) begin
                        first_byte <= cam_data;
                        byte_phase <= 1'b1;
                    end else begin
                        byte_phase <= 1'b0;
                        if (frame_pixel_count < FRAME_PIXELS) begin
                            pixel_rgb565 <= {first_byte, cam_data};
                            pixel_addr   <= frame_pixel_count;
                            pixel_valid  <= 1'b1;
                            frame_pixel_count <= frame_pixel_count + 1'b1;
                        end
                    end
                end else if (!cam_href) begin
                    byte_phase <= 1'b0;
                end
            end
        end
    end
endmodule
