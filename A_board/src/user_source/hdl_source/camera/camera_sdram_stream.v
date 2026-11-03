`timescale 1ns/1ps

module camera_sdram_stream #(
    parameter integer FRAME_PIXELS = 307200,
    parameter integer TIMEOUT_CYCLES = 10_000_000
) (
    input  wire        sd_clk,
    input  wire        sd_rst_n,
    input  wire        cam_pclk,
    input  wire        mode_request,
    input  wire        tf_write_idle,
    input  wire        write_req_ack,
    input  wire        write_finish_toggle,
    input  wire        write_fifo_full,
    input  wire        display_ack_toggle,
    input  wire        pixel_valid,
    input  wire [15:0] pixel_rgb565,
    input  wire        frame_start_toggle,
    input  wire        frame_done_toggle,
    output wire        pause_tf,
    output wire        camera_owner,
    output wire        capture_allowed,
    output wire        write_req,
    output wire        write_en,
    output wire [31:0] write_data,
    output reg         write_abort_toggle,
    output reg         write_bank,
    output reg         published_bank,
    output reg         published_toggle
);
    localparam [3:0] OFF = 0, WAIT_TF = 1, WAIT_FRAME = 2,
                     REQUEST = 3, REQUEST_RELEASE = 4, STREAM = 5,
                     WAIT_FINISH = 6, WAIT_DISPLAY = 7, ABORT = 8;
    reg [3:0] state;
    reg [4:0] settle_count;
    reg [23:0] timeout_count;
    reg [18:0] sent_count;
    reg source_done_seen;
    reg write_done_seen;
    (* ASYNC_REG = "TRUE" *) reg [2:0] start_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] done_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] finish_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] ack_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] overflow_sync;
    (* ASYNC_REG = "TRUE" *) reg [2:0] req_ack_sync;
    wire req_ack = req_ack_sync[2];
    reg ack_seen;
    reg overflow_toggle;
    reg overflow_reported;
    wire fifo_full, fifo_empty;
    wire [31:0] fifo_data;
    wire fifo_reset = !sd_rst_n || (state == OFF) ||
                      (state == WAIT_TF) || (state == WAIT_FRAME) ||
                      (state == ABORT);
    wire start_pulse = start_sync[2] ^ start_sync[1];
    wire done_pulse = done_sync[2] ^ done_sync[1];
    wire finish_pulse = finish_sync[2] ^ finish_sync[1];
    wire overflow_pulse = overflow_sync[2] ^ overflow_sync[1];
    wire timed_out = timeout_count >= TIMEOUT_CYCLES;
    wire fifo_pop = (state == STREAM) && !fifo_empty &&
                    !write_fifo_full && (sent_count < FRAME_PIXELS);

    assign camera_owner = (state != OFF) && (state != WAIT_TF);
    assign pause_tf = mode_request || camera_owner;
    assign capture_allowed = (state == WAIT_FRAME);
    assign write_req = (state == REQUEST);
    assign write_en = fifo_pop;
    assign write_data = fifo_data;

    wire [7:0] red = {pixel_rgb565[15:11], {3{pixel_rgb565[11]}}};
    wire [7:0] green = {pixel_rgb565[10:5], {2{pixel_rgb565[5]}}};
    wire [7:0] blue = {pixel_rgb565[4:0], {3{pixel_rgb565[0]}}};
    wire fifo_push = pixel_valid && !fifo_full && !fifo_reset;

    wfifo_32_32_512 u_camera_cross_clock_fifo (
        .rst(fifo_reset), .clkw(cam_pclk), .clkr(sd_clk),
        .we(fifo_push), .di({red, green, blue, 8'd0}),
        .re(fifo_pop), .dout(fifo_data),
        .valid(), .full_flag(fifo_full), .empty_flag(fifo_empty),
        .afull(), .aempty(), .wrusedw(), .rdusedw()
    );

    always @(posedge cam_pclk or negedge sd_rst_n) begin
        if (!sd_rst_n) begin
            overflow_toggle <= 1'b0;
            overflow_reported <= 1'b0;
        end else if (fifo_reset) begin
            overflow_reported <= 1'b0;
        end else if (pixel_valid && fifo_full && !overflow_reported) begin
            overflow_toggle <= ~overflow_toggle;
            overflow_reported <= 1'b1;
        end
    end

    always @(posedge sd_clk or negedge sd_rst_n) begin
        if (!sd_rst_n) begin
            state <= OFF;
            settle_count <= 5'd0;
            timeout_count <= 24'd0;
            sent_count <= 19'd0;
            source_done_seen <= 1'b0;
            write_done_seen <= 1'b0;
            start_sync <= 3'b000;
            done_sync <= 3'b000;
            finish_sync <= 3'b000;
            ack_sync <= 3'b000;
            overflow_sync <= 3'b000;
            req_ack_sync <= 3'b000;
            ack_seen <= 1'b0;
            write_abort_toggle <= 1'b0;
            write_bank <= 1'b0;
            published_bank <= 1'b0;
            published_toggle <= 1'b0;
        end else begin
            start_sync <= {start_sync[1:0], frame_start_toggle};
            done_sync <= {done_sync[1:0], frame_done_toggle};
            finish_sync <= {finish_sync[1:0], write_finish_toggle};
            ack_sync <= {ack_sync[1:0], display_ack_toggle};
            overflow_sync <= {overflow_sync[1:0], overflow_toggle};
            req_ack_sync <= {req_ack_sync[1:0], write_req_ack};

            if (state == REQUEST || state == REQUEST_RELEASE ||
                state == STREAM || state == WAIT_FINISH)
                timeout_count <= timeout_count + 1'b1;
            else
                timeout_count <= 24'd0;

            if (state == REQUEST || state == REQUEST_RELEASE ||
                state == STREAM || state == WAIT_FINISH) begin
                if (done_pulse)
                    source_done_seen <= 1'b1;
                if (finish_pulse)
                    write_done_seen <= 1'b1;
            end

            case (state)
                OFF: if (mode_request) state <= WAIT_TF;
                WAIT_TF: begin
                    if (!mode_request)
                        state <= OFF;
                    else if (tf_write_idle) begin
                        ack_seen <= ack_sync[2];
                        state <= WAIT_FRAME;
                    end
                end
                WAIT_FRAME: begin
                    if (!mode_request)
                        state <= OFF;
                    else if (start_pulse) begin
                        sent_count <= 19'd0;
                        source_done_seen <= 1'b0;
                        write_done_seen <= 1'b0;
                        state <= REQUEST;
                    end
                end
                REQUEST: begin
                    if (!mode_request || overflow_pulse || timed_out) begin
                        write_abort_toggle <= ~write_abort_toggle;
                        settle_count <= 5'd0;
                        state <= ABORT;
                    end else if (req_ack) begin
                        settle_count <= 5'd0;
                        state <= REQUEST_RELEASE;
                    end
                end
                REQUEST_RELEASE: begin
                    if (!mode_request || overflow_pulse || timed_out) begin
                        write_abort_toggle <= ~write_abort_toggle;
                        settle_count <= 5'd0;
                        state <= ABORT;
                    end else if (settle_count == 5'd15) begin
                        state <= STREAM;
                    end else
                        settle_count <= settle_count + 1'b1;
                end
                STREAM: begin
                    if (!mode_request || overflow_pulse || timed_out) begin
                        write_abort_toggle <= ~write_abort_toggle;
                        settle_count <= 5'd0;
                        state <= ABORT;
                    end else if (sent_count == FRAME_PIXELS)
                        state <= WAIT_FINISH;
                    else if (fifo_pop)
                        sent_count <= sent_count + 1'b1;
                end
                WAIT_FINISH: begin
                    if (!mode_request || overflow_pulse || timed_out) begin
                        write_abort_toggle <= ~write_abort_toggle;
                        settle_count <= 5'd0;
                        state <= ABORT;
                    end else if (source_done_seen && write_done_seen) begin
                        published_bank <= write_bank;
                        published_toggle <= ~published_toggle;
                        state <= WAIT_DISPLAY;
                    end
                end
                WAIT_DISPLAY: begin
                    if (!mode_request)
                        state <= OFF;
                    else if (ack_sync[2] != ack_seen) begin
                        ack_seen <= ack_sync[2];
                        write_bank <= ~write_bank;
                        state <= WAIT_FRAME;
                    end
                end
                ABORT: begin
                    if (settle_count == 5'd31) begin
                        if (mode_request)
                            state <= WAIT_FRAME;
                        else
                            state <= OFF;
                    end else
                        settle_count <= settle_count + 1'b1;
                end
                default: state <= OFF;
            endcase
        end
    end
endmodule

module camera_video_select (
    input  wire video_clk,
    input  wire video_rst_n,
    input  wire mode_request_async,
    input  wire published_toggle_async,
    input  wire published_bank_async,
    input  wire raw_vsync,
    input  wire delayed_vsync,
    output reg  read_mode,
    output reg  read_bank,
    output reg  video_mode,
    output reg  frame_available,
    output reg  display_ack_toggle
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] mode_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] publish_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] bank_sync;
    reg publish_seen;
    reg read_mode_active;
    reg publish_pending;
    reg raw_vsync_q, delayed_vsync_q;
    always @(posedge video_clk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            mode_sync <= 2'b00;
            publish_sync <= 2'b00;
            bank_sync <= 2'b00;
            publish_seen <= 1'b0;
            read_mode_active <= 1'b0;
            publish_pending <= 1'b0;
            raw_vsync_q <= 1'b1;
            delayed_vsync_q <= 1'b1;
            read_mode <= 1'b0;
            read_bank <= 1'b0;
            video_mode <= 1'b0;
            frame_available <= 1'b0;
            display_ack_toggle <= 1'b0;
        end else begin
            mode_sync <= {mode_sync[0], mode_request_async};
            publish_sync <= {publish_sync[0], published_toggle_async};
            bank_sync <= {bank_sync[0], published_bank_async};
            raw_vsync_q <= raw_vsync;
            delayed_vsync_q <= delayed_vsync;
            if (raw_vsync_q && !raw_vsync) begin
                // The SDRAM reader accepted this frame's request before the
                // delayed VS reaches here. Address changes apply next frame.
                read_mode_active <= read_mode;
                read_mode <= mode_sync[1];
                if (!mode_sync[1]) begin
                    if (!read_mode)
                        frame_available <= 1'b0;
                    publish_seen <= publish_sync[1];
                    publish_pending <= 1'b0;
                end else if (publish_pending) begin
                    frame_available <= 1'b1;
                    display_ack_toggle <= ~display_ack_toggle;
                    publish_pending <= 1'b0;
                end else if (publish_sync[1] != publish_seen) begin
                    publish_seen <= publish_sync[1];
                    read_bank <= bank_sync[1];
                    publish_pending <= 1'b1;
                end
            end
            if (delayed_vsync_q && !delayed_vsync)
                video_mode <= read_mode_active;
        end
    end
endmodule
