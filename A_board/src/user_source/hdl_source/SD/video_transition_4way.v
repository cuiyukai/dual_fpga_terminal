`timescale 1ns/1ps

module video_transition_4way #(
    parameter integer FADE_FRAMES = 16,
    parameter integer FLIP_FRAMES = 32,
    parameter integer FRAME_WIDTH = 640,
    parameter integer FRAME_HEIGHT = 480
) (
    input  wire        ctrl_clk,
    input  wire        ctrl_rst_n,
    input  wire        switch_start,
    input  wire [1:0]  switch_target,
    input  wire        index_commit,
    output wire        switch_commit,
    output wire        transition_busy,

    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        player_active,
    input  wire        vsync,
    input  wire        de,
    input  wire [23:0] rgb_in,
    output wire [23:0] rgb_out,
    output wire [1:0]  transition_kind,

    output reg         flip_read_en,
    output reg  [9:0]  flip_read_fold
);
    localparam integer ALPHA_STEP = 256 / FADE_FRAMES;
    localparam integer BLIND_BITS = 5;
    localparam integer BLIND_STEP = ((1 << BLIND_BITS) + FADE_FRAMES - 1) /
                                    FADE_FRAMES;
    localparam integer FLIP_STEP  = FRAME_WIDTH / FLIP_FRAMES;
    localparam integer FLIP_HALF  = FRAME_WIDTH / 2;

    localparam [2:0] S_IDLE   = 3'd0;
    localparam [2:0] S_OUT    = 3'd1;
    localparam [2:0] S_HOLD   = 3'd2;
    localparam [2:0] S_REVEAL = 3'd3;
    localparam [2:0] S_FLIP   = 3'd4;

    reg req_toggle;
    reg [1:0] target_hold;
    reg index_toggle;
    reg index_commit_d;
    reg commit_seen;
    (* ASYNC_REG = "TRUE" *) reg [1:0] commit_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] busy_sync;

    reg [2:0] state;
    reg [5:0] ramp;
    reg [1:0] target_latched;
    reg commit_toggle;
    // request toggle is seen.
    (* ASYNC_REG = "TRUE" *) reg [2:0] req_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] target_sync0;
    (* ASYNC_REG = "TRUE" *) reg [1:0] target_sync1;
    (* ASYNC_REG = "TRUE" *) reg [1:0] index_sync;
    reg req_seen;
    reg index_seen;
    reg index_pending;
    reg [1:0] hold_frames;

    reg vs_q;
    reg de_q;
    reg [9:0] pixel_x;
    reg [9:0] pixel_y;

    reg       flip_on;
    reg [9:0] flip_fold;     // columns >= flip_fold show the incoming page
    reg [9:0] flap_left;     // the lifted old page covers [flap_left, flip_fold)
    reg [3:0] shadow_dark;   // min(flap width, 12)
    reg [5:0] shadow_wide;   // min(flap width, 32)

    wire frame_start = vs_q && !vsync;
    wire frame_end   = de_q && !de && (pixel_y == FRAME_HEIGHT - 1);
    wire transition_running = (state != S_IDLE);
    wire flip_kind = (target_latched == 2'd1);
    wire hold_tick = flip_kind ? frame_end : frame_start;

    // around it and clipped at the left screen edge.
    wire [9:0] next_flap_w = (flip_read_fold < FLIP_HALF) ? flip_read_fold :
                             FRAME_WIDTH - flip_read_fold;

    assign switch_commit = commit_sync[1] != commit_seen;
    assign transition_busy = busy_sync[1];
    assign transition_kind = target_latched;

    // one-cycle event can be lost while crossing clock domains.
    always @(posedge ctrl_clk or negedge ctrl_rst_n) begin
        if (!ctrl_rst_n) begin
            req_toggle  <= 1'b0;
            target_hold <= 2'd0;
            index_toggle <= 1'b0;
            index_commit_d <= 1'b0;
            commit_seen <= 1'b0;
            commit_sync <= 2'b00;
            busy_sync   <= 2'b00;
        end else begin
            commit_sync <= {commit_sync[0], commit_toggle};
            busy_sync   <= {busy_sync[0], transition_running};
            index_commit_d <= index_commit;

            if (switch_start && !busy_sync[1]) begin
                target_hold <= switch_target;
                req_toggle  <= ~req_toggle;
            end

            if (index_commit && !index_commit_d) begin
                index_toggle <= ~index_toggle;
                commit_seen  <= commit_sync[1];
            end
        end
    end

    // only advance at frame boundaries.
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            state          <= S_IDLE;
            ramp           <= 6'd0;
            target_latched <= 2'd0;
            commit_toggle  <= 1'b0;
            req_sync       <= 3'b000;
            target_sync0   <= 2'd0;
            target_sync1   <= 2'd0;
            index_sync     <= 2'b00;
            req_seen       <= 1'b0;
            index_seen     <= 1'b0;
            index_pending  <= 1'b0;
            hold_frames    <= 2'd0;
            flip_read_en   <= 1'b0;
            flip_read_fold <= 10'd0;
            vs_q            <= 1'b1;
            de_q            <= 1'b0;
            pixel_x         <= 10'd0;
            pixel_y         <= 10'd0;
            flip_on         <= 1'b0;
            flip_fold       <= 10'd0;
            flap_left       <= 10'd0;
            shadow_dark     <= 4'd0;
            shadow_wide     <= 6'd0;
        end else begin
            req_sync     <= {req_sync[1:0], req_toggle};
            target_sync0 <= target_hold;
            target_sync1 <= target_sync0;
            index_sync   <= {index_sync[0], index_toggle};
            vs_q         <= vsync;
            de_q         <= de;

            if (!de)
                pixel_x <= 10'd0;
            else
                pixel_x <= pixel_x + 1'b1;

            if (!vsync)
                pixel_y <= 10'd0;
            else if (de_q && !de)
                pixel_y <= pixel_y + 1'b1;

            if (frame_start) begin
                flip_on     <= flip_read_en;
                flip_fold   <= flip_read_fold;
                flap_left   <= flip_read_fold - next_flap_w;
                shadow_dark <= (next_flap_w < 10'd12) ? next_flap_w[3:0] : 4'd12;
                shadow_wide <= (next_flap_w < 10'd32) ? next_flap_w[5:0] : 6'd32;
            end

            if ((state == S_HOLD) && (index_sync[1] != index_seen)) begin
                index_seen    <= index_sync[1];
                index_pending <= 1'b1;
                hold_frames   <= 2'd0;
            end

            case (state)
                S_IDLE: begin
                    if (frame_end)
                        flip_read_en <= 1'b0;
                    if (req_sync[2] != req_seen) begin
                        req_seen <= req_sync[2];
                        if (player_active) begin
                            target_latched <= target_sync1;
                            if (target_sync1 == 2'd1) begin
                                // Non-zero ramp: the first frame end only
                                // arms the reader.
                                ramp  <= 6'd1;
                                state <= S_FLIP;
                            end else begin
                                ramp  <= FADE_FRAMES;
                                state <= S_OUT;
                            end
                        end
                    end
                end

                S_OUT: begin
                    if (frame_start) begin
                        if (ramp == 0) begin
                            state <= S_HOLD;
                            commit_toggle <= ~commit_toggle;
                        end else begin
                            ramp <= ramp - 1'b1;
                        end
                    end
                end

                S_FLIP: begin
                    if (frame_end) begin
                        if (ramp != 0) begin
                            ramp           <= 6'd0;
                            flip_read_en   <= 1'b1;
                            flip_read_fold <= FRAME_WIDTH - FLIP_STEP;
                        end else if (flip_read_fold > FLIP_STEP) begin
                            flip_read_fold <= flip_read_fold - FLIP_STEP;
                        end else begin
                            // The incoming page now fills the screen.
                            flip_read_fold <= 10'd0;
                            commit_toggle  <= ~commit_toggle;
                            state          <= S_HOLD;
                        end
                    end
                end

                S_HOLD: begin
                    if (hold_tick && index_pending) begin
                        if (hold_frames == 2'd1) begin
                            index_pending <= 1'b0;
                            hold_frames   <= 2'd0;
                            ramp  <= 6'd0;
                            if (flip_kind) begin
                                flip_read_en <= 1'b0;
                                state <= S_IDLE;
                            end else begin
                                state <= S_REVEAL;
                            end
                        end else begin
                            hold_frames <= hold_frames + 1'b1;
                        end
                    end
                end

                S_REVEAL: begin
                    if (frame_start) begin
                        if (ramp >= FADE_FRAMES - 1) begin
                            state <= S_IDLE;
                            ramp  <= 6'd0;
                        end else begin
                            ramp <= ramp + 1'b1;
                        end
                    end
                end

                default: begin
                    state <= S_IDLE;
                    ramp  <= 6'd0;
                    flip_read_en <= 1'b0;
                end
            endcase
        end
    end

    wire [13:0] alpha_calc = ramp * ALPHA_STEP;
    wire [7:0] fade_alpha = (ramp >= FADE_FRAMES) ? 8'hff :
                            alpha_calc[7:0];
    wire [15:0] fade_r_product = rgb_in[23:16] * fade_alpha;
    wire [15:0] fade_g_product = rgb_in[15:8]  * fade_alpha;
    wire [15:0] fade_b_product = rgb_in[7:0]   * fade_alpha;
    wire [23:0] faded_rgb = (fade_alpha == 8'hff) ? rgb_in :
                            {fade_r_product[15:8], fade_g_product[15:8],
                             fade_b_product[15:8]};

    wire [6:0] reveal_step = ramp + 1'b1;
    wire [15:0] blind_boundary = reveal_step * BLIND_STEP;
    wire blind_visible = pixel_y[BLIND_BITS-1:0] < blind_boundary;

    // carries into the next channel.
    wire [23:0] rgb_half    = {1'b0, rgb_in[23:17], 1'b0, rgb_in[15:9],
                               1'b0, rgb_in[7:1]};
    wire [23:0] rgb_quarter = {2'b00, rgb_in[23:18], 2'b00, rgb_in[15:10],
                               2'b00, rgb_in[7:2]};
    wire [23:0] rgb_3q      = rgb_half + rgb_quarter;

    wire       on_new_page = pixel_x >= flip_fold;
    wire       on_flap     = pixel_x >= flap_left;
    wire [9:0] new_dist    = pixel_x - flip_fold;
    wire [9:0] flap_dist   = flip_fold - 1'b1 - pixel_x;
    wire [9:0] old_dist    = flap_left - pixel_x;
    wire [7:0] paper_l     = 8'hb0 + ((flap_dist > 10'd63) ? 8'd63 : flap_dist[7:0]);

    reg [23:0] flip_rgb;
    always @* begin
        if (on_new_page)
            flip_rgb = (new_dist < shadow_dark) ? rgb_half :
                       (new_dist < shadow_wide) ? rgb_3q : rgb_in;
        else if (on_flap)
            flip_rgb = {paper_l, paper_l, paper_l - 8'd12};
        else
            flip_rgb = (old_dist <= 10'd8) ? rgb_3q : rgb_in;
    end

    reg [23:0] transition_rgb;
    always @* begin
        if (flip_on) begin
            transition_rgb = flip_rgb;
        end else begin
            case (state)
                S_OUT:    transition_rgb = faded_rgb;
                S_HOLD:   transition_rgb = 24'd0;
                S_REVEAL: transition_rgb = (target_latched == 2'd2) ?
                                           (blind_visible ? rgb_in : 24'd0) :
                                           faded_rgb;
                default:  transition_rgb = rgb_in;
            endcase
        end
    end

    assign rgb_out = de ? transition_rgb : rgb_in;
endmodule
