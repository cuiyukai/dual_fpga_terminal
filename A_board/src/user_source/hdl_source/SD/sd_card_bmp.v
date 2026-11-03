module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd6,
    parameter integer LONG_PRESS_MS     = 1000,
    parameter integer AUTO_PLAY_MS      = 3000,
    parameter integer LOAD_TIMEOUT_MS   = 5000,
    parameter integer SD_STALL_TIMEOUT_MS = 500,
    parameter integer MAX_RETRIES       = 1,
    parameter integer FRAME_PIXELS      = 307200,
    parameter integer SD_RESET_CYCLES   = 16
)(
    input                       clk,
    input                       rst,
    input                       system_ready,
    input                       key_next,
    input                       key_auto,
    input                       screen_next_pulse,
    input                       pause_background,
    input                       camera_mode_active,
    input                       flow_mode,
    input                       test_mode,           // 实体测试：固定第 2 页
    input                       flow_next_pulse,
    input                       flow_prev_pulse,
    input                       flow_play_pulse,
    input                       flow_stop_pulse,     // KEY4 长按：停止轮播，停在当前页
    input       [1:0]           flow_lock,
    output                      write_path_idle,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    input                       write_fifo_full,
    output                      disp_res_valid,
    output [10:0]               disp_res_w,
    output [10:0]               disp_res_h,
    output reg                  display_valid,
    output      [3:0]           state_code,
    output                      osd_auto_enabled,
    output      [7:0]           osd_error_code,

    input                       write_finish_toggle,
    output reg                  write_abort_toggle,
    output reg [2:0]            write_buf_idx,
    output reg [2:0]            disp_buf_idx,

    input                       transition_busy,
    input                       transition_commit,
    output reg                  transition_start,
    input  [1:0]                transition_mode,     // 翻页转场种类（本时钟域电平），每次发起转场时锁存
    output reg [1:0]            transition_target,
    output reg                  transition_index_commit,
    output [2:0]                transition_dest_idx, // 转场目标页：翻书期间读模块提前从该 bank 取新页

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,

    input                       music_enable,
    input                       music_hold,
    input                       alarm_select,
    output                      music_byte_valid,
    output [7:0]                music_byte,
    output                      music_byte_hi,   // 1=16bit 样本的高字节
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);

wire key_next_press;
wire key_auto_press;
wire key_auto_stable;
reg  key_auto_stable_d;
wire key_auto_release;

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire [3:0]       state_code_i;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire             sd_init_done;
wire             bmp_ready;
wire             scan_busy;
wire             scan_done;
wire [7:0]       scan_fs_error;
wire [7:0]       scan_spc;
wire [31:0]      scan_fat_start;
wire [31:0]      scan_data_start;
wire [31:0]      scan_max_cluster;
wire [5:0]       scan_resource_valid;
wire [31:0]      scan_cluster0;
wire [31:0]      scan_cluster1;
wire [31:0]      scan_cluster2;
wire [31:0]      scan_cluster3;
wire [31:0]      scan_cluster4;
wire [31:0]      scan_cluster5;
wire [31:0]      scan_size0;
wire [31:0]      scan_size1;
wire [31:0]      scan_size2;
wire [31:0]      scan_size3;
wire [31:0]      scan_size4;
wire [31:0]      scan_size5;
wire             bmp_load_done;
wire             bmp_load_error;
wire [10:0]      bmp_src_w, bmp_src_h;
reg  [21:0]      res2, res3, res4, res5;   // bank 2~5 原分辨率 {W, H}
wire [2:0]       bmp_error;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_cluster;
reg [31:0]       load_file_size;
reg              scan_kicked;
reg              catalog_valid;
reg              first_image_committed;
reg              auto_play_en;
reg [28:0]       auto_cnt;
reg [2:0]        img_found_count;
reg [2:0]        img_idx;              // 当前真正显示中的图片编号
reg [2:0]        load_idx;             // 当前正在写入的图片编号
reg [5:0]        img_valid_mask;
reg [5:0]        bank_loaded_mask;
wire [21:0]      res_sel = (disp_buf_idx == 3'd2) ? res2 : (disp_buf_idx == 3'd3) ? res3 :
                           (disp_buf_idx == 3'd4) ? res4 : res5;
assign disp_res_valid = display_valid && (disp_buf_idx >= 3'd2) && (disp_buf_idx <= 3'd5) &&
                        bank_loaded_mask[disp_buf_idx];
assign disp_res_w = res_sel[21:11];
assign disp_res_h = res_sel[10:0];
reg [5:0]        bank_failed_mask;
reg [31:0]       img_cluster0;
reg [31:0]       img_cluster1;
reg [31:0]       img_cluster2;
reg [31:0]       img_cluster3;
reg [31:0]       img_cluster4;
reg [31:0]       img_cluster5;
reg [31:0]       img_size0;
reg [31:0]       img_size1;
reg [31:0]       img_size2;
reg [31:0]       img_size3;
reg [31:0]       img_size4;
reg [31:0]       img_size5;
reg [7:0]        catalog_spc;
reg [31:0]       catalog_fat_start;
reg [31:0]       catalog_data_start;
reg [31:0]       catalog_max_cluster;
reg              next_req_pending;
reg              prev_req_pending;
reg              transition_inflight;
reg [2:0]        transition_active_target;
assign transition_dest_idx = transition_active_target;
reg              camera_mode_d;
reg [1:0]        lock_sel_d;
reg              flow_play_en;         // 流程轮播进行中
reg              flow_req_pending;     // 流程跳页请求，目标缓存好后启动转场
reg [2:0]        flow_target;
reg [31:0]       long_press_cnt;
reg              long_press_marked;
reg              load_busy;
reg              load_prepare;
reg [31:0]       load_timeout_cnt;

reg              load_abort;
reg              write_session_started;
reg [3:0]        retry_count;
reg              retry_pending;
reg [5:0]        retry_delay_cnt;
reg [7:0]        sd_reset_cnt;

reg              source_done_seen;
reg              write_done_seen;

// 同步 mem_clk 域的 write_finish_toggle
(* ASYNC_REG = "TRUE" *) reg [2:0] wrfin_tgl_sync;
wire             write_finish_pulse;

(* ASYNC_REG = "TRUE" *) reg [2:0] write_req_ack_sync;

localparam integer AUTO_PLAY_CYCLES = (CLK_FREQ_HZ / 1000) * AUTO_PLAY_MS;
localparam integer LOAD_TIMEOUT_CYCLES = (CLK_FREQ_HZ / 1000) * LOAD_TIMEOUT_MS;
wire auto_tick;
wire [2:0] next_from_current;
wire [1:0] lock_sel;
wire       lock_mode;
wire [2:0] camera_fallback_idx;
wire [2:0] flow_cur;
wire       flow_in_ring;
wire [2:0] flow_ring_next;
wire [2:0] flow_ring_prev;
wire       flow_next_ok;
wire       flow_prev_ok;
wire [2:0] bank_loaded_count;
wire [5:0] available_bank_mask;
wire [5:0] preload_pending_mask;
wire load_total_timeout;
wire load_failure_event;
wire load_failure_needs_sd_reset;
wire sd_path_rst;
wire scanner_sector_req;
wire [31:0] scanner_sector_lba;
wire loader_sector_req;
wire [31:0] loader_sector_lba;
wire adapter_sector_busy;
wire adapter_sector_valid;
wire [7:0] adapter_sector_byte;
wire [8:0] adapter_sector_byte_index;
wire adapter_sector_done;
wire [7:0] adapter_sector_error;
wire adapter_sector_req;
wire [31:0] adapter_sector_lba;
wire adapter_abort;
wire client_a_busy;
wire client_a_valid;
wire client_a_done;
wire music_sector_req;
wire [31:0] music_sector_lba;
wire music_sector_busy;
wire music_sector_valid;
wire music_sector_done;
wire music_streamer_busy;
wire scan_music_valid;
wire [31:0] scan_music_cluster;
wire [31:0] scan_music_size;
reg              music_catalog_valid;
reg [31:0]       music_cluster;
reg [31:0]       music_size;
reg              music_started;
reg              music_start_pulse;
wire             scan_alarm_valid;
wire [27:0]      scan_alarm_cluster;
wire [31:0]      scan_alarm_size;
reg              want_alarm;
reg              playing_alarm;

assign write_path_idle = !load_busy && !load_prepare && !write_req && !bmp_data_wr_en;
assign write_en   = bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};
assign auto_tick  = (auto_cnt == (AUTO_PLAY_CYCLES - 1));
assign available_bank_mask = bank_loaded_mask & img_valid_mask &
                             (camera_mode_active ? 6'b001111 : 6'b111111);
assign next_from_current = next_valid_index(img_idx, available_bank_mask);
assign lock_sel  = test_mode ? 2'd1 : (flow_mode ? 2'd2 : 2'd0);
assign lock_mode = (lock_sel != 2'd0);
// 流程页为 idx2~5（第 3~6 页）；转场途中以转场目标作为当前页
assign flow_cur       = transition_inflight ? transition_active_target : img_idx;
assign flow_in_ring   = (flow_cur >= 3'd2);
assign flow_ring_next = (flow_cur == 3'd5) ? 3'd2 : flow_cur + 3'd1;
assign flow_ring_prev = (flow_cur == 3'd2) ? 3'd5 : flow_cur - 3'd1;
assign flow_next_ok   = flow_next_pulse && (flow_play_en ||
                        !((flow_cur == 3'd2) ? flow_lock[0] : ((flow_cur == 3'd3) && flow_lock[1])));
assign flow_prev_ok   = flow_prev_pulse && (flow_play_en || !((flow_cur == 3'd2) && flow_lock[0]));
assign bank_loaded_count = count_valid_images(available_bank_mask);
assign preload_pending_mask = img_valid_mask &
                              ~(bank_loaded_mask | bank_failed_mask);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
assign key_auto_release = !key_auto_stable_d && key_auto_stable;
assign state_code = display_valid ? ({1'b0, img_idx} + 4'd1) : 4'd0;
assign osd_auto_enabled = auto_play_en || flow_play_en;
assign osd_error_code = (scan_fs_error != 8'h00) ? scan_fs_error :
                        (bank_failed_mask != 6'd0) ?
                        {1'b1, bank_failed_mask[5], 1'b1, bank_failed_mask[4:0]} : 8'h00;
assign load_total_timeout = load_busy &&
                            (load_timeout_cnt >= LOAD_TIMEOUT_CYCLES - 1);
assign load_failure_event = bmp_load_error || load_total_timeout;
assign load_failure_needs_sd_reset = load_total_timeout ||
                                     (bmp_load_error && (bmp_error == 3'd2));
assign sd_path_rst = rst | (sd_reset_cnt != 8'd0);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(key_auto_press),
    .button_stable_out(key_auto_stable)
);

always @(posedge clk or posedge rst) begin
    if (rst)
        write_req_ack_sync <= 3'b000;
    else
        write_req_ack_sync <= {write_req_ack_sync[1:0], write_req_ack};
end

function [2:0] count_valid_images;
    input [5:0] mask;
    begin
        count_valid_images = mask[0] + mask[1] + mask[2] + mask[3] + mask[4] + mask[5];
    end
endfunction

function [2:0] first_valid_index;
    input [5:0] mask;
    begin
        if (mask[0])      first_valid_index = 3'd0;
        else if (mask[1]) first_valid_index = 3'd1;
        else if (mask[2]) first_valid_index = 3'd2;
        else if (mask[3]) first_valid_index = 3'd3;
        else if (mask[4]) first_valid_index = 3'd4;
        else              first_valid_index = 3'd5;
    end
endfunction

assign camera_fallback_idx = (lock_mode && bank_loaded_mask[lock_sel]) ? {1'b0, lock_sel} : first_valid_index({2'b00, bank_loaded_mask[3:0]});

function [2:0] next_valid_index;
    input [2:0] cur;
    input [5:0] mask;
    integer k;
    reg [2:0] cand;
    begin
        next_valid_index = cur;
        for (k = 5; k >= 1; k = k - 1) begin
            cand = (cur + k >= 6) ? (cur + k - 6) : (cur + k);
            if (mask[cand])
                next_valid_index = cand;
        end
    end
endfunction

function [2:0] prev_valid_index;
    input [2:0] cur;
    input [5:0] mask;
    integer k;
    reg [2:0] cand;
    begin
        prev_valid_index = cur;
        for (k = 5; k >= 1; k = k - 1) begin
            cand = (cur >= k) ? (cur - k) : (cur + 6 - k);
            if (mask[cand])
                prev_valid_index = cand;
        end
    end
endfunction

function [31:0] cluster_lut;
    input [2:0] idx;
    begin
        case (idx)
            2'd0: cluster_lut = img_cluster0;
            2'd1: cluster_lut = img_cluster1;
            2'd2: cluster_lut = img_cluster2;
            3'd3: cluster_lut = img_cluster3;
            3'd4: cluster_lut = img_cluster4;
            default: cluster_lut = img_cluster5;
        endcase
    end
endfunction

function [31:0] size_lut;
    input [2:0] idx;
    begin
        case (idx)
            2'd0: size_lut = img_size0;
            2'd1: size_lut = img_size1;
            2'd2: size_lut = img_size2;
            3'd3: size_lut = img_size3;
            3'd4: size_lut = img_size4;
            default: size_lut = img_size5;
        endcase
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_cluster          <= 32'd0;
        load_file_size        <= 32'd0;
        scan_kicked           <= 1'b0;
        catalog_valid         <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 29'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 3'd0;
        load_idx              <= 3'd0;
        write_buf_idx         <= 3'd0;
        disp_buf_idx          <= 3'd0;
        img_valid_mask        <= 6'd0;
        bank_loaded_mask      <= 6'd0;
        bank_failed_mask      <= 6'd0;
        img_cluster0          <= 32'd0;
        img_cluster1          <= 32'd0;
        img_cluster2          <= 32'd0;
        img_cluster3          <= 32'd0;
        img_cluster4          <= 32'd0;
        img_cluster5          <= 32'd0;
        img_size0             <= 32'd0;
        img_size1             <= 32'd0;
        img_size2             <= 32'd0;
        img_size3             <= 32'd0;
        img_size4             <= 32'd0;
        img_size5             <= 32'd0;
        catalog_spc           <= 8'd0;
        catalog_fat_start     <= 32'd0;
        catalog_data_start    <= 32'd0;
        catalog_max_cluster   <= 32'd0;
        music_catalog_valid   <= 1'b0;
        music_cluster         <= 32'd0;
        music_size            <= 32'd0;
        next_req_pending      <= 1'b0;
        prev_req_pending      <= 1'b0;
        transition_start      <= 1'b0;
        transition_target     <= 2'd0;
        transition_index_commit <= 1'b0;
        transition_inflight   <= 1'b0;
        transition_active_target <= 3'd0;
        camera_mode_d         <= 1'b0;
        lock_sel_d            <= 2'd0;
        flow_play_en          <= 1'b0;
        flow_req_pending      <= 1'b0;
        flow_target           <= 3'd0;
        long_press_cnt        <= 32'd0;
        long_press_marked     <= 1'b0;
        key_auto_stable_d     <= 1'b1;
        load_busy             <= 1'b0;
        load_prepare          <= 1'b0;
        load_timeout_cnt      <= 32'd0;
        load_abort            <= 1'b0;
        write_abort_toggle    <= 1'b0;
        write_session_started <= 1'b0;
        retry_count           <= 4'd0;
        retry_pending         <= 1'b0;
        retry_delay_cnt       <= 6'd0;
        sd_reset_cnt          <= 8'd0;
        source_done_seen      <= 1'b0;
        write_done_seen       <= 1'b0;
        display_valid         <= 1'b0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        camera_mode_d <= camera_mode_active;
        key_auto_stable_d <= key_auto_stable;
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        load_abort       <= 1'b0;
        transition_start <= 1'b0;
        transition_index_commit <= 1'b0;

        if (transition_commit && transition_inflight) begin
            disp_buf_idx <= transition_active_target;
            img_idx      <= transition_active_target;
            auto_cnt     <= 29'd0;
            transition_index_commit <= 1'b1;
            transition_inflight <= 1'b0;
        end

        if (sd_reset_cnt != 8'd0)
            sd_reset_cnt <= sd_reset_cnt - 8'd1;
        if (retry_delay_cnt != 6'd0)
            retry_delay_cnt <= retry_delay_cnt - 6'd1;

        if (!sd_init_done) begin
            auto_play_en          <= 1'b0;
            auto_cnt              <= 29'd0;
            next_req_pending      <= 1'b0;
            prev_req_pending      <= 1'b0;
            long_press_cnt        <= 32'd0;
            long_press_marked     <= 1'b0;
            key_auto_stable_d     <= 1'b1;
            load_busy             <= 1'b0;
            load_prepare          <= 1'b0;
            load_timeout_cnt      <= 32'd0;
            load_abort            <= 1'b0;
            source_done_seen      <= 1'b0;
            write_done_seen       <= 1'b0;
            write_session_started <= 1'b0;

            if (!display_valid && !retry_pending) begin
                scan_kicked           <= 1'b0;
                catalog_valid         <= 1'b0;
                first_image_committed <= 1'b0;
                img_found_count       <= 3'd0;
                img_idx               <= 3'd0;
                load_idx              <= 3'd0;
                write_buf_idx         <= 3'd0;
                disp_buf_idx          <= 3'd0;
                img_valid_mask        <= 6'd0;
                bank_loaded_mask      <= 6'd0;
                bank_failed_mask      <= 6'd0;
                img_cluster0          <= 32'd0;
                img_cluster1          <= 32'd0;
                img_cluster2          <= 32'd0;
                img_cluster3          <= 32'd0;
                img_cluster4          <= 32'd0;
                img_cluster5          <= 32'd0;
                img_size0             <= 32'd0;
                img_size1             <= 32'd0;
                img_size2             <= 32'd0;
                img_size3             <= 32'd0;
                img_size4             <= 32'd0;
                img_size5             <= 32'd0;
                catalog_spc           <= 8'd0;
                catalog_fat_start     <= 32'd0;
                catalog_data_start    <= 32'd0;
                catalog_max_cluster   <= 32'd0;
                music_catalog_valid   <= 1'b0;
                music_cluster         <= 32'd0;
                music_size            <= 32'd0;
            end
        end else begin
            if (scan_done && scan_kicked) begin
                if ((scan_fs_error == 8'h00) && (scan_resource_valid != 6'd0)) begin
                    catalog_valid       <= 1'b1;
                    img_valid_mask      <= scan_resource_valid;
                    img_found_count     <= count_valid_images(scan_resource_valid);
                    img_cluster0        <= scan_cluster0;
                    img_cluster1        <= scan_cluster1;
                    img_cluster2        <= scan_cluster2;
                    img_cluster3        <= scan_cluster3;
                    img_cluster4        <= scan_cluster4;
                    img_cluster5        <= scan_cluster5;
                    img_size0           <= scan_size0;
                    img_size1           <= scan_size1;
                    img_size2           <= scan_size2;
                    img_size3           <= scan_size3;
                    img_size4           <= scan_size4;
                    img_size5           <= scan_size5;
                    catalog_spc         <= scan_spc;
                    catalog_fat_start   <= scan_fat_start;
                    catalog_data_start  <= scan_data_start;
                    catalog_max_cluster <= scan_max_cluster;
                    music_catalog_valid <= scan_music_valid;
                    music_cluster       <= scan_music_cluster;
                    music_size          <= scan_music_size;
                    bank_loaded_mask    <= bank_loaded_mask & scan_resource_valid;
                    bank_failed_mask    <= 6'd0;
                end else begin
                    scan_kicked  <= 1'b0;
                    catalog_valid <= 1'b0;
                    sd_reset_cnt <= SD_RESET_CYCLES[7:0];
                end
            end

            if (load_busy && bmp_load_done)
                source_done_seen <= 1'b1;

            // 记住写帧完成脉冲，避免“先写完后源结束”导致脉冲丢失而卡死
            if (load_busy && write_finish_pulse)
                write_done_seen <= 1'b1;

            if (load_busy && write_req_ack_sync[2])
                write_session_started <= 1'b1;

            if (load_failure_event) begin
                load_busy             <= 1'b0;
                load_prepare          <= 1'b0;
                load_timeout_cnt      <= 32'd0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                load_abort            <= 1'b1;
                next_req_pending      <= 1'b0;
                prev_req_pending      <= 1'b0;
                retry_delay_cnt       <= 6'd16;

                if (write_session_started)
                    write_abort_toggle <= ~write_abort_toggle;
                write_session_started <= 1'b0;

                if (retry_count < MAX_RETRIES) begin
                    retry_count   <= retry_count + 4'd1;
                    retry_pending <= 1'b1;
                end else begin
                    retry_count                  <= 4'd0;
                    retry_pending                <= 1'b0;
                    bank_failed_mask[load_idx]   <= 1'b1;
                end

                if (load_failure_needs_sd_reset)
                    sd_reset_cnt <= SD_RESET_CYCLES[7:0];
            end else if (load_busy && source_done_seen && write_done_seen) begin
                load_busy             <= 1'b0;
                load_prepare          <= 1'b0;
                load_timeout_cnt      <= 32'd0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                write_session_started <= 1'b0;
                retry_count           <= 4'd0;
                retry_pending         <= 1'b0;
                bank_loaded_mask[load_idx] <= 1'b1;
                case (load_idx)
                    3'd2: res2 <= {bmp_src_w, bmp_src_h};
                    3'd3: res3 <= {bmp_src_w, bmp_src_h};
                    3'd4: res4 <= {bmp_src_w, bmp_src_h};
                    3'd5: res5 <= {bmp_src_w, bmp_src_h};
                    default: ;
                endcase

                // 第一张完整写入的图片立即显示；后续图片只在后台缓存，
                // 不打断当前画面。每张图片永久对应同编号 bank。
                if (!display_valid) begin
                    disp_buf_idx          <= load_idx;
                    img_idx               <= load_idx;
                    display_valid         <= 1'b1;
                    first_image_committed <= 1'b1;
                end
            end else if (load_busy) begin
                load_timeout_cnt <= load_timeout_cnt + 32'd1;
            end else begin
                load_timeout_cnt <= 32'd0;
            end

            // 上电后扫描 PIC1.BMP 至 PIC6.BMP。
            if (!scan_kicked && !pause_background && system_ready && bmp_ready && !scan_busy &&
                !retry_pending) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                catalog_valid         <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 29'd0;
                img_found_count       <= 3'd0;
                load_idx              <= 3'd0;
                img_valid_mask        <= 6'd0;
                img_cluster0          <= 32'd0;
                img_cluster1          <= 32'd0;
                img_cluster2          <= 32'd0;
                img_cluster3          <= 32'd0;
                img_cluster4          <= 32'd0;
                img_cluster5          <= 32'd0;
                img_size0             <= 32'd0;
                img_size1             <= 32'd0;
                img_size2             <= 32'd0;
                img_size3             <= 32'd0;
                img_size4             <= 32'd0;
                img_size5             <= 32'd0;
                next_req_pending      <= 1'b0;
                prev_req_pending      <= 1'b0;
                long_press_cnt        <= 32'd0;
                long_press_marked     <= 1'b0;
                key_auto_stable_d     <= 1'b1;
                load_busy             <= 1'b0;
                load_prepare          <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                write_session_started <= 1'b0;
                retry_count           <= 4'd0;

                if (!display_valid) begin
                    first_image_committed <= 1'b0;
                    img_idx               <= 3'd0;
                    write_buf_idx         <= 3'd0;
                    disp_buf_idx          <= 3'd0;
                    bank_loaded_mask      <= 6'd0;
                    bank_failed_mask      <= 6'd0;
                end
            end else begin
                // 忙的时候也只记 1 次“下一张”请求，不会累积成连跳两张
                if (!pause_background && (key_next_press || screen_next_pulse) && catalog_valid && !retry_pending &&
                    !lock_mode &&
                    display_valid && (bank_loaded_count > 3'd1))
                    next_req_pending <= 1'b1;

                // KEY2 长按检测：按住达到阈值只产生一次上一张请求
                if (!key_auto_stable) begin
                    if (long_press_cnt < ((CLK_FREQ_HZ / 1000) * LONG_PRESS_MS - 1)) begin
                        long_press_cnt <= long_press_cnt + 32'd1;
                    end else if (!long_press_marked) begin
                        long_press_marked <= 1'b1;
                        if (catalog_valid && display_valid && !lock_mode &&
                            !retry_pending && (bank_loaded_count > 3'd1))
                            prev_req_pending <= 1'b1;
                    end
                end else begin
                    long_press_cnt    <= 32'd0;
                    long_press_marked <= 1'b0;
                end

                if (key_auto_release && !long_press_marked && catalog_valid && !lock_mode &&
                    !retry_pending && (bank_loaded_count > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 29'd0;
                end

                // 自动播放计数：只有当前没有写图任务时才计时
                if (!pause_background && catalog_valid && (auto_play_en || flow_play_en) && display_valid &&
                    !retry_pending && first_image_committed &&
                    !transition_busy && !transition_inflight &&
                    (bank_loaded_count > 3'd1)) begin
                    if (auto_tick)
                        auto_cnt <= 29'd0;
                    else
                        auto_cnt <= auto_cnt + 29'd1;
                end else begin
                    auto_cnt <= 29'd0;
                end

                if (!pause_background && load_prepare && bmp_ready && !load_busy) begin
                    load_cluster          <= cluster_lut(load_idx);
                    load_file_size        <= size_lut(load_idx);
                    load_start_pulse      <= 1'b1;
                    load_busy             <= 1'b1;
                    load_prepare          <= 1'b0;
                    source_done_seen      <= 1'b0;
                    write_done_seen       <= 1'b0;
                    write_session_started <= 1'b0;
                    auto_cnt              <= 29'd0;
                end
                // 失败后优先重试同一张；写通道取消和SD重新初始化完成后再启动。
                else if (!pause_background && retry_pending && (retry_delay_cnt == 6'd0) &&
                    bmp_ready && !load_busy) begin
                    load_start_pulse      <= 1'b1;
                    load_busy             <= 1'b1;
                    source_done_seen      <= 1'b0;
                    write_done_seen       <= 1'b0;
                    write_session_started <= 1'b0;
                    retry_pending         <= 1'b0;
                    auto_cnt              <= 29'd0;
                end
                else if (!pause_background && display_valid && flow_req_pending &&
                         !transition_busy && !transition_inflight &&
                         ((flow_target == img_idx) || available_bank_mask[flow_target] ||
                          bank_failed_mask[flow_target])) begin
                    if ((flow_target != img_idx) && available_bank_mask[flow_target]) begin
                        transition_target        <= transition_mode;
                        transition_active_target <= flow_target;
                        transition_start         <= 1'b1;
                        transition_inflight      <= 1'b1;
                    end
                    flow_req_pending <= 1'b0;
                    auto_cnt         <= 29'd0;
                end
                else if (!pause_background && display_valid && next_req_pending &&
                         !transition_busy && !transition_inflight &&
                         (bank_loaded_count > 3'd1)) begin
                    transition_target        <= transition_mode;
                    transition_active_target <= next_from_current;
                    transition_start         <= 1'b1;
                    transition_inflight      <= 1'b1;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 29'd0;
                end
                else if (!pause_background && display_valid && prev_req_pending &&
                         !transition_busy && !transition_inflight &&
                         (bank_loaded_count > 3'd1)) begin
                    transition_target <= transition_mode;
                    transition_active_target <= prev_valid_index(
                                                           img_idx,
                                                           available_bank_mask);
                    transition_start <= 1'b1;
                    transition_inflight <= 1'b1;
                    prev_req_pending <= 1'b0;
                    auto_cnt         <= 29'd0;
                end
                else if (!pause_background && display_valid && auto_play_en && auto_tick &&
                         !transition_busy && !transition_inflight &&
                         (bank_loaded_count > 3'd1)) begin
                    transition_target        <= transition_mode;
                    transition_active_target <= next_from_current;
                    transition_start         <= 1'b1;
                    transition_inflight      <= 1'b1;
                    auto_cnt                 <= 29'd0;
                end
                // 所有目录中有效但尚未缓存的图片依次写入固定 bank。
                // 失败 bank 在本轮被屏蔽，其他图片仍继续预加载。
                else if (!pause_background && catalog_valid && bmp_ready && !load_busy &&
                         (retry_delay_cnt == 6'd0) &&
                         (preload_pending_mask != 6'd0)) begin
                    load_idx         <= first_valid_index(preload_pending_mask);
                    write_buf_idx    <= first_valid_index(preload_pending_mask);
                    load_prepare     <= 1'b1;
                    retry_count      <= 4'd0;
                    auto_cnt         <= 29'd0;
                end
            end
            if (camera_mode_active && !camera_mode_d) begin
                bank_loaded_mask[5:4] <= 2'b00;
                if (img_idx[2] && bank_loaded_mask[3:0] != 4'd0) begin
                    img_idx      <= camera_fallback_idx;
                    disp_buf_idx <= camera_fallback_idx;
                end
            end
            if (!camera_mode_active && camera_mode_d &&
                img_idx[2] && bank_loaded_mask[3:0] != 4'd0) begin
                img_idx      <= camera_fallback_idx;
                disp_buf_idx <= camera_fallback_idx;
            end
        end

        lock_sel_d <= lock_sel;
        if (lock_sel != lock_sel_d) begin
            flow_req_pending <= 1'b1;
            flow_target      <= {1'b0, lock_sel};
            flow_play_en     <= 1'b0;
            next_req_pending <= 1'b0;
            prev_req_pending <= 1'b0;
            if (lock_mode)
                auto_play_en <= 1'b0;
        end else if (lock_sel == 2'd2) begin
            if (flow_stop_pulse) begin
                flow_play_en     <= 1'b0;
            end else if (flow_play_pulse) begin
                flow_req_pending <= 1'b1;
                flow_target      <= 3'd2;
                flow_play_en     <= 1'b1;
            end else if (flow_next_ok || flow_prev_ok) begin
                flow_req_pending <= 1'b1;
                flow_target      <= !flow_in_ring ? 3'd2 :
                                    flow_next_ok  ? flow_ring_next : flow_ring_prev;
            end else if (flow_play_en && auto_tick) begin
                if (flow_in_ring && (flow_cur != 3'd5)) begin
                    flow_req_pending <= 1'b1;
                    flow_target      <= flow_cur + 3'd1;
                end else begin
                    flow_play_en     <= 1'b0;
                end
            end
        end
    end
end

fat32_volume_scanner u_fat32_scanner (
    .clk                 (clk),
    .rst                 (sd_path_rst),
    .scan_start          (scan_start_pulse),
    .scan_busy           (scan_busy),
    .scan_done           (scan_done),
    .fs_error            (scan_fs_error),
    .sectors_per_cluster (scan_spc),
    .fat_start_lba       (scan_fat_start),
    .data_start_lba      (scan_data_start),
    .max_cluster         (scan_max_cluster),
    .resource_valid      (scan_resource_valid),
    .resource0_cluster   (scan_cluster0),
    .resource1_cluster   (scan_cluster1),
    .resource2_cluster   (scan_cluster2),
    .resource3_cluster   (scan_cluster3),
    .resource4_cluster   (scan_cluster4),
    .resource5_cluster   (scan_cluster5),
    .resource0_size      (scan_size0),
    .resource1_size      (scan_size1),
    .resource2_size      (scan_size2),
    .resource3_size      (scan_size3),
    .resource4_size      (scan_size4),
    .resource5_size      (scan_size5),
    .music_valid         (scan_music_valid),
    .music_cluster       (scan_music_cluster),
    .music_size          (scan_music_size),
    .alarm_valid         (scan_alarm_valid),
    .alarm_cluster       (scan_alarm_cluster),
    .alarm_size          (scan_alarm_size),
    .sector_req          (scanner_sector_req),
    .sector_lba          (scanner_sector_lba),
    .sector_busy         (client_a_busy),
    .sector_valid        (client_a_valid),
    .sector_byte         (adapter_sector_byte),
    .sector_byte_index   (adapter_sector_byte_index),
    .sector_done         (client_a_done),
    .sector_error        (adapter_sector_error)
);

fat32_bmp_loader #(
    .FRAME_PIXELS (FRAME_PIXELS)
) u_fat32_bmp_loader (
    .clk                 (clk),
    .rst                 (sd_path_rst),
    .load_start          (load_start_pulse),
    .load_abort          (load_abort),
    .first_cluster       (load_cluster),
    .directory_file_size(load_file_size),
    .sectors_per_cluster (catalog_spc),
    .fat_start_lba       (catalog_fat_start),
    .data_start_lba      (catalog_data_start),
    .max_cluster         (catalog_max_cluster),
    .bmp_width           (bmp_width),
    .bmp_height          (bmp_height),
    .write_fifo_full     (write_fifo_full),
    .src_w               (bmp_src_w),
    .src_h               (bmp_src_h),
    .ready               (bmp_ready),
    .load_done           (bmp_load_done),
    .load_error          (bmp_load_error),
    .bmp_error           (bmp_error),
    .state_code          (state_code_i),
    .write_req           (write_req),
    .write_req_ack       (write_req_ack_sync[2]),
    .bmp_data_wr_en      (bmp_data_wr_en),
    .bmp_data            (bmp_data),
    .sector_req          (loader_sector_req),
    .sector_lba          (loader_sector_lba),
    .sector_busy         (client_a_busy),
    .sector_valid        (client_a_valid),
    .sector_byte         (adapter_sector_byte),
    .sector_byte_index   (adapter_sector_byte_index),
    .sector_done         (client_a_done),
    .sector_error        (adapter_sector_error)
);

wire music_switch = music_started && (playing_alarm != want_alarm);

always @(posedge clk or posedge sd_path_rst) begin
    if (sd_path_rst) begin
        music_started     <= 1'b0;
        music_start_pulse <= 1'b0;
        want_alarm        <= 1'b0;
        playing_alarm     <= 1'b0;
    end else begin
        music_start_pulse <= 1'b0;
        want_alarm        <= alarm_select && scan_alarm_valid;
        if (!music_enable || music_switch) begin
            music_started <= 1'b0;
        end else if (!music_started && catalog_valid &&
                     (want_alarm || music_catalog_valid) && !music_streamer_busy) begin
            music_start_pulse <= 1'b1;
            music_started     <= 1'b1;
            playing_alarm     <= want_alarm;
        end else if (music_started && playing_alarm && !music_start_pulse &&
                     !music_streamer_busy) begin
            // 警报读到文件尾：下一拍重新起播，形成循环
            music_started <= 1'b0;
        end
    end
end

music_sector_reader u_music_reader (
    .clk                 (clk),
    .rst                 (sd_path_rst),
    .abort               (!music_enable || music_switch),
    .file_start          (music_start_pulse),
    .first_cluster       (playing_alarm ? {4'd0, scan_alarm_cluster} : music_cluster),
    .file_size           (playing_alarm ? scan_alarm_size : music_size),
    .sectors_per_cluster (catalog_spc),
    .fat_start_lba       (catalog_fat_start),
    .data_start_lba      (catalog_data_start),
    .file_busy           (music_streamer_busy),
    .file_valid          (music_byte_valid),
    .file_byte           (music_byte),
    .file_byte_hi        (music_byte_hi),
    .sector_req          (music_sector_req),
    .sector_lba          (music_sector_lba),
    .sector_busy         (music_sector_busy | music_hold),
    .sector_valid        (music_sector_valid),
    .sector_byte         (adapter_sector_byte),
    .sector_byte_index   (adapter_sector_byte_index),
    .sector_done         (music_sector_done),
    .sector_error        (adapter_sector_error)
);

sd_sector_arbiter u_sd_sector_arbiter (
    .clk     (clk),
    .rst     (sd_path_rst),
    .abort   (load_abort),
    .a_req   (scanner_sector_req | loader_sector_req),
    .a_lba   (loader_sector_req ? loader_sector_lba : scanner_sector_lba),
    .a_busy  (client_a_busy),
    .a_valid (client_a_valid),
    .a_done  (client_a_done),
    .b_req   (music_sector_req),
    .b_lba   (music_sector_lba),
    .b_busy  (music_sector_busy),
    .b_valid (music_sector_valid),
    .b_done  (music_sector_done),
    .m_req   (adapter_sector_req),
    .m_lba   (adapter_sector_lba),
    .m_abort (adapter_abort),
    .m_busy  (adapter_sector_busy),
    .m_valid (adapter_sector_valid),
    .m_done  (adapter_sector_done)
);

sd_sector_adapter #(
    .CLK_FREQ_HZ      (CLK_FREQ_HZ),
    .STALL_TIMEOUT_MS (SD_STALL_TIMEOUT_MS)
) u_sd_sector_adapter (
    .clk                    (clk),
    .rst                    (sd_path_rst),
    .abort                  (adapter_abort),
    .sector_req             (adapter_sector_req),
    .sector_lba             (adapter_sector_lba),
    .sector_busy            (adapter_sector_busy),
    .sector_valid           (adapter_sector_valid),
    .sector_byte            (adapter_sector_byte),
    .sector_byte_index      (adapter_sector_byte_index),
    .sector_done            (adapter_sector_done),
    .sector_error           (adapter_sector_error),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end)
);

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (sd_path_rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse,
    output reg  button_stable_out
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
        button_stable_out <= 1'b1;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                button_stable_out <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
