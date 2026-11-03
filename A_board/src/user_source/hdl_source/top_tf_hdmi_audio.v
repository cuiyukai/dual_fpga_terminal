`include "../user_source/hdl_source/tf_audio_player.v"
`include "../user_source/hdl_source/SD/sd_sector_arbiter.v"
`include "../user_source/hdl_source/SD/bmp_fit_scaler.v"
`include "../user_source/hdl_source/res_font_rom.v"
`include "../user_source/hdl_source/src_res_osd.v"
`include "../user_source/hdl_source/SD/music_sector_reader.v"
`include "../user_source/hdl_source/I2S_receiver.v"
`include "../user_source/hdl_source/audio_arc_calculate.v"
`include "al_ip/PLL_HDMI_AUDIO.v"
`include "../user_source/hdl_source/reset_sync.v"
`include "../user_source/hdl_source/sync_2ff.v"
`include "../user_source/hdl_source/SD/video_transition_4way.v"
`include "../user_source/hdl_source/osd_font_rom.v"
`include "../user_source/hdl_source/osd_alpha_blend.v"
`include "../user_source/hdl_source/imu_gyro_wave_osd.v"
`include "../user_source/hdl_source/audio_spectrum_rom.v"
`include "../user_source/hdl_source/audio_spectrum_osd.v"
`include "../user_source/hdl_source/screen_uart_rx.v"
`include "../user_source/hdl_source/imu_link_rx.v"
`include "../user_source/hdl_source/pct_sync.v"
`include "../user_source/hdl_source/audio_volume.v"
`include "../user_source/hdl_source/page_chime.v"
`include "../user_source/hdl_source/brightness_font_rom.v"
`include "../user_source/hdl_source/screen_brightness_osd.v"
`include "../user_source/hdl_source/status_font_rom.v"
`include "../user_source/hdl_source/device_status_osd.v"
`include "../user_source/hdl_source/warn_sprite_rom.v"
`include "../user_source/hdl_source/alarm_ribbon3d.v"
`include "../user_source/hdl_source/alarm_sparks.v"
`include "../user_source/hdl_source/gyro_storm_osd.v"
`include "../user_source/hdl_source/menu_corner_osd.v"
`include "../user_source/hdl_source/flow_checklist.v"
`include "../user_source/hdl_source/test_font_rom.v"
`include "../user_source/hdl_source/camera/ov5640_sccb_master.v"
`include "../user_source/hdl_source/camera/ov5640_rgb565_config.v"
`include "../user_source/hdl_source/camera/ov5640_camera.v"
`include "../user_source/hdl_source/camera/camera_sdram_stream.v"
`include "../user_source/hdl_source/sleep_logo_rom.v"
`include "../user_source/hdl_source/stamp_font_rom.v"
`include "../user_source/hdl_source/timestamp_roll_osd.v"
`include "../user_source/hdl_source/wake_streak_fx.v"

module top #(
    parameter       SW4_PLAY_LEVEL = 1'b0,
    parameter       SW3_SLEEP_LEVEL = 1'b0
)(
    input                       clk,
    input                       rst_n,
    input                       key1,
    input                       key2,
    input                       key3,
    input                       sw4,            // 拨码开关 SW4(C5)：拨到播放位置播放 TF 卡音乐
    input                       sw3,

	output [5:0]                seg_sel,
	output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso,

    // imu_uart_tx(D12) 直接镜像链路数据，电脑串口助手可抓包。
    input                       link_rx,
    output                      imu_uart_tx,
    output                      sleep_link_n,   // J2-16(L1) -> B 板 J2-16：休眠请求，低有效

    output                      cam_pwdn,
    input                       cam_pclk,
    input      [7:0]            cam_data,
    output                      cam_reset_n,
    inout                       cam_sda,
    input                       cam_href,
    output                      cam_scl,
    input                       cam_vsync
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
parameter [ADDR_BITS-1:0] BUF0_ADDR = 21'd0;
parameter [ADDR_BITS-1:0] BUF1_ADDR = 21'd307200;
parameter [ADDR_BITS-1:0] BUF2_ADDR = 21'd614400;
parameter [ADDR_BITS-1:0] BUF3_ADDR = 21'd921600;
parameter [ADDR_BITS-1:0] CAM0_ADDR = 21'd1228800;
parameter [ADDR_BITS-1:0] CAM1_ADDR = 21'd1536000;
parameter [ADDR_BITS-1:0] BUF4_ADDR = CAM0_ADDR;
parameter [ADDR_BITS-1:0] BUF5_ADDR = CAM1_ADDR;

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;
wire sdram_refresh_hold;
wire sdram_access_idle;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_raw;
wire [23:0] vout_data_transition;
wire [23:0] vout_data_base;
wire [23:0] vout_data_status;
wire [23:0] vout_data_wave;
wire [59:0] audio_vis_band;        // 频谱 12 段振幅，驱动第二页警报律动丝带
wire [23:0] vout_data;
wire [23:0] vout_data_dev;
wire        vs_dev;
wire        de_dev;
wire [23:0] vout_data_storm;       // 第二页陀螺仪超速警戒带警报叠加后
wire        vs_storm;
wire        de_storm;
wire [23:0] vout_data_menu;        // 第一页菜单光标角标叠加后
wire        vs_menu;
wire        de_menu;
wire [23:0] vout_data_bright;      // 串口屏亮度缩放后送 HDMI
wire        vs_bright;
wire        de_bright;
wire        display_valid;
wire        osd_auto_enabled;
wire [7:0]  osd_error_code;

wire [3:0]  state_code;
wire [6:0]  seg_data_0;
wire [3:0]  camera_readback_code;
wire        transfx_key_press;
reg  [1:0]  transition_mode;
(* ASYNC_REG = "TRUE" *) reg [1:0] transition_mode_meta;
(* ASYNC_REG = "TRUE" *) reg [1:0] transition_mode_seg;   // clk 域，只供数码管显示
wire [6:0]  transfx_seg;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        sd_card_write_path_idle;
wire        frame_write_abort_toggle;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [2:0]  write_buf_idx;
wire [2:0]  disp_buf_idx;
wire        transition_start;
wire [1:0]  transition_target;
wire        transition_index_commit;
wire        transition_commit;
wire        transition_busy;
wire [1:0]  transition_kind;
wire [2:0]  transition_dest_idx;   // 转场目标页（sd_card_clk 域）
wire        flip_read_en;
wire [9:0]  flip_read_fold;
(* ASYNC_REG = "TRUE" *) reg [2:0] transition_dest_mem_meta;
(* ASYNC_REG = "TRUE" *) reg [2:0] transition_dest_mem_sync;   // ext_mem_clk 域
wire [ADDR_BITS-1:0] flip_read_addr;

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

wire hs_0;
wire vs_0;
wire de_0;

wire imu_sample_valid;
wire imu_calibrated;
wire imu_sensor_error;
wire signed [15:0] imu_gx;
wire signed [15:0] imu_gy;
wire signed [15:0] imu_gz;
wire signed [15:0] imu_roll_deg_x10;
wire signed [15:0] imu_pitch_deg_x10;
wire signed [15:0] imu_yaw_deg_x10;
wire               imu_attitude_valid;
reg                imu_wave_page_sd;       // 第二页：姿态角波形 + 读数卡片（IMU 只在本页运行）
wire               imu_wave_page_vid;
wire               imu_run_rst_n;
wire               imu_osd_rst_n;
wire                link_flow_mode;      // B 板菜单：操作流程模式
wire [6:0]          link_flow_evt_toggle;
wire                link_test_mode;      // B 板菜单：实体测试（第 2 页）
wire [1:0]          link_test_phase;
wire [3:0]          link_test_secs;      // 当前阶段剩余秒数
wire [6:0]          link_test_score;     // 检测评分 0~100（phase 3 有效）
wire [13:0]         link_test_tilt;
wire [19:0]         link_rtc_time;       // B 板 RTC 时间（BCD）：首页滚动时钟显示
wire                link_stamp_toggle;   // B 板主菜单 KEY2 短按事件翻转位：触发首页滚动时钟
wire                test_osd_show;       // 检测倒计时 / 结果可显示（还需当前为第二页）
wire                test_freeze;         // 检测出结果：第二页波形 / 读数冻结
wire                test_alarm_release;  // 实体检测中且不在“检测中”阶段：警戒带警报解除
wire [7:0]          link_menu_state;
wire                menu_corner_show;    // 显示第一页光标角标（还需当前为第一页）
(* ASYNC_REG = "TRUE" *) reg [8:0] flow_sd_meta, flow_sd_sync;
reg                 page_chime_tgl_sd;   // 每次翻页提交翻转一次，触发视频域“叮咚”
reg  [6:0]          flow_evt_sd_d;
wire [6:0]          flow_evt_pulse_sd;
wire [1:0]          flow_lock_sd;        // 流程第 1 / 2 步未勾全：禁止手动下一页
wire                chk_show_sd;         // 流程第 1 / 2 步复选框显示参数（送视频域）
wire                chk_step2_sd;
wire [1:0]          chk_cursor_sd;
wire [3:0]          chk_mask_sd;
wire [6:0]          link_brightness_pct; // B 板串口屏亮度滑块 0~100
wire [6:0]          link_volume_pct;     // B 板串口屏音量滑块 0~100
reg                 bright_page_sd;      // 第一页且转场结束：顶栏显示音量/亮度数值
reg                 status_page_sd;
reg                 status_gyro_only_sd; // 第二页：状态只显示陀螺仪，位置上移到第二页底栏
reg                 storm_page_sd;       // 第二页且转场结束：允许显示超速警戒带警报
wire                imu_frame_valid;     // B 板链路收到一帧校验通过的数据
reg  [24:0]         imu_link_timer;      // 距上一帧有效数据的 clk 周期数
reg                 imu_link_alive;
wire                gyro_ok;             // 链路在线、已校准且无传感器错误
reg                 gyro_disp;           // 状态栏显示用：休眠 / 重新消零期间保持上一次结果
reg  [28:0]         gyro_hold_timer;
reg                 sleep_mode_d;
reg                 cam_probe;           // 上电自检：在摄像头模式之外跑一遍配置 + 回读
reg  [26:0]         cam_probe_timer;
(* ASYNC_REG = "TRUE" *) reg [3:0] cam_rb_meta, cam_rb_sync;
reg  [3:0]          cam_rb_last;
reg                 cam_ok;              // 最近一次回读结果为 5（已验证）
reg                 camera_mode_request;
wire                camera_mode_sd;
wire                camera_mode_video;
wire                camera_frame_available;
wire                camera_init_done;
wire                camera_pixel_valid;
wire [15:0]         camera_pixel_rgb565;
wire [18:0]         camera_pixel_addr;
wire                camera_frame_start_toggle;
wire                camera_frame_done_toggle;
wire                camera_pause_tf;
wire                camera_owner_sd;
wire                camera_capture_allowed;
wire                camera_sdram_write_req;
wire                camera_sdram_write_en;
wire [31:0]         camera_sdram_write_data;
wire                camera_sdram_abort_toggle;
wire                camera_write_bank;
wire                camera_published_bank;
wire                camera_published_toggle;
wire                camera_display_ack_toggle;
wire                camera_read_mode;
wire                camera_read_bank;
wire                write_fifo_full;

// HDMI 1.4b 相关
wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// HDMI 1.4b 音频链路：音源只有 TF 卡音乐（tf_audio_player）
wire        audio_pll_lock;
wire        audio_mclk;
wire        audio_i2s_bclk;
wire        audio_i2s_lrck;
wire        audio_i2s_dout;
wire        music_i2s_bclk;
wire        music_i2s_lrck;
wire        music_i2s_dout;

// SW4 控制 TF 卡音乐播放；第二页警报有效时也自动播放
wire        sw4_stable;
wire        music_on;
wire        storm_alarm_vid;      // 第二页警报有效（video_clk 域）
wire        storm_alarm;          // 同步到 clk 域
reg         music_play;           // SW4 播放 | 警报播放
wire        music_on_sd;
wire        music_on_mclk;
wire        music_hold_sd;
wire        music_byte_valid_sd;
wire [7:0]  music_byte_sd;
wire        music_byte_hi_sd;
wire        audio_valid_raw;
wire [23:0] audio_left_data_raw;
wire [23:0] audio_right_data_raw;
wire        audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
wire        audio_valid_chime;    // 叠加翻页提示音后的采样
wire [23:0] audio_left_chime;
wire [23:0] audio_right_chime;
wire        audio_valid_vol;      // 音量缩放后送 HDMI；频谱仍取缩放前的采样
wire [23:0] audio_left_vol;
wire [23:0] audio_right_vol;
wire        acr_valid_raw;
wire [19:0] acr_cts_raw;
wire [19:0] acr_n_raw;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

reg  [1:0]  audio_enable_video_sync;
wire        audio_enable_video;
wire        audio_video_rst;

// 休眠模式（B 板菜单）
reg         sleep_mode;            // clk 域：1 = 休眠
wire        sleep_sd;              // sleep_mode 同步到 sd_card_clk 域
reg  [6:0]  bright_pct_hold;       // 亮度 / 音量：休眠期间保持进入休眠前的值
reg  [6:0]  volume_pct_hold;
wire [23:0] vout_data_wake;        // 休眠暗屏 / 唤醒光带后
wire        vs_wake;
wire        de_wake;
wire        vs_res;                // 流程页原分辨率角标叠加后送 HDMI
wire        de_res;
wire [23:0] vout_data_res;
wire        disp_res_valid;        // 当前显示页是多分辨率流程图（sd_card_clk 域）
wire [10:0] disp_res_w, disp_res_h;
reg         res_show_sd;
wire        vs_stamp;              // 首页滚动时钟叠加后送 HDMI
wire        de_stamp;
wire [23:0] vout_data_stamp;
wire        storm_alarm_sd;        // 警报同步到 sd_card_clk 域：选择 ALARM.PCM

// 统一复位：TF 图像链路 + HDMI 链路
wire sys_pll_lock;
wire video_pll_lock;
wire startup_release_n;
wire ctrl_rst_n;
wire sdcard_rst_n;
wire mem_rst_n;
wire video_rst_n;
wire hdmi5x_rst_n;
wire audio_rst_n;
wire ctrl_rst;
wire sdcard_rst;
wire mem_rst;
wire video_rst;
wire hdmi5x_rst;
wire audio_rst;
wire sdram_init_done_sd;

reg [19:0] rst_cnt = 20'd0;
reg rst_all_reg = 1'b1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rst_cnt <= 20'd0;
        rst_all_reg <= 1'b1;
    end else if (!rst_all_reg) begin
        rst_all_reg <= 1'b0;
    end else if (sys_pll_lock && video_pll_lock && audio_pll_lock) begin
        // 等待全部 PLL 连续稳定 20ms @ 50MHz
        if (rst_cnt < 20'd1_000_000) begin
            rst_cnt <= rst_cnt + 1'b1;
            rst_all_reg <= 1'b1;
        end else begin
            rst_all_reg <= 1'b0;
        end
    end else begin
        rst_cnt <= 20'd0;
        rst_all_reg <= 1'b1;
    end
end

assign startup_release_n = ~rst_all_reg;

reset_sync u_ctrl_reset (
    .clk        (clk),
    .ext_rst_n  (startup_release_n),
    .rst_n_sync (ctrl_rst_n)
);

reset_sync u_sdcard_reset (
    .clk        (sd_card_clk),
    .ext_rst_n  (startup_release_n),
    .rst_n_sync (sdcard_rst_n)
);

reset_sync u_mem_reset (
    .clk        (ext_mem_clk),
    .ext_rst_n  (startup_release_n),
    .rst_n_sync (mem_rst_n)
);

reset_sync u_video_reset (
    .clk        (video_clk),
    .ext_rst_n  (startup_release_n),
    .rst_n_sync (video_rst_n)
);

reset_sync u_hdmi5x_reset (
    .clk        (hdmi_5x_clk),
    .ext_rst_n  (startup_release_n),
    .rst_n_sync (hdmi5x_rst_n)
);

reset_sync u_audio_reset (
    .clk        (audio_mclk),
    .ext_rst_n  (startup_release_n),
    .rst_n_sync (audio_rst_n)
);

assign ctrl_rst   = ~ctrl_rst_n;
assign sdcard_rst = ~sdcard_rst_n;
assign mem_rst    = ~mem_rst_n;
assign video_rst  = ~video_rst_n;
assign hdmi5x_rst = ~hdmi5x_rst_n;
assign audio_rst  = ~audio_rst_n;

sync_2ff u_sdram_init_done_sync (
    .clk      (sd_card_clk),
    .rst_n    (sdcard_rst_n),
    .async_in (Sdr_init_done),
    .sync_out (sdram_init_done_sd)
);

sync_2ff u_camera_mode_sd_sync (
    .clk      (sd_card_clk),
    .rst_n    (sdcard_rst_n),
    .async_in (camera_mode_request),
    .sync_out (camera_mode_sd)
);

sys_pll sys_pll_m0(
    .refclk     (clk),
    .reset      (~rst_n),
    .extlock    (sys_pll_lock),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .reset      (~rst_n),
    .extlock    (video_pll_lock),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk)
);

// 官方音频 PLL：50MHz -> 12.288MHz 音频主时钟
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (~rst_n),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);

// SW4 是电平开关，用消抖后的稳定电平判断是否播放 TF 卡音乐
ax_debounce #(
    .N        (32),
    .FREQ     (50),
    .MAX_TIME (20)
) u_sw4_debounce (
    .clk            (clk),
    .rst            (ctrl_rst),
    .button_in      (sw4),
    .button_posedge (),
    .button_negedge (),
    .button_out     (sw4_stable)
);

assign music_on = (sw4_stable == SW4_PLAY_LEVEL);

sync_2ff u_storm_alarm_sync (
    .clk      (clk),
    .rst_n    (ctrl_rst_n),
    .async_in (storm_alarm_vid),
    .sync_out (storm_alarm)
);

always @(posedge clk or posedge ctrl_rst) begin
    if (ctrl_rst)
        music_play <= 1'b0;
    else
        music_play <= music_on | storm_alarm;
end

always @(posedge clk or posedge ctrl_rst) begin
    if (ctrl_rst)
        sleep_mode <= 1'b0;
    else
        sleep_mode <= link_menu_state[6] && imu_link_alive;
end

sync_2ff u_sleep_sd_sync (
    .clk      (sd_card_clk),
    .rst_n    (sdcard_rst_n),
    .async_in (sleep_mode),
    .sync_out (sleep_sd)
);

always @(posedge clk or posedge ctrl_rst) begin
    if (ctrl_rst) begin
        bright_pct_hold <= 7'd100;
        volume_pct_hold <= 7'd100;
    end else if (!sleep_mode) begin
        bright_pct_hold <= link_brightness_pct;
        volume_pct_hold <= link_volume_pct;
    end
end

sync_2ff u_music_on_sd_sync (
    .clk      (sd_card_clk),
    .rst_n    (sdcard_rst_n),
    .async_in (music_play),
    .sync_out (music_on_sd)
);

sync_2ff u_storm_alarm_sd_sync (
    .clk      (sd_card_clk),
    .rst_n    (sdcard_rst_n),
    .async_in (storm_alarm),
    .sync_out (storm_alarm_sd)
);

sync_2ff u_music_on_mclk_sync (
    .clk      (audio_mclk),
    .rst_n    (audio_rst_n),
    .async_in (music_play),
    .sync_out (music_on_mclk)
);

// 静音时也送 0 采样，翻页“叮咚”随时可以叠加。
always @(posedge video_clk or posedge video_rst) begin
    if (video_rst)
        audio_enable_video_sync <= 2'b00;
    else
        audio_enable_video_sync <= {audio_enable_video_sync[0], audio_pll_lock};
end

assign audio_enable_video = audio_enable_video_sync[1];
assign audio_video_rst    = video_rst | ~audio_enable_video | ~audio_pll_lock;

assign audio_valid      = audio_enable_video ? audio_valid_raw      : 1'b0;
assign audio_left_data  = audio_enable_video ? audio_left_data_raw  : 24'd0;
assign audio_right_data = audio_enable_video ? audio_right_data_raw : 24'd0;
assign acr_valid        = audio_enable_video ? acr_valid_raw        : 1'b0;
assign acr_cts          = audio_enable_video ? acr_cts_raw          : 20'd0;
assign acr_n            = audio_enable_video ? acr_n_raw            : 20'd0;

always @(posedge clk or posedge ctrl_rst) begin
    if (ctrl_rst)
        camera_mode_request <= 1'b0;
    else
        camera_mode_request <= link_menu_state[5] && imu_link_alive && !sleep_mode;
end

assign menu_corner_show = link_menu_state[3] && imu_link_alive && !sleep_mode;

assign test_osd_show = link_test_mode && imu_link_alive && !sleep_mode;
assign test_freeze   = test_osd_show && (link_test_phase == 2'd3);
assign test_alarm_release = test_osd_show && (link_test_phase != 2'd2);

always @(posedge sd_card_clk or posedge sdcard_rst) begin
    if (sdcard_rst) begin
        flow_sd_meta  <= 9'd0;
        flow_sd_sync  <= 9'd0;
        flow_evt_sd_d <= 7'd0;
        page_chime_tgl_sd <= 1'b0;
    end else begin
        flow_sd_meta  <= {sleep_sd ? flow_sd_meta[8] : (link_test_mode && imu_link_alive),
                          link_flow_evt_toggle,
                          sleep_sd ? flow_sd_meta[0] : (link_flow_mode && imu_link_alive)};
        flow_sd_sync  <= flow_sd_meta;
        flow_evt_sd_d <= flow_sd_sync[7:1];
        if (transition_index_commit)
            page_chime_tgl_sd <= ~page_chime_tgl_sd;
    end
end

assign flow_evt_pulse_sd = sleep_sd ? 7'd0 : (flow_sd_sync[7:1] ^ flow_evt_sd_d);

// 未勾全时 sd_card_bmp 禁止手动下一页（轮播不受限）。
flow_checklist u_flow_checklist (
    .clk        (sd_card_clk),
    .rst        (sdcard_rst),
    .flow_mode  (flow_sd_sync[0]),
    .page_ready (display_valid && !camera_pause_tf && !transition_busy && !sleep_sd),
    .page       (disp_buf_idx),
    .knob_cw    (flow_evt_pulse_sd[4]),
    .knob_ccw   (flow_evt_pulse_sd[5]),
    .knob_press (flow_evt_pulse_sd[6]),
    .lock       (flow_lock_sd),
    .osd_show   (chk_show_sd),
    .osd_step2  (chk_step2_sd),
    .osd_cursor (chk_cursor_sd),
    .osd_mask   (chk_mask_sd)
);

always @(posedge ext_mem_clk or posedge mem_rst) begin
    if (mem_rst)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

always @(posedge ext_mem_clk or posedge mem_rst) begin
    if (mem_rst) begin
        transition_dest_mem_meta <= 3'd0;
        transition_dest_mem_sync <= 3'd0;
    end else begin
        transition_dest_mem_meta <= transition_dest_idx;
        transition_dest_mem_sync <= transition_dest_mem_meta;
    end
end

assign flip_read_addr = (transition_dest_mem_sync == 3'd0) ? BUF0_ADDR :
                        (transition_dest_mem_sync == 3'd1) ? BUF1_ADDR :
                        (transition_dest_mem_sync == 3'd2) ? BUF2_ADDR :
                        (transition_dest_mem_sync == 3'd3) ? BUF3_ADDR :
                        (transition_dest_mem_sync == 3'd4) ? BUF4_ADDR : BUF5_ADDR;

sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),
    .SCAN_TARGET_COUNT (3'd6),
    .FRAME_PIXELS      (FRAME_PIXELS)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (sdcard_rst),
    .system_ready      (sdram_init_done_sd),
    .key_next          (1'b1),
    .key_auto          (1'b1),
    .screen_next_pulse (1'b0),
    .pause_background  (camera_pause_tf),
    .camera_mode_active(camera_mode_sd),
    .flow_mode         (flow_sd_sync[0]),
    .test_mode         (flow_sd_sync[8]),
    .flow_next_pulse   (flow_evt_pulse_sd[0]),
    .flow_prev_pulse   (flow_evt_pulse_sd[1]),
    .flow_play_pulse   (flow_evt_pulse_sd[2]),
    .flow_stop_pulse   (flow_evt_pulse_sd[3]),
    .flow_lock         (flow_lock_sd),
    .write_path_idle   (sd_card_write_path_idle),
    .state_code        (state_code),
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .write_fifo_full   (write_fifo_full),
    .disp_res_valid    (disp_res_valid),
    .disp_res_w        (disp_res_w),
    .disp_res_h        (disp_res_h),
    .display_valid     (display_valid),
    .osd_auto_enabled  (osd_auto_enabled),
    .osd_error_code    (osd_error_code),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_abort_toggle(frame_write_abort_toggle),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),
    .transition_busy   (transition_busy),
    .transition_commit (transition_commit),
    .transition_start  (transition_start),
    .transition_mode   (transition_mode),
    .transition_target (transition_target),
    .transition_index_commit(transition_index_commit),
    .transition_dest_idx(transition_dest_idx),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .music_enable      (music_on_sd),
    .music_hold        (music_hold_sd),
    .alarm_select      (storm_alarm_sd),
    .music_byte_valid  (music_byte_valid_sd),
    .music_byte        (music_byte_sd),
    .music_byte_hi     (music_byte_hi_sd),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso)
);

seg_decoder seg_decoder_m0(
    .bin_data          (state_code),
    .seg_data          (seg_data_0)
);

ax_debounce #(
    .N        (32),
    .FREQ     (100),
    .MAX_TIME (20)
) u_transfx_key_debounce (
    .clk            (sd_card_clk),
    .rst            (sdcard_rst),
    .button_in      (key2),
    .button_posedge (),
    .button_negedge (transfx_key_press),
    .button_out     ()
);

always @(posedge sd_card_clk or posedge sdcard_rst) begin
    if (sdcard_rst)
        transition_mode <= 2'd0;
    else if (transfx_key_press && !sleep_sd)
        transition_mode <= (transition_mode == 2'd2) ? 2'd0 : transition_mode + 2'd1;
end

// 同步到 clk 域只供数码管显示（偶尔一拍中间值肉眼不可见）
always @(posedge clk or negedge ctrl_rst_n) begin
    if (!ctrl_rst_n) begin
        transition_mode_meta <= 2'd0;
        transition_mode_seg  <= 2'd0;
    end else begin
        transition_mode_meta <= transition_mode;
        transition_mode_seg  <= transition_mode_meta;
    end
end

seg_decoder u_transfx_seg_decoder (
    .bin_data ({2'b00, transition_mode_seg} + 4'd1),
    .seg_data (transfx_seg)
);

seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (ctrl_rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        ({1'b1,transfx_seg}),
    .seg_data_1        (8'hff),
    .seg_data_2        (8'hff),
    .seg_data_3        (8'hff),
    .seg_data_4        (8'hff),
    .seg_data_5        ({1'b1,seg_data_0})
);

video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (video_rst),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (video_rst),
    .read_en           (video_read_en),
    .read_data         (video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs),
    .vs_r              (vs),
    .de_r              (de),
    .vout_data         (vout_data_raw)
);

ov5640_camera #(
    .CLK_HZ(50_000_000),
    .WIDTH (640),
    .HEIGHT(480)
) u_ov5640_camera (
    .clk              (clk),
    .rst_n            (ctrl_rst_n),
    .enable           (camera_mode_request),
    .probe            (cam_probe),
    .sleep            (sleep_mode),
    .capture_allowed  (camera_capture_allowed),
    .cam_pclk         (cam_pclk),
    .cam_vsync        (cam_vsync),
    .cam_href         (cam_href),
    .cam_data         (cam_data),
    .cam_pwdn         (cam_pwdn),
    .cam_reset_n      (cam_reset_n),
    .cam_scl          (cam_scl),
    .cam_sda          (cam_sda),
    .init_done        (camera_init_done),
    .readback_code    (camera_readback_code),
    .pixel_valid      (camera_pixel_valid),
    .pixel_rgb565     (camera_pixel_rgb565),
    .pixel_addr       (camera_pixel_addr),
    .frame_start_toggle(camera_frame_start_toggle),
    .frame_done_toggle(camera_frame_done_toggle)
);

camera_sdram_stream #(.FRAME_PIXELS(307200)) u_camera_sdram_stream (
    .sd_clk              (sd_card_clk),
    .sd_rst_n            (sdcard_rst_n),
    .cam_pclk            (cam_pclk),
    .mode_request        (camera_mode_sd),
    .tf_write_idle       (sd_card_write_path_idle),
    .write_req_ack       (sd_card_write_req_ack),
    .write_finish_toggle (frame_write_toggle_mem),
    .write_fifo_full     (write_fifo_full),
    .display_ack_toggle (camera_display_ack_toggle),
    .pixel_valid         (camera_pixel_valid),
    .pixel_rgb565        (camera_pixel_rgb565),
    .frame_start_toggle  (camera_frame_start_toggle),
    .frame_done_toggle   (camera_frame_done_toggle),
    .pause_tf            (camera_pause_tf),
    .camera_owner        (camera_owner_sd),
    .capture_allowed     (camera_capture_allowed),
    .write_req           (camera_sdram_write_req),
    .write_en            (camera_sdram_write_en),
    .write_data          (camera_sdram_write_data),
    .write_abort_toggle  (camera_sdram_abort_toggle),
    .write_bank          (camera_write_bank),
    .published_bank      (camera_published_bank),
    .published_toggle    (camera_published_toggle)
);

camera_video_select u_camera_video_select (
    .video_clk             (video_clk),
    .video_rst_n           (video_rst_n),
    .mode_request_async    (camera_mode_request),
    .published_toggle_async(camera_published_toggle),
    .published_bank_async  (camera_published_bank),
    .raw_vsync             (vs_0),
    .delayed_vsync         (vs),
    .read_mode             (camera_read_mode),
    .read_bank             (camera_read_bank),
    .video_mode            (camera_mode_video),
    .frame_available       (camera_frame_available),
    .display_ack_toggle    (camera_display_ack_toggle)
);

always @(posedge sd_card_clk or negedge sdcard_rst_n) begin
    if (!sdcard_rst_n) begin
        imu_wave_page_sd     <= 1'b0;
        bright_page_sd       <= 1'b0;
        status_page_sd       <= 1'b0;
        status_gyro_only_sd  <= 1'b0;
        storm_page_sd        <= 1'b0;
        res_show_sd          <= 1'b0;
    end else begin
        res_show_sd          <= disp_res_valid && !camera_pause_tf && !transition_busy && !sleep_sd;
        imu_wave_page_sd     <= display_valid && (disp_buf_idx == 3'd1) &&
                                !camera_pause_tf;
        bright_page_sd       <= display_valid && (disp_buf_idx == 3'd0) &&
                                !camera_pause_tf && !transition_busy;
        status_page_sd       <= display_valid && (disp_buf_idx[2:1] == 2'd0) &&
                                !camera_pause_tf && !transition_busy;
        status_gyro_only_sd  <= (disp_buf_idx == 3'd1);
        storm_page_sd        <= display_valid && (disp_buf_idx == 3'd1) &&
                                !camera_pause_tf && !transition_busy;
    end
end

sync_2ff u_imu_wave_page_vid_sync (
    .clk      (video_clk),
    .rst_n    (video_rst_n),
    .async_in (imu_wave_page_sd),
    .sync_out (imu_wave_page_vid)
);

reset_sync u_imu_page_reset (
    .clk        (clk),
    .ext_rst_n  (ctrl_rst_n && imu_wave_page_sd && !camera_mode_request),
    .rst_n_sync (imu_run_rst_n)
);

reset_sync u_imu_osd_page_reset (
    .clk        (video_clk),
    .ext_rst_n  (video_rst_n && imu_run_rst_n && !camera_mode_video),
    .rst_n_sync (imu_osd_rst_n)
);

video_transition_4way #(
    .FADE_FRAMES (16),
    .FRAME_WIDTH (640),
    .FRAME_HEIGHT(480)
) u_video_transition_4way (
    .ctrl_clk         (sd_card_clk),
    .ctrl_rst_n       (sdcard_rst_n),
    .switch_start     (transition_start),
    .switch_target    (transition_target),
    .index_commit     (transition_index_commit),
    .switch_commit    (transition_commit),
    .transition_busy  (transition_busy),
    .pix_clk          (video_clk),
    .pix_rst_n        (video_rst_n),
    .player_active    (display_valid),
    .vsync            (vs),
    .de               (de),
    .rgb_in           (vout_data_raw),
    .rgb_out          (vout_data_transition),
    .transition_kind  (transition_kind),
    .flip_read_en     (flip_read_en),
    .flip_read_fold   (flip_read_fold)
);

wire display_valid_vid;
wire camera_mode_request_vid;

sync_2ff u_display_valid_vid_sync (
    .clk      (video_clk),
    .rst_n    (video_rst_n),
    .async_in (display_valid),
    .sync_out (display_valid_vid)
);

sync_2ff u_camera_mode_request_vid_sync (
    .clk      (video_clk),
    .rst_n    (video_rst_n),
    .async_in (camera_mode_request),
    .sync_out (camera_mode_request_vid)
);

assign vout_data_base = camera_mode_video ?
                        (camera_frame_available ? vout_data_raw : 24'd0) :
                        (camera_mode_request_vid ? 24'd0 :
                         (display_valid_vid ? vout_data_transition : 24'd0));

assign vout_data_status = vout_data_base;

imu_gyro_wave_osd #(
    .PLOT_X0    (64),
    .PLOT_Y0    (93),
    .PLOT_W     (546),
    .PLOT_H     (201),
    .PEAK_X0    (25),
    .PEAK_Y0    (305),
    .PEAK_W     (589),
    .PEAK_H     (104)
) u_imu_gyro_wave_osd (
    .ctrl_clk       (clk),
    .ctrl_rst_n     (imu_run_rst_n),
    .attitude_valid (imu_attitude_valid && !test_freeze),
    .calibrated     (imu_calibrated),
    .roll_deg_x10   (imu_roll_deg_x10),
    .pitch_deg_x10  (imu_pitch_deg_x10),
    .yaw_deg_x10    (imu_yaw_deg_x10),
    .pix_clk        (video_clk),
    .pix_rst_n      (imu_osd_rst_n),
    .de             (de),
    .display_enable (imu_wave_page_vid),
    .test_show_async  (test_osd_show),
    .test_phase_async (link_test_phase),
    .test_secs_async  (link_test_secs),
    .test_score_async (link_test_score),
    .test_tilt_async  (link_test_tilt),
    .rgb_in         (vout_data_status),
    .rgb_out        (vout_data_wave)
);

// 第二页叠加姿态角波形和读数卡片；各叠加层按页使能。
assign vout_data = imu_osd_rst_n ? vout_data_wave : vout_data_status;

device_status_osd u_device_status_osd (
    .pix_clk       (video_clk),
    .pix_rst_n     (video_rst_n),
    .show_async    (status_page_sd),
    .gyro_only_async (status_gyro_only_sd),
    .gyro_ok_async (gyro_disp),
    .cam_ok_async  (cam_ok),
    .vsync         (vs),
    .de            (de),
    .rgb_in        (vout_data),
    .vsync_out     (vs_dev),
    .de_out        (de_dev),
    .rgb_out       (vout_data_dev)
);

gyro_storm_osd u_gyro_storm_osd (
    .ctrl_clk      (clk),
    .ctrl_rst_n    (imu_run_rst_n),
    .attitude_valid(imu_attitude_valid),
    .calibrated    (imu_calibrated),
    .roll_deg_x10  (imu_roll_deg_x10),
    .pitch_deg_x10 (imu_pitch_deg_x10),
    .yaw_deg_x10   (imu_yaw_deg_x10),
    .pix_clk       (video_clk),
    .pix_rst_n     (video_rst_n),
    .show_async    (storm_page_sd),
    .release_async (test_alarm_release),
    .vsync         (vs_dev),
    .de            (de_dev),
    .rgb_in        (vout_data_dev),
    .vis_band      (audio_vis_band),
    .vsync_out     (vs_storm),
    .de_out        (de_storm),
    .rgb_out       (vout_data_storm),
    .alarm_active  (storm_alarm_vid)
);

menu_corner_osd u_menu_corner_osd (
    .pix_clk      (video_clk),
    .pix_rst_n    (video_rst_n),
    .show_async   (bright_page_sd && menu_corner_show),
    .cursor_async (link_menu_state[2:0]),
    .blink_async  (link_menu_state[4]),
    .chk_show_async   (chk_show_sd),
    .chk_step2_async  (chk_step2_sd),
    .chk_cursor_async (chk_cursor_sd),
    .chk_mask_async   (chk_mask_sd),
    .vsync        (vs_storm),
    .de           (de_storm),
    .rgb_in       (vout_data_storm),
    .vsync_out    (vs_menu),
    .de_out       (de_menu),
    .rgb_out      (vout_data_menu)
);

// 模块内部寄存 4 拍，vs/de 同步延迟后一起送 RGB 转 AXIS。
screen_brightness_osd u_screen_brightness_osd (
    .pix_clk       (video_clk),
    .pix_rst_n     (video_rst_n),
    .pct_async     (bright_pct_hold),
    .vol_pct_async (volume_pct_hold),
    .show_async    (bright_page_sd),
    .vsync         (vs_menu),
    .de            (de_menu),
    .rgb_in        (vout_data_menu),
    .vsync_out     (vs_bright),
    .de_out        (de_bright),
    .rgb_out       (vout_data_bright)
);

wake_streak_fx #(
    .DIM_GAIN    (6)
) u_wake_streak_fx (
    .pix_clk     (video_clk),
    .pix_rst_n   (video_rst_n),
    .sleep_async (sleep_mode),
    .vsync       (vs_bright),
    .de          (de_bright),
    .rgb_in      (vout_data_bright),
    .vsync_out   (vs_wake),
    .de_out      (de_wake),
    .rgb_out     (vout_data_wake)
);

audio_spectrum_osd u_audio_spectrum_osd (
    .pix_clk        (video_clk),
    .pix_rst_n      (video_rst_n),
    .de             (de_wake),
    .audio_valid    (audio_valid),
    .audio_left     (audio_left_data),
    .audio_right    (audio_right_data),
    .vis_band       (audio_vis_band)
);

timestamp_roll_osd u_timestamp_roll_osd (
    .pix_clk     (video_clk),
    .pix_rst_n   (video_rst_n),
    .sleep_async (sleep_mode),
    .show_async  (bright_page_sd && menu_corner_show),
    .trig_async  (link_stamp_toggle),
    .time_async  (link_rtc_time),
    .vsync       (vs_wake),
    .de          (de_wake),
    .rgb_in      (vout_data_wake),
    .vsync_out   (vs_stamp),
    .de_out      (de_stamp),
    .rgb_out     (vout_data_stamp)
);

// 放在视频链最后，不随亮度 / 休眠变化；模块内部寄存 2 拍。
src_res_osd u_src_res_osd (
    .pix_clk     (video_clk),
    .pix_rst_n   (video_rst_n),
    .show_async  (res_show_sd),
    .w_async     (disp_res_w),
    .h_async     (disp_res_h),
    .vsync       (vs_stamp),
    .de          (de_stamp),
    .rgb_in      (vout_data_stamp),
    .vsync_out   (vs_res),
    .de_out      (de_res),
    .rgb_out     (vout_data_res)
);

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (mem_rst),
    .refresh_hold      (sdram_refresh_hold),
    .access_idle       (sdram_access_idle),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),
    .read_addr_0       (camera_read_mode ? CAM0_ADDR :
                        ((disp_buf_idx == 3'd4) ? BUF4_ADDR : BUF0_ADDR)),
    .read_addr_1       (camera_read_mode ? CAM1_ADDR :
                        ((disp_buf_idx == 3'd5) ? BUF5_ADDR : BUF1_ADDR)),
    .read_addr_2       (BUF2_ADDR),
    .read_addr_3       (BUF3_ADDR),
    .read_addr_index   (camera_read_mode ? {1'b0, camera_read_bank} : disp_buf_idx[1:0]),
    .read_len          (FRAME_PIXELS),
    .read_flip_en      (flip_read_en && !camera_read_mode),
    .read_flip_fold    (flip_read_fold),
    .read_flip_addr    (flip_read_addr),
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),
    .write_fifo_full   (write_fifo_full),

    .write_clk         (sd_card_clk),
    .write_v_flip      (!camera_owner_sd),
    .write_req         (camera_owner_sd ? camera_sdram_write_req : sd_card_write_req),
    .write_abort_toggle(camera_sdram_abort_toggle ^ frame_write_abort_toggle),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (camera_owner_sd ? CAM0_ADDR :
                        ((write_buf_idx == 3'd4) ? BUF4_ADDR : BUF0_ADDR)),
    .write_addr_1      (camera_owner_sd ? CAM1_ADDR :
                        ((write_buf_idx == 3'd5) ? BUF5_ADDR : BUF1_ADDR)),
    .write_addr_2      (BUF2_ADDR),
    .write_addr_3      (BUF3_ADDR),
    .write_addr_index  (camera_owner_sd ? {1'b0, camera_write_bank} : write_buf_idx[1:0]),
    .write_len         (FRAME_PIXELS),
    .write_en          (camera_owner_sd ? camera_sdram_write_en : sd_card_write_en),
    .write_data        (camera_owner_sd ? camera_sdram_write_data : sd_card_write_data)
);

sdram U3(
    .App_access_idle   (sdram_access_idle),
    .App_refresh_hold  (sdram_refresh_hold),
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (mem_rst),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

tf_audio_player u_tf_audio_player (
    .wr_clk      (sd_card_clk),
    .wr_rst      (sdcard_rst),
    .wr_play     (music_on_sd),
    .byte_valid  (music_byte_valid_sd),
    .byte_data   (music_byte_sd),
    .byte_hi     (music_byte_hi_sd),
    .hold        (music_hold_sd),
    .I_mclk      (audio_mclk),
    .I_rst       (audio_rst),
    .rd_play     (music_on_mclk),
    .O_i2s_BCLK  (music_i2s_bclk),
    .O_i2s_LRCK  (music_i2s_lrck),
    .O_i2s_DOUT  (music_i2s_dout)
);

assign audio_i2s_bclk = music_i2s_bclk;
assign audio_i2s_lrck = music_i2s_lrck;
assign audio_i2s_dout = music_i2s_dout;

I2S_receiver u_I2S_receiver(
    .I_clk              (video_clk),
    .I_rst              (audio_video_rst),
    .I_i2s_BCLK         (audio_i2s_bclk),
    .I_i2s_LRCK         (audio_i2s_lrck),
    .I_i2s_DOUT         (audio_i2s_dout),
    .O_audio_valid      (audio_valid_raw),
    .O_audio_left_data  (audio_left_data_raw),
    .O_audio_right_data (audio_right_data_raw)
);

page_chime u_page_chime (
    .clk         (video_clk),
    .rst_n       (video_rst_n),
    .trig_toggle (page_chime_tgl_sd),
    .page_async  (disp_buf_idx),
    .valid_in    (audio_valid),
    .left_in     (audio_left_data),
    .right_in    (audio_right_data),
    .valid_out   (audio_valid_chime),
    .left_out    (audio_left_chime),
    .right_out   (audio_right_chime)
);

audio_volume u_audio_volume (
    .clk       (video_clk),
    .rst_n     (video_rst_n),
    .pct_async (volume_pct_hold),
    .valid_in  (audio_valid_chime),
    .left_in   (audio_left_chime),
    .right_in  (audio_right_chime),
    .valid_out (audio_valid_vol),
    .left_out  (audio_left_vol),
    .right_out (audio_right_vol)
);

audio_arc_calculate #(
    .ACR_N (6144)
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (audio_video_rst),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid_raw),
    .O_acr_cts     (acr_cts_raw),
    .O_acr_n       (acr_n_raw)
);

video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (video_rst),
    .I_vs          (vs_res),
    .I_de          (de_res),
    .I_rgb         (vout_data_res),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (video_rst),
    .O_pulse (edid_trig)
);

// ===================== HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (video_rst),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid_vol),
    .I_audio_left_data  (audio_left_vol),
    .I_audio_right_data (audio_right_vol),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_pixel_rst        (video_rst),
    .I_serial_rst       (hdmi5x_rst),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

imu_link_rx #(
    .CLK_HZ (50_000_000),
    .BAUD   (115_200)
) u_imu_link_rx (
    .clk            (clk),
    .rst_n          (ctrl_rst_n),
    .rx             (link_rx),
    .sample_valid   (imu_sample_valid),
    .attitude_valid (imu_attitude_valid),
    .calibrated     (imu_calibrated),
    .sensor_error   (imu_sensor_error),
    .gx             (imu_gx),
    .gy             (imu_gy),
    .gz             (imu_gz),
    .roll_deg_x10   (imu_roll_deg_x10),
    .pitch_deg_x10  (imu_pitch_deg_x10),
    .yaw_deg_x10    (imu_yaw_deg_x10),
    .brightness_pct (link_brightness_pct),
    .volume_pct     (link_volume_pct),
    .frame_valid    (imu_frame_valid),
    .flow_mode      (link_flow_mode),
    .flow_evt_toggle(link_flow_evt_toggle),
    .test_mode      (link_test_mode),
    .test_phase     (link_test_phase),
    .test_secs      (link_test_secs),
    .test_score     (link_test_score),
    .test_tilt      (link_test_tilt),
    .menu_state     (link_menu_state),
    .rtc_time       (link_rtc_time),
    .stamp_toggle   (link_stamp_toggle)
);

always @(posedge clk or negedge ctrl_rst_n) begin
    if (!ctrl_rst_n) begin
        imu_link_timer <= 25'd0;
        imu_link_alive <= 1'b0;
    end else if (imu_frame_valid) begin
        imu_link_timer <= 25'd0;
        imu_link_alive <= 1'b1;
    end else if (imu_link_timer == 25'd24_999_999) begin
        imu_link_alive <= 1'b0;
    end else begin
        imu_link_timer <= imu_link_timer + 1'b1;
    end
end

assign gyro_ok = imu_link_alive && imu_calibrated && !imu_sensor_error;

always @(posedge clk or negedge ctrl_rst_n) begin
    if (!ctrl_rst_n) begin
        gyro_disp       <= 1'b0;
        gyro_hold_timer <= 29'd0;
    end else if (sleep_mode) begin
        gyro_disp       <= 1'b0;
        gyro_hold_timer <= 29'd0;
    end else if (gyro_ok) begin
        gyro_disp       <= 1'b1;
        gyro_hold_timer <= 29'd0;
    end else if (imu_link_alive && !imu_sensor_error && gyro_disp &&
                 gyro_hold_timer != 29'd499_999_999) begin
        gyro_hold_timer <= gyro_hold_timer + 1'b1;
    end else begin
        gyro_disp       <= 1'b0;
    end
end

always @(posedge clk or negedge ctrl_rst_n) begin
    if (!ctrl_rst_n) begin
        cam_probe       <= 1'b1;
        cam_probe_timer <= 27'd0;
        cam_rb_meta     <= 4'd0;
        cam_rb_sync     <= 4'd0;
        cam_rb_last     <= 4'd0;
        cam_ok          <= 1'b0;
        sleep_mode_d    <= 1'b0;
    end else begin
        cam_rb_meta  <= camera_readback_code;
        cam_rb_sync  <= cam_rb_meta;
        cam_rb_last  <= cam_rb_sync;
        sleep_mode_d <= sleep_mode;
        if (sleep_mode) begin
            cam_ok <= 1'b0;
        end else if (cam_rb_sync == cam_rb_last && cam_rb_sync != 4'd0) begin
            cam_ok    <= (cam_rb_sync == 4'd5);
            cam_probe <= 1'b0;
        end
        if (sleep_mode_d && !sleep_mode) begin
            cam_probe       <= 1'b1;
            cam_probe_timer <= 27'd0;
        end else if (cam_probe && !sleep_mode) begin
            if (cam_probe_timer == 27'd99_999_999)
                cam_probe <= 1'b0;
            else
                cam_probe_timer <= cam_probe_timer + 1'b1;
        end
    end
end

assign imu_uart_tx = link_rx;

// 休眠请求送 B 板（低有效；B 板该脚上拉，未接线时保持清醒）
assign sleep_link_n = ~sleep_mode;

endmodule
