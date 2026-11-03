
# ============================================================
# merged_top_tf_hdmi_audio.sdc
# 用于：TF 图片轮播 + HDMI(视频+音频) 整合工程
#
# 合并原则：
# 1) 保留 top.sdc 里的系统主时钟与 derive_clocks
# 2) 把 timing(2).sdc 里的 pixel/serial 时钟改成整合工程实际名字
# 3) 增加 hdmi_5x_clk / audio_mclk 的命名
# 4) 不再对内部 net 直接 create_clock，避免和 derive_clocks 重复
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    你的原 top.sdc 里 clk=50MHz，所以这里仍然用 20ns
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
# ------------------------------------------------------------
derive_clocks

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    下面这些路径是按你当前工程实例名写的：
#      sys_pll      -> sys_pll_m0
#      video_pll    -> video_pll_m0
#      音频 PLL     -> u_audio_pll
#
#    如果综合后实例路径略有不同，就按 Timing Analyzer 里的实际 pin 路径改。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {audio_mclk}  -source [get_ports {clk}] -master_clock {clk} [get_pins {u_audio_pll/pll_inst.clkc[2]}]


# ------------------------------------------------------------
# 4. 从原 timing(2).sdc 迁移过来的 HDMI 像素/串行时钟约束
#
#    原文件是：
#      create_clock -name pixel_clk  ... [get_nets {S_pixel_clk}]
#      create_clock -name serial_clk ... [get_nets {S_serial_clk}]
#      set_clock_groups -exclusive ...
#
#    在整合工程里，这两个时钟已经由 video_pll 产生，并通过 derive_clocks 建出来了，
#    所以这里只保留“组关系”，不再重复 create_clock。
#
#    这里沿用原作者的写法，把 video_clk 和 hdmi_5x_clk 设成 exclusive。
#    如果后面 Timing Analyzer 明确显示两者之间需要做同步时序分析，再把这一条去掉。
# ------------------------------------------------------------
set_clock_groups -exclusive \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]

# 图片编号从 sd_card_clk 域进入独立 audio_mclk 域，首级由双触发器同步。
# 只排除同步器首级的异步采样路径，后级仍按 audio_mclk 正常分析。
set_false_path -to [get_regs -nowarn {*/S_media_id_meta[*]}]

# TF 音乐环形缓冲的读写指针以格雷码 2FF 跨 sd_card_clk / audio_mclk，
# 每次只变 1 位，只排除同步器首级，后级仍按本域正常分析。
set_false_path -to [get_regs -nowarn {*/rd_gray_meta[*]}]
set_false_path -to [get_regs -nowarn {*/wr_gray_meta[*]}]

# KEY3 音频开关从 50MHz 控制域分别进入音频和视频域。
# 只排除两条双触发器同步链的首级。
set_false_path -to [get_regs -nowarn {audio_enable_mclk_sync[0]}]
set_false_path -to [get_regs -nowarn {audio_enable_video_sync[0]}]

# 翻页提示音的页号从 sd_card_clk 域进入 video_clk 域：翻转位触发前已稳定，只排除同步器首级。
set_false_path -to [get_regs -nowarn {*/page_meta[*]}]

# 写取消事件从 sd_card_clk 域以 toggle 方式进入 ext_mem_clk 域。
# 只排除同步器首级，后两级和取消脉冲逻辑仍按 ext_mem_clk 分析。
set_false_path -to [get_regs -nowarn {*/write_abort_sync[0]}]

# 帧写完成 toggle 和写请求应答进入 sd_card_clk 域的同步器首级。
set_false_path -to [get_regs -nowarn {*/wrfin_tgl_sync[0]}]
set_false_path -to [get_regs -nowarn {*/write_req_ack_sync[0]}]

# Per-domain asynchronous reset assertion is not a synchronous data path.
# Restrict the exception to the reset source, so reset_pipe[0] ->
# reset_pipe[1] remains a normally timed data path.
set_false_path \
    -from [get_regs -nowarn {rst_all_reg}] \
    -to   [get_regs -nowarn {*/reset_pipe[*]}]

# SDRAM-ready is a slow level crossing into sd_card_clk. Only its first
# sampling stage is asynchronous; sync_pipe[0] -> sync_pipe[1] remains timed.
set_false_path -to [get_regs -nowarn {*/sync_pipe[0]}]

# 翻书转场：video_clk 域的 {flip_read_en, flip_read_fold} 只在帧尾变化，sd_card_clk 域的
# 目标页号在发起转场时变化，读模块都要到之后的帧请求才锁存，只排除同步器第一级。
set_false_path -to [get_regs -nowarn {*/read_flip_meta[*]}]
set_false_path -to [get_regs -nowarn {transition_dest_mem_meta[*]}]



# ------------------------------------------------------------
# 6. 可选：如果你的 frame_read_write 内部异步 FIFO 已经完全处理好了跨域，
#    且报告里总出现 video_clk / sd_card_clk / ext_mem_clk 之间的大量伪违例，
#    可以打开下面这组异步分组。
#
#    建议先不要开，先看报告；
#    只有确认这些跨域都只经过 FIFO / 同步器时再开。
# ------------------------------------------------------------
# set_clock_groups -asynchronous \
#     -group [get_clocks {sd_card_clk}] \
#     -group [get_clocks {ext_mem_clk}] \
#     -group [get_clocks {video_clk}] \
#     -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 7. 其余 Input/Output delay 先保持空白
#    你当前两个原始 sdc 也都没有写这部分。
# ------------------------------------------------------------

# ------------------------------------------------------------
# 8. OV5640 DVP input timing only
#    RGB565 configuration produces an approximately 50 MHz PCLK from the
#    board's existing 25 MHz XCLK. Camera capture is asynchronous to HDMI.
# ------------------------------------------------------------
create_clock -name cam_pclk -period 20.000 -waveform {0 10} [get_ports {cam_pclk}]

set_input_delay -clock {cam_pclk} -min 0.000 \
    [get_ports {cam_data[*] cam_href cam_vsync}]
set_input_delay -clock {cam_pclk} -max 8.000 \
    [get_ports {cam_data[*] cam_href cam_vsync}]

set_clock_groups -asynchronous \
    -group [get_clocks {cam_pclk}] \
    -group [get_clocks {clk sd_card_clk ext_mem_clk video_clk hdmi_5x_clk audio_mclk}]

# Only the first stages of camera control/status synchronizers are excluded.
set_false_path -to [get_regs -nowarn {*/enable_pclk_sync[0]}]
set_false_path -to [get_regs -nowarn {*/init_pclk_sync[0]}]
set_false_path -to [get_regs -nowarn {*/mode_sync[0]}]
set_false_path -to [get_regs -nowarn {*/frame_toggle_sync[0]}]
