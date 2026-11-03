# 双 FPGA 工业培训与姿态检测终端

基于两块 EG4S20BG256（安路 Tang Dynasty 6.2）的 HDMI 多媒体终端。两块板通过 UART（115200，27 字节帧）互联，**必须配套烧录**。

| 目录 | 板卡 | 功能 |
|---|---|---|
| `A_board/` | A 板 | TF 卡图片 / 音乐读取、多分辨率图片自适应缩放、SDRAM 帧缓存、HDMI 音视频输出、OV5640 摄像头、OSD 叠加与特效 |
| `B_board/` | B 板 | 旋钮菜单、按键、IMU963RA 姿态检测、数码管 RTC、链路发送 |

## 编译

用 TD 打开 `A_board/src/td_project/lab_ex5_i2s_v1.0.al`、`B_board/src/td_project/lab_ex5_i2s_v1.0.al` 直接编译。
A 板所需的综合 / 布局布线参数已写在工程文件中。工程目录请放在**英文路径**下编译。

## TF 卡

把 `A_board/tf_card/` 下的文件拷到 TF 卡根目录（FAT32）：

| 文件 | 说明 |
|---|---|
| `PIC1.BMP` ~ `PIC6.BMP` | 24 位 BMP，支持 320×240 ~ 1280×960，加载时自动缩放到 640×480 |
| `MUSIC.PCM` | 背景音乐，48kHz 16 位 PCM |
| `ALARM.PCM` | 报警音，48kHz 16 位 PCM |

## 仿真

`A_board/sim/`、`B_board/sim/` 为 iverilog 测试平台。
