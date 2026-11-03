module fat32_bmp_loader #(
    parameter integer FRAME_PIXELS = 307200
)(
    input             clk,
    input             rst,
    input             load_start,
    input             load_abort,
    input      [31:0] first_cluster,
    input      [31:0] directory_file_size,
    input      [7:0]  sectors_per_cluster,
    input      [31:0] fat_start_lba,
    input      [31:0] data_start_lba,
    input      [31:0] max_cluster,
    input      [15:0] bmp_width,        // 已不使用（保留端口兼容）
    input      [15:0] bmp_height,
    input             write_fifo_full,  // SDRAM 写 FIFO 满（本时钟域）
    output     [10:0] src_w,            // 本张图原分辨率（加载成功后有效）
    output     [10:0] src_h,
    output            ready,
    output reg        load_done,
    output reg        load_error,
    output reg [2:0]  bmp_error,
    output reg [3:0]  state_code,

    output reg        write_req,
    input             write_req_ack,
    output            bmp_data_wr_en,
    output     [23:0] bmp_data,

    output            sector_req,
    output     [31:0] sector_lba,
    input             sector_busy,
    input             sector_valid,
    input      [7:0]  sector_byte,
    input      [8:0]  sector_byte_index,
    input             sector_done,
    input      [7:0]  sector_error
);

localparam [2:0] ERR_NONE      = 3'd0;
localparam [2:0] ERR_HEADER    = 3'd1;
localparam [2:0] ERR_SD_PATH   = 3'd2;
localparam [2:0] ERR_TRUNCATED = 3'd3;
localparam [2:0] ERR_FAT       = 3'd4;
localparam [2:0] ERR_SCALE     = 3'd5;   // 分辨率不在支持范围 / 缩放出错

localparam ST_IDLE       = 3'd0;
localparam ST_HEADER     = 3'd1;
localparam ST_VALIDATE   = 3'd2;
localparam ST_WRITE_WAIT = 3'd3;
localparam ST_DATA       = 3'd4;
localparam ST_FINISH     = 3'd5;
localparam ST_CHECK      = 3'd6;
localparam ST_CFG        = 3'd7;

localparam [31:0] EXPECTED_PIXEL_BYTES = FRAME_PIXELS * 3;

reg [2:0]  state;
reg        streamer_start;
reg        streamer_abort;
reg [31:0] stream_size;
reg [31:0] cluster_latched;
reg [31:0] directory_size_latched;
reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] header_file_size;
reg [31:0] pixel_offset;
reg [31:0] dib_header_size;
reg [31:0] width;
reg [31:0] height;
reg [15:0] planes;
reg [15:0] bit_count;
reg [31:0] compression;
reg [1:0]  pixel_byte_index;
reg [15:0] pix_acc;            // 已收到的 B / G 字节
reg [10:0] pix_col;            // 行内像素号
reg [1:0]  pad_left;           // 行尾还需跳过的填充字节
reg        cfg_wait;           // ST_CFG 内：0 = 等参数，1 = 等输出完成
reg        cfg_pulse, cfg_seen;
reg        scaler_run;
reg        format_ok;
reg        length_pre_ok;
reg [31:0] pixel_room;
reg [15:0] pixel_skip;         // BMP 像素偏移（54 起，头部校验保证 < 64K）

wire       streamer_busy;
wire       streamer_done;
wire       streamer_valid;
wire [7:0] streamer_byte;
wire [31:0] streamer_byte_offset;
wire [7:0] streamer_error;
wire       header_format_valid;
wire       pixel_stream_byte;

assign ready = (state == ST_IDLE);
assign header_format_valid = (header_0 == "B") && (header_1 == "M") &&
                             (dib_header_size >= 32'd40) &&
                             (width[31:11] == 21'd0) && (height[31:11] == 21'd0) &&   // 正高度（自下而上）且 < 2048
                             (pixel_offset[31:16] == 16'd0) &&
                             (planes == 16'd1) && (bit_count == 16'd24) &&
                             (compression == 32'd0) &&
                             (pixel_offset >= 32'd54);
// 收齐 H 行后其余字节忽略
wire       scaler_busy, scaler_ok, scaler_rows_done, scaler_done, scaler_err;
assign pixel_stream_byte = streamer_valid && (pixel_skip == 16'd0) && (pad_left == 2'd0) && !scaler_rows_done;
wire       pixel_word = (state == ST_DATA) && pixel_stream_byte && (pixel_byte_index == 2'd2);
assign src_w = width[10:0];
assign src_h = height[10:0];

bmp_fit_scaler u_scaler (
    .clk           (clk),
    .rst           (rst),
    .cfg_start     (cfg_pulse),
    .cfg_w         (width[10:0]),
    .cfg_h         (height[10:0]),
    .cfg_busy      (scaler_busy),
    .cfg_ok        (scaler_ok),
    .abort         (load_abort || (state == ST_IDLE)),
    .run           (scaler_run),
    .src_valid     (pixel_word),
    .src_pix       ({streamer_byte, pix_acc}),           // {R, G, B}
    .out_full      (write_fifo_full),
    .out_en        (bmp_data_wr_en),
    .out_pix       (bmp_data),
    .src_rows_done (scaler_rows_done),
    .done          (scaler_done),
    .err           (scaler_err)
);

fat32_file_streamer u_streamer (
    .clk                 (clk),
    .rst                 (rst),
    .abort               (streamer_abort),
    .file_start          (streamer_start),
    .first_cluster       (cluster_latched),
    .file_size           (stream_size),
    .sectors_per_cluster (sectors_per_cluster),
    .fat_start_lba       (fat_start_lba),
    .data_start_lba      (data_start_lba),
    .max_cluster         (max_cluster),
    .file_busy           (streamer_busy),
    .file_done           (streamer_done),
    .file_valid          (streamer_valid),
    .file_byte           (streamer_byte),
    .file_byte_offset    (streamer_byte_offset),
    .fs_error            (streamer_error),
    .sector_req          (sector_req),
    .sector_lba          (sector_lba),
    .sector_busy         (sector_busy),
    .sector_valid        (sector_valid),
    .sector_byte         (sector_byte),
    .sector_byte_index   (sector_byte_index),
    .sector_done         (sector_done),
    .sector_error        (sector_error)
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state                  <= ST_IDLE;
        streamer_start         <= 1'b0;
        streamer_abort         <= 1'b0;
        stream_size            <= 32'd0;
        cluster_latched        <= 32'd0;
        directory_size_latched <= 32'd0;
        header_0               <= 8'd0;
        header_1               <= 8'd0;
        header_file_size       <= 32'd0;
        pixel_offset           <= 32'd54;
        dib_header_size        <= 32'd0;
        width                  <= 32'd0;
        height                 <= 32'd0;
        planes                 <= 16'd0;
        bit_count              <= 16'd0;
        compression            <= 32'd0;
        pixel_byte_index       <= 2'd0;
        pix_acc                <= 16'd0;
        pix_col                <= 11'd0;
        pad_left               <= 2'd0;
        cfg_wait               <= 1'b0;
        cfg_pulse              <= 1'b0;
        cfg_seen               <= 1'b0;
        scaler_run             <= 1'b0;
        format_ok              <= 1'b0;
        length_pre_ok          <= 1'b0;
        pixel_room             <= 32'd0;
        pixel_skip             <= 16'd0;
        load_done              <= 1'b0;
        load_error             <= 1'b0;
        bmp_error              <= ERR_NONE;
        state_code             <= 4'd1;
        write_req              <= 1'b0;
    end else begin
        streamer_start <= 1'b0;
        streamer_abort <= 1'b0;
        load_done      <= 1'b0;
        load_error     <= 1'b0;
        cfg_pulse      <= 1'b0;

        if (load_abort) begin
            streamer_abort <= 1'b1;
            write_req      <= 1'b0;
            scaler_run     <= 1'b0;
            state           <= ST_IDLE;
            state_code      <= 4'd1;
        end else begin
            if (state == ST_HEADER && streamer_valid) begin
                case (streamer_byte_offset)
                    32'd0:  header_0 <= streamer_byte;
                    32'd1:  header_1 <= streamer_byte;
                    32'd2:  header_file_size[7:0]   <= streamer_byte;
                    32'd3:  header_file_size[15:8]  <= streamer_byte;
                    32'd4:  header_file_size[23:16] <= streamer_byte;
                    32'd5:  header_file_size[31:24] <= streamer_byte;
                    32'd10: pixel_offset[7:0]       <= streamer_byte;
                    32'd11: pixel_offset[15:8]      <= streamer_byte;
                    32'd12: pixel_offset[23:16]     <= streamer_byte;
                    32'd13: pixel_offset[31:24]     <= streamer_byte;
                    32'd14: dib_header_size[7:0]    <= streamer_byte;
                    32'd15: dib_header_size[15:8]   <= streamer_byte;
                    32'd16: dib_header_size[23:16]  <= streamer_byte;
                    32'd17: dib_header_size[31:24]  <= streamer_byte;
                    32'd18: width[7:0]              <= streamer_byte;
                    32'd19: width[15:8]             <= streamer_byte;
                    32'd20: width[23:16]            <= streamer_byte;
                    32'd21: width[31:24]            <= streamer_byte;
                    32'd22: height[7:0]             <= streamer_byte;
                    32'd23: height[15:8]            <= streamer_byte;
                    32'd24: height[23:16]           <= streamer_byte;
                    32'd25: height[31:24]           <= streamer_byte;
                    32'd26: planes[7:0]             <= streamer_byte;
                    32'd27: planes[15:8]            <= streamer_byte;
                    32'd28: bit_count[7:0]          <= streamer_byte;
                    32'd29: bit_count[15:8]         <= streamer_byte;
                    32'd30: compression[7:0]        <= streamer_byte;
                    32'd31: compression[15:8]       <= streamer_byte;
                    32'd32: compression[23:16]      <= streamer_byte;
                    32'd33: compression[31:24]      <= streamer_byte;
                    default: ;
                endcase
            end

            if ((state == ST_DATA) && streamer_valid) begin
                if (pixel_skip != 16'd0)
                    pixel_skip <= pixel_skip - 16'd1;
                else if (pad_left != 2'd0)
                    pad_left <= pad_left - 2'd1;
            end

            if ((state == ST_DATA) && pixel_stream_byte) begin
                case (pixel_byte_index)
                    2'd0: begin
                        pix_acc[7:0]     <= streamer_byte;
                        pixel_byte_index <= 2'd1;
                    end
                    2'd1: begin
                        pix_acc[15:8]    <= streamer_byte;
                        pixel_byte_index <= 2'd2;
                    end
                    default: begin
                        pixel_byte_index <= 2'd0;
                        if (pix_col == width[10:0] - 11'd1) begin
                            pix_col  <= 11'd0;
                            pad_left <= width[1:0];          // 行字节数 3W 对齐到 4 需补 W & 3 个字节
                        end else
                            pix_col <= pix_col + 11'd1;
                    end
                endcase
            end

            case (state)
                ST_IDLE: begin
                    state_code <= 4'd1;
                    write_req <= 1'b0;
                    if (load_start) begin
                        cluster_latched        <= first_cluster;
                        directory_size_latched <= directory_file_size;
                        stream_size            <= (directory_file_size < 32'd512) ?
                                                  directory_file_size : 32'd512;
                        header_0               <= 8'd0;
                        header_1               <= 8'd0;
                        header_file_size       <= 32'd0;
                        pixel_offset           <= 32'd54;
                        dib_header_size        <= 32'd0;
                        width                  <= 32'd0;
                        height                 <= 32'd0;
                        planes                 <= 16'd0;
                        bit_count              <= 16'd0;
                        compression            <= 32'd0;
                        pixel_byte_index       <= 2'd0;
                        bmp_error              <= ERR_NONE;
                        streamer_start         <= 1'b1;
                        state                  <= ST_HEADER;
                    end
                end

                ST_HEADER: begin
                    state_code <= 4'd2;
                    if (streamer_done) begin
                        if (streamer_error != 8'h00) begin
                            bmp_error  <= (streamer_error[7:4] == 4'h1) ?
                                          ERR_SD_PATH : ERR_FAT;
                            load_error <= 1'b1;
                            state      <= ST_IDLE;
                        end else begin
                            state <= ST_VALIDATE;
                        end
                    end
                end

                ST_VALIDATE: begin
                    format_ok     <= header_format_valid;
                    length_pre_ok <= (directory_size_latched >= 32'd54) &&
                                     (header_file_size <= directory_size_latched) &&
                                     (pixel_offset < header_file_size);
                    pixel_room    <= header_file_size - pixel_offset;
                    state         <= ST_CHECK;
                end

                ST_CHECK: begin
                    if (!format_ok) begin
                        bmp_error  <= ERR_HEADER;
                        load_error <= 1'b1;
                        state      <= ST_IDLE;
                    end else if (!length_pre_ok) begin
                        bmp_error  <= ERR_TRUNCATED;
                        load_error <= 1'b1;
                        state      <= ST_IDLE;
                    end else begin
                        // 先让缩放器按 W x H 算参数（约 50 拍），支持再申请写通道
                        cfg_pulse <= 1'b1;
                        cfg_seen  <= 1'b0;
                        cfg_wait  <= 1'b0;
                        state     <= ST_CFG;
                    end
                end

                ST_CFG: begin
                    if (!cfg_wait) begin
                        // 等参数：cfg_busy 先升起（cfg_seen）再落下
                        if (scaler_busy)
                            cfg_seen <= 1'b1;
                        else if (cfg_seen) begin
                            if (scaler_ok) begin
                                write_req <= 1'b1;
                                state     <= ST_WRITE_WAIT;
                            end else begin
                                bmp_error  <= ERR_SCALE;
                                load_error <= 1'b1;
                                state      <= ST_IDLE;
                            end
                        end
                    end else begin
                        // 源文件已读完：等缩放器把 640 x 480 全部写出
                        if (scaler_err) begin
                            bmp_error  <= ERR_SCALE;
                            load_error <= 1'b1;
                            scaler_run <= 1'b0;
                            state      <= ST_FINISH;
                        end else if (scaler_done) begin
                            load_done  <= 1'b1;
                            scaler_run <= 1'b0;
                            state      <= ST_FINISH;
                        end
                    end
                end

                ST_WRITE_WAIT: begin
                    state_code <= 4'd3;
                    if (write_req_ack) begin
                        write_req        <= 1'b0;
                        stream_size      <= header_file_size;
                        pixel_byte_index <= 2'd0;
                        pix_col          <= 11'd0;
                        pad_left         <= 2'd0;
                        pixel_skip       <= pixel_offset[15:0];
                        scaler_run       <= 1'b1;
                        streamer_start   <= 1'b1;
                        state            <= ST_DATA;
                    end
                end

                ST_DATA: begin
                    state_code <= 4'd4;
                    if (scaler_err) begin
                        bmp_error      <= ERR_SCALE;
                        load_error     <= 1'b1;
                        scaler_run     <= 1'b0;
                        streamer_abort <= 1'b1;
                        state          <= ST_FINISH;
                    end else if (streamer_done) begin
                        if (streamer_error != 8'h00) begin
                            bmp_error  <= (streamer_error[7:4] == 4'h1) ?
                                          ERR_SD_PATH : ERR_FAT;
                            load_error <= 1'b1;
                            scaler_run <= 1'b0;
                            state      <= ST_FINISH;
                        end else if (!scaler_rows_done) begin
                            bmp_error  <= ERR_TRUNCATED;
                            load_error <= 1'b1;
                            scaler_run <= 1'b0;
                            state      <= ST_FINISH;
                        end else begin
                            cfg_wait <= 1'b1;            // 去 ST_CFG 等缩放器写完
                            state    <= ST_CFG;
                        end
                    end
                end

                ST_FINISH: begin
                    state_code <= 4'd1;
                    state      <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
end

endmodule
