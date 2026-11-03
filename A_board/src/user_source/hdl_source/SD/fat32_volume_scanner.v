module fat32_volume_scanner #(
    parameter integer MAX_ROOT_CLUSTERS = 4096
)(
    input             clk,
    input             rst,
    input             scan_start,
    output            scan_busy,
    output reg        scan_done,
    output reg [7:0]  fs_error,
    output reg [7:0]  sectors_per_cluster,
    output reg [31:0] fat_start_lba,
    output reg [31:0] data_start_lba,
    output reg [31:0] max_cluster,
    output reg [5:0]  resource_valid,
    output reg [31:0] resource0_cluster,
    output reg [31:0] resource1_cluster,
    output reg [31:0] resource2_cluster,
    output reg [31:0] resource3_cluster,
    output reg [31:0] resource4_cluster,
    output reg [31:0] resource5_cluster,
    output reg [31:0] resource0_size,
    output reg [31:0] resource1_size,
    output reg [31:0] resource2_size,
    output reg [31:0] resource3_size,
    output reg [31:0] resource4_size,
    output reg [31:0] resource5_size,
    output reg        music_valid,
    output reg [31:0] music_cluster,
    output reg [31:0] music_size,
    output reg        alarm_valid,
    output reg [27:0] alarm_cluster,
    output reg [31:0] alarm_size,

    output reg        sector_req,
    output reg [31:0] sector_lba,
    input             sector_busy,
    input             sector_valid,
    input      [7:0]  sector_byte,
    input      [8:0]  sector_byte_index,
    input             sector_done,
    input      [7:0]  sector_error
);

localparam [7:0] ERR_NONE             = 8'h00;
localparam [7:0] ERR_SECTOR           = 8'h10;
localparam [7:0] ERR_BOOT_SIGNATURE   = 8'h20;
localparam [7:0] ERR_UNSUPPORTED_BPB  = 8'h21;
localparam [7:0] ERR_RESOURCE_MISSING = 8'h22;
localparam [7:0] ERR_FAT_LOOP         = 8'h23;
localparam [7:0] ERR_CLUSTER_RANGE    = 8'h24;
localparam [7:0] ERR_ROOT_EXHAUSTED   = 8'h26;

localparam ST_IDLE          = 5'd0;
localparam ST_READ0_REQ     = 5'd1;
localparam ST_READ0_WAIT    = 5'd2;
localparam ST_READ0_PARSE   = 5'd3;
localparam ST_BOOT_REQ      = 5'd4;
localparam ST_BOOT_WAIT     = 5'd5;
localparam ST_BOOT_PARSE    = 5'd6;
localparam ST_LAYOUT        = 5'd7;
localparam ST_ROOT_ADDR     = 5'd8;
localparam ST_DIR_REQ       = 5'd9;
localparam ST_DIR_WAIT      = 5'd10;
localparam ST_DIR_NEXT      = 5'd11;
localparam ST_ROOT_FAT_ADDR = 5'd12;
localparam ST_ROOT_FAT_REQ  = 5'd13;
localparam ST_ROOT_FAT_WAIT = 5'd14;
localparam ST_ROOT_FAT_PARSE= 5'd15;
localparam ST_FINISH        = 5'd16;
localparam ST_ROOT_CHECK    = 5'd17;
localparam ST_DATA_LAYOUT   = 5'd18;
localparam ST_DATA_COUNT    = 5'd19;
localparam ST_CLUSTER_LIMIT = 5'd20;
localparam ST_BPB_BASIC     = 5'd21;
localparam ST_BPB_FAT32     = 5'd22;
localparam ST_LAYOUT_CHECK  = 5'd23;
localparam ST_READ0_KIND    = 5'd24;
localparam ST_PARTITION0    = 5'd25;
localparam ST_PARTITION1    = 5'd26;
localparam ST_PARTITION2    = 5'd27;
localparam ST_PARTITION3    = 5'd28;
localparam ST_LAYOUT_SUM    = 5'd29;
localparam ST_ROOT_FAT_DECIDE = 5'd30;
localparam ST_ROOT_SHIFT    = 5'd31;

reg [4:0] state;
reg [7:0] sector_buffer [0:511];
reg [7:0] dir_entry [0:30];
reg [31:0] partition_lba;
reg [31:0] total_sectors;
reg [31:0] fat_sectors;
reg [31:0] fat_span;
reg [31:0] data_sector_count;
reg [15:0] reserved_sectors;
reg [7:0]  fat_count;
reg [15:0] bytes_per_sector;
reg [15:0] root_entry_count;
reg [15:0] fat16_sectors;
reg [31:0] root_cluster;
reg [31:0] current_root_cluster;
reg [31:0] root_start_cluster;
reg [31:0] root_cluster_count;
reg [7:0]  sector_in_cluster;
reg [2:0]  spc_shift;
reg [8:0]  fat_entry_offset;
reg [7:0]  fat_byte0;
reg [7:0]  fat_byte1;
reg [7:0]  fat_byte2;
reg [7:0]  fat_byte3;
reg        directory_end_seen;
reg [31:0] layout_end;          // reserved_sectors + fat_span
reg [2:0]  shift_left;          // 簇号 -> LBA 还需左移的位数
reg        next_is_eoc;
reg        next_bad_range;
reg        next_is_loop;
reg        root_full;

wire [31:0] next_root_cluster;
wire        boot_signature_valid;

assign scan_busy = (state != ST_IDLE);
assign next_root_cluster = {4'd0, fat_byte3[3:0], fat_byte2,
                            fat_byte1, fat_byte0};
assign boot_signature_valid = (sector_buffer[510] == 8'h55) &&
                              (sector_buffer[511] == 8'haa);

function [15:0] le16;
    input integer offset;
    begin
        le16 = {sector_buffer[offset+1], sector_buffer[offset]};
    end
endfunction

function [31:0] le32;
    input integer offset;
    begin
        le32 = {sector_buffer[offset+3], sector_buffer[offset+2],
                sector_buffer[offset+1], sector_buffer[offset]};
    end
endfunction

function valid_spc;
    input [7:0] value;
    begin
        valid_spc = (value != 0) &&
                    ((value & (value - 1'b1)) == 0);
    end
endfunction

function is_fat32_type;
    input [7:0] value;
    begin
        is_fat32_type = (value == 8'h0b) || (value == 8'h0c);
    end
endfunction

function [2:0] get_spc_shift;
    input [7:0] value;
    begin
        case (value)
            8'd1:   get_spc_shift = 3'd0;
            8'd2:   get_spc_shift = 3'd1;
            8'd4:   get_spc_shift = 3'd2;
            8'd8:   get_spc_shift = 3'd3;
            8'd16:  get_spc_shift = 3'd4;
            8'd32:  get_spc_shift = 3'd5;
            8'd64:  get_spc_shift = 3'd6;
            default:get_spc_shift = 3'd7;
        endcase
    end
endfunction

wire dir_regular_entry;
wire dir_name_common;
wire dir_pic1;
wire dir_pic2;
wire dir_pic3;
wire dir_pic4;
wire dir_pic5;
wire dir_pic6;
wire dir_music;
wire dir_alarm;
wire dir_file_ok;
wire [31:0] dir_first_cluster;
wire [31:0] dir_file_size;

assign dir_regular_entry = (dir_entry[0] != 8'h00) &&
                           (dir_entry[0] != 8'he5) &&
                           (dir_entry[11] != 8'h0f) &&
                           !dir_entry[11][4] && !dir_entry[11][3];
assign dir_name_common = (dir_entry[0] == "P") &&
                         (dir_entry[1] == "I") &&
                         (dir_entry[2] == "C") &&
                         (dir_entry[4] == " ") &&
                         (dir_entry[5] == " ") &&
                         (dir_entry[6] == " ") &&
                         (dir_entry[7] == " ") &&
                         (dir_entry[8] == "B") &&
                         (dir_entry[9] == "M") &&
                         (dir_entry[10] == "P");
assign dir_pic1 = dir_regular_entry && dir_name_common &&
                  (dir_entry[3] == "1");
assign dir_pic2 = dir_regular_entry && dir_name_common &&
                  (dir_entry[3] == "2");
assign dir_pic3 = dir_regular_entry && dir_name_common &&
                  (dir_entry[3] == "3");
assign dir_pic4 = dir_regular_entry && dir_name_common &&
                  (dir_entry[3] == "4");
assign dir_pic5 = dir_regular_entry && dir_name_common &&
                  (dir_entry[3] == "5");
assign dir_pic6 = dir_regular_entry && dir_name_common &&
                  (dir_entry[3] == "6");
assign dir_music = dir_regular_entry &&
                   (dir_entry[0] == "M") && (dir_entry[1] == "U") &&
                   (dir_entry[2] == "S") && (dir_entry[3] == "I") &&
                   (dir_entry[4] == "C") && (dir_entry[5] == " ") &&
                   (dir_entry[6] == " ") && (dir_entry[7] == " ") &&
                   (dir_entry[8] == "P") && (dir_entry[9] == "C") &&
                   (dir_entry[10] == "M");
assign dir_alarm = dir_regular_entry &&
                   (dir_entry[0] == "A") && (dir_entry[1] == "L") &&
                   (dir_entry[2] == "A") && (dir_entry[3] == "R") &&
                   (dir_entry[4] == "M") && (dir_entry[5] == " ") &&
                   (dir_entry[6] == " ") && (dir_entry[7] == " ") &&
                   (dir_entry[8] == "P") && (dir_entry[9] == "C") &&
                   (dir_entry[10] == "M");
assign dir_first_cluster = {dir_entry[21], dir_entry[20],
                            dir_entry[27], dir_entry[26]};
assign dir_file_size = {sector_byte, dir_entry[30], dir_entry[29],
                        dir_entry[28]};
reg dir_pic1_q, dir_pic2_q, dir_pic3_q, dir_pic4_q, dir_pic5_q, dir_pic6_q;
reg dir_music_q, dir_alarm_q;
reg dir_cluster_ok_q;

always @(posedge clk) begin
    dir_pic1_q       <= dir_pic1;
    dir_pic2_q       <= dir_pic2;
    dir_pic3_q       <= dir_pic3;
    dir_pic4_q       <= dir_pic4;
    dir_pic5_q       <= dir_pic5;
    dir_pic6_q       <= dir_pic6;
    dir_music_q      <= dir_music;
    dir_alarm_q      <= dir_alarm;
    dir_cluster_ok_q <= (dir_first_cluster >= 2) &&
                        (dir_first_cluster <= max_cluster);
end

// 簇号范围与非空检查各文件共用一份
assign dir_file_ok = dir_cluster_ok_q && (dir_file_size != 0);

always @(posedge clk) begin
    if (sector_valid && ((state == ST_READ0_WAIT) ||
                         (state == ST_BOOT_WAIT)))
        sector_buffer[sector_byte_index] <= sector_byte;

    if ((state == ST_DIR_WAIT) && sector_valid &&
        (sector_byte_index[4:0] != 5'd31))
        dir_entry[sector_byte_index[4:0]] <= sector_byte;
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state                 <= ST_IDLE;
        scan_done             <= 1'b0;
        fs_error              <= ERR_NONE;
        sectors_per_cluster   <= 8'd0;
        fat_start_lba         <= 32'd0;
        data_start_lba        <= 32'd0;
        max_cluster           <= 32'd0;
        resource_valid        <= 6'd0;
        resource0_cluster     <= 32'd0;
        resource1_cluster     <= 32'd0;
        resource2_cluster     <= 32'd0;
        resource3_cluster     <= 32'd0;
        resource4_cluster     <= 32'd0;
        resource5_cluster     <= 32'd0;
        resource0_size        <= 32'd0;
        resource1_size        <= 32'd0;
        resource2_size        <= 32'd0;
        resource3_size        <= 32'd0;
        resource4_size        <= 32'd0;
        resource5_size        <= 32'd0;
        music_valid           <= 1'b0;
        music_cluster         <= 32'd0;
        music_size            <= 32'd0;
        alarm_valid           <= 1'b0;
        alarm_cluster         <= 28'd0;
        alarm_size            <= 32'd0;
        sector_req            <= 1'b0;
        sector_lba            <= 32'd0;
        partition_lba         <= 32'd0;
        total_sectors         <= 32'd0;
        fat_sectors           <= 32'd0;
        fat_span              <= 32'd0;
        data_sector_count     <= 32'd0;
        reserved_sectors      <= 16'd0;
        fat_count             <= 8'd0;
        bytes_per_sector      <= 16'd0;
        root_entry_count      <= 16'd0;
        fat16_sectors         <= 16'd0;
        root_cluster          <= 32'd0;
        current_root_cluster  <= 32'd0;
        root_start_cluster    <= 32'd0;
        root_cluster_count    <= 32'd0;
        sector_in_cluster     <= 8'd0;
        spc_shift             <= 3'd0;
        fat_entry_offset      <= 9'd0;
        fat_byte0             <= 8'd0;
        fat_byte1             <= 8'd0;
        fat_byte2             <= 8'd0;
        fat_byte3             <= 8'd0;
        directory_end_seen    <= 1'b0;
        layout_end            <= 32'd0;
        shift_left            <= 3'd0;
        next_is_eoc           <= 1'b0;
        next_bad_range        <= 1'b0;
        next_is_loop          <= 1'b0;
        root_full             <= 1'b0;
    end else begin
        scan_done <= 1'b0;
        sector_req <= 1'b0;

        if ((state == ST_DIR_WAIT) && sector_valid &&
            (sector_byte_index[4:0] == 5'd31)) begin
                if (dir_entry[0] == 8'h00)
                    directory_end_seen <= 1'b1;

                if (dir_pic1_q && dir_file_ok) begin
                    resource_valid[0]    <= 1'b1;
                    resource0_cluster    <= dir_first_cluster;
                    resource0_size       <= dir_file_size;
                end
                if (dir_pic2_q && dir_file_ok) begin
                    resource_valid[1]    <= 1'b1;
                    resource1_cluster    <= dir_first_cluster;
                    resource1_size       <= dir_file_size;
                end
                if (dir_pic3_q && dir_file_ok) begin
                    resource_valid[2]    <= 1'b1;
                    resource2_cluster    <= dir_first_cluster;
                    resource2_size       <= dir_file_size;
                end
                if (dir_pic4_q && dir_file_ok) begin
                    resource_valid[3]    <= 1'b1;
                    resource3_cluster    <= dir_first_cluster;
                    resource3_size       <= dir_file_size;
                end
                if (dir_pic5_q && dir_file_ok) begin
                    resource_valid[4]    <= 1'b1;
                    resource4_cluster    <= dir_first_cluster;
                    resource4_size       <= dir_file_size;
                end
                if (dir_pic6_q && dir_file_ok) begin
                    resource_valid[5]    <= 1'b1;
                    resource5_cluster    <= dir_first_cluster;
                    resource5_size       <= dir_file_size;
                end
                if (dir_music_q && dir_file_ok) begin
                    music_valid          <= 1'b1;
                    music_cluster        <= dir_first_cluster;
                    music_size           <= dir_file_size;
                end
                if (dir_alarm_q && dir_file_ok) begin
                    alarm_valid          <= 1'b1;
                    alarm_cluster        <= dir_first_cluster[27:0];
                    alarm_size           <= dir_file_size;
                end
        end

        if ((state == ST_ROOT_FAT_WAIT) && sector_valid) begin
            if (sector_byte_index == fat_entry_offset)
                fat_byte0 <= sector_byte;
            else if (sector_byte_index == fat_entry_offset + 9'd1)
                fat_byte1 <= sector_byte;
            else if (sector_byte_index == fat_entry_offset + 9'd2)
                fat_byte2 <= sector_byte;
            else if (sector_byte_index == fat_entry_offset + 9'd3)
                fat_byte3 <= sector_byte;
        end

        case (state)
            ST_IDLE: if (scan_start) begin
                fs_error           <= ERR_NONE;
                resource_valid     <= 6'd0;
                resource0_cluster  <= 32'd0;
                resource1_cluster  <= 32'd0;
                resource2_cluster  <= 32'd0;
                resource3_cluster  <= 32'd0;
                resource4_cluster  <= 32'd0;
                resource5_cluster  <= 32'd0;
                resource0_size     <= 32'd0;
                resource1_size     <= 32'd0;
                resource2_size     <= 32'd0;
                resource3_size     <= 32'd0;
                resource4_size     <= 32'd0;
                resource5_size     <= 32'd0;
                music_valid        <= 1'b0;
                music_cluster      <= 32'd0;
                music_size         <= 32'd0;
                alarm_valid        <= 1'b0;
                alarm_cluster      <= 28'd0;
                alarm_size         <= 32'd0;
                directory_end_seen <= 1'b0;
                sector_lba         <= 32'd0;
                state              <= ST_READ0_REQ;
            end

            ST_READ0_REQ: if (!sector_busy) begin
                sector_req <= 1'b1;
                state      <= ST_READ0_WAIT;
            end

            ST_READ0_WAIT: if (sector_done) begin
                if (sector_error != ERR_NONE) begin
                    fs_error <= sector_error;
                    state    <= ST_FINISH;
                end else begin
                    state <= ST_READ0_PARSE;
                end
            end

            ST_READ0_PARSE: begin
                if (!boot_signature_valid) begin
                    fs_error <= ERR_BOOT_SIGNATURE;
                    state    <= ST_FINISH;
                end else begin
                    state <= ST_READ0_KIND;
                end
            end

            ST_READ0_KIND: begin
                if ((le16(11) == 16'd512) &&
                             valid_spc(sector_buffer[13]) &&
                             (le16(17) == 0) && (le16(22) == 0) &&
                             (le32(36) != 0) && (le32(44) >= 2)) begin
                    partition_lba <= 32'd0;
                    state         <= ST_BOOT_PARSE;
                end else begin
                    state <= ST_PARTITION0;
                end
            end

            ST_PARTITION0: begin
                if (is_fat32_type(sector_buffer[450]) && (le32(454) != 0)) begin
                    partition_lba <= le32(454);
                    sector_lba    <= le32(454);
                    state         <= ST_BOOT_REQ;
                end else begin
                    state <= ST_PARTITION1;
                end
            end

            ST_PARTITION1: begin
                if (is_fat32_type(sector_buffer[466]) && (le32(470) != 0)) begin
                    partition_lba <= le32(470);
                    sector_lba    <= le32(470);
                    state         <= ST_BOOT_REQ;
                end else begin
                    state <= ST_PARTITION2;
                end
            end

            ST_PARTITION2: begin
                if (is_fat32_type(sector_buffer[482]) && (le32(486) != 0)) begin
                    partition_lba <= le32(486);
                    sector_lba    <= le32(486);
                    state         <= ST_BOOT_REQ;
                end else begin
                    state <= ST_PARTITION3;
                end
            end

            ST_PARTITION3: begin
                if (is_fat32_type(sector_buffer[498]) && (le32(502) != 0)) begin
                    partition_lba <= le32(502);
                    sector_lba    <= le32(502);
                    state         <= ST_BOOT_REQ;
                end else begin
                    fs_error <= ERR_UNSUPPORTED_BPB;
                    state    <= ST_FINISH;
                end
            end

            ST_BOOT_REQ: if (!sector_busy) begin
                sector_req <= 1'b1;
                state      <= ST_BOOT_WAIT;
            end

            ST_BOOT_WAIT: if (sector_done) begin
                if (sector_error != ERR_NONE) begin
                    fs_error <= sector_error;
                    state    <= ST_FINISH;
                end else if (!boot_signature_valid) begin
                    fs_error <= ERR_BOOT_SIGNATURE;
                    state    <= ST_FINISH;
                end else begin
                    state <= ST_BOOT_PARSE;
                end
            end

            ST_BOOT_PARSE: begin
                bytes_per_sector    <= le16(11);
                sectors_per_cluster <= sector_buffer[13];
                reserved_sectors    <= le16(14);
                fat_count           <= sector_buffer[16];
                root_entry_count    <= le16(17);
                fat16_sectors       <= le16(22);
                total_sectors       <= le32(32);
                fat_sectors         <= le32(36);
                root_cluster        <= le32(44);
                spc_shift           <= get_spc_shift(sector_buffer[13]);
                state               <= ST_BPB_BASIC;
            end

            ST_BPB_BASIC: begin
                if ((bytes_per_sector != 16'd512) ||
                    !valid_spc(sectors_per_cluster) ||
                    (reserved_sectors == 0) ||
                    ((fat_count != 1) && (fat_count != 2))) begin
                    fs_error <= ERR_UNSUPPORTED_BPB;
                    state    <= ST_FINISH;
                end else begin
                    state <= ST_BPB_FAT32;
                end
            end

            ST_BPB_FAT32: begin
                if ((root_entry_count != 0) || (fat16_sectors != 0) ||
                    (total_sectors == 0) || (fat_sectors == 0) ||
                    (root_cluster < 2)) begin
                    fs_error <= ERR_UNSUPPORTED_BPB;
                    state    <= ST_FINISH;
                end else begin
                    state <= ST_LAYOUT;
                end
            end

            ST_LAYOUT: begin
                fat_span      <= (fat_count == 2) ?
                                 (fat_sectors << 1) : fat_sectors;
                fat_start_lba <= partition_lba + reserved_sectors;
                state         <= ST_LAYOUT_SUM;
            end

            // 加法和比较分两拍，避免 fat_span -> fs_error 的长链
            ST_LAYOUT_SUM: begin
                layout_end <= {16'd0, reserved_sectors} + fat_span;
                state      <= ST_LAYOUT_CHECK;
            end

            ST_LAYOUT_CHECK: begin
                if (total_sectors <= layout_end) begin
                    fs_error <= ERR_UNSUPPORTED_BPB;
                    state    <= ST_FINISH;
                end else begin
                    state <= ST_DATA_LAYOUT;
                end
            end

            ST_DATA_LAYOUT: begin
                data_start_lba    <= fat_start_lba + fat_span;
                data_sector_count <= total_sectors - reserved_sectors;
                state             <= ST_DATA_COUNT;
            end

            ST_DATA_COUNT: begin
                data_sector_count <= data_sector_count - fat_span;
                state             <= ST_CLUSTER_LIMIT;
            end

            ST_CLUSTER_LIMIT: begin
                max_cluster          <= (data_sector_count >> spc_shift) + 32'd1;
                current_root_cluster <= root_cluster;
                root_start_cluster   <= root_cluster;
                root_cluster_count   <= 32'd1;
                sector_in_cluster    <= 8'd0;
                state                <= ST_ROOT_CHECK;
            end

            ST_ROOT_CHECK: begin
                if ((root_cluster < 2) || (root_cluster > max_cluster)) begin
                    fs_error <= ERR_CLUSTER_RANGE;
                    state    <= ST_FINISH;
                end else begin
                    state <= ST_ROOT_ADDR;
                end
            end

            ST_ROOT_ADDR: begin
                sector_lba <= current_root_cluster - 32'd2;
                shift_left <= spc_shift;
                state      <= ST_ROOT_SHIFT;
            end

            ST_ROOT_SHIFT: begin
                if (shift_left != 3'd0) begin
                    sector_lba <= sector_lba << 1;
                    shift_left <= shift_left - 3'd1;
                end else begin
                    sector_lba <= sector_lba + data_start_lba;
                    state      <= ST_DIR_REQ;
                end
            end

            ST_DIR_REQ: if (!sector_busy) begin
                sector_req <= 1'b1;
                state      <= ST_DIR_WAIT;
            end

            ST_DIR_WAIT: if (sector_done) begin
                if (sector_error != ERR_NONE) begin
                    fs_error <= sector_error;
                    state    <= ST_FINISH;
                end else begin
                    state <= ST_DIR_NEXT;
                end
            end

            ST_DIR_NEXT: begin
                if (((resource_valid == 6'h3f) && music_valid && alarm_valid) ||
                    directory_end_seen) begin
                    if (resource_valid == 6'd0)
                        fs_error <= ERR_RESOURCE_MISSING;
                    state <= ST_FINISH;
                end else if (sector_in_cluster + 8'd1 < sectors_per_cluster) begin
                    sector_in_cluster <= sector_in_cluster + 8'd1;
                    sector_lba        <= sector_lba + 32'd1;
                    state             <= ST_DIR_REQ;
                end else begin
                    sector_in_cluster <= 8'd0;
                    state             <= ST_ROOT_FAT_ADDR;
                end
            end

            ST_ROOT_FAT_ADDR: begin
                fat_entry_offset <= {current_root_cluster[6:0], 2'b00};
                sector_lba       <= fat_start_lba + (current_root_cluster >> 7);
                fat_byte0        <= 8'd0;
                fat_byte1        <= 8'd0;
                fat_byte2        <= 8'd0;
                fat_byte3        <= 8'd0;
                state            <= ST_ROOT_FAT_REQ;
            end

            ST_ROOT_FAT_REQ: if (!sector_busy) begin
                sector_req <= 1'b1;
                state      <= ST_ROOT_FAT_WAIT;
            end

            ST_ROOT_FAT_WAIT: if (sector_done) begin
                if (sector_error != ERR_NONE) begin
                    fs_error <= sector_error;
                    state    <= ST_FINISH;
                end else begin
                    state <= ST_ROOT_FAT_PARSE;
                end
            end

            ST_ROOT_FAT_PARSE: begin
                next_is_eoc    <= (next_root_cluster >= 32'h0ffffff8);
                next_bad_range <= (next_root_cluster < 2) ||
                                  (next_root_cluster == 32'h0ffffff7) ||
                                  (next_root_cluster > max_cluster);
                next_is_loop   <= (next_root_cluster == current_root_cluster) ||
                                  (next_root_cluster == root_start_cluster);
                root_full      <= (root_cluster_count >= MAX_ROOT_CLUSTERS);
                state          <= ST_ROOT_FAT_DECIDE;
            end

            ST_ROOT_FAT_DECIDE: begin
                if (next_is_eoc) begin
                    if (resource_valid == 6'd0)
                        fs_error <= ERR_RESOURCE_MISSING;
                    state <= ST_FINISH;
                end else if (next_bad_range) begin
                    fs_error <= ERR_CLUSTER_RANGE;
                    state    <= ST_FINISH;
                end else if (next_is_loop) begin
                    fs_error <= ERR_FAT_LOOP;
                    state    <= ST_FINISH;
                end else if (root_full) begin
                    fs_error <= ERR_ROOT_EXHAUSTED;
                    state    <= ST_FINISH;
                end else begin
                    current_root_cluster <= next_root_cluster;
                    root_cluster_count   <= root_cluster_count + 32'd1;
                    state                <= ST_ROOT_ADDR;
                end
            end

            ST_FINISH: begin
                scan_done <= 1'b1;
                state     <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
