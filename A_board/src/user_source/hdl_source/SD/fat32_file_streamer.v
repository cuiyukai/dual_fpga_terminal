module fat32_file_streamer #(
    parameter integer MAX_VISITED_CLUSTERS = 8192
)(
    input             clk,
    input             rst,
    input             abort,
    input             file_start,
    input      [31:0] first_cluster,
    input      [31:0] file_size,
    input      [7:0]  sectors_per_cluster,
    input      [31:0] fat_start_lba,
    input      [31:0] data_start_lba,
    input      [31:0] max_cluster,
    output            file_busy,
    output reg        file_done,
    output reg        file_valid,
    output reg [7:0]  file_byte,
    output reg [31:0] file_byte_offset,
    output reg [7:0]  fs_error,

    output reg        sector_req,
    output reg [31:0] sector_lba,
    input             sector_busy,
    input             sector_valid,
    input      [7:0]  sector_byte,
    input      [8:0]  sector_byte_index,
    input             sector_done,
    input      [7:0]  sector_error
);

localparam [7:0] ERR_NONE              = 8'h00;
localparam [7:0] ERR_SECTOR            = 8'h10;
localparam [7:0] ERR_FAT_LOOP          = 8'h23;
localparam [7:0] ERR_CLUSTER_RANGE     = 8'h24;
localparam [7:0] ERR_EARLY_EOF         = 8'h25;
localparam [7:0] ERR_VISITED_EXHAUSTED = 8'h26;

localparam ST_IDLE         = 4'd0;
localparam ST_CLUSTER_ADDR = 4'd1;
localparam ST_DATA_REQ     = 4'd2;
localparam ST_DATA_WAIT    = 4'd3;
localparam ST_FAT_ADDR     = 4'd4;
localparam ST_FAT_REQ      = 4'd5;
localparam ST_FAT_WAIT     = 4'd6;
localparam ST_FAT_PARSE    = 4'd7;
localparam ST_FINISH       = 4'd8;
localparam ST_DATA_DECIDE  = 4'd9;
localparam ST_CLUSTER_SHIFT = 4'd10;
localparam ST_FAT_DECIDE   = 4'd11;

reg [3:0]  state;
reg [31:0] current_cluster;
reg [31:0] start_cluster;
reg [31:0] remaining_bytes;
reg [7:0]  sector_in_cluster;
reg [2:0]  spc_shift;
reg [2:0]  shift_left;          // 簇号 -> LBA 还需左移的位数
reg [8:0]  fat_entry_offset;
reg [7:0]  fat_byte0;
reg [7:0]  fat_byte1;
reg [7:0]  fat_byte2;
reg [7:0]  fat_byte3;
reg [31:0] chain_count;
// 下一簇的检查结果，ST_FAT_PARSE 寄存、ST_FAT_DECIDE 使用
reg        next_is_eoc;
reg        next_bad_range;
reg        next_is_loop;
reg        chain_full;

wire [31:0] next_cluster_value;
wire        spc_is_valid;

assign file_busy = (state != ST_IDLE);
assign next_cluster_value = {4'd0, fat_byte3[3:0], fat_byte2,
                             fat_byte1, fat_byte0};
assign spc_is_valid = (sectors_per_cluster != 0) &&
                      ((sectors_per_cluster & (sectors_per_cluster - 1'b1)) == 0);

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

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state              <= ST_IDLE;
        file_done          <= 1'b0;
        file_valid         <= 1'b0;
        file_byte          <= 8'd0;
        file_byte_offset   <= 32'd0;
        fs_error           <= ERR_NONE;
        sector_req         <= 1'b0;
        sector_lba         <= 32'd0;
        current_cluster    <= 32'd0;
        start_cluster      <= 32'd0;
        remaining_bytes    <= 32'd0;
        sector_in_cluster  <= 8'd0;
        spc_shift          <= 3'd0;
        shift_left         <= 3'd0;
        fat_entry_offset   <= 9'd0;
        fat_byte0          <= 8'd0;
        fat_byte1          <= 8'd0;
        fat_byte2          <= 8'd0;
        fat_byte3          <= 8'd0;
        chain_count        <= 32'd0;
        next_is_eoc        <= 1'b0;
        next_bad_range     <= 1'b0;
        next_is_loop       <= 1'b0;
        chain_full         <= 1'b0;
    end else begin
        file_done  <= 1'b0;
        file_valid <= 1'b0;
        sector_req <= 1'b0;

        if (abort) begin
            state <= ST_IDLE;
        end else begin
            if ((state == ST_DATA_WAIT) && sector_valid &&
                (remaining_bytes != 0)) begin
                file_valid       <= 1'b1;
                file_byte        <= sector_byte;
                file_byte_offset <= file_size - remaining_bytes;
                remaining_bytes  <= remaining_bytes - 32'd1;
            end

            if ((state == ST_FAT_WAIT) && sector_valid) begin
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
                ST_IDLE: if (file_start) begin
                    fs_error          <= ERR_NONE;
                    file_byte_offset  <= 32'd0;
                    remaining_bytes   <= file_size;
                    sector_in_cluster <= 8'd0;
                    spc_shift         <= get_spc_shift(sectors_per_cluster);
                    current_cluster   <= first_cluster;
                    start_cluster     <= first_cluster;
                    chain_count       <= 32'd1;
                    if (file_size == 0) begin
                        state <= ST_FINISH;
                    end else if (!spc_is_valid || (first_cluster < 2) ||
                                 (first_cluster > max_cluster)) begin
                        fs_error <= ERR_CLUSTER_RANGE;
                        state    <= ST_FINISH;
                    end else begin
                        state <= ST_CLUSTER_ADDR;
                    end
                end

                ST_CLUSTER_ADDR: begin
                    sector_lba <= current_cluster - 32'd2;
                    shift_left <= spc_shift;
                    state      <= ST_CLUSTER_SHIFT;
                end

                ST_CLUSTER_SHIFT: begin
                    if (shift_left != 3'd0) begin
                        sector_lba <= sector_lba << 1;
                        shift_left <= shift_left - 3'd1;
                    end else begin
                        sector_lba <= sector_lba + data_start_lba;
                        state      <= ST_DATA_REQ;
                    end
                end

                ST_DATA_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state      <= ST_DATA_WAIT;
                end

                ST_DATA_WAIT: if (sector_done) begin
                    if (sector_error != ERR_NONE) begin
                        fs_error <= sector_error;
                        state    <= ST_FINISH;
                    end else begin
                        state <= ST_DATA_DECIDE;
                    end
                end

                ST_DATA_DECIDE: begin
                    if (remaining_bytes == 0) begin
                        state <= ST_FINISH;
                    end else if (sector_in_cluster + 8'd1 < sectors_per_cluster) begin
                        sector_in_cluster <= sector_in_cluster + 8'd1;
                        sector_lba        <= sector_lba + 32'd1;
                        state             <= ST_DATA_REQ;
                    end else begin
                        sector_in_cluster <= 8'd0;
                        state             <= ST_FAT_ADDR;
                    end
                end

                ST_FAT_ADDR: begin
                    fat_entry_offset <= {current_cluster[6:0], 2'b00};
                    sector_lba       <= fat_start_lba + (current_cluster >> 7);
                    fat_byte0        <= 8'd0;
                    fat_byte1        <= 8'd0;
                    fat_byte2        <= 8'd0;
                    fat_byte3        <= 8'd0;
                    state            <= ST_FAT_REQ;
                end

                ST_FAT_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state      <= ST_FAT_WAIT;
                end

                ST_FAT_WAIT: if (sector_done) begin
                    if (sector_error != ERR_NONE) begin
                        fs_error <= sector_error;
                        state    <= ST_FINISH;
                    end else begin
                        state <= ST_FAT_PARSE;
                    end
                end

                ST_FAT_PARSE: begin
                    next_is_eoc    <= (next_cluster_value >= 32'h0ffffff8);
                    next_bad_range <= (next_cluster_value < 2) ||
                                      (next_cluster_value == 32'h0ffffff7) ||
                                      (next_cluster_value > max_cluster);
                    next_is_loop   <= (next_cluster_value == current_cluster) ||
                                      (next_cluster_value == start_cluster);
                    chain_full     <= (chain_count >= MAX_VISITED_CLUSTERS);
                    state          <= ST_FAT_DECIDE;
                end

                ST_FAT_DECIDE: begin
                    if (next_is_eoc) begin
                        fs_error <= ERR_EARLY_EOF;
                        state    <= ST_FINISH;
                    end else if (next_bad_range) begin
                        fs_error <= ERR_CLUSTER_RANGE;
                        state    <= ST_FINISH;
                    end else if (next_is_loop) begin
                        fs_error <= ERR_FAT_LOOP;
                        state    <= ST_FINISH;
                    end else if (chain_full) begin
                        fs_error <= ERR_VISITED_EXHAUSTED;
                        state    <= ST_FINISH;
                    end else begin
                        current_cluster <= next_cluster_value;
                        chain_count     <= chain_count + 32'd1;
                        state           <= ST_CLUSTER_ADDR;
                    end
                end

                ST_FINISH: begin
                    file_done <= 1'b1;
                    state     <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
end

endmodule
