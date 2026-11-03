module music_sector_reader (
    input             clk,
    input             rst,
    input             abort,
    input             file_start,
    input      [31:0] first_cluster,
    input      [31:0] file_size,
    input      [7:0]  sectors_per_cluster,   // FAT32 规定为 2 的幂
    input      [31:0] fat_start_lba,
    input      [31:0] data_start_lba,
    output            file_busy,
    output            file_valid,
    output     [7:0]  file_byte,
    output            file_byte_hi,          // 扇区从偶数偏移开始，index[0] 即高低字节

    output reg        sector_req,
    output reg [31:0] sector_lba,
    input             sector_busy,
    input             sector_valid,
    input      [7:0]  sector_byte,
    input      [8:0]  sector_byte_index,
    input             sector_done,
    input      [7:0]  sector_error
);

localparam [2:0] ST_IDLE      = 3'd0;
localparam [2:0] ST_ADDR      = 3'd1;
localparam [2:0] ST_SHIFT     = 3'd2;
localparam [2:0] ST_DATA_REQ  = 3'd3;
localparam [2:0] ST_DATA_WAIT = 3'd4;
localparam [2:0] ST_FAT_REQ   = 3'd5;
localparam [2:0] ST_FAT_WAIT  = 3'd6;
localparam [2:0] ST_FAT_PARSE = 3'd7;

reg [2:0]  state;
reg [27:0] cluster;
reg [27:0] next_cluster;
reg [22:0] sectors_left;       // 含最后一个不满的扇区
reg [8:0]  tail_bytes;         // 最后一个扇区的有效字节数，0 表示满扇区
reg [7:0]  spc_cnt;            // 簇号转 LBA 时的移位计数
reg [7:0]  sector_in_cluster;  // 本簇剩余扇区数

reg  last_sector;              // sectors_left == 1
reg  tail_full;                // tail_bytes == 0
reg  cluster_end;              // sector_in_cluster == 1
// 起播检查晚一拍：file_size / first_cluster 在起播后保持不变，
// 下一拍 IDLE 直接用寄存好的检查结果
reg  start_ok;
wire byte_in_file = !last_sector || tail_full ||
                    (sector_byte_index < tail_bytes);

// start_ok 也算忙，避免上层在起播的下一拍误判“已读完”
assign file_busy    = (state != ST_IDLE) || start_ok;
assign file_valid   = (state == ST_DATA_WAIT) && sector_valid && byte_in_file;
assign file_byte    = sector_byte;
assign file_byte_hi = sector_byte_index[0];

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= ST_IDLE;
        sector_req        <= 1'b0;
        sector_lba        <= 32'd0;
        cluster           <= 28'd0;
        next_cluster      <= 28'd0;
        sectors_left      <= 23'd0;
        tail_bytes        <= 9'd0;
        spc_cnt           <= 8'd0;
        sector_in_cluster <= 8'd0;
        last_sector       <= 1'b0;
        tail_full         <= 1'b0;
        cluster_end       <= 1'b0;
        start_ok          <= 1'b0;
    end else begin
        sector_req  <= 1'b0;
        last_sector <= (sectors_left == 23'd1);
        tail_full   <= (tail_bytes == 9'd0);
        cluster_end <= (sector_in_cluster == 8'd1);
        start_ok    <= !abort && (state == ST_IDLE) && file_start &&
                       (file_size != 32'd0) && (first_cluster >= 32'd2) &&
                       (sectors_per_cluster != 8'd0);

        if (abort) begin
            state <= ST_IDLE;
        end else begin
            case (state)
                ST_IDLE: if (start_ok) begin
                    cluster      <= first_cluster[27:0];
                    sectors_left <= file_size[31:9] + (|file_size[8:0]);
                    tail_bytes   <= file_size[8:0];
                    state        <= ST_ADDR;
                end

                ST_ADDR: begin
                    sector_lba        <= {4'd0, cluster} - 32'd2;
                    spc_cnt           <= sectors_per_cluster;
                    sector_in_cluster <= sectors_per_cluster;
                    state             <= ST_SHIFT;
                end

                ST_SHIFT: begin
                    if (spc_cnt[7:1] != 7'd0) begin
                        sector_lba <= sector_lba << 1;
                        spc_cnt    <= spc_cnt >> 1;
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
                    sectors_left      <= sectors_left - 23'd1;
                    sector_in_cluster <= sector_in_cluster - 8'd1;
                    if ((sector_error != 8'h00) || last_sector) begin
                        state <= ST_IDLE;
                    end else if (!cluster_end) begin
                        sector_lba <= sector_lba + 32'd1;
                        state      <= ST_DATA_REQ;
                    end else begin
                        // 本簇读完，查 FAT 表找下一簇
                        sector_lba   <= fat_start_lba + {11'd0, cluster[27:7]};
                        next_cluster <= 28'd0;
                        state        <= ST_FAT_REQ;
                    end
                end

                ST_FAT_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state      <= ST_FAT_WAIT;
                end

                ST_FAT_WAIT: begin
                    if (sector_valid && (sector_byte_index[8:2] == cluster[6:0])) begin
                        case (sector_byte_index[1:0])
                            2'd0: next_cluster[7:0]   <= sector_byte;
                            2'd1: next_cluster[15:8]  <= sector_byte;
                            2'd2: next_cluster[23:16] <= sector_byte;
                            2'd3: next_cluster[27:24] <= sector_byte[3:0];
                        endcase
                    end
                    if (sector_done)
                        state <= (sector_error != 8'h00) ? ST_IDLE : ST_FAT_PARSE;
                end

                // 坏簇、结束标记或非法簇号都直接停止
                ST_FAT_PARSE: begin
                    if ((next_cluster < 28'd2) || (next_cluster >= 28'hffffff7)) begin
                        state <= ST_IDLE;
                    end else begin
                        cluster <= next_cluster;
                        state   <= ST_ADDR;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
end

endmodule
