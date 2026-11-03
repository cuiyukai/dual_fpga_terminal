module sd_sector_adapter #(
    parameter integer CLK_FREQ_HZ      = 100_000_000,
    parameter integer STALL_TIMEOUT_MS = 500
)(
    input             clk,
    input             rst,
    input             abort,

    input             sector_req,
    input      [31:0] sector_lba,
    output            sector_busy,
    output reg        sector_valid,
    output reg [7:0]  sector_byte,
    output reg [8:0]  sector_byte_index,
    output reg        sector_done,
    output reg [7:0]  sector_error,

    output reg        sd_sec_read,
    output reg [31:0] sd_sec_read_addr,
    input      [7:0]  sd_sec_read_data,
    input             sd_sec_read_data_valid,
    input             sd_sec_read_end
);

localparam [7:0] ERR_NONE    = 8'h00;
localparam [7:0] ERR_TIMEOUT = 8'h10;
localparam [7:0] ERR_SHORT   = 8'h11;
localparam integer STALL_TIMEOUT_CYCLES =
    (CLK_FREQ_HZ / 1000) * STALL_TIMEOUT_MS;

reg        busy;
reg [9:0]  byte_count;
reg [31:0] stall_count;
wire [10:0] bytes_at_end;

assign sector_busy = busy;
assign bytes_at_end = {1'b0, byte_count} +
                      (sd_sec_read_data_valid ? 11'd1 : 11'd0);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        busy              <= 1'b0;
        byte_count        <= 10'd0;
        stall_count       <= 32'd0;
        sector_valid      <= 1'b0;
        sector_byte       <= 8'd0;
        sector_byte_index <= 9'd0;
        sector_done       <= 1'b0;
        sector_error      <= ERR_NONE;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
    end else begin
        sector_valid <= 1'b0;
        sector_done  <= 1'b0;

        if (abort) begin
            busy         <= 1'b0;
            sd_sec_read  <= 1'b0;
            byte_count   <= 10'd0;
            stall_count  <= 32'd0;
            sector_error <= ERR_NONE;
        end else if (!busy) begin
            sd_sec_read <= 1'b0;
            stall_count <= 32'd0;
            if (sector_req) begin
                busy             <= 1'b1;
                byte_count       <= 10'd0;
                sector_error     <= ERR_NONE;
                sd_sec_read_addr <= sector_lba;
                sd_sec_read      <= 1'b1;
            end
        end else begin
            sd_sec_read <= 1'b1;

            if (sd_sec_read_data_valid) begin
                sector_valid      <= 1'b1;
                sector_byte       <= sd_sec_read_data;
                sector_byte_index <= byte_count[8:0];
                byte_count        <= byte_count + 10'd1;
                stall_count       <= 32'd0;
            end else if (sd_sec_read_end) begin
                stall_count <= 32'd0;
            end else if (stall_count < STALL_TIMEOUT_CYCLES) begin
                stall_count <= stall_count + 32'd1;
            end

            if (sd_sec_read_end) begin
                busy         <= 1'b0;
                sd_sec_read  <= 1'b0;
                sector_done  <= 1'b1;
                sector_error <= (bytes_at_end == 11'd512) ? ERR_NONE : ERR_SHORT;
            end else if (stall_count >= STALL_TIMEOUT_CYCLES - 1) begin
                busy         <= 1'b0;
                sd_sec_read  <= 1'b0;
                sector_done  <= 1'b1;
                sector_error <= ERR_TIMEOUT;
            end
        end
    end
end

endmodule
