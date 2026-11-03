module sd_sector_arbiter (
    input             clk,
    input             rst,
    input             abort,          // 图片加载失败中止，只作用于 A

    input             a_req,
    input      [31:0] a_lba,
    output            a_busy,
    output            a_valid,
    output            a_done,

    input             b_req,
    input      [31:0] b_lba,
    output            b_busy,
    output            b_valid,
    output            b_done,

    output reg        m_req,
    output reg [31:0] m_lba,
    output            m_abort,
    input             m_busy,
    input             m_valid,
    input             m_done
);

localparam [1:0] OWN_NONE = 2'd0;
localparam [1:0] OWN_A    = 2'd1;
localparam [1:0] OWN_B    = 2'd2;

reg [1:0]  owner;
reg        a_pend;
reg        b_pend;
reg [31:0] a_lba_latched;
reg [31:0] b_lba_latched;

wire bus_busy = (owner != OWN_NONE) || a_pend || b_pend || m_busy || m_req;

assign a_busy  = bus_busy;
assign b_busy  = bus_busy;
assign a_valid = m_valid && (owner == OWN_A);
assign a_done  = m_done  && (owner == OWN_A);
assign b_valid = m_valid && (owner == OWN_B);
assign b_done  = m_done  && (owner == OWN_B);
// 音乐扇区正在读时不打断它，让它自然结束
assign m_abort = abort && (owner != OWN_B);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        owner         <= OWN_NONE;
        a_pend        <= 1'b0;
        b_pend        <= 1'b0;
        a_lba_latched <= 32'd0;
        b_lba_latched <= 32'd0;
        m_req         <= 1'b0;
        m_lba         <= 32'd0;
    end else begin
        m_req <= 1'b0;

        if (a_req) begin
            a_pend        <= 1'b1;
            a_lba_latched <= a_lba;
        end
        if (b_req) begin
            b_pend        <= 1'b1;
            b_lba_latched <= b_lba;
        end

        if (m_done)
            owner <= OWN_NONE;

        if (owner == OWN_NONE && !m_busy && !m_req) begin
            // 音乐有实时性要求，优先；每次只占一个扇区，不会饿死图片加载
            if (b_pend) begin
                owner  <= OWN_B;
                b_pend <= 1'b0;
                m_req  <= 1'b1;
                m_lba  <= b_lba_latched;
            end else if (a_pend) begin
                owner  <= OWN_A;
                a_pend <= 1'b0;
                m_req  <= 1'b1;
                m_lba  <= a_lba_latched;
            end
        end

        if (abort) begin
            a_pend <= 1'b0;
            if (owner == OWN_A)
                owner <= OWN_NONE;
        end
    end
end

endmodule
