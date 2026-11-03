`timescale 1ns / 1ps

module reset_sync (
    input  wire clk,
    input  wire ext_rst_n,
    output wire rst_n_sync
);

(* ASYNC_REG = "TRUE" *) reg [1:0] reset_pipe;

always @(posedge clk or negedge ext_rst_n) begin
    if (!ext_rst_n)
        reset_pipe <= 2'b00;
    else
        reset_pipe <= {reset_pipe[0], 1'b1};
end

assign rst_n_sync = reset_pipe[1];

endmodule
