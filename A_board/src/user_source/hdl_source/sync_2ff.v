`timescale 1ns / 1ps

module sync_2ff #(
    parameter RESET_VAL = 1'b0
)(
    input  wire clk,
    input  wire rst_n,
    input  wire async_in,
    output wire sync_out
);

(* ASYNC_REG = "TRUE" *) reg [1:0] sync_pipe;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        sync_pipe <= {2{RESET_VAL}};
    else
        sync_pipe <= {sync_pipe[0], async_in};
end

assign sync_out = sync_pipe[1];

endmodule
