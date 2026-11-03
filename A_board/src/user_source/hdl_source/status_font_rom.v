`timescale 1ns/1ps

module status_font_rom (
    input  wire         clk,
    input  wire [6:0]   addr,
    output reg  [55:0]  row_bits
);
    reg [55:0] rom [0:127];
    integer    i;

    initial begin
        for (i = 0; i < 128; i = i + 1)
            rom[i] = 56'h0;
        rom[1] = 56'h02ffffffffffd0;
        rom[2] = 56'h02f10000000000;
        rom[3] = 56'h02f00000000000;
        rom[4] = 56'h02f00000000c60;
        rom[5] = 56'h02f00000000c60;
        rom[6] = 56'h02ffffffffff60;
        rom[7] = 56'h02f10000000c60;
        rom[8] = 56'h00000000000c60;
        rom[9] = 56'h78000000000c60;
        rom[10] = 56'h8b000000000c60;
        rom[11] = 56'h4e300000002e50;
        rom[12] = 56'h0bfffffffffc10;
        rom[13] = 56'h00122222222000;
        rom[17] = 56'h00877800049000;
        rom[18] = 56'h0960785eeeeee9;
        rom[19] = 56'h00006900000000;
        rom[20] = 56'h6ffffffaeeeec0;
        rom[21] = 56'h000f5a095002c0;
        rom[22] = 56'h000f4c095002c0;
        rom[23] = 56'h000f2e09eeeec0;
        rom[24] = 56'h000f0c30b68000;
        rom[25] = 56'h930f0887a685a0;
        rom[26] = 56'hb50f01eb368096;
        rom[27] = 56'h981f007b06a000;
        rom[28] = 56'h3dfb000a42df40;
        rom[33] = 56'h00000b40006900;
        rom[34] = 56'h0b4ddfed601e20;
        rom[35] = 56'h03d20b401c16a0;
        rom[36] = 56'h004d2b400780b5;
        rom[37] = 56'h6dddfeddd2adea;
        rom[38] = 56'h00002b60001a10;
        rom[39] = 56'h06feeeeb38b9c2;
        rom[40] = 56'h0690005ca537b6;
        rom[41] = 56'h06edddea000000;
        rom[42] = 56'h0690005a1da741;
        rom[43] = 56'h06feeeea0259c7;
        rom[44] = 56'h0690005a000000;
        rom[49] = 56'h00000088000000;
        rom[50] = 56'h00000088000000;
        rom[51] = 56'h0dffffffffffc0;
        rom[52] = 56'h00000088000000;
        rom[53] = 56'h00000088000000;
        rom[54] = 56'h9ffffffffffff9;
        rom[55] = 56'h000006ee700000;
        rom[56] = 56'h00005a88970000;
        rom[57] = 56'h0018a08808a100;
        rom[58] = 56'h06d70088005d60;
        rom[59] = 56'h7a2000880002a7;
        rom[60] = 56'h00000088000000;
        rom[65] = 56'h0afeddddd705a0;
        rom[66] = 56'h019d504a003e40;
        rom[67] = 56'h0002ceb2006800;
        rom[68] = 56'h0cddefdde40000;
        rom[69] = 56'h0c303b00a40ffb;
        rom[70] = 56'h0cddefdde40f10;
        rom[71] = 56'h0c303b00a40f10;
        rom[72] = 56'h0cddefdde40f10;
        rom[73] = 56'h0c414b00a40f10;
        rom[74] = 56'h07ff7b00a41f50;
        rom[75] = 56'h1110000114c9e6;
        rom[76] = 56'h8ffffffeec6047;
        rom[81] = 56'h004d00000002b0;
        rom[82] = 56'h004d0000001d70;
        rom[83] = 56'h9ffffffffb4a00;
        rom[84] = 56'h004d0000000000;
        rom[85] = 56'h004d0001b00ffa;
        rom[86] = 56'h004d000b900f10;
        rom[87] = 56'h004d006d100f10;
        rom[88] = 56'h004d0094000f10;
        rom[89] = 56'h004e1110000f10;
        rom[90] = 56'h001dff90001f40;
        rom[91] = 56'h1111222125c8e5;
        rom[92] = 56'h8effffeedb4046;
    end

    always @(posedge clk)
        row_bits <= rom[addr];
endmodule
