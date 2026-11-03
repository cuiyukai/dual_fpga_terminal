`timescale 1ns/1ps

module brightness_font_rom (
    input  wire         clk,
    input  wire [7:0]   addr,
    output reg  [35:0]  row_bits
);
    reg [35:0] rom [0:255];
    integer    i;

    initial begin
        for (i = 0; i < 256; i = i + 1)
            rom[i] = 36'h0;
        rom[1] = 36'h006ded700;
        rom[2] = 36'h04fc8bf70;
        rom[3] = 36'h08f100db0;
        rom[4] = 36'h0ae000cc0;
        rom[5] = 36'h0ae000cc0;
        rom[6] = 36'h0ae000cc0;
        rom[7] = 36'h0ae000cc0;
        rom[8] = 36'h0ae000cc0;
        rom[9] = 36'h08f100db0;
        rom[10] = 36'h04fc8bf60;
        rom[11] = 36'h006ded700;
        rom[17] = 36'h000ee7000;
        rom[18] = 36'h000eef400;
        rom[19] = 36'h000ea4300;
        rom[20] = 36'h000ea0000;
        rom[21] = 36'h000ea0000;
        rom[22] = 36'h000ea0000;
        rom[23] = 36'h000ea0000;
        rom[24] = 36'h000ea0000;
        rom[25] = 36'h000ea0000;
        rom[26] = 36'h000ea0000;
        rom[27] = 36'h000ea0000;
        rom[33] = 36'h03bed8000;
        rom[34] = 36'h1ee8af900;
        rom[35] = 36'h4f600ae00;
        rom[36] = 36'h3f7000000;
        rom[37] = 36'h0ce100000;
        rom[38] = 36'h03fb00000;
        rom[39] = 36'h006f80000;
        rom[40] = 36'h000af5000;
        rom[41] = 36'h0001de200;
        rom[42] = 36'h3888afd00;
        rom[43] = 36'h6ffffff00;
        rom[49] = 36'h02bee9100;
        rom[50] = 36'h0de9af900;
        rom[51] = 36'h2f7008b00;
        rom[52] = 36'h1f7000000;
        rom[53] = 36'h08e850000;
        rom[54] = 36'h03df90000;
        rom[55] = 36'h1ea100000;
        rom[56] = 36'h5f4000000;
        rom[57] = 36'h4f5006c20;
        rom[58] = 36'h1ee89ec00;
        rom[59] = 36'h03beea200;
        rom[65] = 36'h003f60000;
        rom[66] = 36'h000bd0000;
        rom[67] = 36'h0004f6000;
        rom[68] = 36'h0000cd000;
        rom[69] = 36'h00005f500;
        rom[70] = 36'h07f10cd00;
        rom[71] = 36'h07f105f50;
        rom[72] = 36'h1bf544ec0;
        rom[73] = 36'h5fffffff0;
        rom[74] = 36'h1bf544440;
        rom[75] = 36'h07f100000;
        rom[81] = 36'h03fffff90;
        rom[82] = 36'h028888f90;
        rom[83] = 36'h000000f90;
        rom[84] = 36'h004cebf90;
        rom[85] = 36'h01ee8cf90;
        rom[86] = 36'h06f401740;
        rom[87] = 36'h08f100000;
        rom[88] = 36'h08f100000;
        rom[89] = 36'h06f400c90;
        rom[90] = 36'h01ee8bf50;
        rom[91] = 36'h003ced600;
        rom[97] = 36'h00ae10000;
        rom[98] = 36'h002f90000;
        rom[99] = 36'h0009f2000;
        rom[100] = 36'h0002fa000;
        rom[101] = 36'h0047bf300;
        rom[102] = 36'h09ffff900;
        rom[103] = 36'h1fc12dd00;
        rom[104] = 36'h4f5009f00;
        rom[105] = 36'h2f700ae00;
        rom[106] = 36'h0ce9af900;
        rom[107] = 36'h02bee9100;
        rom[113] = 36'h1ffffff20;
        rom[114] = 36'h0fd88bf20;
        rom[115] = 36'h0be007f20;
        rom[116] = 36'h06f407f20;
        rom[117] = 36'h02f900000;
        rom[118] = 36'h00cd00000;
        rom[119] = 36'h007f30000;
        rom[120] = 36'h003f80000;
        rom[121] = 36'h000dc0000;
        rom[122] = 36'h0009f2000;
        rom[123] = 36'h0004f7000;
        rom[129] = 36'h006ded600;
        rom[130] = 36'h05fc8cf50;
        rom[131] = 36'h0ae101ea0;
        rom[132] = 36'h09e101e90;
        rom[133] = 36'h02ec8ce20;
        rom[134] = 36'h01cfffc10;
        rom[135] = 36'h0af414fa0;
        rom[136] = 36'h0ea000ae0;
        rom[137] = 36'h0dc000cd0;
        rom[138] = 36'h07fc8cf70;
        rom[139] = 36'h007dfd700;
        rom[145] = 36'h02aed8000;
        rom[146] = 36'h0cf9af800;
        rom[147] = 36'h2f800be00;
        rom[148] = 36'h4f5008f00;
        rom[149] = 36'h2f800be00;
        rom[150] = 36'h0cf9af900;
        rom[151] = 36'h06feea100;
        rom[152] = 36'h00da00000;
        rom[153] = 36'h005f40000;
        rom[154] = 36'h000cc0000;
        rom[155] = 36'h0005f6000;
    end

    always @(posedge clk)
        row_bits <= rom[addr];
endmodule
