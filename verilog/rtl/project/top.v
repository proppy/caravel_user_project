module top (
	clk,
	Start,
	reset,
	FoundKeyNum
);
	input wire clk;
	input wire Start;
	input wire reset;
	output wire FoundKeyNum;
	wire [63:0] DESkey_input;
	wire [63:0] ciphertext_gen;
	wire [63:0] ciphertext_store;
	wire down;
	wire load;
	wire Found;
	wire UP;
	wire en1;
	wire en2;
	wire [55:0] in;
	genvar index;
	assign down = 1'b0;
	assign load = 1'b0;
	assign in = 56'h00000000000000;
	wire [63:0] plaintext;
	wire [63:0] ciphertext;
	wire [55:0] count;
	wire [63:0] Key;
	assign plaintext = 64'h2e987cc94ae92c64;
	assign ciphertext = 64'h6758707f20e55b79;
	control ctrl(
		.clk(~clk),
		.reset(reset),
		.Found(Found),
		.Start(Start),
		.UP(UP),
		.en1(en1),
		.en2(en2),
		.FoundKeyNum(FoundKeyNum)
	);
	UDL_Count #(.WIDTH(56)) p0(
		.clk(clk),
		.rst(reset),
		.up(UP),
		.down(down),
		.load(load),
		.in(in),
		.out(count)
	);
	genParity8 genParity8(
		.in(count),
		.out(DESkey_input)
	);
	DES p1(
		.key(DESkey_input),
		.plaintext(plaintext),
		.encrypt(1'b1),
		.ciphertext(ciphertext_gen)
	);
	flopenr #(.WIDTH(64)) p2a(
		.clk(clk),
		.reset(reset),
		.en(en1),
		.d(ciphertext_gen),
		.q(ciphertext_store)
	);
	comparator #(.WIDTH(64)) p3(
		.a(ciphertext_store),
		.b(ciphertext),
		.eq(Found)
	);
	flopenr #(.WIDTH(64)) p4(
		.clk(clk),
		.reset(reset),
		.en(en1),
		.d(DESkey_input),
		.q(Key)
	);
endmodule
module genParity (
	in,
	out
);
	input wire [6:0] in;
	output wire [7:0] out;
	assign out[0] = ~^in;
	assign out[7:1] = in;
endmodule
module genParity8 (
	in,
	out
);
	input wire [55:0] in;
	output wire [63:0] out;
	genvar index;
	generate
		for (index = 0; index < 8; index = index + 1) begin : genblk1
			genParity genParity(
				.in(in[7 * index+:7]),
				.out(out[8 * index+:8])
			);
		end
	endgenerate
endmodule
module flopenr (
	clk,
	reset,
	en,
	d,
	q
);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire en;
	input wire [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	always @(posedge clk or posedge reset)
		if (reset)
			q <= {WIDTH {1'b0}};
		else if (en)
			q <= d;
endmodule
module GenerateKeys (
	Key,
	SubKey1,
	SubKey2,
	SubKey3,
	SubKey4,
	SubKey5,
	SubKey6,
	SubKey7,
	SubKey8,
	SubKey9,
	SubKey10,
	SubKey11,
	SubKey12,
	SubKey13,
	SubKey14,
	SubKey15,
	SubKey16
);
	input wire [63:0] Key;
	output wire [47:0] SubKey1;
	output wire [47:0] SubKey2;
	output wire [47:0] SubKey3;
	output wire [47:0] SubKey4;
	output wire [47:0] SubKey5;
	output wire [47:0] SubKey6;
	output wire [47:0] SubKey7;
	output wire [47:0] SubKey8;
	output wire [47:0] SubKey9;
	output wire [47:0] SubKey10;
	output wire [47:0] SubKey11;
	output wire [47:0] SubKey12;
	output wire [47:0] SubKey13;
	output wire [47:0] SubKey14;
	output wire [47:0] SubKey15;
	output wire [47:0] SubKey16;
	wire [27:0] left_block0;
	wire [27:0] left_block1;
	wire [27:0] left_block2;
	wire [27:0] left_block3;
	wire [27:0] left_block4;
	wire [27:0] left_block5;
	wire [27:0] left_block6;
	wire [27:0] left_block7;
	wire [27:0] left_block8;
	wire [27:0] left_block9;
	wire [27:0] left_block10;
	wire [27:0] left_block11;
	wire [27:0] left_block12;
	wire [27:0] left_block13;
	wire [27:0] left_block14;
	wire [27:0] left_block15;
	wire [27:0] left_block16;
	wire [27:0] right_block0;
	wire [27:0] right_block1;
	wire [27:0] right_block2;
	wire [27:0] right_block3;
	wire [27:0] right_block4;
	wire [27:0] right_block5;
	wire [27:0] right_block6;
	wire [27:0] right_block7;
	wire [27:0] right_block8;
	wire [27:0] right_block9;
	wire [27:0] right_block10;
	wire [27:0] right_block11;
	wire [27:0] right_block12;
	wire [27:0] right_block13;
	wire [27:0] right_block14;
	wire [27:0] right_block15;
	wire [27:0] right_block16;
	PC1 b1(
		.key(Key),
		.left_block(left_block0),
		.right_block(right_block0)
	);
	assign left_block1 = {left_block0[26:0], left_block0[27]};
	assign right_block1 = {right_block0[26:0], right_block0[27]};
	PC2 b2(
		.left_block(left_block1),
		.right_block(right_block1),
		.subkey(SubKey1)
	);
	assign left_block2 = {left_block1[26:0], left_block1[27]};
	assign right_block2 = {right_block1[26:0], right_block1[27]};
	PC2 b3(
		.left_block(left_block2),
		.right_block(right_block2),
		.subkey(SubKey2)
	);
	assign left_block3 = {left_block2[25:0], left_block2[27:26]};
	assign right_block3 = {right_block2[25:0], right_block2[27:26]};
	PC2 b4(
		.left_block(left_block3),
		.right_block(right_block3),
		.subkey(SubKey3)
	);
	assign left_block4 = {left_block3[25:0], left_block3[27:26]};
	assign right_block4 = {right_block3[25:0], right_block3[27:26]};
	PC2 b5(
		.left_block(left_block4),
		.right_block(right_block4),
		.subkey(SubKey4)
	);
	assign left_block5 = {left_block4[25:0], left_block4[27:26]};
	assign right_block5 = {right_block4[25:0], right_block4[27:26]};
	PC2 b6(
		.left_block(left_block5),
		.right_block(right_block5),
		.subkey(SubKey5)
	);
	assign left_block6 = {left_block5[25:0], left_block5[27:26]};
	assign right_block6 = {right_block5[25:0], right_block5[27:26]};
	PC2 b7(
		.left_block(left_block6),
		.right_block(right_block6),
		.subkey(SubKey6)
	);
	assign left_block7 = {left_block6[25:0], left_block6[27:26]};
	assign right_block7 = {right_block6[25:0], right_block6[27:26]};
	PC2 b8(
		.left_block(left_block7),
		.right_block(right_block7),
		.subkey(SubKey7)
	);
	assign left_block8 = {left_block7[25:0], left_block7[27:26]};
	assign right_block8 = {right_block7[25:0], right_block7[27:26]};
	PC2 b9(
		.left_block(left_block8),
		.right_block(right_block8),
		.subkey(SubKey8)
	);
	assign left_block9 = {left_block8[26:0], left_block8[27]};
	assign right_block9 = {right_block8[26:0], right_block8[27]};
	PC2 b10(
		.left_block(left_block9),
		.right_block(right_block9),
		.subkey(SubKey9)
	);
	assign left_block10 = {left_block9[25:0], left_block9[27:26]};
	assign right_block10 = {right_block9[25:0], right_block9[27:26]};
	PC2 b11(
		.left_block(left_block10),
		.right_block(right_block10),
		.subkey(SubKey10)
	);
	assign left_block11 = {left_block10[25:0], left_block10[27:26]};
	assign right_block11 = {right_block10[25:0], right_block10[27:26]};
	PC2 b12(
		.left_block(left_block11),
		.right_block(right_block11),
		.subkey(SubKey11)
	);
	assign left_block12 = {left_block11[25:0], left_block11[27:26]};
	assign right_block12 = {right_block11[25:0], right_block11[27:26]};
	PC2 b13(
		.left_block(left_block12),
		.right_block(right_block12),
		.subkey(SubKey12)
	);
	assign left_block13 = {left_block12[25:0], left_block12[27:26]};
	assign right_block13 = {right_block12[25:0], right_block12[27:26]};
	PC2 b14(
		.left_block(left_block13),
		.right_block(right_block13),
		.subkey(SubKey13)
	);
	assign left_block14 = {left_block13[25:0], left_block13[27:26]};
	assign right_block14 = {right_block13[25:0], right_block13[27:26]};
	PC2 b15(
		.left_block(left_block14),
		.right_block(right_block14),
		.subkey(SubKey14)
	);
	assign left_block15 = {left_block14[25:0], left_block14[27:26]};
	assign right_block15 = {right_block14[25:0], right_block14[27:26]};
	PC2 b16(
		.left_block(left_block15),
		.right_block(right_block15),
		.subkey(SubKey15)
	);
	assign left_block16 = {left_block15[26:0], left_block15[27]};
	assign right_block16 = {right_block15[26:0], right_block15[27]};
	PC2 b17(
		.left_block(left_block16),
		.right_block(right_block16),
		.subkey(SubKey16)
	);
endmodule
module PC1 (
	key,
	left_block,
	right_block
);
	input wire [63:0] key;
	output wire [27:0] left_block;
	output wire [27:0] right_block;
	wire [55:0] out_block;
	assign {left_block, right_block} = out_block;
	assign out_block[55] = key[7];
	assign out_block[54] = key[15];
	assign out_block[53] = key[23];
	assign out_block[52] = key[31];
	assign out_block[51] = key[39];
	assign out_block[50] = key[47];
	assign out_block[49] = key[55];
	assign out_block[48] = key[63];
	assign out_block[47] = key[6];
	assign out_block[46] = key[14];
	assign out_block[45] = key[22];
	assign out_block[44] = key[30];
	assign out_block[43] = key[38];
	assign out_block[42] = key[46];
	assign out_block[41] = key[54];
	assign out_block[40] = key[62];
	assign out_block[39] = key[5];
	assign out_block[38] = key[13];
	assign out_block[37] = key[21];
	assign out_block[36] = key[29];
	assign out_block[35] = key[37];
	assign out_block[34] = key[45];
	assign out_block[33] = key[53];
	assign out_block[32] = key[61];
	assign out_block[31] = key[4];
	assign out_block[30] = key[12];
	assign out_block[29] = key[20];
	assign out_block[28] = key[28];
	assign out_block[27] = key[1];
	assign out_block[26] = key[9];
	assign out_block[25] = key[17];
	assign out_block[24] = key[25];
	assign out_block[23] = key[33];
	assign out_block[22] = key[41];
	assign out_block[21] = key[49];
	assign out_block[20] = key[57];
	assign out_block[19] = key[2];
	assign out_block[18] = key[10];
	assign out_block[17] = key[18];
	assign out_block[16] = key[26];
	assign out_block[15] = key[34];
	assign out_block[14] = key[42];
	assign out_block[13] = key[50];
	assign out_block[12] = key[58];
	assign out_block[11] = key[3];
	assign out_block[10] = key[11];
	assign out_block[9] = key[19];
	assign out_block[8] = key[27];
	assign out_block[7] = key[35];
	assign out_block[6] = key[43];
	assign out_block[5] = key[51];
	assign out_block[4] = key[59];
	assign out_block[3] = key[36];
	assign out_block[2] = key[44];
	assign out_block[1] = key[52];
	assign out_block[0] = key[60];
endmodule
module PC2 (
	left_block,
	right_block,
	subkey
);
	input wire [27:0] left_block;
	input wire [27:0] right_block;
	output wire [47:0] subkey;
	wire [55:0] combined;
	assign combined = {left_block, right_block};
	assign subkey[47] = combined[42];
	assign subkey[46] = combined[39];
	assign subkey[45] = combined[45];
	assign subkey[44] = combined[32];
	assign subkey[43] = combined[55];
	assign subkey[42] = combined[51];
	assign subkey[41] = combined[53];
	assign subkey[40] = combined[28];
	assign subkey[39] = combined[41];
	assign subkey[38] = combined[50];
	assign subkey[37] = combined[35];
	assign subkey[36] = combined[46];
	assign subkey[35] = combined[33];
	assign subkey[34] = combined[37];
	assign subkey[33] = combined[44];
	assign subkey[32] = combined[52];
	assign subkey[31] = combined[30];
	assign subkey[30] = combined[48];
	assign subkey[29] = combined[40];
	assign subkey[28] = combined[49];
	assign subkey[27] = combined[29];
	assign subkey[26] = combined[36];
	assign subkey[25] = combined[43];
	assign subkey[24] = combined[54];
	assign subkey[23] = combined[15];
	assign subkey[22] = combined[4];
	assign subkey[21] = combined[25];
	assign subkey[20] = combined[19];
	assign subkey[19] = combined[9];
	assign subkey[18] = combined[1];
	assign subkey[17] = combined[26];
	assign subkey[16] = combined[16];
	assign subkey[15] = combined[5];
	assign subkey[14] = combined[11];
	assign subkey[13] = combined[23];
	assign subkey[12] = combined[8];
	assign subkey[11] = combined[12];
	assign subkey[10] = combined[7];
	assign subkey[9] = combined[17];
	assign subkey[8] = combined[0];
	assign subkey[7] = combined[22];
	assign subkey[6] = combined[3];
	assign subkey[5] = combined[10];
	assign subkey[4] = combined[14];
	assign subkey[3] = combined[6];
	assign subkey[2] = combined[20];
	assign subkey[1] = combined[27];
	assign subkey[0] = combined[24];
endmodule
module SF (
	inp_block,
	out_block
);
	input wire [31:0] inp_block;
	output wire [31:0] out_block;
	assign out_block[31] = inp_block[16];
	assign out_block[30] = inp_block[25];
	assign out_block[29] = inp_block[12];
	assign out_block[28] = inp_block[11];
	assign out_block[27] = inp_block[3];
	assign out_block[26] = inp_block[20];
	assign out_block[25] = inp_block[4];
	assign out_block[24] = inp_block[15];
	assign out_block[23] = inp_block[31];
	assign out_block[22] = inp_block[17];
	assign out_block[21] = inp_block[9];
	assign out_block[20] = inp_block[6];
	assign out_block[19] = inp_block[27];
	assign out_block[18] = inp_block[14];
	assign out_block[17] = inp_block[1];
	assign out_block[16] = inp_block[22];
	assign out_block[15] = inp_block[30];
	assign out_block[14] = inp_block[24];
	assign out_block[13] = inp_block[8];
	assign out_block[12] = inp_block[18];
	assign out_block[11] = inp_block[0];
	assign out_block[10] = inp_block[5];
	assign out_block[9] = inp_block[29];
	assign out_block[8] = inp_block[23];
	assign out_block[7] = inp_block[13];
	assign out_block[6] = inp_block[19];
	assign out_block[5] = inp_block[2];
	assign out_block[4] = inp_block[26];
	assign out_block[3] = inp_block[10];
	assign out_block[2] = inp_block[21];
	assign out_block[1] = inp_block[28];
	assign out_block[0] = inp_block[7];
endmodule
module EF (
	inp_block,
	out_block
);
	input wire [31:0] inp_block;
	output wire [47:0] out_block;
	assign out_block[47] = inp_block[0];
	assign out_block[46] = inp_block[31];
	assign out_block[45] = inp_block[30];
	assign out_block[44] = inp_block[29];
	assign out_block[43] = inp_block[28];
	assign out_block[42] = inp_block[27];
	assign out_block[41] = inp_block[28];
	assign out_block[40] = inp_block[27];
	assign out_block[39] = inp_block[26];
	assign out_block[38] = inp_block[25];
	assign out_block[37] = inp_block[24];
	assign out_block[36] = inp_block[23];
	assign out_block[35] = inp_block[24];
	assign out_block[34] = inp_block[23];
	assign out_block[33] = inp_block[22];
	assign out_block[32] = inp_block[21];
	assign out_block[31] = inp_block[20];
	assign out_block[30] = inp_block[19];
	assign out_block[29] = inp_block[20];
	assign out_block[28] = inp_block[19];
	assign out_block[27] = inp_block[18];
	assign out_block[26] = inp_block[17];
	assign out_block[25] = inp_block[16];
	assign out_block[24] = inp_block[15];
	assign out_block[23] = inp_block[16];
	assign out_block[22] = inp_block[15];
	assign out_block[21] = inp_block[14];
	assign out_block[20] = inp_block[13];
	assign out_block[19] = inp_block[12];
	assign out_block[18] = inp_block[11];
	assign out_block[17] = inp_block[12];
	assign out_block[16] = inp_block[11];
	assign out_block[15] = inp_block[10];
	assign out_block[14] = inp_block[9];
	assign out_block[13] = inp_block[8];
	assign out_block[12] = inp_block[7];
	assign out_block[11] = inp_block[8];
	assign out_block[10] = inp_block[7];
	assign out_block[9] = inp_block[6];
	assign out_block[8] = inp_block[5];
	assign out_block[7] = inp_block[4];
	assign out_block[6] = inp_block[3];
	assign out_block[5] = inp_block[4];
	assign out_block[4] = inp_block[3];
	assign out_block[3] = inp_block[2];
	assign out_block[2] = inp_block[1];
	assign out_block[1] = inp_block[0];
	assign out_block[0] = inp_block[31];
endmodule
module feistel (
	inp_block,
	subkey,
	out_block
);
	input wire [31:0] inp_block;
	input wire [47:0] subkey;
	output wire [31:0] out_block;
	wire [47:0] ep_out;
	wire [47:0] sbox_pre;
	wire [3:0] sb1out;
	wire [3:0] sb2out;
	wire [3:0] sb3out;
	wire [3:0] sb4out;
	wire [3:0] sb5out;
	wire [3:0] sb6out;
	wire [3:0] sb7out;
	wire [3:0] sb8out;
	wire [5:0] sb1in;
	wire [5:0] sb2in;
	wire [5:0] sb3in;
	wire [5:0] sb4in;
	wire [5:0] sb5in;
	wire [5:0] sb6in;
	wire [5:0] sb7in;
	wire [5:0] sb8in;
	EF b1(
		.inp_block(inp_block),
		.out_block(ep_out)
	);
	assign sbox_pre = ep_out ^ subkey;
	assign sb8in = sbox_pre[5:0];
	assign sb7in = sbox_pre[11:6];
	assign sb6in = sbox_pre[17:12];
	assign sb5in = sbox_pre[23:18];
	assign sb4in = sbox_pre[29:24];
	assign sb3in = sbox_pre[35:30];
	assign sb2in = sbox_pre[41:36];
	assign sb1in = sbox_pre[47:42];
	S1_Box s1(
		.inp_bits(sb1in),
		.out_bits(sb1out)
	);
	S2_Box s2(
		.inp_bits(sb2in),
		.out_bits(sb2out)
	);
	S3_Box s3(
		.inp_bits(sb3in),
		.out_bits(sb3out)
	);
	S4_Box s4(
		.inp_bits(sb4in),
		.out_bits(sb4out)
	);
	S5_Box s5(
		.inp_bits(sb5in),
		.out_bits(sb5out)
	);
	S6_Box s6(
		.inp_bits(sb6in),
		.out_bits(sb6out)
	);
	S7_Box s7(
		.inp_bits(sb7in),
		.out_bits(sb7out)
	);
	S8_Box s8(
		.inp_bits(sb8in),
		.out_bits(sb8out)
	);
	SF b2(
		.inp_block({sb1out, sb2out, sb3out, sb4out, sb5out, sb6out, sb7out, sb8out}),
		.out_block(out_block)
	);
endmodule
module round (
	inp_block,
	subkey,
	out_block
);
	input wire [63:0] inp_block;
	input wire [47:0] subkey;
	output wire [63:0] out_block;
	wire [31:0] Left0;
	wire [31:0] Right0;
	wire [31:0] Left1;
	wire [31:0] Right1;
	wire [31:0] f_out;
	assign {Left0, Right0} = inp_block;
	feistel fk(
		.inp_block(Right0),
		.subkey(subkey),
		.out_block(f_out)
	);
	assign Right1 = f_out ^ Left0;
	assign Left1 = Right0;
	assign out_block = {Left1, Right1};
endmodule
module IP (
	inp_block,
	out_block
);
	input wire [63:0] inp_block;
	output wire [63:0] out_block;
	assign out_block[63] = inp_block[6];
	assign out_block[62] = inp_block[14];
	assign out_block[61] = inp_block[22];
	assign out_block[60] = inp_block[30];
	assign out_block[59] = inp_block[38];
	assign out_block[58] = inp_block[46];
	assign out_block[57] = inp_block[54];
	assign out_block[56] = inp_block[62];
	assign out_block[55] = inp_block[4];
	assign out_block[54] = inp_block[12];
	assign out_block[53] = inp_block[20];
	assign out_block[52] = inp_block[28];
	assign out_block[51] = inp_block[36];
	assign out_block[50] = inp_block[44];
	assign out_block[49] = inp_block[52];
	assign out_block[48] = inp_block[60];
	assign out_block[47] = inp_block[2];
	assign out_block[46] = inp_block[10];
	assign out_block[45] = inp_block[18];
	assign out_block[44] = inp_block[26];
	assign out_block[43] = inp_block[34];
	assign out_block[42] = inp_block[42];
	assign out_block[41] = inp_block[50];
	assign out_block[40] = inp_block[58];
	assign out_block[39] = inp_block[0];
	assign out_block[38] = inp_block[8];
	assign out_block[37] = inp_block[16];
	assign out_block[36] = inp_block[24];
	assign out_block[35] = inp_block[32];
	assign out_block[34] = inp_block[40];
	assign out_block[33] = inp_block[48];
	assign out_block[32] = inp_block[56];
	assign out_block[31] = inp_block[7];
	assign out_block[30] = inp_block[15];
	assign out_block[29] = inp_block[23];
	assign out_block[28] = inp_block[31];
	assign out_block[27] = inp_block[39];
	assign out_block[26] = inp_block[47];
	assign out_block[25] = inp_block[55];
	assign out_block[24] = inp_block[63];
	assign out_block[23] = inp_block[5];
	assign out_block[22] = inp_block[13];
	assign out_block[21] = inp_block[21];
	assign out_block[20] = inp_block[29];
	assign out_block[19] = inp_block[37];
	assign out_block[18] = inp_block[45];
	assign out_block[17] = inp_block[53];
	assign out_block[16] = inp_block[61];
	assign out_block[15] = inp_block[3];
	assign out_block[14] = inp_block[11];
	assign out_block[13] = inp_block[19];
	assign out_block[12] = inp_block[27];
	assign out_block[11] = inp_block[35];
	assign out_block[10] = inp_block[43];
	assign out_block[9] = inp_block[51];
	assign out_block[8] = inp_block[59];
	assign out_block[7] = inp_block[1];
	assign out_block[6] = inp_block[9];
	assign out_block[5] = inp_block[17];
	assign out_block[4] = inp_block[25];
	assign out_block[3] = inp_block[33];
	assign out_block[2] = inp_block[41];
	assign out_block[1] = inp_block[49];
	assign out_block[0] = inp_block[57];
endmodule
module FP (
	inp_block,
	out_block
);
	input wire [63:0] inp_block;
	output wire [63:0] out_block;
	assign out_block[63] = inp_block[24];
	assign out_block[62] = inp_block[56];
	assign out_block[61] = inp_block[16];
	assign out_block[60] = inp_block[48];
	assign out_block[59] = inp_block[8];
	assign out_block[58] = inp_block[40];
	assign out_block[57] = inp_block[0];
	assign out_block[56] = inp_block[32];
	assign out_block[55] = inp_block[25];
	assign out_block[54] = inp_block[57];
	assign out_block[53] = inp_block[17];
	assign out_block[52] = inp_block[49];
	assign out_block[51] = inp_block[9];
	assign out_block[50] = inp_block[41];
	assign out_block[49] = inp_block[1];
	assign out_block[48] = inp_block[33];
	assign out_block[47] = inp_block[26];
	assign out_block[46] = inp_block[58];
	assign out_block[45] = inp_block[18];
	assign out_block[44] = inp_block[50];
	assign out_block[43] = inp_block[10];
	assign out_block[42] = inp_block[42];
	assign out_block[41] = inp_block[2];
	assign out_block[40] = inp_block[34];
	assign out_block[39] = inp_block[27];
	assign out_block[38] = inp_block[59];
	assign out_block[37] = inp_block[19];
	assign out_block[36] = inp_block[51];
	assign out_block[35] = inp_block[11];
	assign out_block[34] = inp_block[43];
	assign out_block[33] = inp_block[3];
	assign out_block[32] = inp_block[35];
	assign out_block[31] = inp_block[28];
	assign out_block[30] = inp_block[60];
	assign out_block[29] = inp_block[20];
	assign out_block[28] = inp_block[52];
	assign out_block[27] = inp_block[12];
	assign out_block[26] = inp_block[44];
	assign out_block[25] = inp_block[4];
	assign out_block[24] = inp_block[36];
	assign out_block[23] = inp_block[29];
	assign out_block[22] = inp_block[61];
	assign out_block[21] = inp_block[21];
	assign out_block[20] = inp_block[53];
	assign out_block[19] = inp_block[13];
	assign out_block[18] = inp_block[45];
	assign out_block[17] = inp_block[5];
	assign out_block[16] = inp_block[37];
	assign out_block[15] = inp_block[30];
	assign out_block[14] = inp_block[62];
	assign out_block[13] = inp_block[22];
	assign out_block[12] = inp_block[54];
	assign out_block[11] = inp_block[14];
	assign out_block[10] = inp_block[46];
	assign out_block[9] = inp_block[6];
	assign out_block[8] = inp_block[38];
	assign out_block[7] = inp_block[31];
	assign out_block[6] = inp_block[63];
	assign out_block[5] = inp_block[23];
	assign out_block[4] = inp_block[55];
	assign out_block[3] = inp_block[15];
	assign out_block[2] = inp_block[47];
	assign out_block[1] = inp_block[7];
	assign out_block[0] = inp_block[39];
endmodule
module S1_Box (
	inp_bits,
	out_bits
);
	input wire [5:0] inp_bits;
	output reg [3:0] out_bits;
	always @(*)
		case ({{inp_bits[5], inp_bits[0]}, inp_bits[4:1]})
			6'd0: out_bits = 4'd14;
			6'd1: out_bits = 4'd4;
			6'd2: out_bits = 4'd13;
			6'd3: out_bits = 4'd1;
			6'd4: out_bits = 4'd2;
			6'd5: out_bits = 4'd15;
			6'd6: out_bits = 4'd11;
			6'd7: out_bits = 4'd8;
			6'd8: out_bits = 4'd3;
			6'd9: out_bits = 4'd10;
			6'd10: out_bits = 4'd6;
			6'd11: out_bits = 4'd12;
			6'd12: out_bits = 4'd5;
			6'd13: out_bits = 4'd9;
			6'd14: out_bits = 4'd0;
			6'd15: out_bits = 4'd7;
			6'd16: out_bits = 4'd0;
			6'd17: out_bits = 4'd15;
			6'd18: out_bits = 4'd7;
			6'd19: out_bits = 4'd4;
			6'd20: out_bits = 4'd14;
			6'd21: out_bits = 4'd2;
			6'd22: out_bits = 4'd13;
			6'd23: out_bits = 4'd1;
			6'd24: out_bits = 4'd10;
			6'd25: out_bits = 4'd6;
			6'd26: out_bits = 4'd12;
			6'd27: out_bits = 4'd11;
			6'd28: out_bits = 4'd9;
			6'd29: out_bits = 4'd5;
			6'd30: out_bits = 4'd3;
			6'd31: out_bits = 4'd8;
			6'd32: out_bits = 4'd4;
			6'd33: out_bits = 4'd1;
			6'd34: out_bits = 4'd14;
			6'd35: out_bits = 4'd8;
			6'd36: out_bits = 4'd13;
			6'd37: out_bits = 4'd6;
			6'd38: out_bits = 4'd2;
			6'd39: out_bits = 4'd11;
			6'd40: out_bits = 4'd15;
			6'd41: out_bits = 4'd12;
			6'd42: out_bits = 4'd9;
			6'd43: out_bits = 4'd7;
			6'd44: out_bits = 4'd3;
			6'd45: out_bits = 4'd10;
			6'd46: out_bits = 4'd5;
			6'd47: out_bits = 4'd0;
			6'd48: out_bits = 4'd15;
			6'd49: out_bits = 4'd12;
			6'd50: out_bits = 4'd8;
			6'd51: out_bits = 4'd2;
			6'd52: out_bits = 4'd4;
			6'd53: out_bits = 4'd9;
			6'd54: out_bits = 4'd1;
			6'd55: out_bits = 4'd7;
			6'd56: out_bits = 4'd5;
			6'd57: out_bits = 4'd11;
			6'd58: out_bits = 4'd3;
			6'd59: out_bits = 4'd14;
			6'd60: out_bits = 4'd10;
			6'd61: out_bits = 4'd0;
			6'd62: out_bits = 4'd6;
			6'd63: out_bits = 4'd13;
			default: out_bits = 4'd0;
		endcase
endmodule
module S2_Box (
	inp_bits,
	out_bits
);
	input wire [5:0] inp_bits;
	output reg [3:0] out_bits;
	always @(*)
		case ({{inp_bits[5], inp_bits[0]}, inp_bits[4:1]})
			6'd0: out_bits = 4'd15;
			6'd1: out_bits = 4'd1;
			6'd2: out_bits = 4'd8;
			6'd3: out_bits = 4'd14;
			6'd4: out_bits = 4'd6;
			6'd5: out_bits = 4'd11;
			6'd6: out_bits = 4'd3;
			6'd7: out_bits = 4'd4;
			6'd8: out_bits = 4'd9;
			6'd9: out_bits = 4'd7;
			6'd10: out_bits = 4'd2;
			6'd11: out_bits = 4'd13;
			6'd12: out_bits = 4'd12;
			6'd13: out_bits = 4'd0;
			6'd14: out_bits = 4'd5;
			6'd15: out_bits = 4'd10;
			6'd16: out_bits = 4'd3;
			6'd17: out_bits = 4'd13;
			6'd18: out_bits = 4'd4;
			6'd19: out_bits = 4'd7;
			6'd20: out_bits = 4'd15;
			6'd21: out_bits = 4'd2;
			6'd22: out_bits = 4'd8;
			6'd23: out_bits = 4'd14;
			6'd24: out_bits = 4'd12;
			6'd25: out_bits = 4'd0;
			6'd26: out_bits = 4'd1;
			6'd27: out_bits = 4'd10;
			6'd28: out_bits = 4'd6;
			6'd29: out_bits = 4'd9;
			6'd30: out_bits = 4'd11;
			6'd31: out_bits = 4'd5;
			6'd32: out_bits = 4'd0;
			6'd33: out_bits = 4'd14;
			6'd34: out_bits = 4'd7;
			6'd35: out_bits = 4'd11;
			6'd36: out_bits = 4'd10;
			6'd37: out_bits = 4'd4;
			6'd38: out_bits = 4'd13;
			6'd39: out_bits = 4'd1;
			6'd40: out_bits = 4'd5;
			6'd41: out_bits = 4'd8;
			6'd42: out_bits = 4'd12;
			6'd43: out_bits = 4'd6;
			6'd44: out_bits = 4'd9;
			6'd45: out_bits = 4'd3;
			6'd46: out_bits = 4'd2;
			6'd47: out_bits = 4'd15;
			6'd48: out_bits = 4'd13;
			6'd49: out_bits = 4'd8;
			6'd50: out_bits = 4'd10;
			6'd51: out_bits = 4'd1;
			6'd52: out_bits = 4'd3;
			6'd53: out_bits = 4'd15;
			6'd54: out_bits = 4'd4;
			6'd55: out_bits = 4'd2;
			6'd56: out_bits = 4'd11;
			6'd57: out_bits = 4'd6;
			6'd58: out_bits = 4'd7;
			6'd59: out_bits = 4'd12;
			6'd60: out_bits = 4'd0;
			6'd61: out_bits = 4'd5;
			6'd62: out_bits = 4'd14;
			6'd63: out_bits = 4'd9;
			default: out_bits = 4'd0;
		endcase
endmodule
module S3_Box (
	inp_bits,
	out_bits
);
	input wire [5:0] inp_bits;
	output reg [3:0] out_bits;
	always @(*)
		case ({{inp_bits[5], inp_bits[0]}, inp_bits[4:1]})
			6'd0: out_bits = 4'd10;
			6'd1: out_bits = 4'd0;
			6'd2: out_bits = 4'd9;
			6'd3: out_bits = 4'd14;
			6'd4: out_bits = 4'd6;
			6'd5: out_bits = 4'd3;
			6'd6: out_bits = 4'd15;
			6'd7: out_bits = 4'd5;
			6'd8: out_bits = 4'd1;
			6'd9: out_bits = 4'd13;
			6'd10: out_bits = 4'd12;
			6'd11: out_bits = 4'd7;
			6'd12: out_bits = 4'd11;
			6'd13: out_bits = 4'd4;
			6'd14: out_bits = 4'd2;
			6'd15: out_bits = 4'd8;
			6'd16: out_bits = 4'd13;
			6'd17: out_bits = 4'd7;
			6'd18: out_bits = 4'd0;
			6'd19: out_bits = 4'd9;
			6'd20: out_bits = 4'd3;
			6'd21: out_bits = 4'd4;
			6'd22: out_bits = 4'd6;
			6'd23: out_bits = 4'd10;
			6'd24: out_bits = 4'd2;
			6'd25: out_bits = 4'd8;
			6'd26: out_bits = 4'd5;
			6'd27: out_bits = 4'd14;
			6'd28: out_bits = 4'd12;
			6'd29: out_bits = 4'd11;
			6'd30: out_bits = 4'd15;
			6'd31: out_bits = 4'd1;
			6'd32: out_bits = 4'd13;
			6'd33: out_bits = 4'd6;
			6'd34: out_bits = 4'd4;
			6'd35: out_bits = 4'd9;
			6'd36: out_bits = 4'd8;
			6'd37: out_bits = 4'd15;
			6'd38: out_bits = 4'd3;
			6'd39: out_bits = 4'd0;
			6'd40: out_bits = 4'd11;
			6'd41: out_bits = 4'd1;
			6'd42: out_bits = 4'd2;
			6'd43: out_bits = 4'd12;
			6'd44: out_bits = 4'd5;
			6'd45: out_bits = 4'd10;
			6'd46: out_bits = 4'd14;
			6'd47: out_bits = 4'd7;
			6'd48: out_bits = 4'd1;
			6'd49: out_bits = 4'd10;
			6'd50: out_bits = 4'd13;
			6'd51: out_bits = 4'd0;
			6'd52: out_bits = 4'd6;
			6'd53: out_bits = 4'd9;
			6'd54: out_bits = 4'd8;
			6'd55: out_bits = 4'd7;
			6'd56: out_bits = 4'd4;
			6'd57: out_bits = 4'd15;
			6'd58: out_bits = 4'd14;
			6'd59: out_bits = 4'd3;
			6'd60: out_bits = 4'd11;
			6'd61: out_bits = 4'd5;
			6'd62: out_bits = 4'd2;
			6'd63: out_bits = 4'd12;
			default: out_bits = 4'd0;
		endcase
endmodule
module S4_Box (
	inp_bits,
	out_bits
);
	input wire [5:0] inp_bits;
	output reg [3:0] out_bits;
	always @(*)
		case ({{inp_bits[5], inp_bits[0]}, inp_bits[4:1]})
			6'd0: out_bits = 4'd7;
			6'd1: out_bits = 4'd13;
			6'd2: out_bits = 4'd14;
			6'd3: out_bits = 4'd3;
			6'd4: out_bits = 4'd0;
			6'd5: out_bits = 4'd6;
			6'd6: out_bits = 4'd9;
			6'd7: out_bits = 4'd10;
			6'd8: out_bits = 4'd1;
			6'd9: out_bits = 4'd2;
			6'd10: out_bits = 4'd8;
			6'd11: out_bits = 4'd5;
			6'd12: out_bits = 4'd11;
			6'd13: out_bits = 4'd12;
			6'd14: out_bits = 4'd4;
			6'd15: out_bits = 4'd15;
			6'd16: out_bits = 4'd13;
			6'd17: out_bits = 4'd8;
			6'd18: out_bits = 4'd11;
			6'd19: out_bits = 4'd5;
			6'd20: out_bits = 4'd6;
			6'd21: out_bits = 4'd15;
			6'd22: out_bits = 4'd0;
			6'd23: out_bits = 4'd3;
			6'd24: out_bits = 4'd4;
			6'd25: out_bits = 4'd7;
			6'd26: out_bits = 4'd2;
			6'd27: out_bits = 4'd12;
			6'd28: out_bits = 4'd1;
			6'd29: out_bits = 4'd10;
			6'd30: out_bits = 4'd14;
			6'd31: out_bits = 4'd9;
			6'd32: out_bits = 4'd10;
			6'd33: out_bits = 4'd6;
			6'd34: out_bits = 4'd9;
			6'd35: out_bits = 4'd0;
			6'd36: out_bits = 4'd12;
			6'd37: out_bits = 4'd11;
			6'd38: out_bits = 4'd7;
			6'd39: out_bits = 4'd13;
			6'd40: out_bits = 4'd15;
			6'd41: out_bits = 4'd1;
			6'd42: out_bits = 4'd3;
			6'd43: out_bits = 4'd14;
			6'd44: out_bits = 4'd5;
			6'd45: out_bits = 4'd2;
			6'd46: out_bits = 4'd8;
			6'd47: out_bits = 4'd4;
			6'd48: out_bits = 4'd3;
			6'd49: out_bits = 4'd15;
			6'd50: out_bits = 4'd0;
			6'd51: out_bits = 4'd6;
			6'd52: out_bits = 4'd10;
			6'd53: out_bits = 4'd1;
			6'd54: out_bits = 4'd13;
			6'd55: out_bits = 4'd8;
			6'd56: out_bits = 4'd9;
			6'd57: out_bits = 4'd4;
			6'd58: out_bits = 4'd5;
			6'd59: out_bits = 4'd11;
			6'd60: out_bits = 4'd12;
			6'd61: out_bits = 4'd7;
			6'd62: out_bits = 4'd2;
			6'd63: out_bits = 4'd14;
			default: out_bits = 4'd0;
		endcase
endmodule
module S5_Box (
	inp_bits,
	out_bits
);
	input wire [5:0] inp_bits;
	output reg [3:0] out_bits;
	always @(*)
		case ({{inp_bits[5], inp_bits[0]}, inp_bits[4:1]})
			6'd0: out_bits = 4'd2;
			6'd1: out_bits = 4'd12;
			6'd2: out_bits = 4'd4;
			6'd3: out_bits = 4'd1;
			6'd4: out_bits = 4'd7;
			6'd5: out_bits = 4'd10;
			6'd6: out_bits = 4'd11;
			6'd7: out_bits = 4'd6;
			6'd8: out_bits = 4'd8;
			6'd9: out_bits = 4'd5;
			6'd10: out_bits = 4'd3;
			6'd11: out_bits = 4'd15;
			6'd12: out_bits = 4'd13;
			6'd13: out_bits = 4'd0;
			6'd14: out_bits = 4'd14;
			6'd15: out_bits = 4'd9;
			6'd16: out_bits = 4'd14;
			6'd17: out_bits = 4'd11;
			6'd18: out_bits = 4'd2;
			6'd19: out_bits = 4'd12;
			6'd20: out_bits = 4'd4;
			6'd21: out_bits = 4'd7;
			6'd22: out_bits = 4'd13;
			6'd23: out_bits = 4'd1;
			6'd24: out_bits = 4'd5;
			6'd25: out_bits = 4'd0;
			6'd26: out_bits = 4'd15;
			6'd27: out_bits = 4'd10;
			6'd28: out_bits = 4'd3;
			6'd29: out_bits = 4'd9;
			6'd30: out_bits = 4'd8;
			6'd31: out_bits = 4'd6;
			6'd32: out_bits = 4'd4;
			6'd33: out_bits = 4'd2;
			6'd34: out_bits = 4'd1;
			6'd35: out_bits = 4'd11;
			6'd36: out_bits = 4'd10;
			6'd37: out_bits = 4'd13;
			6'd38: out_bits = 4'd7;
			6'd39: out_bits = 4'd8;
			6'd40: out_bits = 4'd15;
			6'd41: out_bits = 4'd9;
			6'd42: out_bits = 4'd12;
			6'd43: out_bits = 4'd5;
			6'd44: out_bits = 4'd6;
			6'd45: out_bits = 4'd3;
			6'd46: out_bits = 4'd0;
			6'd47: out_bits = 4'd14;
			6'd48: out_bits = 4'd11;
			6'd49: out_bits = 4'd8;
			6'd50: out_bits = 4'd12;
			6'd51: out_bits = 4'd7;
			6'd52: out_bits = 4'd1;
			6'd53: out_bits = 4'd14;
			6'd54: out_bits = 4'd2;
			6'd55: out_bits = 4'd13;
			6'd56: out_bits = 4'd6;
			6'd57: out_bits = 4'd15;
			6'd58: out_bits = 4'd0;
			6'd59: out_bits = 4'd9;
			6'd60: out_bits = 4'd10;
			6'd61: out_bits = 4'd4;
			6'd62: out_bits = 4'd5;
			6'd63: out_bits = 4'd3;
			default: out_bits = 4'd0;
		endcase
endmodule
module S6_Box (
	inp_bits,
	out_bits
);
	input wire [5:0] inp_bits;
	output reg [3:0] out_bits;
	always @(*)
		case ({{inp_bits[5], inp_bits[0]}, inp_bits[4:1]})
			6'd0: out_bits = 4'd12;
			6'd1: out_bits = 4'd1;
			6'd2: out_bits = 4'd10;
			6'd3: out_bits = 4'd15;
			6'd4: out_bits = 4'd9;
			6'd5: out_bits = 4'd2;
			6'd6: out_bits = 4'd6;
			6'd7: out_bits = 4'd8;
			6'd8: out_bits = 4'd0;
			6'd9: out_bits = 4'd13;
			6'd10: out_bits = 4'd3;
			6'd11: out_bits = 4'd4;
			6'd12: out_bits = 4'd14;
			6'd13: out_bits = 4'd7;
			6'd14: out_bits = 4'd5;
			6'd15: out_bits = 4'd11;
			6'd16: out_bits = 4'd10;
			6'd17: out_bits = 4'd15;
			6'd18: out_bits = 4'd4;
			6'd19: out_bits = 4'd2;
			6'd20: out_bits = 4'd7;
			6'd21: out_bits = 4'd12;
			6'd22: out_bits = 4'd9;
			6'd23: out_bits = 4'd5;
			6'd24: out_bits = 4'd6;
			6'd25: out_bits = 4'd1;
			6'd26: out_bits = 4'd13;
			6'd27: out_bits = 4'd14;
			6'd28: out_bits = 4'd0;
			6'd29: out_bits = 4'd11;
			6'd30: out_bits = 4'd3;
			6'd31: out_bits = 4'd8;
			6'd32: out_bits = 4'd9;
			6'd33: out_bits = 4'd14;
			6'd34: out_bits = 4'd15;
			6'd35: out_bits = 4'd5;
			6'd36: out_bits = 4'd2;
			6'd37: out_bits = 4'd8;
			6'd38: out_bits = 4'd12;
			6'd39: out_bits = 4'd3;
			6'd40: out_bits = 4'd7;
			6'd41: out_bits = 4'd0;
			6'd42: out_bits = 4'd4;
			6'd43: out_bits = 4'd10;
			6'd44: out_bits = 4'd1;
			6'd45: out_bits = 4'd13;
			6'd46: out_bits = 4'd11;
			6'd47: out_bits = 4'd6;
			6'd48: out_bits = 4'd4;
			6'd49: out_bits = 4'd3;
			6'd50: out_bits = 4'd2;
			6'd51: out_bits = 4'd12;
			6'd52: out_bits = 4'd9;
			6'd53: out_bits = 4'd5;
			6'd54: out_bits = 4'd15;
			6'd55: out_bits = 4'd10;
			6'd56: out_bits = 4'd11;
			6'd57: out_bits = 4'd14;
			6'd58: out_bits = 4'd1;
			6'd59: out_bits = 4'd7;
			6'd60: out_bits = 4'd6;
			6'd61: out_bits = 4'd0;
			6'd62: out_bits = 4'd8;
			6'd63: out_bits = 4'd13;
			default: out_bits = 4'd0;
		endcase
endmodule
module S7_Box (
	inp_bits,
	out_bits
);
	input wire [5:0] inp_bits;
	output reg [3:0] out_bits;
	always @(*)
		case ({{inp_bits[5], inp_bits[0]}, inp_bits[4:1]})
			6'd0: out_bits = 4'd4;
			6'd1: out_bits = 4'd11;
			6'd2: out_bits = 4'd2;
			6'd3: out_bits = 4'd14;
			6'd4: out_bits = 4'd15;
			6'd5: out_bits = 4'd0;
			6'd6: out_bits = 4'd8;
			6'd7: out_bits = 4'd13;
			6'd8: out_bits = 4'd3;
			6'd9: out_bits = 4'd12;
			6'd10: out_bits = 4'd9;
			6'd11: out_bits = 4'd7;
			6'd12: out_bits = 4'd5;
			6'd13: out_bits = 4'd10;
			6'd14: out_bits = 4'd6;
			6'd15: out_bits = 4'd1;
			6'd16: out_bits = 4'd13;
			6'd17: out_bits = 4'd0;
			6'd18: out_bits = 4'd11;
			6'd19: out_bits = 4'd7;
			6'd20: out_bits = 4'd4;
			6'd21: out_bits = 4'd9;
			6'd22: out_bits = 4'd1;
			6'd23: out_bits = 4'd10;
			6'd24: out_bits = 4'd14;
			6'd25: out_bits = 4'd3;
			6'd26: out_bits = 4'd5;
			6'd27: out_bits = 4'd12;
			6'd28: out_bits = 4'd2;
			6'd29: out_bits = 4'd15;
			6'd30: out_bits = 4'd8;
			6'd31: out_bits = 4'd6;
			6'd32: out_bits = 4'd1;
			6'd33: out_bits = 4'd4;
			6'd34: out_bits = 4'd11;
			6'd35: out_bits = 4'd13;
			6'd36: out_bits = 4'd12;
			6'd37: out_bits = 4'd3;
			6'd38: out_bits = 4'd7;
			6'd39: out_bits = 4'd14;
			6'd40: out_bits = 4'd10;
			6'd41: out_bits = 4'd15;
			6'd42: out_bits = 4'd6;
			6'd43: out_bits = 4'd8;
			6'd44: out_bits = 4'd0;
			6'd45: out_bits = 4'd5;
			6'd46: out_bits = 4'd9;
			6'd47: out_bits = 4'd2;
			6'd48: out_bits = 4'd6;
			6'd49: out_bits = 4'd11;
			6'd50: out_bits = 4'd13;
			6'd51: out_bits = 4'd8;
			6'd52: out_bits = 4'd1;
			6'd53: out_bits = 4'd4;
			6'd54: out_bits = 4'd10;
			6'd55: out_bits = 4'd7;
			6'd56: out_bits = 4'd9;
			6'd57: out_bits = 4'd5;
			6'd58: out_bits = 4'd0;
			6'd59: out_bits = 4'd15;
			6'd60: out_bits = 4'd14;
			6'd61: out_bits = 4'd2;
			6'd62: out_bits = 4'd3;
			6'd63: out_bits = 4'd12;
			default: out_bits = 4'd0;
		endcase
endmodule
module S8_Box (
	inp_bits,
	out_bits
);
	input wire [5:0] inp_bits;
	output reg [3:0] out_bits;
	always @(*)
		case ({{inp_bits[5], inp_bits[0]}, inp_bits[4:1]})
			6'd0: out_bits = 4'd13;
			6'd1: out_bits = 4'd2;
			6'd2: out_bits = 4'd8;
			6'd3: out_bits = 4'd4;
			6'd4: out_bits = 4'd6;
			6'd5: out_bits = 4'd15;
			6'd6: out_bits = 4'd11;
			6'd7: out_bits = 4'd1;
			6'd8: out_bits = 4'd10;
			6'd9: out_bits = 4'd9;
			6'd10: out_bits = 4'd3;
			6'd11: out_bits = 4'd14;
			6'd12: out_bits = 4'd5;
			6'd13: out_bits = 4'd0;
			6'd14: out_bits = 4'd12;
			6'd15: out_bits = 4'd7;
			6'd16: out_bits = 4'd1;
			6'd17: out_bits = 4'd15;
			6'd18: out_bits = 4'd13;
			6'd19: out_bits = 4'd8;
			6'd20: out_bits = 4'd10;
			6'd21: out_bits = 4'd3;
			6'd22: out_bits = 4'd7;
			6'd23: out_bits = 4'd4;
			6'd24: out_bits = 4'd12;
			6'd25: out_bits = 4'd5;
			6'd26: out_bits = 4'd6;
			6'd27: out_bits = 4'd11;
			6'd28: out_bits = 4'd0;
			6'd29: out_bits = 4'd14;
			6'd30: out_bits = 4'd9;
			6'd31: out_bits = 4'd2;
			6'd32: out_bits = 4'd7;
			6'd33: out_bits = 4'd11;
			6'd34: out_bits = 4'd4;
			6'd35: out_bits = 4'd1;
			6'd36: out_bits = 4'd9;
			6'd37: out_bits = 4'd12;
			6'd38: out_bits = 4'd14;
			6'd39: out_bits = 4'd2;
			6'd40: out_bits = 4'd0;
			6'd41: out_bits = 4'd6;
			6'd42: out_bits = 4'd10;
			6'd43: out_bits = 4'd13;
			6'd44: out_bits = 4'd15;
			6'd45: out_bits = 4'd3;
			6'd46: out_bits = 4'd5;
			6'd47: out_bits = 4'd8;
			6'd48: out_bits = 4'd2;
			6'd49: out_bits = 4'd1;
			6'd50: out_bits = 4'd14;
			6'd51: out_bits = 4'd7;
			6'd52: out_bits = 4'd4;
			6'd53: out_bits = 4'd10;
			6'd54: out_bits = 4'd8;
			6'd55: out_bits = 4'd13;
			6'd56: out_bits = 4'd15;
			6'd57: out_bits = 4'd12;
			6'd58: out_bits = 4'd9;
			6'd59: out_bits = 4'd0;
			6'd60: out_bits = 4'd3;
			6'd61: out_bits = 4'd5;
			6'd62: out_bits = 4'd6;
			6'd63: out_bits = 4'd11;
			default: out_bits = 4'd0;
		endcase
endmodule
module DES (
	key,
	plaintext,
	encrypt,
	ciphertext
);
	input wire [63:0] key;
	input wire [63:0] plaintext;
	input wire encrypt;
	output wire [63:0] ciphertext;
	wire [47:0] SubKey1;
	wire [47:0] SubKey2;
	wire [47:0] SubKey3;
	wire [47:0] SubKey4;
	wire [47:0] SubKey5;
	wire [47:0] SubKey6;
	wire [47:0] SubKey7;
	wire [47:0] SubKey8;
	wire [47:0] SubKey9;
	wire [47:0] SubKey10;
	wire [47:0] SubKey11;
	wire [47:0] SubKey12;
	wire [47:0] SubKey13;
	wire [47:0] SubKey14;
	wire [47:0] SubKey15;
	wire [47:0] SubKey16;
	wire [47:0] SubKeyM1;
	wire [47:0] SubKeyM2;
	wire [47:0] SubKeyM3;
	wire [47:0] SubKeyM4;
	wire [47:0] SubKeyM5;
	wire [47:0] SubKeyM6;
	wire [47:0] SubKeyM7;
	wire [47:0] SubKeyM8;
	wire [47:0] SubKeyM9;
	wire [47:0] SubKeyM10;
	wire [47:0] SubKeyM11;
	wire [47:0] SubKeyM12;
	wire [47:0] SubKeyM13;
	wire [47:0] SubKeyM14;
	wire [47:0] SubKeyM15;
	wire [47:0] SubKeyM16;
	wire [63:0] ip_out;
	wire [63:0] r1_out;
	wire [63:0] r2_out;
	wire [63:0] r3_out;
	wire [63:0] r4_out;
	wire [63:0] r5_out;
	wire [63:0] r6_out;
	wire [63:0] r7_out;
	wire [63:0] r8_out;
	wire [63:0] r9_out;
	wire [63:0] r10_out;
	wire [63:0] r11_out;
	wire [63:0] r12_out;
	wire [63:0] r13_out;
	wire [63:0] r14_out;
	wire [63:0] r15_out;
	wire [63:0] r16_out;
	GenerateKeys k1(
		.Key(key),
		.SubKey1(SubKey1),
		.SubKey2(SubKey2),
		.SubKey3(SubKey3),
		.SubKey4(SubKey4),
		.SubKey5(SubKey5),
		.SubKey6(SubKey6),
		.SubKey7(SubKey7),
		.SubKey8(SubKey8),
		.SubKey9(SubKey9),
		.SubKey10(SubKey10),
		.SubKey11(SubKey11),
		.SubKey12(SubKey12),
		.SubKey13(SubKey13),
		.SubKey14(SubKey14),
		.SubKey15(SubKey15),
		.SubKey16(SubKey16)
	);
	assign {SubKeyM1, SubKeyM2, SubKeyM3, SubKeyM4, SubKeyM5, SubKeyM6, SubKeyM7, SubKeyM8, SubKeyM9, SubKeyM10, SubKeyM11, SubKeyM12, SubKeyM13, SubKeyM14, SubKeyM15, SubKeyM16} = (encrypt ? {SubKey1, SubKey2, SubKey3, SubKey4, SubKey5, SubKey6, SubKey7, SubKey8, SubKey9, SubKey10, SubKey11, SubKey12, SubKey13, SubKey14, SubKey15, SubKey16} : {SubKey16, SubKey15, SubKey14, SubKey13, SubKey12, SubKey11, SubKey10, SubKey9, SubKey8, SubKey7, SubKey6, SubKey5, SubKey4, SubKey3, SubKey2, SubKey1});
	IP b1(
		.inp_block(plaintext),
		.out_block(ip_out)
	);
	round b2(
		.inp_block(ip_out),
		.subkey(SubKeyM1),
		.out_block(r1_out)
	);
	round b3(
		.inp_block(r1_out),
		.subkey(SubKeyM2),
		.out_block(r2_out)
	);
	round b4(
		.inp_block(r2_out),
		.subkey(SubKeyM3),
		.out_block(r3_out)
	);
	round b5(
		.inp_block(r3_out),
		.subkey(SubKeyM4),
		.out_block(r4_out)
	);
	round b6(
		.inp_block(r4_out),
		.subkey(SubKeyM5),
		.out_block(r5_out)
	);
	round b7(
		.inp_block(r5_out),
		.subkey(SubKeyM6),
		.out_block(r6_out)
	);
	round b8(
		.inp_block(r6_out),
		.subkey(SubKeyM7),
		.out_block(r7_out)
	);
	round b9(
		.inp_block(r7_out),
		.subkey(SubKeyM8),
		.out_block(r8_out)
	);
	round b10(
		.inp_block(r8_out),
		.subkey(SubKeyM9),
		.out_block(r9_out)
	);
	round b11(
		.inp_block(r9_out),
		.subkey(SubKeyM10),
		.out_block(r10_out)
	);
	round b12(
		.inp_block(r10_out),
		.subkey(SubKeyM11),
		.out_block(r11_out)
	);
	round b13(
		.inp_block(r11_out),
		.subkey(SubKeyM12),
		.out_block(r12_out)
	);
	round b14(
		.inp_block(r12_out),
		.subkey(SubKeyM13),
		.out_block(r13_out)
	);
	round b15(
		.inp_block(r13_out),
		.subkey(SubKeyM14),
		.out_block(r14_out)
	);
	round b16(
		.inp_block(r14_out),
		.subkey(SubKeyM15),
		.out_block(r15_out)
	);
	round b17(
		.inp_block(r15_out),
		.subkey(SubKeyM16),
		.out_block(r16_out)
	);
	FP FP(
		.inp_block({r16_out[31:0], r16_out[63:32]}),
		.out_block(ciphertext)
	);
endmodule
module UDL_Count (
	clk,
	rst,
	up,
	down,
	load,
	in,
	out
);
	parameter WIDTH = 8;
	input wire clk;
	input wire rst;
	input wire up;
	input wire down;
	input wire load;
	input wire [WIDTH - 1:0] in;
	output wire [WIDTH - 1:0] out;
	reg [WIDTH - 1:0] next;
	flop #(.WIDTH(WIDTH)) count(
		.clk(clk),
		.d(next),
		.q(out)
	);
	always @(*)
		if (rst)
			next = {WIDTH {1'b0}};
		else if (load)
			next = in;
		else if (up)
			next = out + 1'b1;
		else if (down)
			next = out - 1'b1;
		else
			next = out;
endmodule
module flop (
	clk,
	d,
	q
);
	parameter WIDTH = 8;
	input wire clk;
	input wire [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	always @(posedge clk) q <= d;
endmodule
module control (
	clk,
	reset,
	Found,
	Start,
	UP,
	en1,
	en2,
	FoundKeyNum
);
	input wire clk;
	input wire reset;
	input wire Start;
	input wire Found;
	output reg en1;
	output reg en2;
	output reg UP;
	output reg FoundKeyNum;
	reg [1:0] state;
	reg [1:0] nextstate;
	always @(posedge clk or posedge reset)
		if (reset)
			state <= 2'd0;
		else
			state <= nextstate;
	always @(*)
		case (state)
			2'd0: begin
				UP = 1'b0;
				en1 = 1'b0;
				en2 = 1'b0;
				FoundKeyNum = 1'b0;
				if (Start == 1'b1)
					nextstate = 2'd2;
				else
					nextstate = 2'd0;
			end
			2'd1: begin
				UP = 1'b1;
				en1 = 1'b0;
				en2 = 1'b0;
				FoundKeyNum = 1'b0;
				nextstate = 2'd2;
			end
			2'd2: begin
				UP = 1'b0;
				en1 = 1'b1;
				en2 = 1'b0;
				FoundKeyNum = 1'b0;
				if (Found == 1'b1)
					nextstate = 2'd3;
				else
					nextstate = 2'd1;
			end
			2'd3: begin
				UP = 1'b0;
				en1 = 1'b0;
				en2 = 1'b1;
				FoundKeyNum = 1'b1;
				if (Start == 1'b1)
					nextstate = 2'd3;
				else
					nextstate = 2'd0;
			end
			default: begin
				UP = 1'b0;
				en1 = 1'b0;
				en2 = 1'b0;
				nextstate = 2'd0;
				FoundKeyNum = 1'b0;
			end
		endcase
endmodule
module comparator (
	a,
	b,
	eq
);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] a;
	input wire [WIDTH - 1:0] b;
	output wire eq;
	assign eq = a == b;
endmodule