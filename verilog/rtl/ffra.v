module ffra 
#(
    parameter BITS=8
) (
`ifdef USE_POWER_PINS
    input vdd,
    input vss,
`endif
    input clk,
    input rst, // unused, but amaranth still creates it
    input [BITS-1:0] a,
    input [BITS-1:0] b,
    input [BITS*2-1:0] ci,
    output [BITS*2-1:0] o
);
    reg [BITS*2-1:0] o_tmp[3:0];

    always @(posedge clk) begin
	o_tmp[3] = o_tmp[2];
	o_tmp[2] = o_tmp[1];
	o_tmp[1] = o_tmp[0];
	o_tmp[0] = (a * b) + ci;
    end

    assign o = o_tmp[3];
endmodule
