module reset(clk, reset);
	parameter WIDTH = 8;

	input clk;
	output reg reset = 1;

	reg [WIDTH - 1 : 0]   out = 0;
	wire clk;

	always @(posedge clk)
		if (reset == 1)
			out <= out + 1;

	always @(posedge clk)
		if (out[4:0] == 5'b11111)
			reset <= 0;
endmodule /* reset */

module marsohod3(
	input  CLK100MHZ,
	output [7:0] LED,
	inout [7:0] IO,
	input KEY1

//	input FTDI_BD0,		/* SK_i, TCK_i, TXD_i */
//	output FTDI_BD1,	/* DO_o, TDI_o, RXD_o */
//	input FTDI_BD2,		/* DI_i, TDO_i, RTS#_i */
//	input FTDI_BD3,		/* CS_i, TMS_i, CTS#_i */

	);

	wire clk10m, clk24m, clk1m;

	altpll0 pll(
		.inclk0(CLK100MHZ),
		.c0(clk10m),
		.c1(clk24m),
		.c2(clk1m)
	);

	wire [35:0] cout;
	counter counter(.out(cout), .clk(clk10m), .reset(0));
	wire slow_clock = cout[8];

	wire my_reset;
	assign LED[0] = my_reset;
	reset mreset(slow_clock, my_reset);

	mips32r1_soc soc(
		.clock(clk10m),
		.reset(my_reset),
		.wb_iadr_o(),
		.uart_rx(IO[7]),
		.uart_tx(IO[5])
	);

endmodule

module counter(out, clk, reset);

	parameter WIDTH = 36;

	output [WIDTH - 1 : 0] out;
	input clk, reset;

	reg [WIDTH - 1 : 0] out;
	wire clk, reset;

	always @(posedge clk)
		if (reset)
			out <= 0;
		else
			out <= out + 1;

endmodule /* counter */
