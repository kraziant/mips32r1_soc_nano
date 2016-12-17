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

module de0_nano(
	input  CLOCK_50,
	input  [3:0]  SW,
	input  [1:0]  KEY,
	output [7:0]  LED,
	inout  [33:0] GPIO_0,
	inout  [33:0] GPIO_1);

	reg reset = 0;
	wire clk10m;
	wire pll_locked;

	altpll0 pll10MHz(
		.inclk0(CLOCK_50),
		.c0(clk10m)
	);
	

	wire [35:0] cout;
	counter counter(.out(cout), .clk(clk10m), .reset(0));

	//wire led_oe = ((cout[0] & cout[1]) & ~my_reset);
	wire led_oe = (cout[0] & cout[1]);

	wire slow_clock = cout[8];
	assign LED[7] = slow_clock;
	assign LED[6] = slow_clock;

	wire my_reset;
	assign LED[1] = my_reset;
	reset mreset(slow_clock, my_reset);

	wire [31:0] wb_iadr_o;

	mips32r1_soc #(
		.MEMFILE ("nmon.be.10MHz.9600.txt")
	)
	soc(
		.clock(clk10m),
		.reset(my_reset),
		.wb_iadr_o(wb_iadr_o),
		.uart_rx(GPIO_0[32]),
		.uart_tx(GPIO_0[33])
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
