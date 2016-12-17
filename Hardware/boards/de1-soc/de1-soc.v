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

module de1_soc(
	input  CLOCK_50,
	output [6:0]  HEX0,
	output [6:0]  HEX1,
	output [6:0]  HEX2,
	output [6:0]  HEX3,
	output [6:0]  HEX4,
	output [6:0]  HEX5,
	input  [9:0]  SW,
	input  [3:0]  KEY,
	output [9:0]  LEDR,
	inout  [35:0] GPIO_0,
	inout  [35:0] GPIO_1);

	reg reset = 0;
	wire clk10m;
	wire pll_locked;

	pll10MHz pll_10(
		.refclk(CLOCK_50),
		.rst(reset),
		.outclk_0(clk10m),
		.locked(pll_locked));

	wire [35:0] cout;
	counter counter(.out(cout), .clk(clk10m), .reset(0));

	//wire led_oe = ((cout[0] & cout[1]) & ~my_reset);
	wire led_oe = (cout[0] & cout[1]);

	//hexLEDDriver segment0(.value(cout[15:12]), .out(HEX0), .oe(led_oe));
	//hexLEDDriver segment1(.value(cout[19:16]), .out(HEX1), .oe(led_oe));
	//hexLEDDriver segment2(.value(cout[23:20]), .out(HEX2), .oe(led_oe));
	//hexLEDDriver segment3(.value(cout[27:24]), .out(HEX3), .oe(led_oe));
	//hexLEDDriver segment4(.value(cout[31:28]), .out(HEX4), .oe(led_oe));
	//hexLEDDriver segment5(.value(cout[35:32]), .out(HEX5), .oe(led_oe));

	hexLEDDriver segment0(.value(wb_iadr_o[3:0]), .out(HEX0), .oe(led_oe));
	hexLEDDriver segment1(.value(wb_iadr_o[7:4]), .out(HEX1), .oe(led_oe));
	hexLEDDriver segment2(.value(wb_iadr_o[11:8]), .out(HEX2), .oe(led_oe));
	hexLEDDriver segment3(.value(wb_iadr_o[15:12]), .out(HEX3), .oe(led_oe));
	hexLEDDriver segment4(.value(wb_iadr_o[19:16]), .out(HEX4), .oe(led_oe));
	hexLEDDriver segment5(.value(wb_iadr_o[23:20]), .out(HEX5), .oe(led_oe));

	wire slow_clock = cout[8];
	assign LEDR[9] = slow_clock;
	assign LEDR[6] = slow_clock;


	wire my_reset;
	assign LEDR[1] = my_reset;
	reset mreset(slow_clock, my_reset);

	wire [31:0] wb_iadr_o;

	mips32r1_soc #(
		.MEMFILE ("nmon.be.10MHz.9600.txt")
	)
	soc(
		.clock(clk10m),
		.reset(my_reset),
		.wb_iadr_o(wb_iadr_o),
		.uart_rx(GPIO_0[3]),
		.uart_tx(GPIO_0[1])
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
