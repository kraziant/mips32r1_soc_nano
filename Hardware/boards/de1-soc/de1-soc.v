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

module soc(
	input  clock_50MHz,
	input  reset,
	output [31:0] wb_iadr_o,
	input  uart_rx,
	output uart_tx
	);

	wire wb_clk;
	assign wb_clk = clock_50MHz;
	wire wb_rst;
	assign wb_rst = reset;

`include "wb_intercon.vh"

	rom bootrom(
		.wb_clk(wb_clk),
		.wb_rst(wb_rst),

		.wb_adr_i(wb_m2s_rom0_adr[10:2]),
		.wb_stb_i(wb_m2s_rom0_stb),
		.wb_cyc_i(wb_m2s_rom0_cyc),
		.wb_dat_o(wb_s2m_rom0_dat),
		.wb_ack_o(wb_s2m_rom0_ack)
	);

	uart_top uart16550(
		.wb_clk_i(wb_clk),
		.wb_rst_i(wb_rst),

		.wb_adr_i(wb_m2s_uart0_adr[4:0]),
		.wb_dat_i(wb_m2s_uart0_dat),
		.wb_sel_i(wb_m2s_uart0_sel),
		.wb_we_i(wb_m2s_uart0_we),
		.wb_cyc_i(wb_m2s_uart0_cyc),
		.wb_stb_i(wb_m2s_uart0_stb),
		.wb_dat_o(wb_s2m_uart0_dat),
		.wb_ack_o(wb_s2m_uart0_ack),

		.stx_pad_o(uart_tx),
		.srx_pad_i(uart_rx)
	);

	mips32r1_wb mips32r1_wb(
		.wb_clk_i(clock_50MHz),
		.wb_rst_i(reset),

		.iwbm_adr_o(wb_m2s_mips32r1_i_adr),
		.iwbm_dat_i(wb_s2m_mips32r1_i_dat),
		.iwbm_stb_o(wb_m2s_mips32r1_i_stb),
		.iwbm_ack_i(wb_s2m_mips32r1_i_ack),
		.iwbm_cyc_o(wb_m2s_mips32r1_i_cyc),

		.dwbm_adr_o(wb_m2s_mips32r1_d_adr),
		.dwbm_dat_i(wb_s2m_mips32r1_d_dat),
		.dwbm_stb_o(wb_m2s_mips32r1_d_stb),
		.dwbm_ack_i(wb_s2m_mips32r1_d_ack),
		.dwbm_cyc_o(wb_m2s_mips32r1_d_cyc),
		.dwbm_dat_o(wb_m2s_mips32r1_d_dat),
		.dwbm_we_o(wb_m2s_mips32r1_d_we),
		.dwbm_sel_o(wb_m2s_mips32r1_d_sel)
	);

	assign wb_iadr_o = wb_m2s_mips32r1_i_adr;
endmodule

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

	soc soc(
		.clock_50MHz(clk10m),
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
