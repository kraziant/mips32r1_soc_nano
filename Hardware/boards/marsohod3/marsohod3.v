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
	input  clock,
	input  reset,
	output [31:0] wb_iadr_o,
	input  uart_rx,
	output uart_tx
	);

	wire wb_clk;
	assign wb_clk = clock;
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
		.wb_clk_i(wb_clk),
		.wb_rst_i(wb_rst),

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

	soc soc(
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
