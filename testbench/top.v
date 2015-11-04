`timescale 1ns / 1ps

module soc(
	input  clock_50MHz,
	input  reset_n
	);

	wire reset;
	assign reset = !reset_n;

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

	wire uart_tx;
	wire uart_rx;

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
endmodule

module Top;

	reg clk = 0;
	always
	begin
		#5 clk = !clk;
	end

	reg reset = 1;
	initial
	begin
		#100 reset = 0;
		#100 reset = 1;
	end

	initial
		#10000 $finish;

	initial
	begin
		$dumpfile("Top.vcd");
		$dumpvars(0, Top);
	end

	soc soc(
		.clock_50MHz(clk),
		.reset_n(reset)
	);

endmodule
