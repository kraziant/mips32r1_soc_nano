module mips32r1_soc #(
	parameter MEMFILE = ""
	)
	(
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

localparam WB_BOOTROM_MEM_DEPTH = 1024;

	wb_bootrom #(
		.DEPTH (WB_BOOTROM_MEM_DEPTH),
		.MEMFILE (MEMFILE)
	)
	bootrom(
		.wb_clk_i(wb_clk),
		.wb_rst_i(wb_rst),

		.wb_adr_i(wb_m2s_rom0_adr),
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
