`timescale 1ns / 1ps

module PLL_50MHz_to_50MHz_100MHz(
	input areset,
	input inclk0,
	output c0,
	output c1,
	output locked
);

	reg c0;
	wire c1;
	reg locked;

	assign c1 = areset ? 1'b0 : inclk0;

	always @(posedge inclk0)
	begin
		if (areset)
			c0 = 0;
		else
			c0 <= !c0;
	end

	always @areset
		if (areset)
		begin
			c0 = 0;
			locked = 0;
		end
		else
		begin
			locked = 1;
		end

endmodule

module mips32r1_bus_if_wb32(
	/* mips32r1 interface */
	input [29:0] cpu_addr,
	input cpu_read,
	output [31:0] cpu_value_o,
	output cpu_ack,
	input [3:0] cpu_we,
	input [31:0] cpu_value_i,

	/* wishbone */
	input wb_rst_i,
	input wb_clk_i,

	output [31:0] wb_adr_o,
	output [31:0] wb_dat_o,
	input [31:0] wb_dat_i,
	output wb_we_o,
	output [3:0] wb_sel_o,
	output wb_stb_o,
	input wb_ack_i,
	output wb_cyc_o
);

	reg [2:0] state;

`define IDLE 3'b000
`define WBSTART 3'b001
`define WBEND 3'b010

	reg [31:0] wb_adr_o;
	reg [31:0] wb_dat_o;
	reg wb_we_o;
	reg [3:0] wb_sel_o;
	reg wb_stb_o;
	reg wb_cyc_o;

	reg cpu_ack;
	reg [31:0] cpu_value_o;

	wire we;
	assign we = (cpu_we[0] | cpu_we[1] | cpu_we[2] | cpu_we[3]);

	always @(posedge wb_clk_i)
	if (wb_rst_i)
	begin
		state <= `IDLE;

		wb_adr_o <= 0;
		wb_dat_o <= 0;
		wb_we_o <= 0;
		wb_sel_o <= 0;
		wb_stb_o <= 0;
		wb_cyc_o <= 0;

		cpu_ack <= 0;
		cpu_value_o <= 0;
	end
	else
	begin
		case (state)
		`IDLE:
			if (cpu_read | we)
			begin
				wb_adr_o <= { cpu_addr[29:0], 2'b00 };
				wb_dat_o <= cpu_value_o;
				wb_we_o <= (we & !cpu_read);
				wb_sel_o <= cpu_we;

				wb_stb_o <= 1'b1;
				wb_cyc_o <= 1'b1;
				state <= `WBSTART;
			end
			else
			begin
				cpu_ack <= 1'b0;

				wb_stb_o <= 1'b0;
				wb_cyc_o <= 1'b0;
				wb_we_o <= 1'b0;
			end
		`WBSTART:
			if (wb_ack_i)
			begin
				cpu_value_o <= wb_dat_i;
				cpu_ack <= 1'b1;

				state <= `WBEND;

				wb_stb_o <= 1'b0;
				wb_cyc_o <= 1'b0;
				wb_we_o <= 1'b0;
			end

		`WBEND:
			if (!(cpu_read | we))
			begin
				cpu_ack <= 1'b0;

				state <= `IDLE;
			end

		default:
			state <= `IDLE;
		endcase
	end

endmodule

module mips32r1_wb(
	input wb_rst_i,
	input wb_clk_i,

	output [31:0] iwbm_adr_o,
	input [31:0] iwbm_dat_i,
	output [3:0] iwbm_sel_o,
	output iwbm_stb_o,
	input iwbm_ack_i,
	output iwbm_cyc_o,

	output [31:0] dwbm_adr_o,
	output [31:0] dwbm_dat_o,
	input [31:0] dwbm_dat_i,
	output dwbm_we_o,
	output [3:0] dwbm_sel_o,
	output dwbm_stb_o,
	input dwbm_ack_i,
	output dwbm_cyc_o
);

	wire [31:0] Handy_Instr_Address;

	wire [29:0] MIPS32_InstMem_Address;
	wire [31:0] MIPS32_InstMem_In;
	wire MIPS32_InstMem_Read;
	wire MIPS32_InstMem_AckA;

	wire [29:0] MIPS32_DataMem_Address;
	wire [31:0] MIPS32_DataMem_In;
	wire [31:0] MIPS32_DataMem_Out;
	wire [3:0] MIPS32_DataMem_Write;
	wire MIPS32_DataMem_Read;
	wire MIPS32_DataMem_Ack;

	reg [4:0] MIPS32_Interrupts = 5'h0;
	reg MIPS32_NMI = 0;

	reg [3:0] icpu_we = 0;

	mips32r1_bus_if_wb32 ibus_bridge(
		.cpu_addr	(MIPS32_InstMem_Address),
		.cpu_read	(MIPS32_InstMem_Read),
		.cpu_value_o	(MIPS32_InstMem_In),
		.cpu_ack	(MIPS32_InstMem_AckA),
		.cpu_we		(icpu_we),

		.wb_rst_i(wb_rst_i),
		.wb_clk_i(wb_clk_i),

		.wb_adr_o(iwbm_adr_o),
		.wb_dat_i(iwbm_dat_i),
		.wb_stb_o(iwbm_stb_o),
		.wb_ack_i(iwbm_ack_i),
		.wb_cyc_o(iwbm_cyc_o)
	);

	assign Handy_Instr_Address[31:2] = MIPS32_InstMem_Address[29:0];
	assign Handy_Instr_Address[1:0] = 2'b00;

	mips32r1_bus_if_wb32 dbus_bridge(
		.cpu_addr	(MIPS32_DataMem_Address),
		.cpu_read	(MIPS32_DataMem_Read),
		.cpu_value_o	(MIPS32_DataMem_In),
		.cpu_ack	(MIPS32_DataMem_Ack),
		.cpu_value_i	(MIPS32_DataMem_Out),
		.cpu_we		(MIPS32_DataMem_Write),

		.wb_rst_i(wb_rst_i),
		.wb_clk_i(wb_clk_i),

		.wb_adr_o(dwbm_adr_o),
		.wb_dat_i(dwbm_dat_i),
		.wb_we_o(dwbm_we_o),
		.wb_sel_o(dwbm_sel_o),
		.wb_stb_o(dwbm_stb_o),
		.wb_ack_i(dwbm_ack_i),
		.wb_cyc_o(dwbm_cyc_o)
	);

	Processor MIPS32(
		.clock			(wb_clk_i),
		.reset			(wb_rst_i),

		.Interrupts		(MIPS32_Interrupts),
		.NMI			(MIPS32_NMI),

		.InstMem_In		(MIPS32_InstMem_In),
		.InstMem_Address	(MIPS32_InstMem_Address),
		.InstMem_Ack		(MIPS32_InstMem_AckA),
		.InstMem_Read		(MIPS32_InstMem_Read),

		.DataMem_Address	(MIPS32_DataMem_Address),
		.DataMem_Read		(MIPS32_DataMem_Read),
		.DataMem_In		(MIPS32_DataMem_In),
		.DataMem_Ack		(MIPS32_DataMem_Ack),
		.DataMem_Write		(MIPS32_DataMem_Write),
		.DataMem_Out		(MIPS32_DataMem_Out)
	);

endmodule

module cpu_top(
	input  clock_50MHz,
	input  reset_n
	);

	wire reset;
	assign reset = !reset_n;
	wire clock, clock2x;

	PLL_50MHz_to_50MHz_100MHz Clock_Generator(
		.areset (reset),
		.inclk0 (clock_50MHz),
		.c0     (clock),
		.c1     (clock2x)
	);

	wire [31:0] brom_wb_adr_i;
	wire brom_wb_stb_i;
	wire brom_wb_cyc_i;
	wire brom_wb_ack_o;
	wire [31:0] brom_wb_dat_o;

	rom bootrom(
		.wb_clk(clock_50MHz),
		.wb_rst(reset),

		.wb_adr_i(brom_wb_adr_i[10:2]),
		.wb_stb_i(brom_wb_stb_i),
		.wb_cyc_i(brom_wb_cyc_i),
		.wb_dat_o(brom_wb_dat_o),
		.wb_ack_o(brom_wb_ack_o)
	);

	wire uart_wb_we_i;
	wire uart_wb_stb_i;
	wire uart_wb_cyc_i;
	wire uart_wb_ack_o;
	wire [31:0] uart_wb_adr_i;
	wire [31:0] uart_wb_dat_i;
	wire [31:0] uart_wb_dat_o;
	wire [3:0] uart_wb_sel_i;

	wire uart_tx;
	wire uart_rx;

	uart_top uart16550(
		.wb_clk_i(clock_50MHz),
		.wb_rst_i(reset),

		.wb_we_i(uart_wb_we_i),
		.wb_stb_i(uart_wb_stb_i),
		.wb_cyc_i(uart_wb_cyc_i),
		.wb_ack_o(uart_wb_ack_o),
		.wb_adr_i(uart_wb_adr_i[4:0]),
		.wb_dat_i(uart_wb_dat_i),
		.wb_dat_o(uart_wb_dat_o),
		.wb_sel_i(uart_wb_sel_i),

		.stx_pad_o(uart_tx),
		.srx_pad_i(uart_rx)
	);

	mips32r1_wb mips32r1_wb(
		.wb_clk_i(clock_50MHz),
		.wb_rst_i(reset),

		.iwbm_adr_o(brom_wb_adr_i),
		.iwbm_dat_i(brom_wb_dat_o),
		.iwbm_stb_o(brom_wb_stb_i),
		.iwbm_ack_i(brom_wb_ack_o),
		.iwbm_cyc_o(brom_wb_cyc_i),

		.dwbm_adr_o(uart_wb_adr_i),
		.dwbm_dat_i(uart_wb_dat_o),
		.dwbm_stb_o(uart_wb_stb_i),
		.dwbm_ack_i(uart_wb_ack_o),
		.dwbm_cyc_o(uart_wb_cyc_i),
		.dwbm_dat_o(uart_wb_dat_i),
		.dwbm_we_o(uart_wb_we_i),
		.dwbm_sel_o(uart_wb_sel_i)
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
		#100000 $finish;

	initial
	begin
		$dumpfile("Top.vcd");
		$dumpvars(0, Top);
	end

	cpu_top cpu(
		.clock_50MHz(clk),
		.reset_n(reset)
	);

endmodule
