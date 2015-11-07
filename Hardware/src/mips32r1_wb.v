module mips32r1_bus_if_wb32(
	/* mips32r1 interface */
	input [29:0] cpu_addr,
	input cpu_read,
	output reg [31:0] cpu_value_o,
	output reg cpu_ack,
	input [3:0] cpu_we,
	input [31:0] cpu_value_i,

	/* wishbone */
	input wb_rst_i,
	input wb_clk_i,

	output reg [31:0] wb_adr_o,
	output reg [31:0] wb_dat_o,
	input [31:0] wb_dat_i,
	output reg wb_we_o,
	output reg [3:0] wb_sel_o,
	output reg wb_stb_o,
	input wb_ack_i,
	output reg wb_cyc_o
);

	reg [2:0] state;

`define IDLE 3'b000
`define WBSTART 3'b001
`define WBEND 3'b010

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
				wb_dat_o <= cpu_value_i;
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
		/* FIXME: mips32r1 has no TLB/MMU so drop two most significant bits to get phys addr */
		.cpu_addr	({3'b000, MIPS32_DataMem_Address[26:0]}),
		.cpu_read	(MIPS32_DataMem_Read),
		.cpu_value_o	(MIPS32_DataMem_In),
		.cpu_ack	(MIPS32_DataMem_Ack),
		.cpu_value_i	(MIPS32_DataMem_Out),
		.cpu_we		(MIPS32_DataMem_Write),

		.wb_rst_i(wb_rst_i),
		.wb_clk_i(wb_clk_i),

		.wb_adr_o(dwbm_adr_o),
		.wb_dat_i(dwbm_dat_i),
		.wb_dat_o(dwbm_dat_o),
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
