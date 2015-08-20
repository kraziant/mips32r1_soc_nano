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

module Inst_RAM(
    input [29:0] addr,
    input read,
    output [31:0] value,
    output ack
);

	reg ack;
	reg [31:0] value;

	reg [31:0] mem[0:15];
	initial $readmemh("memory.hex", mem);

	always @(*)
	begin
		if (read == 1'b1)
		begin
			//value <= 32'b00000000;
			value <= mem[addr[3:0]];
			ack <= 1;
		end
		else
		begin
			ack <= 0;
		end
	end

endmodule

module cpu_top(
	input  clock_50MHz,
	input  reset_n
	);

	wire reset;
	assign reset = !reset_n;
	wire clock, clock2x;
	wire PLL_Locked;

	PLL_50MHz_to_50MHz_100MHz Clock_Generator(
		.areset (reset),
		.inclk0 (clock_50MHz),
		.c0     (clock),
		.c1     (clock2x),
		.locked (PLL_Locked)
	);

    // MIPS Processor Signals
//    reg  [31:0] MIPS32_DataMem_In;
    wire [31:0] MIPS32_DataMem_Out, MIPS32_InstMem_In;
    wire [29:0] MIPS32_DataMem_Address, MIPS32_InstMem_Address;
    wire [3:0]  MIPS32_DataMem_WE;
    wire        MIPS32_DataMem_Read, MIPS32_InstMem_Read;
    //reg         MIPS32_DataMem_Ack;
    //wire [4:0]  MIPS32_Interrupts;
    //wire        MIPS32_NMI;
    wire [7:0]  MIPS32_IP;
    wire        MIPS32_IO_WE;

    reg [4:0]  MIPS32_Interrupts = 5'h0;
    reg MIPS32_NMI = 0;
    reg  [31:0] MIPS32_DataMem_In = 32'h00000000;
    reg         MIPS32_DataMem_Ack = 0;

	wire MIPS32_InstMem_AckA;

	Inst_RAM iram(
		.addr	(MIPS32_InstMem_Address),
		.read	(MIPS32_InstMem_Read),
		.value	(MIPS32_InstMem_In),
		.ack	(MIPS32_InstMem_AckA)
	);

     // MIPS-32 Core
    Processor MIPS32 (
        .clock            (clock_50MHz),
        .reset            (reset),
        .Interrupts       (MIPS32_Interrupts),
        .NMI              (MIPS32_NMI),
        .DataMem_In       (MIPS32_DataMem_In),
        .DataMem_Ack    (MIPS32_DataMem_Ack),
        .DataMem_Read     (MIPS32_DataMem_Read),
        .DataMem_Write    (MIPS32_DataMem_WE),
        .DataMem_Address  (MIPS32_DataMem_Address),
        .DataMem_Out      (MIPS32_DataMem_Out),

        .InstMem_In       (MIPS32_InstMem_In),
        .InstMem_Address  (MIPS32_InstMem_Address),
        .InstMem_Ack    (MIPS32_InstMem_AckA),
        .InstMem_Read     (MIPS32_InstMem_Read),

        .IP               (MIPS32_IP)
    );

	initial
	begin
		#250;
		#10 MIPS32_DataMem_Ack = 1;
		#10 MIPS32_DataMem_Ack = 0;
		#20;
		#10 MIPS32_DataMem_Ack = 1;
		#10 MIPS32_DataMem_Ack = 0;
	end
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
		#1000 $finish;

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
