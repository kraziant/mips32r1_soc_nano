`timescale 1ns / 1ps

module Top;

	reg clk = 0;
	always
	begin
		#5 clk = !clk;
	end

	reg reset = 0;
	initial
	begin
		#100 reset = 1;
		#100 reset = 0;
	end

	initial
		#10000 $finish;

	initial
	begin
		$dumpfile("Top.vcd");
		$dumpvars(0, Top);
	end

	mips32r1_soc #(
		.MEMFILE ("nmon.be.10MHz.9600.txt")
	)
	soc (
		.clock(clk),
		.reset(reset)
	);

endmodule
