#!/bin/sh -e

iverilog -I MIPS32 \
	Top3.v \
	MIPS32/Add.v \
	MIPS32/ALU.v \
	MIPS32/Compare.v \
	MIPS32/Control.v \
	MIPS32/CPZero.v \
	MIPS32/Divide.v \
	MIPS32/EXMEM_Stage.v \
	MIPS32/Hazard_Detection.v \
	MIPS32/IDEX_Stage.v \
	MIPS32/IFID_Stage.v \
	MIPS32/MemControl.v \
	MIPS32/MEMWB_Stage.v \
	MIPS32/Mux2.v \
	MIPS32/Mux4.v \
	MIPS32/Processor.v \
	MIPS32/RegisterFile.v \
	MIPS32/Register.v \
	MIPS32/TrapDetect.v

./a.out

gtkwave Top.vcd top.gtkw
