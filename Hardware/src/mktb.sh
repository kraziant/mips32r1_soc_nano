#!/bin/sh -e


iverilog \
	-g2001 \
	-I MIPS32 \
	-I uart16550-1.5 \
	Top3.v \
	rom.v \
	wb_ram.v \
	uart16550-1.5/uart_defines.v \
	uart16550-1.5/raminfr.v \
	uart16550-1.5/uart_sync_flops.v \
	uart16550-1.5/uart_regs.v \
	uart16550-1.5/uart_wb.v \
	uart16550-1.5/uart_debug_if.v \
	uart16550-1.5/uart_receiver.v \
	uart16550-1.5/uart_top.v \
	uart16550-1.5/uart_rfifo.v \
	uart16550-1.5/uart_tfifo.v \
	uart16550-1.5/uart_transmitter.v \
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
