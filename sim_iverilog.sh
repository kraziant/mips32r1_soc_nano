#!/bin/sh -e

SRC=$(pwd)/Hardware/src

cd testbench
iverilog \
	-g2001 \
	-I $SRC/MIPS32 \
	-I $SRC/uart16550-1.5 \
	-DPRESCALER_PRESET_HARD \
	-DPRESCALER_HIGH_PRESET=0 \
	-DPRESCALER_LOW_PRESET=2 \
	-I $SRC/verilog_utils \
	-I $SRC/wb_intercon \
	top.v \
	$SRC/mips32r1_soc.v \
	$SRC/mips32r1_wb.v \
	$SRC/verilog-arbiter/src/arbiter.v \
	$SRC/wb_intercon/rtl/verilog/wb_arbiter.v \
	$SRC/wb_intercon/rtl/verilog/wb_mux.v \
	$SRC/wb_intercon/rtl/verilog/wb_data_resize.v \
	$SRC/wb_intercon/wb_intercon.v \
	$SRC/rom.v \
	$SRC/uart16550-1.5/uart_defines.v \
	$SRC/uart16550-1.5/raminfr.v \
	$SRC/uart16550-1.5/uart_sync_flops.v \
	$SRC/uart16550-1.5/uart_regs.v \
	$SRC/uart16550-1.5/uart_wb.v \
	$SRC/uart16550-1.5/uart_debug_if.v \
	$SRC/uart16550-1.5/uart_receiver.v \
	$SRC/uart16550-1.5/uart_top.v \
	$SRC/uart16550-1.5/uart_rfifo.v \
	$SRC/uart16550-1.5/uart_tfifo.v \
	$SRC/uart16550-1.5/uart_transmitter.v \
	$SRC/MIPS32/Add.v \
	$SRC/MIPS32/ALU.v \
	$SRC/MIPS32/Compare.v \
	$SRC/MIPS32/Control.v \
	$SRC/MIPS32/CPZero.v \
	$SRC/MIPS32/Divide.v \
	$SRC/MIPS32/EXMEM_Stage.v \
	$SRC/MIPS32/Hazard_Detection.v \
	$SRC/MIPS32/IDEX_Stage.v \
	$SRC/MIPS32/IFID_Stage.v \
	$SRC/MIPS32/MemControl.v \
	$SRC/MIPS32/MEMWB_Stage.v \
	$SRC/MIPS32/Mux2.v \
	$SRC/MIPS32/Mux4.v \
	$SRC/MIPS32/Processor.v \
	$SRC/MIPS32/RegisterFile.v \
	$SRC/MIPS32/Register.v \
	$SRC/MIPS32/TrapDetect.v

./a.out

gtkwave Top.vcd top.gtkw
