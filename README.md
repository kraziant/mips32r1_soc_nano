MIPS32 Release 1 Wishbone SoC
=============================

This is a Wishbone System-on-Chip (SoC) version
of the mips32r1_soc_nano project for Altera's development boards

Features
--------

- Pipelined MIPS32 bare-metal processor from https://github.com/grantea/mips32r1_core
- 10 MHz core
- 1 KB boot ROM with nmon software
- UART

Requirements
------------

- Terasic DE0-Nano, Terasic DC1-SoC, Marsohod2 or Marsohod3 board
- Altera Quartus II software
- build utilities (make, etc.)
- Serial port to 3.3V UART hardware
- Icarus Verilog & GTKWave software for simulation

Getting Started
---------------

`Hardware/README` contains instructions for building the SoC.
