# Create a Quartus II project for simple flashing LEDs demo
# on the Marsohod2 board.
#
# Arg 1: Project name
# Arg 2: Source directory

if { $::argc != 2 } {
    puts "Error: Insufficient or invalid options passed to script \"[file tail $argv0]\"."
    exit 1
}

set proj [lindex $::argv 0]
set src  [lindex $::argv 1]
set family "Cyclone III"
set part   "EP3C10E144C8"
set top    "marsohod2"

# Create a new project
project_new -family $family -part $part $proj
set_global_assignment -name TOP_LEVEL_ENTITY $top

source ../scripts/marsohodx_list.tcl

marsohodx_list $src
marsohodx_list ../src

set_global_assignment -name SEARCH_PATH "../src/MIPS32"
set_global_assignment -name SEARCH_PATH "../src/uart16550-1.5"
set_global_assignment -name SEARCH_PATH "../src/verilog_utils"
set_global_assignment -name SEARCH_PATH "../src/wb_intercon"

set_global_assignment -name FLOW_ENABLE_IO_ASSIGNMENT_ANALYSIS ON

# Pin constraints
set_location_assignment PIN_25 -to CLK100MHZ

set_location_assignment PIN_79 -to LED[3]
set_location_assignment PIN_83 -to LED[2]
set_location_assignment PIN_84 -to LED[1]
set_location_assignment PIN_85 -to LED[0]

set_global_assignment -name CYCLONEII_RESERVE_NCEO_AFTER_CONFIGURATION "USE AS REGULAR IO"
set_location_assignment PIN_101 -to IO[0]
set_location_assignment PIN_103 -to IO[1]
set_location_assignment PIN_104 -to IO[2]
set_location_assignment PIN_105 -to IO[3]
set_location_assignment PIN_106 -to IO[4]
set_location_assignment PIN_110 -to IO[5]
set_location_assignment PIN_111 -to IO[6]
set_location_assignment PIN_112 -to IO[7]

set_location_assignment PIN_24 -to FTDI_BD0
set_location_assignment PIN_28 -to FTDI_BD1
set_location_assignment PIN_11 -to FTDI_BD2
set_location_assignment PIN_10 -to FTDI_BD3
