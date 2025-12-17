//Copyright (C)2014-2025 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.9 
//Created Time: 2025-12-08 23:04:29
create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk}]
