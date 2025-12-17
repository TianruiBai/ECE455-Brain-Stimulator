module gw_gao(
    \rx_data[7] ,
    \rx_data[6] ,
    \rx_data[5] ,
    \rx_data[4] ,
    \rx_data[3] ,
    \rx_data[2] ,
    \rx_data[1] ,
    \rx_data[0] ,
    \rx_byte[7] ,
    \rx_byte[6] ,
    \rx_byte[5] ,
    \rx_byte[4] ,
    \rx_byte[3] ,
    \rx_byte[2] ,
    \rx_byte[1] ,
    \rx_byte[0] ,
    \payload_72[71] ,
    \payload_72[70] ,
    \payload_72[69] ,
    \payload_72[68] ,
    \payload_72[67] ,
    \payload_72[66] ,
    \payload_72[65] ,
    \payload_72[64] ,
    \payload_72[63] ,
    \payload_72[62] ,
    \payload_72[61] ,
    \payload_72[60] ,
    \payload_72[59] ,
    \payload_72[58] ,
    \payload_72[57] ,
    \payload_72[56] ,
    \payload_72[55] ,
    \payload_72[54] ,
    \payload_72[53] ,
    \payload_72[52] ,
    \payload_72[51] ,
    \payload_72[50] ,
    \payload_72[49] ,
    \payload_72[48] ,
    \payload_72[47] ,
    \payload_72[46] ,
    \payload_72[45] ,
    \payload_72[44] ,
    \payload_72[43] ,
    \payload_72[42] ,
    \payload_72[41] ,
    \payload_72[40] ,
    \payload_72[39] ,
    \payload_72[38] ,
    \payload_72[37] ,
    \payload_72[36] ,
    \payload_72[35] ,
    \payload_72[34] ,
    \payload_72[33] ,
    \payload_72[32] ,
    \payload_72[31] ,
    \payload_72[30] ,
    \payload_72[29] ,
    \payload_72[28] ,
    \payload_72[27] ,
    \payload_72[26] ,
    \payload_72[25] ,
    \payload_72[24] ,
    \payload_72[23] ,
    \payload_72[22] ,
    \payload_72[21] ,
    \payload_72[20] ,
    \payload_72[19] ,
    \payload_72[18] ,
    \payload_72[17] ,
    \payload_72[16] ,
    \payload_72[15] ,
    \payload_72[14] ,
    \payload_72[13] ,
    \payload_72[12] ,
    \payload_72[11] ,
    \payload_72[10] ,
    \payload_72[9] ,
    \payload_72[8] ,
    \payload_72[7] ,
    \payload_72[6] ,
    \payload_72[5] ,
    \payload_72[4] ,
    \payload_72[3] ,
    \payload_72[2] ,
    \payload_72[1] ,
    \payload_72[0] ,
    \pkt_cmd[3] ,
    \pkt_cmd[2] ,
    \pkt_cmd[1] ,
    \pkt_cmd[0] ,
    \channels[3] ,
    \channels[2] ,
    \channels[1] ,
    \channels[0] ,
    \waveform[1] ,
    \waveform[0] ,
    \phase[20] ,
    \phase[19] ,
    \phase[18] ,
    \phase[17] ,
    \phase[16] ,
    \phase[15] ,
    \phase[14] ,
    \phase[13] ,
    \phase[12] ,
    \phase[11] ,
    \phase[10] ,
    \phase[9] ,
    \phase[8] ,
    \phase[7] ,
    \phase[6] ,
    \phase[5] ,
    \phase[4] ,
    \phase[3] ,
    \phase[2] ,
    \phase[1] ,
    \phase[0] ,
    \frequency[19] ,
    \frequency[18] ,
    \frequency[17] ,
    \frequency[16] ,
    \frequency[15] ,
    \frequency[14] ,
    \frequency[13] ,
    \frequency[12] ,
    \frequency[11] ,
    \frequency[10] ,
    \frequency[9] ,
    \frequency[8] ,
    \frequency[7] ,
    \frequency[6] ,
    \frequency[5] ,
    \frequency[4] ,
    \frequency[3] ,
    \frequency[2] ,
    \frequency[1] ,
    \frequency[0] ,
    \currents[20] ,
    \currents[19] ,
    \currents[18] ,
    \currents[17] ,
    \currents[16] ,
    \currents[15] ,
    \currents[14] ,
    \currents[13] ,
    \currents[12] ,
    \currents[11] ,
    \currents[10] ,
    \currents[9] ,
    \currents[8] ,
    \currents[7] ,
    \currents[6] ,
    \currents[5] ,
    \currents[4] ,
    \currents[3] ,
    \currents[2] ,
    \currents[1] ,
    \currents[0] ,
    \led0_bus[1] ,
    \led0_bus[0] ,
    \led1_bus[1] ,
    \led1_bus[0] ,
    clkout0,
    tms_pad_i,
    tck_pad_i,
    tdi_pad_i,
    tdo_pad_o
);

input \rx_data[7] ;
input \rx_data[6] ;
input \rx_data[5] ;
input \rx_data[4] ;
input \rx_data[3] ;
input \rx_data[2] ;
input \rx_data[1] ;
input \rx_data[0] ;
input \rx_byte[7] ;
input \rx_byte[6] ;
input \rx_byte[5] ;
input \rx_byte[4] ;
input \rx_byte[3] ;
input \rx_byte[2] ;
input \rx_byte[1] ;
input \rx_byte[0] ;
input \payload_72[71] ;
input \payload_72[70] ;
input \payload_72[69] ;
input \payload_72[68] ;
input \payload_72[67] ;
input \payload_72[66] ;
input \payload_72[65] ;
input \payload_72[64] ;
input \payload_72[63] ;
input \payload_72[62] ;
input \payload_72[61] ;
input \payload_72[60] ;
input \payload_72[59] ;
input \payload_72[58] ;
input \payload_72[57] ;
input \payload_72[56] ;
input \payload_72[55] ;
input \payload_72[54] ;
input \payload_72[53] ;
input \payload_72[52] ;
input \payload_72[51] ;
input \payload_72[50] ;
input \payload_72[49] ;
input \payload_72[48] ;
input \payload_72[47] ;
input \payload_72[46] ;
input \payload_72[45] ;
input \payload_72[44] ;
input \payload_72[43] ;
input \payload_72[42] ;
input \payload_72[41] ;
input \payload_72[40] ;
input \payload_72[39] ;
input \payload_72[38] ;
input \payload_72[37] ;
input \payload_72[36] ;
input \payload_72[35] ;
input \payload_72[34] ;
input \payload_72[33] ;
input \payload_72[32] ;
input \payload_72[31] ;
input \payload_72[30] ;
input \payload_72[29] ;
input \payload_72[28] ;
input \payload_72[27] ;
input \payload_72[26] ;
input \payload_72[25] ;
input \payload_72[24] ;
input \payload_72[23] ;
input \payload_72[22] ;
input \payload_72[21] ;
input \payload_72[20] ;
input \payload_72[19] ;
input \payload_72[18] ;
input \payload_72[17] ;
input \payload_72[16] ;
input \payload_72[15] ;
input \payload_72[14] ;
input \payload_72[13] ;
input \payload_72[12] ;
input \payload_72[11] ;
input \payload_72[10] ;
input \payload_72[9] ;
input \payload_72[8] ;
input \payload_72[7] ;
input \payload_72[6] ;
input \payload_72[5] ;
input \payload_72[4] ;
input \payload_72[3] ;
input \payload_72[2] ;
input \payload_72[1] ;
input \payload_72[0] ;
input \pkt_cmd[3] ;
input \pkt_cmd[2] ;
input \pkt_cmd[1] ;
input \pkt_cmd[0] ;
input \channels[3] ;
input \channels[2] ;
input \channels[1] ;
input \channels[0] ;
input \waveform[1] ;
input \waveform[0] ;
input \phase[20] ;
input \phase[19] ;
input \phase[18] ;
input \phase[17] ;
input \phase[16] ;
input \phase[15] ;
input \phase[14] ;
input \phase[13] ;
input \phase[12] ;
input \phase[11] ;
input \phase[10] ;
input \phase[9] ;
input \phase[8] ;
input \phase[7] ;
input \phase[6] ;
input \phase[5] ;
input \phase[4] ;
input \phase[3] ;
input \phase[2] ;
input \phase[1] ;
input \phase[0] ;
input \frequency[19] ;
input \frequency[18] ;
input \frequency[17] ;
input \frequency[16] ;
input \frequency[15] ;
input \frequency[14] ;
input \frequency[13] ;
input \frequency[12] ;
input \frequency[11] ;
input \frequency[10] ;
input \frequency[9] ;
input \frequency[8] ;
input \frequency[7] ;
input \frequency[6] ;
input \frequency[5] ;
input \frequency[4] ;
input \frequency[3] ;
input \frequency[2] ;
input \frequency[1] ;
input \frequency[0] ;
input \currents[20] ;
input \currents[19] ;
input \currents[18] ;
input \currents[17] ;
input \currents[16] ;
input \currents[15] ;
input \currents[14] ;
input \currents[13] ;
input \currents[12] ;
input \currents[11] ;
input \currents[10] ;
input \currents[9] ;
input \currents[8] ;
input \currents[7] ;
input \currents[6] ;
input \currents[5] ;
input \currents[4] ;
input \currents[3] ;
input \currents[2] ;
input \currents[1] ;
input \currents[0] ;
input \led0_bus[1] ;
input \led0_bus[0] ;
input \led1_bus[1] ;
input \led1_bus[0] ;
input clkout0;
input tms_pad_i;
input tck_pad_i;
input tdi_pad_i;
output tdo_pad_o;

wire \rx_data[7] ;
wire \rx_data[6] ;
wire \rx_data[5] ;
wire \rx_data[4] ;
wire \rx_data[3] ;
wire \rx_data[2] ;
wire \rx_data[1] ;
wire \rx_data[0] ;
wire \rx_byte[7] ;
wire \rx_byte[6] ;
wire \rx_byte[5] ;
wire \rx_byte[4] ;
wire \rx_byte[3] ;
wire \rx_byte[2] ;
wire \rx_byte[1] ;
wire \rx_byte[0] ;
wire \payload_72[71] ;
wire \payload_72[70] ;
wire \payload_72[69] ;
wire \payload_72[68] ;
wire \payload_72[67] ;
wire \payload_72[66] ;
wire \payload_72[65] ;
wire \payload_72[64] ;
wire \payload_72[63] ;
wire \payload_72[62] ;
wire \payload_72[61] ;
wire \payload_72[60] ;
wire \payload_72[59] ;
wire \payload_72[58] ;
wire \payload_72[57] ;
wire \payload_72[56] ;
wire \payload_72[55] ;
wire \payload_72[54] ;
wire \payload_72[53] ;
wire \payload_72[52] ;
wire \payload_72[51] ;
wire \payload_72[50] ;
wire \payload_72[49] ;
wire \payload_72[48] ;
wire \payload_72[47] ;
wire \payload_72[46] ;
wire \payload_72[45] ;
wire \payload_72[44] ;
wire \payload_72[43] ;
wire \payload_72[42] ;
wire \payload_72[41] ;
wire \payload_72[40] ;
wire \payload_72[39] ;
wire \payload_72[38] ;
wire \payload_72[37] ;
wire \payload_72[36] ;
wire \payload_72[35] ;
wire \payload_72[34] ;
wire \payload_72[33] ;
wire \payload_72[32] ;
wire \payload_72[31] ;
wire \payload_72[30] ;
wire \payload_72[29] ;
wire \payload_72[28] ;
wire \payload_72[27] ;
wire \payload_72[26] ;
wire \payload_72[25] ;
wire \payload_72[24] ;
wire \payload_72[23] ;
wire \payload_72[22] ;
wire \payload_72[21] ;
wire \payload_72[20] ;
wire \payload_72[19] ;
wire \payload_72[18] ;
wire \payload_72[17] ;
wire \payload_72[16] ;
wire \payload_72[15] ;
wire \payload_72[14] ;
wire \payload_72[13] ;
wire \payload_72[12] ;
wire \payload_72[11] ;
wire \payload_72[10] ;
wire \payload_72[9] ;
wire \payload_72[8] ;
wire \payload_72[7] ;
wire \payload_72[6] ;
wire \payload_72[5] ;
wire \payload_72[4] ;
wire \payload_72[3] ;
wire \payload_72[2] ;
wire \payload_72[1] ;
wire \payload_72[0] ;
wire \pkt_cmd[3] ;
wire \pkt_cmd[2] ;
wire \pkt_cmd[1] ;
wire \pkt_cmd[0] ;
wire \channels[3] ;
wire \channels[2] ;
wire \channels[1] ;
wire \channels[0] ;
wire \waveform[1] ;
wire \waveform[0] ;
wire \phase[20] ;
wire \phase[19] ;
wire \phase[18] ;
wire \phase[17] ;
wire \phase[16] ;
wire \phase[15] ;
wire \phase[14] ;
wire \phase[13] ;
wire \phase[12] ;
wire \phase[11] ;
wire \phase[10] ;
wire \phase[9] ;
wire \phase[8] ;
wire \phase[7] ;
wire \phase[6] ;
wire \phase[5] ;
wire \phase[4] ;
wire \phase[3] ;
wire \phase[2] ;
wire \phase[1] ;
wire \phase[0] ;
wire \frequency[19] ;
wire \frequency[18] ;
wire \frequency[17] ;
wire \frequency[16] ;
wire \frequency[15] ;
wire \frequency[14] ;
wire \frequency[13] ;
wire \frequency[12] ;
wire \frequency[11] ;
wire \frequency[10] ;
wire \frequency[9] ;
wire \frequency[8] ;
wire \frequency[7] ;
wire \frequency[6] ;
wire \frequency[5] ;
wire \frequency[4] ;
wire \frequency[3] ;
wire \frequency[2] ;
wire \frequency[1] ;
wire \frequency[0] ;
wire \currents[20] ;
wire \currents[19] ;
wire \currents[18] ;
wire \currents[17] ;
wire \currents[16] ;
wire \currents[15] ;
wire \currents[14] ;
wire \currents[13] ;
wire \currents[12] ;
wire \currents[11] ;
wire \currents[10] ;
wire \currents[9] ;
wire \currents[8] ;
wire \currents[7] ;
wire \currents[6] ;
wire \currents[5] ;
wire \currents[4] ;
wire \currents[3] ;
wire \currents[2] ;
wire \currents[1] ;
wire \currents[0] ;
wire \led0_bus[1] ;
wire \led0_bus[0] ;
wire \led1_bus[1] ;
wire \led1_bus[0] ;
wire clkout0;
wire tms_pad_i;
wire tck_pad_i;
wire tdi_pad_i;
wire tdo_pad_o;
wire tms_i_c;
wire tck_i_c;
wire tdi_i_c;
wire tdo_o_c;
wire [9:0] control0;
wire gao_jtag_tck;
wire gao_jtag_reset;
wire run_test_idle_er1;
wire run_test_idle_er2;
wire shift_dr_capture_dr;
wire update_dr;
wire pause_dr;
wire enable_er1;
wire enable_er2;
wire gao_jtag_tdi;
wire tdo_er1;

IBUF tms_ibuf (
    .I(tms_pad_i),
    .O(tms_i_c)
);

IBUF tck_ibuf (
    .I(tck_pad_i),
    .O(tck_i_c)
);

IBUF tdi_ibuf (
    .I(tdi_pad_i),
    .O(tdi_i_c)
);

OBUF tdo_obuf (
    .I(tdo_o_c),
    .O(tdo_pad_o)
);

GW_JTAG  u_gw_jtag(
    .tms_pad_i(tms_i_c),
    .tck_pad_i(tck_i_c),
    .tdi_pad_i(tdi_i_c),
    .tdo_pad_o(tdo_o_c),
    .tck_o(gao_jtag_tck),
    .test_logic_reset_o(gao_jtag_reset),
    .run_test_idle_er1_o(run_test_idle_er1),
    .run_test_idle_er2_o(run_test_idle_er2),
    .shift_dr_capture_dr_o(shift_dr_capture_dr),
    .update_dr_o(update_dr),
    .pause_dr_o(pause_dr),
    .enable_er1_o(enable_er1),
    .enable_er2_o(enable_er2),
    .tdi_o(gao_jtag_tdi),
    .tdo_er1_i(tdo_er1),
    .tdo_er2_i(1'b0)
);

gw_con_top  u_icon_top(
    .tck_i(gao_jtag_tck),
    .tdi_i(gao_jtag_tdi),
    .tdo_o(tdo_er1),
    .rst_i(gao_jtag_reset),
    .control0(control0[9:0]),
    .enable_i(enable_er1),
    .shift_dr_capture_dr_i(shift_dr_capture_dr),
    .update_dr_i(update_dr)
);

ao_top u_ao_top(
    .control(control0[9:0]),
    .data_i({\rx_data[7] ,\rx_data[6] ,\rx_data[5] ,\rx_data[4] ,\rx_data[3] ,\rx_data[2] ,\rx_data[1] ,\rx_data[0] ,\rx_byte[7] ,\rx_byte[6] ,\rx_byte[5] ,\rx_byte[4] ,\rx_byte[3] ,\rx_byte[2] ,\rx_byte[1] ,\rx_byte[0] ,\payload_72[71] ,\payload_72[70] ,\payload_72[69] ,\payload_72[68] ,\payload_72[67] ,\payload_72[66] ,\payload_72[65] ,\payload_72[64] ,\payload_72[63] ,\payload_72[62] ,\payload_72[61] ,\payload_72[60] ,\payload_72[59] ,\payload_72[58] ,\payload_72[57] ,\payload_72[56] ,\payload_72[55] ,\payload_72[54] ,\payload_72[53] ,\payload_72[52] ,\payload_72[51] ,\payload_72[50] ,\payload_72[49] ,\payload_72[48] ,\payload_72[47] ,\payload_72[46] ,\payload_72[45] ,\payload_72[44] ,\payload_72[43] ,\payload_72[42] ,\payload_72[41] ,\payload_72[40] ,\payload_72[39] ,\payload_72[38] ,\payload_72[37] ,\payload_72[36] ,\payload_72[35] ,\payload_72[34] ,\payload_72[33] ,\payload_72[32] ,\payload_72[31] ,\payload_72[30] ,\payload_72[29] ,\payload_72[28] ,\payload_72[27] ,\payload_72[26] ,\payload_72[25] ,\payload_72[24] ,\payload_72[23] ,\payload_72[22] ,\payload_72[21] ,\payload_72[20] ,\payload_72[19] ,\payload_72[18] ,\payload_72[17] ,\payload_72[16] ,\payload_72[15] ,\payload_72[14] ,\payload_72[13] ,\payload_72[12] ,\payload_72[11] ,\payload_72[10] ,\payload_72[9] ,\payload_72[8] ,\payload_72[7] ,\payload_72[6] ,\payload_72[5] ,\payload_72[4] ,\payload_72[3] ,\payload_72[2] ,\payload_72[1] ,\payload_72[0] ,\pkt_cmd[3] ,\pkt_cmd[2] ,\pkt_cmd[1] ,\pkt_cmd[0] ,\channels[3] ,\channels[2] ,\channels[1] ,\channels[0] ,\waveform[1] ,\waveform[0] ,\phase[20] ,\phase[19] ,\phase[18] ,\phase[17] ,\phase[16] ,\phase[15] ,\phase[14] ,\phase[13] ,\phase[12] ,\phase[11] ,\phase[10] ,\phase[9] ,\phase[8] ,\phase[7] ,\phase[6] ,\phase[5] ,\phase[4] ,\phase[3] ,\phase[2] ,\phase[1] ,\phase[0] ,\frequency[19] ,\frequency[18] ,\frequency[17] ,\frequency[16] ,\frequency[15] ,\frequency[14] ,\frequency[13] ,\frequency[12] ,\frequency[11] ,\frequency[10] ,\frequency[9] ,\frequency[8] ,\frequency[7] ,\frequency[6] ,\frequency[5] ,\frequency[4] ,\frequency[3] ,\frequency[2] ,\frequency[1] ,\frequency[0] ,\currents[20] ,\currents[19] ,\currents[18] ,\currents[17] ,\currents[16] ,\currents[15] ,\currents[14] ,\currents[13] ,\currents[12] ,\currents[11] ,\currents[10] ,\currents[9] ,\currents[8] ,\currents[7] ,\currents[6] ,\currents[5] ,\currents[4] ,\currents[3] ,\currents[2] ,\currents[1] ,\currents[0] ,\led0_bus[1] ,\led0_bus[0] ,\led1_bus[1] ,\led1_bus[0] }),
    .clk_i(clkout0)
);

endmodule
