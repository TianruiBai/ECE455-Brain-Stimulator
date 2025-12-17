// current pipeline is with 32-byte assembler no parser for now.
// Pipeline: UART_rx -> rdy edge -> rx_FIFO_Buffer -> assembler -> stim_ctrl_fsm

// Might add a TX transmission later, but will need to see how this does...
module top (
  input  wire  clk,       // 50 MHz system clock
  input  wire  rst,     // active-low reset

  inout  wire  I2C_SCL,
  output  wire  I2C_SDA,

  input  wire [3:0]  dac_miso,
  input  wire [3:0]  adc_miso,

  // Pmod BLE pins
  input  wire  bt_rx,     // BLE TX -> FPGA RX
  output  wire  rst_bt, // output for reset for the bluetooth module
  output wire bt_tx,      // FPGA TX -> BLE RX

  output logic [3:0]  dac_sclk,
  output logic [3:0]  dac_ss_n,
  output logic [3:0]  dac_mosi,

  // ADC Interface
  output logic [3:0]  adc_sclk,
  output logic [3:0]  adc_ss_n,
  output logic [3:0]  adc_mosi,

  output logic led0,
  output logic led1
  
);
logic clkout0, clkout1;
Gowin_PLL iPLL0(
        .clkin(clk), //input  clkin
        .clkout0(clkout0), //output  clkout0 //100MHZ
        .clkout1(clkout1) //output  clkout1 80MHZ
);

logic rst_FF0, rst_FF1, rst_bt_n, rst_n;
always_ff @(posedge clk) begin
    rst_FF0 <= rst;
    rst_bt_n <= ~rst_FF0;
end
assign rst_bt = rst_bt_n;

logic main_rst_FF, main_rst_FF2;
always_ff @(posedge clk) begin
    main_rst_FF <= rst;
    rst_n <= ~main_rst_FF;
end
  // reg [23:0] hb;
  // always_ff @(posedge clk or negedge rst_n) begin
  //   if (!rst_n) hb <= '0;
  //   else        hb <= hb + 24'd1;
  // end
  // assign led0 = hb[23];


  logic pair_ok = 1'b1; // forgot what this is, we'll leave it here for now

  // UART RX parses raw bytes from bt_rx
  logic        rdy;          // byte ready from UART_rx
  logic        clr_rdy;      // pulse to clear rdy, we drive this
  logic [7:0]  rx_data;      // byte value from UART_rx

  UART_rx u_rx (
    .clk     (clk),
    .rst_n   (rst_bt_n),
    .RX      (bt_rx),
    .clr_rdy (clr_rdy),
    .rx_data (rx_data),
    .rdy     (rdy)
  );

  // rdy rising edge -> one-cycle rx_valid
  // latch rx_byte same cycle.
  logic       rdy_q;
  logic       rx_valid;
  logic [7:0] rx_byte;

  always_ff @(posedge clk or negedge rst_bt_n) begin
    if (!rst_bt_n) begin
      rdy_q   <= 1'b0;
      rx_byte <= 8'h00;
    end else begin
      rdy_q <= rdy;
      if (rdy & ~rdy_q) begin
        rx_byte <= rx_data;       // Exactly once
      end
    end
  end

  // Not implementing for now, to lessen the complexity of the code.
  // wire fifo_full, fifo_empty;
  // wire [7:0] fifo_dout;

  // rx_fifo #(.DEPTH(32)) u_fifo (
  //   .clk    (clk),
  //   .rst_n  (rst_n),
  //   .din    (rx_byte),
  //   .wr_en  (rx_valid),
  //   .full   (fifo_full),
  //   .dout   (fifo_dout),
  //   .rd_en  (asm_ready && !fifo_empty),
  //   .empty  (fifo_empty)
  // );
  assign rx_valid = (rdy & ~rdy_q);  // 1-cycle pulse on new byte
  assign clr_rdy  = rx_valid;        // clear UART_rx ready the same cycle

  // 32-byte assembler
  // Collects 32 bytes -> payload_256 + pkt_valid
  logic        pkt_valid;
  logic [71:0] payload_72;

  // first byte -> [7:0], second -> [15:8], ... last - > [31:24]
// changed to 9 bytes
  rx_assembler_32B #(
    .BYTES(9)
  ) u_asm (
    .clk        (clk),
    .rst_n      (rst_bt_n),        // active-low reset
    .byte_valid (rx_valid),
    .byte_in    (rx_byte),
    .asm_ready  (),
    .payload    (payload_72),
    .pkt_valid  (pkt_valid),
    .dropped    (/* open */)
  );

// parses the packet
  logic        pkt_ready;
  logic [3:0]  pkt_cmd;
  logic [3:0]  channels;
  logic [1:0]  waveform;
  logic [20:0] phase;
  logic [19:0] frequency;
  logic [20:0] currents;
  logic        frame_err;
  pkt_parser #(
    .PAYLOAD_BYTES(9)
  ) pkt_parser(
    .clk        (clk),
    .rst        (~rst_n),

    // From rx assembler
    .pkt_valid  (pkt_valid),
    .payload    (payload_72),

    .pkt_ready  (pkt_ready),
    .pkt_cmd    (pkt_cmd),
    .channels   (channels),
    .waveform   (waveform),
    .phase      (phase),
    .frequency  (frequency),
    .currents   (currents),

    .frame_err  (frame_err)
  );

  logic         cmd_executed;
  logic [1:0] led0_bus, led1_bus;
  cmd_exec execute_cmd(
    .clk          (clk),
    .rst          (~rst_n),

    .pkt_ready    (pkt_ready),
    .pkt_cmd      (pkt_cmd),
    .channels     (channels),
    .wave_type    (waveform),
    .phase        (phase),
    .frequency    (frequency),
    .currents     (currents),

    .cmd_executed (cmd_executed),

    .dac_sclk     (dac_sclk),
    .dac_ss_n     (dac_ss_n),
    .dac_mosi     (dac_mosi),
    .dac_miso     (dac_miso),

    .adc_sclk     (adc_sclk),
    .adc_ss_n     (adc_ss_n),
    .adc_mosi     (adc_mosi),
    .adc_miso     (adc_miso),
    .led0         (led0_bus),
    .led1         (led1_bus)
  );

assign led0 = |led0_bus;
assign led1 = |led1_bus;


   //Stim control FSM
   //Tie off pkt_type and crc_ok since we aren't parsing yet
//  logic        afe_start;
//  logic [31:0] afe_cmd;
//  logic        afe_busy, afe_done, afe_fault;
//  logic        stim_enable;
//  logic        tx_ack, tx_nack;
//  logic [7:0]  status_code;

   //Tie off AFE for now
//  assign afe_busy  = 1'b0;
//  assign afe_done  = 1'b0;
//  assign afe_fault = 1'b0;

   //Dummy ties for unused parser fields
//  wire [1:0] pkt_type = 2'b00;
//  wire       crc_ok   = 1'b1;
  logic        sample_req;
  logic        status_ready, status_err;
  logic [15:0] vbat, vsys, ibat, isys, psys;
  logic [7:0]  state0;

  ip2363_status batt0 (
    .clk          (clk),
    .rst_n        (rst_n),
    .sample_req   (sample_req),
    .status_ready (status_ready),
    .error        (status_err),
    .i2c_sda      (I2C_SDA),
    .i2c_scl      (I2C_SCL),
    .vbat_mV      (vbat),
    .vsys_mV      (vsys),
    .ibat_mA      (ibat),
    .isys_mA      (isys),
    .psys_10mW    (psys),
    .state0       (state0)
  );

//  stim_ctrl_fsm u_fsm (
//    .clk         (clkout0),
//    .rst         (~rst_n),        // this FSM expects active-high reset
    // link
//    .pair_ok     (pair_ok),
//    .disconnect  (1'b0),
//    .timeout     (1'b0),
     //packet interface
//    .pkt_ready   (pkt_valid),     // drive from assembler complete
//    .pkt_type    (pkt_type),      // tied-off
//    .crc_ok      (crc_ok),        // tied-off
//    .payload     (payload_72),
    // AFE
//    .afe_start   (afe_start),
//    .afe_cmd     (afe_cmd),
//    .afe_busy    (afe_busy),
//    .afe_done    (afe_done),
//    .afe_fault   (afe_fault),
    // stim
//    .stim_enable (stim_enable),
    // app status -> TX
//    .tx_ack      (tx_ack),
//    .tx_nack     (tx_nack),
//    .status_code (status_code)
//  );

  // ACK/NACK formatter → UART TX (optional debug)
  logic        tx_valid;
  logic [7:0]  tx_byte;
  logic        tx_done;

  // TX:  TBD
  // Implementation ideas: Have a command to send an status report over to the android device.
  UART_tx u_tx (
    .clk     (clk),
    .rst_n   (rst_bt_n),           // active-low reset
    .TX      (bt_tx),
    .trmt    (tx_valid),        // "send this byte now"
    .tx_data (tx_byte),
    .tx_done (tx_done)          // "byte accepted/done"
  );

endmodule
