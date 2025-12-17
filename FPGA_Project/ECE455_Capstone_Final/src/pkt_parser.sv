// packet payload structure (MSB -> LSB):
// [71:68] cmd(4) | [67:64] channels(4) | [63:43] phase(21) | [42:21] frequency(22) | [20:0] currents(21)
module pkt_parser #(
  parameter int PAYLOAD_BYTES = 9
) (
  input  logic        clk,
  input  logic        rst,          // active-high synchronous reset

  // From UART RX / assembler
  input  logic        pkt_valid,    // pulse or level: we'll trigger on rising edge
  input  logic [71:0] payload,

  // To stim FSM
  output logic        pkt_ready,    // 1-cycle pulse when new fields are latched
  output logic [3:0]  pkt_cmd,
  output logic [3:0]  channels,
  output logic [1:0]  waveform,
  output logic [20:0] phase,
  output logic [19:0] frequency,
  output logic [20:0] currents,

  // Optional debug
  output logic        frame_err
);

  // Rising-edge detect on pkt_valid
  logic pkt_valid_d;
  always_ff @(posedge clk) begin
    if (rst) begin
      pkt_valid_d <= 1'b0;
    end else begin
      pkt_valid_d <= pkt_valid;
    end
  end
  wire pkt_valid_rise = pkt_valid & ~pkt_valid_d;

  // Latch outputs on rising edge
  always_ff @(posedge clk) begin
    if (rst) begin
      pkt_cmd   <= '0;
      channels  <= '0;
      phase     <= '0;
      waveform  <= '0;
      frequency <= '0;
      currents  <= '0;
      pkt_ready <= 1'b0;
      frame_err <= 1'b0;
    end else begin
      // default: deassert single-cycle pulse
      pkt_ready <= 1'b0;

      if (pkt_valid_rise) begin
        // Bitfield extraction (MSB -> LSB as documented)
        pkt_cmd   <= payload[71:68];
        channels  <= payload[67:64];
        phase     <= payload[63:43];
        waveform  <= payload[42:41];
        frequency <= payload[40:21];
        currents  <= payload[20:0];

        pkt_ready <= 1'b1;

        // Placeholder for any sanity checks you want (e.g., reserved bits, ranges)
        frame_err <= 1'b0;
      end
    end
  end
endmodule
