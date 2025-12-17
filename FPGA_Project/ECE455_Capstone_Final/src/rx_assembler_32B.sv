`default_nettype none
module rx_assembler_32B #(
  parameter int BYTES       = 9,      // fixed payload size
  parameter bit MSB_FIRST   = 1'b1,   // 0: first byte -> [7:0], 1: first byte -> [BYTES*8-1 : BYTES*8-8]
  parameter byte HEADER_BYTE= 8'hAA   // start-of-frame marker (not included in payload)
)(
  input  wire                clk,
  input  wire                rst_n,        // active-low async reset

  // Byte stream in (1-cycle pulse when rx byte is valid)
  input  wire                byte_valid,
  input  wire [7:0]          byte_in,
  output logic               asm_ready,    // always 1 (no backpressure)

  // Output packet
  output logic [BYTES*8-1:0] payload,
  output logic               pkt_valid,    // 1-cycle pulse when BYTES collected

  output logic               dropped       // kept for interface compatibility (always 0 here)
);

  // 2-state FSM: SEEK -> ACCUM to create payload based on packet transmitted.
  typedef enum logic [0:0] {SEEK, ACCUM} state_t;
  state_t state;

  // Index 0..BYTES-1
  localparam int W = (BYTES <= 1) ? 1 : $clog2(BYTES);
  logic [W-1:0] idx;

  // Always ready / no drop logic in this lean version
  assign asm_ready = 1'b1;
  assign dropped   = 1'b0;

  // Computes which byte-lane to write this cycle
  // If MSB_FIRST: first byte after header goes into the topmost lane
  function automatic [31:0] lane_index(input logic [W-1:0] i);
    if (MSB_FIRST)
      lane_index = (BYTES-1) - i;
    else
      lane_index = i;
  endfunction

  // Main FSM
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state     <= SEEK;
      idx       <= '0;
      payload   <= '0;
      pkt_valid <= 1'b0;
    end else begin
      pkt_valid <= 1'b0;  // default each cycle

      if (byte_valid) begin
        unique case (state)
          SEEK: begin
            // Wait for header, do not store byte
            if (byte_in == HEADER_BYTE) begin
              payload <= '0;     // clear buffer on sync
              idx     <= '0;
              state   <= ACCUM;
            end
            // else stay in SEEK
          end

          ACCUM: begin
            // Store this byte into the proper byte-lane
            payload[ 8*lane_index(idx) +: 8 ] <= byte_in;

            // Advance index and finish packet accum
            if (idx == BYTES-1) begin
              pkt_valid <= 1'b1;  // pulse exactly 1 cycle with the full payload registered
              idx       <= '0;
              state     <= SEEK;  // look for the next header
            end else begin
              idx <= idx + 1'b1;
            end
          end
        endcase
      end // if byte_valid
    end
  end

endmodule
`default_nettype wire
