module UART_rx #(
    parameter int unsigned CLK_HZ = 50_000_000,
    parameter int unsigned BAUD   = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,     // active-low reset
    input  wire       RX,        // serial in
    input  wire       clr_rdy,   // pulse to clear rdy
    output wire [7:0] rx_data,   // received byte
    output reg        rdy        // goes high when a byte is ready
);

  // ===== Derived constants =====
  localparam int unsigned CLKS_PER_BIT = CLK_HZ / BAUD;         // ≈ 868
  localparam int unsigned FULL_RELOAD  = CLKS_PER_BIT - 1;      // 867
  localparam int unsigned HALF_RELOAD  = (CLKS_PER_BIT/2) - 1;  // 433
  localparam int CNTW = $clog2(CLKS_PER_BIT + 1);               // Counter width

  // ===== State encoding =====
  localparam IDLE = 1'b0, RECEIVE = 1'b1;
  reg state, next_state;

  // ===== RX synchronizer & falling-edge detect =====
  reg RX_a, RX_b;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      RX_a <= 1'b1;
      RX_b <= 1'b1;
    end else begin
      RX_a <= RX;
      RX_b <= RX_a;
    end
  end
  wire rx_falling_edge = RX_b & ~RX_a;

  // ===== Controls and counters =====
  reg start;
  reg receiving;
  reg set_rdy;
  reg [3:0] bit_cnt;

  // ===== Bit counter =====
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      bit_cnt <= 4'b0;
    else if (start)
      bit_cnt <= 4'b0;
    else if (shift)
      bit_cnt <= bit_cnt + 4'd1;
  end

  // ===== Baud/sample timer =====
  reg  [CNTW-1:0] baud_cnt;
  wire [CNTW-1:0] init_baud_cnt = start ? HALF_RELOAD : FULL_RELOAD;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      baud_cnt <= '0;
    else if (start || shift)
      baud_cnt <= init_baud_cnt;
    else if (receiving)
      baud_cnt <= baud_cnt - 1'b1;
  end

  // Time to sample
  wire shift = receiving && (baud_cnt == 0);

  // ===== Shift register =====
  reg [9:0] rx_shift_reg;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      rx_shift_reg <= 10'b0;
    else if (shift)
      rx_shift_reg <= {RX_b, rx_shift_reg[9:1]};
  end
  assign rx_data = rx_shift_reg[8:1];

  // ===== State register =====
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      state <= IDLE;
    else
      state <= next_state;
  end

  // ===== Next-state / control logic =====
  always @(*) begin
    next_state = state;
    start      = 1'b1;
    receiving  = 1'b0;
    set_rdy    = 1'b0;

    case (state)
      IDLE: begin
        if (rx_falling_edge) begin
          receiving  = 1'b1;
          next_state = RECEIVE;
        end
      end

      RECEIVE: begin
        receiving = 1'b1;
        start     = 1'b0;
        if (bit_cnt == 4'hA) begin
          set_rdy    = 1'b1;
          next_state = IDLE;
        end
      end
    endcase
  end

  // ===== rdy flag =====
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      rdy <= 1'b0;
    else if (clr_rdy)
      rdy <= 1'b0;
    else if (set_rdy)
      rdy <= 1'b1;
  end

endmodule