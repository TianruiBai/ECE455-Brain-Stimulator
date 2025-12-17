module stim_ctrl_fsm #(
  parameter int TMO_PAIR_S = 10_000_000, // example clk cycles
  parameter int TMO_PKT_S  = 2_000_000,
  parameter int TMO_AFE_S  = 5_000_000
) (
  input  logic        clk,
  input  logic        rst,

  // BLE/link interface
  input  logic        pair_ok,
  input  logic        disconnect,
  input  logic        timeout,
  input  logic        pkt_ready,
  input  logic [1:0]  pkt_type,     // 0=START, 1=ADJUST, 2=STOP
  input  logic        crc_ok,
  input  logic [71:0]payload,

  // AFE interface SPI/I2C command executor
  output logic        afe_start,
  output logic [31:0] afe_cmd,
  input  logic        afe_busy,
  input  logic        afe_done,
  input  logic        afe_fault,

  // Control to stim path
  output logic        stim_enable,

  // Status back to app
  output logic        tx_ack,        // 1-cycle pulse on success
  output logic        tx_nack,
  output logic [7:0]  status_code
);

  typedef enum logic [3:0] {
    S_RESET,
    S_WAIT_PAIR,     // advertising, waiting to pair
    S_SLEEP,         // non-advertising idle (timeout path)
    S_IDLE_P,        // paired idle
    S_RX_VALIDATE,
    S_EXECUTE_CFG,
    S_RUN_MON,
    S_STOPPING,
    S_ERROR
  } state_t;

  state_t     st, st_n;
  logic [31:0] tmo;

  // simple decoder
  localparam logic [1:0] P_START = 2'd0;
  localparam logic [1:0] P_ADJ   = 2'd1;
  localparam logic [1:0] P_STOP  = 2'd2;

  // combinational next-state / outputs
  always_comb begin
    // defaults
    st_n        = st;
    afe_start   = 1'b0;
    afe_cmd     = '0;
    stim_enable = 1'b0;
    tx_ack      = 1'b0;
    tx_nack     = 1'b0;
    status_code = 8'h00;

    unique case (st)
      S_RESET: begin
        st_n = S_WAIT_PAIR;
      end

      S_WAIT_PAIR: begin
        // Only state that times out locally via tmo
        if (disconnect)         st_n = S_RESET;
        else if (pair_ok)       st_n = S_IDLE_P;
        else if (timeout)       st_n = S_SLEEP;          // external timeout
        else if (tmo==TMO_PAIR_S) st_n = S_SLEEP;        // local timeout
      end

      S_SLEEP: begin
        // non-advertising idle leave only on reset
        st_n = S_SLEEP;
      end

      S_IDLE_P: begin
        if (disconnect)               st_n = S_RESET;
        else if (pkt_ready)           st_n = S_RX_VALIDATE;
      end

      S_RX_VALIDATE: begin
        if (disconnect) begin
          st_n = S_STOPPING;
        end else if (!crc_ok) begin
          tx_nack     = 1'b1;
          status_code = 8'hC1; // CRC error
          st_n        = S_IDLE_P;
        end else begin
          st_n = S_EXECUTE_CFG;
        end
      end

      S_EXECUTE_CFG: begin
        // map payload to afe_cmd here
        afe_cmd = payload[31:0];   // placeholder
        if (!afe_busy) begin
          afe_start = 1'b1;        // kick once when idle
        end
        if (afe_done) begin
          tx_ack = 1'b1;
          st_n   = (pkt_type == P_START) ? S_RUN_MON : S_IDLE_P;
        end else if (afe_fault) begin
          tx_nack     = 1'b1;
          status_code = 8'hA1;
          st_n        = S_ERROR;
        end
      end

      S_RUN_MON: begin
        stim_enable = 1'b1;
        if (disconnect)                           st_n = S_RESET;
        else if (pkt_ready && pkt_type==P_ADJ)    st_n = S_RX_VALIDATE;
        else if (pkt_ready && pkt_type==P_STOP)   st_n = S_STOPPING;
        else if (afe_fault)                       st_n = S_ERROR;
      end

      S_STOPPING: begin
        // send safe commands to disable output
        afe_cmd   = 32'h0000;
        if (!afe_busy) afe_start = 1'b1;
        if (afe_done) begin
          tx_ack = 1'b1;
          st_n   = S_IDLE_P;
        end
      end

      S_ERROR: begin
        tx_nack     = 1'b1;
        status_code = 8'hEE;
        if (disconnect) st_n = S_RESET;
        else            st_n = S_IDLE_P;
      end

      default: st_n = S_RESET;
    endcase
  end

  // state and  timeout counters
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      st  <= S_RESET;
      tmo <= '0;
    end else begin
      st <= st_n;

      // reset timer on state change
      if (st != st_n) begin
        tmo <= 32'd0;
      end else begin
        // only count while waiting to pair
        if (st == S_WAIT_PAIR && tmo != TMO_PAIR_S)
          tmo <= tmo + 32'd1;
      end
    end
  end

endmodule
