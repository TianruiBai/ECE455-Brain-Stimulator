module ip2363_status #(
  parameter int F_CLK_HZ   = 50_000_000,
  parameter int I2C_HZ     = 100_000,
  parameter [6:0] SLV_ADDR = 7'h75
)(
  input  wire       clk,
  input  wire       rst_n,

  input  wire       sample_req,
  output logic       status_ready,
  output logic       error,

  inout  wire        i2c_sda,
  output wire        i2c_scl,

  output logic [15:0] vbat_mV,
  output logic [15:0] vsys_mV,
  output logic [15:0] ibat_mA,
  output logic [15:0] isys_mA,
  output logic [15:0] psys_10mW,
  output logic [7:0]  state0
);

  // IP2363 register addresses (LSB address used for 16-bit reads)
  localparam logic [7:0] REG_STATE0 = 8'h31;

  localparam logic [7:0] REG_VBAT_L = 8'h50; // VBAT LSB, then MSB auto-increment
  localparam logic [7:0] REG_VSYS_L = 8'h52;
  localparam logic [7:0] REG_IBAT_L = 8'h6E;
  localparam logic [7:0] REG_ISYS_L = 8'h70;
  localparam logic [7:0] REG_PSYS_L = 8'h74;


  logic        i2c_start;
  logic        i2c_rd_nwr;
  logic [7:0]  i2c_reg_addr;
  logic [15:0] i2c_wr_data;
  logic        i2c_rd_16bit;

  logic [15:0] i2c_rd_data;
  logic        i2c_busy, i2c_done, i2c_err;

  // default write data unused (for now)
  assign i2c_wr_data = 16'h0000;

  I2C #(
    .F_CLK_HZ  (F_CLK_HZ),
    .I2C_HZ    (I2C_HZ),
    .SLV_ADDR  (SLV_ADDR)
  ) i2c0 (
    .clk      (clk),
    .rst_n    (rst_n),
    .start    (i2c_start),
    .rd_nwr   (i2c_rd_nwr),
    .reg_addr (i2c_reg_addr),
    .wr_data  (i2c_wr_data),
    .rd_16bit (i2c_rd_16bit),
    .rd_data  (i2c_rd_data),
    .busy     (i2c_busy),
    .done     (i2c_done),
    .err      (i2c_err),
    .SCL      (i2c_scl),
    .SDA      (i2c_sda)
  );

  // Edge detect sample_req
  logic sample_req_d;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) sample_req_d <= 1'b0;
    else        sample_req_d <= sample_req;
  end

  wire sample_req_pulse = sample_req & ~sample_req_d;

  // sequence through all needed registers
  typedef enum logic [3:0] {
    ST_IDLE,
    ST_RD_STATE0_START,
    ST_RD_STATE0_WAIT,

    ST_RD_VBAT_START,
    ST_RD_VBAT_WAIT,

    ST_RD_VSYS_START,
    ST_RD_VSYS_WAIT,

    ST_RD_IBAT_START,
    ST_RD_IBAT_WAIT,

    ST_RD_ISYS_START,
    ST_RD_ISYS_WAIT,

    ST_RD_PSYS_START,
    ST_RD_PSYS_WAIT,

    ST_DONE,
    ST_ERROR
  } state_t;

  state_t state;

  // Main control
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state        <= ST_IDLE;
      i2c_start    <= 1'b0;
      i2c_rd_nwr   <= 1'b1;
      i2c_reg_addr <= 8'h00;
      i2c_rd_16bit <= 1'b0;

      vbat_mV      <= 16'h0000;
      vsys_mV      <= 16'h0000;
      ibat_mA      <= 16'h0000;
      isys_mA      <= 16'h0000;
      psys_10mW    <= 16'h0000;
      state0       <= 8'h00;

      status_ready <= 1'b0;
      error        <= 1'b0;
    end else begin
      // Defaults each cycle
      i2c_start    <= 1'b0;
      status_ready <= 1'b0;
      error        <= 1'b0;

      case (state)
        ST_IDLE: begin
          if (sample_req_pulse) begin
            // Start with STATE0 (8-bit)
            if (!i2c_busy) begin
              i2c_rd_nwr   <= 1'b1;         // read
              i2c_reg_addr <= REG_STATE0;
              i2c_rd_16bit <= 1'b0;         // 8-bit
              i2c_start    <= 1'b1;
              state        <= ST_RD_STATE0_START;
            end
          end
        end
        // STATE0 (8-bit)
        ST_RD_STATE0_START: begin
          // Just wait for I2C to run; nothing else to do here
          if (i2c_done) begin
            if (i2c_err) begin
              error <= 1'b1;
              state <= ST_ERROR;
            end else begin
              state0 <= i2c_rd_data[7:0];
              state  <= ST_RD_VBAT_START;
            end
          end
        end

        // VBAT (16-bit: mV)
        ST_RD_VBAT_START: begin
          if (!i2c_busy) begin
            i2c_rd_nwr   <= 1'b1;
            i2c_reg_addr <= REG_VBAT_L;
            i2c_rd_16bit <= 1'b1;      // 16-bit read
            i2c_start    <= 1'b1;
            state        <= ST_RD_VBAT_WAIT;
          end
        end

        ST_RD_VBAT_WAIT: begin
          if (i2c_done) begin
            if (i2c_err) begin
              error <= 1'b1;
              state <= ST_ERROR;
            end else begin
              vbat_mV <= i2c_rd_data;
              state   <= ST_RD_VSYS_START;
            end
          end
        end

        // VSYS (16-bit: mV)
        ST_RD_VSYS_START: begin
          if (!i2c_busy) begin
            i2c_rd_nwr   <= 1'b1;
            i2c_reg_addr <= REG_VSYS_L;
            i2c_rd_16bit <= 1'b1;
            i2c_start    <= 1'b1;
            state        <= ST_RD_VSYS_WAIT;
          end
        end

        ST_RD_VSYS_WAIT: begin
          if (i2c_done) begin
            if (i2c_err) begin
              error <= 1'b1;
              state <= ST_ERROR;
            end else begin
              vsys_mV <= i2c_rd_data;
              state   <= ST_RD_IBAT_START;
            end
          end
        end

        // IBAT (16-bit: mA)
        ST_RD_IBAT_START: begin
          if (!i2c_busy) begin
            i2c_rd_nwr   <= 1'b1;
            i2c_reg_addr <= REG_IBAT_L;
            i2c_rd_16bit <= 1'b1;
            i2c_start    <= 1'b1;
            state        <= ST_RD_IBAT_WAIT;
          end
        end

        ST_RD_IBAT_WAIT: begin
          if (i2c_done) begin
            if (i2c_err) begin
              error <= 1'b1;
              state <= ST_ERROR;
            end else begin
              ibat_mA <= i2c_rd_data;
              state   <= ST_RD_ISYS_START;
            end
          end
        end

        // ISYS (16-bit: mA)
        ST_RD_ISYS_START: begin
          if (!i2c_busy) begin
            i2c_rd_nwr   <= 1'b1;
            i2c_reg_addr <= REG_ISYS_L;
            i2c_rd_16bit <= 1'b1;
            i2c_start    <= 1'b1;
            state        <= ST_RD_ISYS_WAIT;
          end
        end

        ST_RD_ISYS_WAIT: begin
          if (i2c_done) begin
            if (i2c_err) begin
              error <= 1'b1;
              state <= ST_ERROR;
            end else begin
              isys_mA <= i2c_rd_data;
              state   <= ST_RD_PSYS_START;
            end
          end
        end
        // PSYS (16-bit: units of 10 mW)
        ST_RD_PSYS_START: begin
          if (!i2c_busy) begin
            i2c_rd_nwr   <= 1'b1;
            i2c_reg_addr <= REG_PSYS_L;
            i2c_rd_16bit <= 1'b1;
            i2c_start    <= 1'b1;
            state        <= ST_RD_PSYS_WAIT;
          end
        end

        ST_RD_PSYS_WAIT: begin
          if (i2c_done) begin
            if (i2c_err) begin
              error <= 1'b1;
              state <= ST_ERROR;
            end else begin
              psys_10mW <= i2c_rd_data;
              state     <= ST_DONE;
            end
          end
        end

        ST_DONE: begin
          status_ready <= 1'b1;   // one-cycle pulse
          state        <= ST_IDLE;
        end

        ST_ERROR: begin
          // Just signal error, then return to IDLE.
          status_ready <= 1'b1;
          state        <= ST_IDLE;
        end

        default: state <= ST_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
