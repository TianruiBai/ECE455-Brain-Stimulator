// I2C master for IP2363-style register reads/writes.
// - Single transaction at a time
// - Write:  [ADDR+W] [reg_addr] [data_byte]
// - Read8:  [ADDR+W] [reg_addr] RESTART [ADDR+R] [data]
// - Read16: same, but read 2 bytes (low, then high)
// Assumes 7-bit slave address = 0x75 (IP2363), standard mode (~100 kHz).

module I2C #(
  parameter int  F_CLK_HZ   = 50_000_000,  // system clock
  parameter int  I2C_HZ     = 100_000,     // target SCL frequency
  parameter logic [6:0] SLV_ADDR = 7'h75  // IP2363 7-bit address
)(
  input  wire        clk,
  input  wire        rst_n,      // async active-low

  // Command interface
  input  wire        start,      // pulse 1 clk to launch a transaction
  input  wire        rd_nwr,     // 0 = write, 1 = read
  input  wire [7:0]  reg_addr,   // IP2363 register address
  input  wire [15:0] wr_data,    // data to write (low 8 bits used)
  input  wire        rd_16bit,   // for reads: 0=8-bit, 1=16-bit

  output logic [15:0] rd_data,   // read-back data (8 or 16 bits)
  output logic        busy,      // high while transaction in progress
  output logic        done,      // 1-cycle pulse at end of transaction
  output logic        err,       // 1-cycle pulse if NACK or error

  // I2C lines
  output wire         SCL,
  inout  wire         SDA
);

  // ------------------------------------------------------------
  // Open-drain SDA
  // ------------------------------------------------------------
  logic sda_oe;      // 1 = drive SDA low, 0 = release
  wire  sda_in;
  assign SDA    = sda_oe ? 1'b0 : 1'bz;
  assign sda_in = SDA;

  // SCL is always driven (no clock stretching handled here)
  logic scl_o;
  assign SCL = scl_o;

  // ------------------------------------------------------------
  // Clock divider for SCL phases (4 phases per SCL period)
  // ------------------------------------------------------------
  localparam int DIV = (F_CLK_HZ / (I2C_HZ * 4)); // ticks per quarter cycle
  logic [$clog2(DIV):0] div_cnt;
  logic [1:0]           phase;    // 0: low, 1: rising, 2: high, 3: falling

  wire tick = (div_cnt == DIV-1);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      div_cnt <= '0;
      phase   <= 2'd0;
    end else begin
      if (busy) begin
        if (tick) begin
          div_cnt <= '0;
          phase   <= phase + 2'd1;
        end else begin
          div_cnt <= div_cnt + 1'b1;
        end
      end else begin
        div_cnt <= '0;
        phase   <= 2'd0;
      end
    end
  end

  // SCL generation from phase
  always_comb begin
    case (phase)
      2'd0, 2'd3: scl_o = 1'b0; // low
      default:    scl_o = 1'b1; // high during phases 1,2
    endcase
  end

  // Helpers
  wire phase_low    = (phase == 2'd0);
  wire phase_high   = (phase == 2'd2);
  wire bit_sample   = phase_high & tick; // sample SDA when clock high
  wire bit_update   = phase_low  & tick; // change SDA when clock low

  // ------------------------------------------------------------
  // FSM & bit counters
  // ------------------------------------------------------------
  typedef enum logic [4:0] {
    ST_IDLE,

    ST_START,
    ST_ADDR_W,     // send SLV_ADDR + W
    ST_ADDR_W_ACK,

    ST_REG,        // send register address
    ST_REG_ACK,

    ST_DATA_W,     // send data byte (write)
    ST_DATA_W_ACK,

    ST_REP_START,  // repeated start for read
    ST_ADDR_R,     // send SLV_ADDR + R
    ST_ADDR_R_ACK,

    ST_READ_BYTE,  // read one byte (8 bits)
    ST_READ_ACK,   // send ACK/NACK for read byte

    ST_STOP,
    ST_DONE,
    ST_ERROR
  } state_t;

  state_t state;

  logic [3:0]  bit_cnt;         // bit index 7..0
  logic [7:0]  tx_byte;
  logic [7:0]  rx_byte;
  logic [15:0] rd_data_r;
  logic        second_byte;     // for 16-bit read
  logic        is_read;         // latched rd_nwr
  logic [7:0]  reg_addr_r;
  logic [7:0]  wr_data_lsb;

  // Edge detect for start
  logic start_d;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) start_d <= 1'b0;
    else        start_d <= start;
  end
  wire start_pulse = start & ~start_d;

  // ------------------------------------------------------------
  // Main FSM and datapath
  // ------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state        <= ST_IDLE;
      bit_cnt      <= 4'd0;
      sda_oe       <= 1'b0;
      tx_byte      <= 8'h00;
      rx_byte      <= 8'h00;
      rd_data_r    <= 16'h0000;
      second_byte  <= 1'b0;
      is_read      <= 1'b0;
      reg_addr_r   <= 8'h00;
      wr_data_lsb  <= 8'h00;
      busy         <= 1'b0;
      done         <= 1'b0;
      err          <= 1'b0;
      rd_data      <= 16'h0000;
    end else begin
      done <= 1'b0;
      err  <= 1'b0;

      case (state)
        // ----------------------------------------------------
        ST_IDLE: begin
          sda_oe <= 1'b0;       // SDA released
          busy   <= 1'b0;
          if (start_pulse) begin
            // Latch command
            is_read     <= rd_nwr;
            reg_addr_r  <= reg_addr;
            wr_data_lsb <= wr_data[7:0];
            second_byte <= 1'b0;
            busy        <= 1'b1;
            // START condition: SDA low while SCL high
            state       <= ST_START;
          end
        end

        // ----------------------------------------------------
        // START condition: drive SDA low while SCL low, then go to first byte
        ST_START: begin
          if (bit_update) begin
            sda_oe  <= 1'b1;   // pull SDA low
            bit_cnt <= 4'd7;
            // First byte = address + W
            tx_byte <= {SLV_ADDR, 1'b0};
            state   <= ST_ADDR_W;
          end
        end

        // ----------------------------------------------------
        // Send address+W byte
        ST_ADDR_W: begin
          if (bit_update) begin
            // output MSB first
            sda_oe <= ~tx_byte[bit_cnt]; // 0 -> drive low, 1 -> release
          end
          if (bit_sample) begin
            if (bit_cnt == 4'd0) begin
              // finished byte, now release SDA for ACK
              sda_oe <= 1'b0; // release for ACK bit
              state  <= ST_ADDR_W_ACK;
            end else begin
              bit_cnt <= bit_cnt - 4'd1;
            end
          end
        end

        // ----------------------------------------------------
        // Sample ACK for addr+W
        ST_ADDR_W_ACK: begin
          if (bit_sample) begin
            if (sda_in) begin
              // NACK
              err   <= 1'b1;
              state <= ST_ERROR;
            end else begin
              // ACK ok -> send reg address
              bit_cnt <= 4'd7;
              tx_byte <= reg_addr_r;
              state   <= ST_REG;
            end
          end
        end

        // ----------------------------------------------------
        // Send register address
        ST_REG: begin
          if (bit_update) begin
            sda_oe <= ~tx_byte[bit_cnt];
          end
          if (bit_sample) begin
            if (bit_cnt == 4'd0) begin
              sda_oe <= 1'b0; // release for ACK
              state  <= ST_REG_ACK;
            end else begin
              bit_cnt <= bit_cnt - 4'd1;
            end
          end
        end

        // ----------------------------------------------------
        // Sample ACK for register address
        ST_REG_ACK: begin
          if (bit_sample) begin
            if (sda_in) begin
              err   <= 1'b1;
              state <= ST_ERROR;
            end else begin
              if (is_read) begin
                // Repeated START for read
                state <= ST_REP_START;
              end else begin
                // Write: send data byte
                bit_cnt <= 4'd7;
                tx_byte <= wr_data_lsb;
                state   <= ST_DATA_W;
              end
            end
          end
        end

        // ----------------------------------------------------
        // Write data byte
        ST_DATA_W: begin
          if (bit_update) begin
            sda_oe <= ~tx_byte[bit_cnt];
          end
          if (bit_sample) begin
            if (bit_cnt == 4'd0) begin
              sda_oe <= 1'b0; // release for ACK
              state  <= ST_DATA_W_ACK;
            end else begin
              bit_cnt <= bit_cnt - 4'd1;
            end
          end
        end

        // ----------------------------------------------------
        // Sample ACK for data byte
        ST_DATA_W_ACK: begin
          if (bit_sample) begin
            if (sda_in) begin
              err   <= 1'b1;
              state <= ST_ERROR;
            end else begin
              // Successful write: go to STOP
              state <= ST_STOP;
            end
          end
        end

        // ----------------------------------------------------
        // Repeated START (for read)
        ST_REP_START: begin
          if (bit_update) begin
            // Ensure SDA low while SCL low -> repeated start sequence
            sda_oe  <= 1'b1;
            bit_cnt <= 4'd7;
            tx_byte <= {SLV_ADDR, 1'b1}; // address + R
            state   <= ST_ADDR_R;
          end
        end

        // ----------------------------------------------------
        // Send address+R
        ST_ADDR_R: begin
          if (bit_update) begin
            sda_oe <= ~tx_byte[bit_cnt];
          end
          if (bit_sample) begin
            if (bit_cnt == 4'd0) begin
              sda_oe <= 1'b0; // release for ACK
              state  <= ST_ADDR_R_ACK;
            end else begin
              bit_cnt <= bit_cnt - 4'd1;
            end
          end
        end

        // ----------------------------------------------------
        // Sample ACK for addr+R
        ST_ADDR_R_ACK: begin
          if (bit_sample) begin
            if (sda_in) begin
              err   <= 1'b1;
              state <= ST_ERROR;
            end else begin
              // Start reading first byte
              bit_cnt <= 4'd7;
              rx_byte <= 8'h00;
              state   <= ST_READ_BYTE;
            end
          end
        end

        // ----------------------------------------------------
        // Read 8 bits (one byte)
        ST_READ_BYTE: begin
          // make sure SDA is released while reading
          if (bit_update) begin
            sda_oe <= 1'b0;
          end
          if (bit_sample) begin
            rx_byte[bit_cnt] <= sda_in;
            if (bit_cnt == 4'd0) begin
              // Finished byte, decide ACK or NACK
              if (rd_16bit && !second_byte) begin
                // store low byte
                rd_data_r[7:0] <= rx_byte;
                second_byte    <= 1'b1;
                state          <= ST_READ_ACK;
              end else begin
                // last byte: high byte or single byte
                if (rd_16bit && second_byte)
                  rd_data_r[15:8] <= rx_byte;
                else
                  rd_data_r[7:0] <= rx_byte;

                state <= ST_READ_ACK;
              end
            end else begin
              bit_cnt <= bit_cnt - 4'd1;
            end
          end
        end

        // ----------------------------------------------------
        // Send ACK (for more bytes) or NACK (for last byte)
        ST_READ_ACK: begin
          if (bit_update) begin
            if (rd_16bit && !second_byte) begin
              // We just read low byte; ACK it (drive SDA low)
              sda_oe <= 1'b1;
            end else begin
              // Last byte -> NACK (release SDA high)
              sda_oe <= 1'b0;
            end
          end
          if (bit_sample) begin
            // After ACK/NACK bit, either read another byte or STOP
            if (rd_16bit && !second_byte) begin
              // Prepare for second byte
              bit_cnt <= 4'd7;
              rx_byte <= 8'h00;
              state   <= ST_READ_BYTE;
            end else begin
              // All done -> STOP
              state <= ST_STOP;
            end
          end
        end

        // ----------------------------------------------------
        // STOP condition: SDA high while SCL high
        ST_STOP: begin
          if (bit_update) begin
            sda_oe <= 1'b0; // release SDA
            state  <= ST_DONE;
          end
        end

        // ----------------------------------------------------
        ST_DONE: begin
          busy    <= 1'b0;
          done    <= 1'b1;
          rd_data <= rd_data_r;
          sda_oe  <= 1'b0;
          state   <= ST_IDLE;
        end

        // ----------------------------------------------------
        ST_ERROR: begin
          busy   <= 1'b0;
          done   <= 1'b1;
          err    <= 1'b1;
          sda_oe <= 1'b0;
          state  <= ST_IDLE;
        end

        default: state <= ST_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
