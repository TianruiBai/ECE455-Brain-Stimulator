module rx_fifo #(
  parameter int WIDTH = 8,        // data width by bytes
  parameter int DEPTH = 32        // number of entries in the FIFO
)(
  input  wire                 clk,
  input  wire                 rst_n,

  // write side
  input  wire [WIDTH-1:0]     din,
  input  wire                 wr_en,   // write request
  output wire                 full,    // 1 when FIFO cannot accept data

  // read side
  output logic [WIDTH-1:0]    dout,    // data output
  input  wire                 rd_en,   // read request flashes for 1 cycle
  output wire                 empty   // 1 when FIFO has no data
);

  // Address width for DEPTH entries
  localparam int AW = (DEPTH <= 1) ? 1 : $clog2(DEPTH);
  // Counter width: 0..DEPTH
  localparam int CW = $clog2(DEPTH + 1);

  // Storage
  logic [WIDTH-1:0] mem [0:DEPTH-1];

  // Pointers and count
  logic [AW-1:0] widx, ridx;
  logic [CW-1:0] count;

  // Status flags
  assign empty = (count == 0);
  assign full  = (count == DEPTH);

  // Determine legal read/write this cycle
  wire do_write = wr_en && !full;
  wire do_read  = rd_en && !empty;

  // WRITE: store at widx, then advance widx (wraps at DEPTH-1)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      widx     <= '0;
    end else begin
      if (do_write) begin
        mem[widx] <= din;
        if (widx == DEPTH-1) widx <= '0;
        else                 widx <= widx + 1'b1;
      end else if (wr_en && full) begin
        // attempted write while full
      end
    end
  end

  // READ: present mem[ridx] to dout, then advance ridx (wrap)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ridx      <= '0;
      dout      <= '0;
    end else begin
      if (do_read) begin
        dout <= mem[ridx];
        if (ridx == DEPTH-1) ridx <= '0;
        else                 ridx <= ridx + 1'b1;
      end else if (rd_en && empty) begin
        // attempted read while empty
      end
    end
  end

  // COUNT: increment on write, decrement on read (both → no change)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      count <= '0;
    end else begin
      unique case ({do_write, do_read})
        2'b10: count <= count + 1'b1; // write only
        2'b01: count <= count - 1'b1; // read only
        default: /* 2'b00 or 2'b11 */ count <= count; // no change
      endcase
    end
  end

endmodule
`default_nettype wire
