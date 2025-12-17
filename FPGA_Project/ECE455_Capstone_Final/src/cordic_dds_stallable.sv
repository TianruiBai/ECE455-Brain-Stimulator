/**
 * @brief CORDIC DDS Stallable Core
 * (MODIFIED for S1.15 Q-format to fix peak overflow and add x0 saturation)
 */
module cordic_dds_stallable # (
   parameter DW = 16,               /* Data width */
   parameter PIPE_DEPTH = 16       /* Pipeline depth (stages 0 to 13) */
)
(
   input             clk,
   input             pipe_en,       // Global enable for all pipeline stages
   input  [DW-1:0]     i_amp_val,     // UQ.16 Amplitude Value
   input [DW-1:0]    phase_i,       /* Phase (top 16 bits) */
   output [DW:0]     sin_o, cos_o,  /* Function value output (S1.15) */
   output [DW:0]     err_o         /* Phase Error output (S1.15) */
);

   // --- NEW: S1.15 (1.15) Q-format Constants ---
   // 1/K for 14 stages (i=0..13) in S1.30 format
   localparam [31:0] limP_32b_s1_30 = 32'h4db79155; // 0.60725 * 2^30
   
   // S1.15 min/max values (17-bit)
   localparam signed [DW:0] S1_15_MAX = 17'h0_FFFF; // +1.9999...
   localparam signed [DW:0] S1_15_MIN = 17'h1_0000; // -2.0

   // --- Registers and Wires (all are S1.15) ---
   reg [DW:0] cos_r=0, sin_o_r=0;
   reg [DW:0] x[PIPE_DEPTH:0];
   reg [DW:0] y[PIPE_DEPTH:0];
   reg [DW:0] z[PIPE_DEPTH:0];
   reg [DW:0] atan_rom[PIPE_DEPTH:0];
   reg [1:0] quadrant [PIPE_DEPTH:0];
   integer i;

   // --- High-Precision Amplitude Scaling Logic ---
   logic signed [DW:0]   x0_input;
   logic signed [DW:0]   x0_input_raw;
   logic signed [DW+31:0]   x0_full_product;

   // x0 = i_amp_val * limP
   // (UQ.16) * (S1.30) = S1.46
   assign x0_full_product = (i_amp_val * limP_32b_s1_30)<<1;
   
   // Convert S1.46 product to S1.15 (17-bit) by shifting right 31 bits
   // (A*L / 2^46) -> (A*L / 2^46) * 2^15 = A*L / 2^31
   /* verilator lint_off WIDTHEXPAND */
   assign x0_input_raw = x0_full_product[DW+31:32]; 
   /* verilator lint_on WIDTHEXPAND */

   // --- ATAN ROM (S1.15 Format) ---
   // Angle is scaled by PI. Range is [-pi, +pi).
   // Our atan_rom values are [0, pi/4), so they fit in S1.15.
   // This always_comb block *is* the ROM
   initial begin
      atan_rom[0] = 8189;
      atan_rom[1] = 4834;
      atan_rom[2] = 2554;
      atan_rom[3] = 1296;
      atan_rom[4] = 650;
      atan_rom[5] = 325;
      atan_rom[6] = 162;
      atan_rom[7] = 81;
      atan_rom[8] = 40;
      atan_rom[9] = 20;
      atan_rom[10] = 10;
      atan_rom[11] = 5;
      atan_rom[12] = 2;
      atan_rom[13] = 1;
      atan_rom[14] = 1;
      atan_rom[15] = 0;
   end


   // ================= //
   // Pipeline stages   //
   // ================= //
   
   // --- Stage 0 (MODIFIED for S1.15) ---
   always @ (posedge clk) begin // stage 0
      if (pipe_en) begin
         x[0] <= x0_input_raw;//{1'b0,limP_32b_s1_30[31:16]}; // Use saturated, scaled amplitude
         y[0] <= 0;
         
         // --- ANGLE FIX (S1.15) ---
         // phase_i[13:0] is N (14-bit angle for [0, pi/2)).
         // z_S1.15 = ( (N / 2^14) * (pi/2) / pi ) * 2^15
         // z_S1.15 = ( N / 2^15 ) * 2^15 = N
         // So, we just zero-extend N to 17 bits.
         z[0] <= {3'b0, phase_i[DW-3:0]};
      end
   end

   // --- Stage 1 (Unchanged) ---
   // Note: The shifts (>>>) are arithmetic and work correctly
   // on the signed S1.15 values.
   always @ (posedge clk) begin // stage 1
      if (pipe_en) begin // Add this 'if'
         x[1] <= x[0] - y[0];
         y[1] <= x[0] + y[0];
         z[1] <= z[0] - atan_rom[0];
      end
   end

   // --- Stages 2 to PIPE_DEPTH (Unchanged) ---
   generate
      genvar k;
      for(k=1; k<PIPE_DEPTH; k=k+1) begin
         // --- MODIFICATION ---
         always @ (posedge clk) begin
            if (pipe_en) begin // Add this 'if'
               if (z[k][DW]) begin 
                  x[k+1] <= x[k] + {{k{y[k][DW]}},y[k][DW:k]};
                  y[k+1] <= y[k] - {{k{x[k][DW]}},x[k][DW:k]};
                  z[k+1] <= z[k] + atan_rom[k];
               end else begin
                  x[k+1] <= x[k] - {{k{y[k][DW]}},y[k][DW:k]};
                  y[k+1] <= y[k] + {{k{x[k][DW]}},x[k][DW:k]};
                  z[k+1] <= z[k] - atan_rom[k];
               end
            end
         end
      end
   endgenerate

   // ================= //
   // Count quadrant    //
   // ================= //
   
   always @ (posedge clk) begin
      if (pipe_en) begin
         quadrant[0] <= phase_i[DW-1:DW-2];
      end
   end
   
   generate
      genvar j;
      for(j=0; j<PIPE_DEPTH; j=j+1) begin
         always @ (posedge clk) begin
            if (pipe_en) begin
               quadrant[j+1] <= quadrant[j];
            end
         end
      end
   endgenerate

   // ================= //
   // Adjust quadrant   //
   // ================= //
   
   // (Unchanged quadrant adjustment logic)
   // This logic is now safe because S1.15 can represent
   // both +1.0 and -1.0, so the 2's complement negation
   // (~) will not overflow.
   always @ (posedge clk)
      if (pipe_en) begin
         case(quadrant[PIPE_DEPTH])
            2'b00: begin
               cos_r <= x[PIPE_DEPTH]; /* cos */
               sin_o_r <= y[PIPE_DEPTH]; /* sin */
            end
            2'b01: begin
               cos_r <= ~(y[PIPE_DEPTH]) + 1'b1; /* -sin */
               sin_o_r <= x[PIPE_DEPTH]; /* cos */
            end
            2'b10: begin
               cos_r <= ~(x[PIPE_DEPTH]) + 1'b1; /* -cos */
               sin_o_r <= ~(y[PIPE_DEPTH]) + 1'b1; /* -sin */
            end
            default: begin // 2'b11
               cos_r <= y[PIPE_DEPTH]; /* sin */
               sin_o_r <= ~(x[PIPE_DEPTH]) + 1'b1; /* -cos */
            end
         endcase
      end
   
   assign cos_o = cos_r;
   assign sin_o = sin_o_r;
   assign err_o = z[PIPE_DEPTH];

endmodule