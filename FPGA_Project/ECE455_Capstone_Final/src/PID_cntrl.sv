module PID #(
    parameter int P_COEFF = 1, // in the range 1 to 8
    parameter int I_COEFF = 1, // in the range 0 to 2
    parameter int D_COEFF = 0  // in the range 0 to 64
)(
    input  logic              clk,
    input  logic              rst_n,
    input  logic signed [16:0] error,
    output logic        [15:0] val_out
);

    // ============================================================
    // STAGE 1: State Updates & Pre-computation
    // ============================================================
    
    // --- Integrator Logic (Accumulator) ---
    logic signed [23:0] integrator;
    logic signed [23:0] nxt_integrator;
    
    // Sign extend error for integrator addition
    assign nxt_integrator = integrator + {{7{error[16]}}, error};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integrator <= 24'h000000;
        end else begin
            // Original logic: reset if negative, else accumulate
            integrator <= (nxt_integrator[23]) ? 24'h000000 : nxt_integrator;
        end
    end

    // --- History Buffer (Derivative tracking) ---
    logic signed [16:0] prev_error1, prev_error2, prev_error3, prev_error4;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_error1 <= '0;
            prev_error2 <= '0;
            prev_error3 <= '0;
            prev_error4 <= '0;
        end else begin
            prev_error1 <= error;
            prev_error2 <= prev_error1;
            prev_error3 <= prev_error2;
            prev_error4 <= prev_error3;
        end
    end

    // --- Pipeline Registers for Stage 1 -> Stage 2 ---
    // We must register the data needed for the multiplications in the next stage
    logic signed [16:0] st1_error;
    logic signed [17:0] st1_error_diff; // (error - prev_error4)
    logic signed [11:0] st1_integrator_slice; 

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st1_error            <= '0;
            st1_error_diff       <= '0;
            st1_integrator_slice <= '0;
        end else begin
            st1_error            <= error;
            // Pre-calculate D-term difference here to save time in multiplier stage
            st1_error_diff       <= $signed(error) - $signed(prev_error4);
            // Grab the relevant bits for I-term
            st1_integrator_slice <= integrator[22:11];
        end
    end

    // ============================================================
    // STAGE 2: Multiplication (The Heavy Lifting)
    // ============================================================
    
    logic signed [19:0] st2_P_term;
    logic signed [23:0] st2_D_term;
    logic signed [13:0] st2_I_term;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st2_P_term <= '0;
            st2_D_term <= '0;
            st2_I_term <= '0;
        end else begin
            // Perform Multiplications based on Stage 1 data
            /* verilator lint_off WIDTHTRUNC */
            st2_P_term <= $signed(P_COEFF) * st1_error;
            st2_I_term <= $signed(I_COEFF) * st1_integrator_slice;
            st2_D_term <= $signed(D_COEFF) * st1_error_diff;
            /* verilator lint_on WIDTHTRUNC */
        end
    end

    // ============================================================
    // STAGE 3: Summation & Saturation
    // ============================================================

    logic signed [23:0] PID_sum;
    
    // Combinational sum logic (intermediate)
    // Note: P_term shifted right by 2 as per original spec
    /* verilator lint_off WIDTHEXPAND */
    wire signed [23:0] next_PID_sum = (st2_P_term >>> 2) + st2_I_term + st2_D_term;
    /* verilator lint_on WIDTHEXPAND */

    // Final Output Register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            val_out <= 16'h0000;
        end else begin
            // Saturate logic based on the calculated sum
            if (next_PID_sum[23]) begin
                 // Negative -> 0
                val_out <= 16'h0000;
            end else if (|next_PID_sum[23:16]) begin
                 // Overflow positive -> Max
                val_out <= 16'hFFFF;
            end else begin
                 // Valid range
                val_out <= next_PID_sum[15:0];
            end
        end
    end

endmodule