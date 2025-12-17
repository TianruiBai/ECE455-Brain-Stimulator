/**
 * @brief Implements an analog front-end PID amplitude controller.
 *
 * RE-EVALUATION:
 * 1. This module is refactored to work with an NCO that accepts an
 * amplitude control input (i_amp_val).
 * 2. The "Fast-Path" (per-sample scaling) has been REMOVED.
 * 3. The "Slow-Loop" (MAV PID) is PRESERVED.
 * 4. A new "Amplitude Output Logic" section combines the feed-forward
 * gain (from current_val) with the PID output (pid_out) to
 * generate the 'o_nco_amp_val' for the nco_generator.
 *
 * REFACTOR:
 * 5. The internal MAV/Peak detector logic has been *replaced* by the
 * 'SinePeakCurrentCalc' module. The PID loop now compares the
 * 'current_val' setpoint directly against the 'measured_current'
 * from the new module.
 */
 `timescale 1ns / 1ps
module analog_front_end (
    input  logic             clk,           // System clock
    input  logic             rst_n,         // System reset, active low

    // Control Inputs
    input  logic [15:0]      current_val,   // Target amplitude (0-65535 maps to 0-5mA)
    input  logic [15:0]      sample_dat,    // Sampled voltage from ADC (PV)

    // Control Output
    output logic [15:0]      o_nco_amp_val, // PID-controlled amplitude for NCO (0-65535)

    // Upstream Handshake (Input from ADC)
    output logic             req_o,         // Handshake: This module is ready for data
    input  logic             ack_i          // Handshake: Upstream (ADC) has valid data
);

    // --- Constants ---
    
    // NCO Amplitude (Feed-Forward) Constants
    // This value maps the 0-65535 current_val to the NCO's 0-65535 amplitude
    // (Note: The user-provided code sets this to 65535, implying a 1:1 FF map)
    localparam C_NCO_AMP_SCALE    = 16'd52429;

    // //////////////////////////////////////////////////////////////////////////
    // /// SLOW-LOOP: PEAK CURRENT CONTROL (Runs per ADC sample)              ///
    // //////////////////////////////////////////////////////////////////////////

    logic signed [16:0] pid_error;
    logic signed [15:0] pid_out;         // PID correction output
    logic [15:0]        measured_current;  // NEW: Output from SinePeakCurrentCalc

    // 1. Instantiate the Sine Peak to Current Calculator
    //    This module takes the raw, offset ADC waveform and calculates
    //    the corresponding current based on its detected peak.
    //    The output 'measured_current' is stable between peaks.
    peak_proc u_peak_calc (
        .clk        (clk),
        .rst_n      (rst_n),
        .sample_dat (sample_dat),
        .fb_current (measured_current) // This is the Process Variable (PV)
    );

    // 2. Register current_val to align with PID loop
    //    This register holds the Setpoint (SP).
    logic [15:0] current_val_reg;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_val_reg <= '0;
        end else if (ack_i) begin
            current_val_reg <= current_val;
        end
    end
    
    // 3. Calculate PID Error
    //    Compare the Setpoint (current_val_reg) with the 
    //    Process Variable (measured_current).
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pid_error <= '0;
        end else begin
            if (ack_i) begin // Only update PID loop on new ADC sample
                // Compare CURRENT setpoint with CURRENT measurement
                pid_error <= $signed({1'b0, current_val_reg}) - $signed({1'b0, measured_current});
            end
        end
    end

    // 4. Instantiate PID Controller
    localparam P_coeff = 3;
    localparam I_coeff = 1; // I=1 is required to fix steady-state error
    localparam D_coeff = 58;

    PID #(
        .P_COEFF(P_coeff),
        .I_COEFF(I_coeff),
        .D_COEFF(D_coeff)
    ) PID_inst (
        .clk    (clk),
        .rst_n  (rst_n),
        //.enable (ack_i), // PID only updates when ADC data is valid
        .error  (pid_error),
        .val_out(pid_out)
    );


    // //////////////////////////////////////////////////////////////////////////
    // /// HANDSHAKE LOGIC (Simplified)                                     ///
    // //////////////////////////////////////////////////////////////////////////

    // This module is always ready for the next ADC sample
    assign req_o = 1'b1;


    // //////////////////////////////////////////////////////////////////////////
    // /// AMPLITUDE OUTPUT LOGIC (Replaces Fast-Path)                      ///
    // //////////////////////////////////////////////////////////////////////////

    logic [15:0]      ff_gain;
    logic signed [16:0] final_nco_amp;
    logic [15:0]      final_nco_amp_clipped;
    logic [31:0]      mult_tmp_ff;
    
    // 1. Calculate Feed-Forward (FF) amplitude
    //    This is the base amplitude based on the user's setpoint
    //    (Note: Using the 1:1 mapping from the provided source)
    assign mult_tmp_ff = current_val_reg * C_NCO_AMP_SCALE;
    assign ff_gain = mult_tmp_ff[31:16];

    // 2. Apply PID trim (correction) to FF gain
    //    This adds the slow-loop correction to the feed-forward value
    assign final_nco_amp = $signed({1'b0, ff_gain}) + $signed(pid_out);

    // 3. Saturate the final amplitude for the NCO (0-65535)
    assign final_nco_amp_clipped = (final_nco_amp < 0) ? 16'd0 :
                                   (final_nco_amp > 65535) ? 16'd65535 :
                                   final_nco_amp[15:0];
                                     
    // 4. Assign to final NCO amplitude output
    assign o_nco_amp_val = final_nco_amp_clipped;

endmodule