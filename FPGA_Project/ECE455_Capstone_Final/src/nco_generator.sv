/**
 * @brief NCO Generator Wrapper
 *
 * This module wraps the pipelined 'nco' phase-to-amplitude converter
 * and adds a phase accumulator to create a full signal generator.
 *
 * It takes a frequency word and phase offset, and generates a
 * continuous sine/cosine output wave.
 */
module nco_generator (
    input  logic        clk,
    input  logic        reset_n,
    
    // --- Control Inputs ---
    input  logic [31:0] i_freq_word,    // Controls output frequency
    input  logic [31:0] i_phase_offset, // Controls output phase shift DO NOT USE
    input  logic [15:0] i_amp_val,      // Controls output amplitude
    
    // --- Data Output Port (Sine/Cosine) ---
    output logic [31:0] o_nco_dat,      // Output data: [31:16]=Sine, [15:0]=Cosine
    output logic        o_nco_dat_req,  // Output data is valid
    input  logic        i_nco_dat_ack   // Downstream module is ready for data
);

    // --- Internal Handshake Signals ---
    // Connect to the NCO's input (target) port
    logic [31:0] nco_phase_in;
    logic        nco_phase_req;
    logic        nco_phase_ack;
    
    // Connect to the NCO's output (initiator) port
    logic [31:0] nco_data_out;
    logic        nco_data_req;
    logic        nco_data_ack;

    // --- Phase Accumulator ---
    reg [31:0] phase_acc_r;
    
    // The 'nco' module's input buffer is ready when req & ack are both high.
    // This is our signal to update the phase accumulator.
    wire update_phase_acc = (nco_phase_req & nco_phase_ack);
    
    always @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            phase_acc_r <= 32'd0;
        end else if (update_phase_acc) begin
            // Increment the phase accumulator on every cycle the NCO
            // is ready to accept a new phase value.
            phase_acc_r <= phase_acc_r + i_freq_word;
        end
    end
    
    // The final phase sent to the NCO is the accumulated value + static offset
    assign nco_phase_in = phase_acc_r + i_phase_offset;

    // --- Input Handshake Logic (Wrapper-to-NCO) ---
    
    // We are always ready to send a new phase value to the NCO.
    // The NCO will signal 'nco_phase_ack' when it's ready to accept it.
    assign nco_phase_req = 1'b1; 

    // --- Output Handshake Logic (NCO-to-Downstream) ---
    
    // Simply pass the NCO's output handshake signals directly to the wrapper's output.
    assign o_nco_dat     = nco_data_out;
    assign o_nco_dat_req = nco_data_req;
    assign nco_data_ack  = i_nco_dat_ack;

    // --- NCO Module Instantiation ---
    
    cordic_dds_top u_nco (
        .clk(clk),
        .reset_n(reset_n),
        
        // Connect to NCO's input port
        .t_angle_dat(nco_phase_in),
        .t_angle_req(nco_phase_req),
        .i_amp_val(i_amp_val),
        .t_angle_ack(nco_phase_ack),
        
        // Connect to NCO's output port
        .i_nco_dat(nco_data_out),
        .i_nco_req(nco_data_req),
        .i_nco_ack(nco_data_ack)
    );
        
endmodule