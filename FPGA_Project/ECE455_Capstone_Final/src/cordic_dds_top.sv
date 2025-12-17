/**
 * @brief CORDIC DDS Adapter
 *
 * This module wraps the 'cordic_dds_stallable' module to give it an
 * elastic pipeline handshake interface identical to the 'nco.v' module.
 *
 * It manages the fixed 15-cycle latency of the stallable CORDIC core
 * and correctly handles the req/ack stall signals.
 *
 * It adapts the 32-bit phase input to 16-bits for the CORDIC
 * and expands the 17-bit CORDIC output to 32-bits.
 */
module cordic_dds_top (
    // per node (target / initiator)
    input              clk,
    input              reset_n,
    input       [31:0] t_angle_dat, // 32-bit input
    input              t_angle_req,
    input logic [15:0] i_amp_val,
    output             t_angle_ack,
    output      [31:0] i_nco_dat,   // 32-bit output
    output             i_nco_req,
    input              i_nco_ack
);

    // --- CORDIC Parameters ---
    localparam CORDIC_DW = 16;
    localparam CORDIC_PIPE_DEPTH = 16;
    localparam LATENCY = CORDIC_PIPE_DEPTH + 1; // Total 15 cycles

    // --- Internal Signals ---
    logic        pipe_en;         // Enable for the stallable CORDIC
    logic        accept_new_data; // t_angle_req & t_angle_ack
    logic [CORDIC_DW-1:0] phase_i_reg;   // Registered phase for CORDIC
    
    // --- CORDIC Outputs ---
    logic [CORDIC_DW:0] sin_o_w;
    logic [CORDIC_DW:0] cos_o_w;
    
    // --- Handshake & Latency Logic ---
    reg [LATENCY-1:0] valid_pipe_r; // Tracks the 'valid' bit through the pipeline
    reg i_nco_req_r;              // Output request register
    
    wire output_valid = valid_pipe_r[LATENCY-1]; // 'valid' bit emerging from pipeline
    
    // The entire internal pipeline (including the 'valid_pipe_r')
    // stalls if the output register is full and not being read.
    assign pipe_en = !i_nco_req_r | i_nco_ack;
    
    // We can accept new data only if the pipeline is moving.
    assign t_angle_ack = pipe_en;
    
    assign accept_new_data = t_angle_req & t_angle_ack;

    // --- Input Register ---
    // Latch the 16 MSBs of the phase word when a new
    // transaction is accepted.
    always @(posedge clk) begin
        if (accept_new_data) begin
            // Use top 16 bits of the 32-bit phase
            phase_i_reg <= t_angle_dat[31:16]; 
        end
    end

    // --- Valid Bit Pipeline ---
    // This shift register acts in parallel to the CORDIC pipeline.
    // It must be stalled by the *same* 'pipe_en' signal.
    always @(posedge clk) begin
        if (reset_n == 1'b0) begin
            valid_pipe_r <= '0;
        end else if (pipe_en) begin
            valid_pipe_r[0] <= accept_new_data;
            for (int i = 1; i < LATENCY; i = i + 1) begin
                valid_pipe_r[i] <= valid_pipe_r[i-1];
            end
        end
    end

    // --- Output Request Logic ---
    assign i_nco_req = i_nco_req_r;
    
    always @(posedge clk or negedge reset_n) begin
        if (reset_n == 1'b0) begin
            i_nco_req_r <= 1'b0;
        end else begin
            if (i_nco_ack) begin
                i_nco_req_r <= 1'b0; // Clear request if data is taken
            end
            if (pipe_en && output_valid) begin
                i_nco_req_r <= 1'b1; // Set request if new valid data arrives
            end
            // Note: if ack and valid arrive same cycle, 'valid' wins,
            // which is correct behavior for a new item.
            // If pipe_en is 0, req holds its value unless ack is high.
        end
    end

    // --- CORDIC Module Instantiation ---
    cordic_dds_stallable #(
        .DW(CORDIC_DW),
        .PIPE_DEPTH(CORDIC_PIPE_DEPTH)
    ) u_cordic_dds (
        .clk(clk),
        .pipe_en(pipe_en),
        .phase_i(phase_i_reg),
        .i_amp_val(i_amp_val),
        .sin_o(sin_o_w),
        .cos_o(cos_o_w),
        .err_o() // Not connected
    );
    
    // --- Output Assignment ---
    // Pack the 17-bit sin/cos into the 32-bit output vector
    // [31:16] = Sine (Imaginary)
    // [15:0]  = Cosine (Real)
    // We sign-extend the 17-bit values to 16 bits by truncating the LSB.
    // This is a loss of precision but matches the 16-bit nco.v format.
    assign i_nco_dat[31:16] = sin_o_w[16:1];
    assign i_nco_dat[15:0]  = cos_o_w[16:1];

endmodule