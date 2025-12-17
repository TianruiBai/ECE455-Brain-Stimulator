/**
 * @brief Triangle/Sawtooth Waveform Generator with req/ack Handshake
 *
 * Modified:
 * - Added max_amp input for variable amplitude.
 * - Output is now 16-bit signed.
 */
module tri_sawtooth_gen #(
    parameter INPUT_BITS  = 16,
    parameter OUTPUT_BITS = 16
)(
    input logic clk,
    input logic rst_n,

    // Control Inputs
    input logic accum,                    // Enable for accumulation
    input logic [INPUT_BITS-1:0] ramp_up_freq,    // Slope up increment
    input logic [INPUT_BITS-1:0] ramp_down_freq,  // Slope down increment
    input logic [INPUT_BITS*2-1:0] on_cnt,
    input logic [INPUT_BITS*2-1:0] off_cnt,
    
    // NEW: Amplitude Control (16-bit unsigned input)
    input logic [15:0] max_amp, 

    // Data Output (Modified to Signed)
    output logic signed [OUTPUT_BITS-1:0] wave_out, 

    // Handshake Ports
    input logic ack,                    
    output logic req                     
);

    // --- Internal Registers ---
    typedef enum { RAMP_UP, ON_CNT, RAMP_DOWN, OFF_CNT } state_t;
    state_t cur_state, nxt_state;

    logic [INPUT_BITS*2 - 1:0] bit_cnt;
    
    // 1. NCO Phase Accumulator (Using extra bit for overflow detection)
    logic [INPUT_BITS:0] nco_phase_accum;
    
    // 2. Output Waveform Accumulator 
    logic [INPUT_BITS:0] wave_accum; 

    // --- Combinational Logic ---
    
    logic [INPUT_BITS:0] effective_phase;
    logic ramp_direction; // 0 = ramp up, 1 = ramp down

    // --- AMPLITUDE SCALING LOGIC (NEW) ---
    // We align the 16-bit max_amp to the MSBs of the INPUT_BITS accumulator.
    // If INPUT_BITS is 32 and max_amp is 16-bit: Shift left by 16.
    localparam SHIFT_AMT = INPUT_BITS - 16;
    
    logic [INPUT_BITS:0] internal_max_limit;
    assign internal_max_limit = {1'b0, max_amp, {SHIFT_AMT{1'b0}}};

    // Handshake "go" signal
    logic do_accum, accum_stage, clr_nco;
    assign do_accum = accum & (~req | ack);

    logic [INPUT_BITS:0] nxt_nco_accum;

    // Calculate next step based on direction
    assign nxt_nco_accum = nco_phase_accum + (ramp_direction ? (-ramp_down_freq) : ramp_up_freq);

    // --- NCO Saturation Logic (Modified) ---
    // Instead of saturation at 32'hFFFFFFFF, we saturate at internal_max_limit
    always_comb begin
        if (ramp_direction == 0) begin // Ramp Up
            // If next step exceeds max limit, clamp to max limit
            if (nxt_nco_accum >= internal_max_limit) 
                effective_phase = internal_max_limit;
            else 
                effective_phase = nxt_nco_accum;
        end else begin // Ramp Down
            // If next step underflows (MSB set in INPUT_BITS+1 width), clamp to 0
            if (nxt_nco_accum[INPUT_BITS]) 
                effective_phase = '0;
            else 
                effective_phase = nxt_nco_accum;
        end
    end

    // --- Sequential Logic (NCO) ---
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            nco_phase_accum <= '0;
        end else if (clr_nco) begin
            nco_phase_accum <= '0;
        end else if (do_accum & accum_stage) begin
            nco_phase_accum <= effective_phase;
        end 
    end

    // --- Sequential Logic (Output Wave) ---
    logic do_wave_accum;
    logic [INPUT_BITS:0] effective_wave_accum;
    
    // The wave accumulator follows the phase accumulator (simple pass-through in this architecture)
    // We duplicate the saturation check here for state machine flags
    assign effective_wave_accum = effective_phase;

    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            wave_accum <= '0;
        end else if (do_accum & do_wave_accum) begin
            wave_accum <= effective_wave_accum;
        end
    end

    logic cnt, clr_cnt;
    always_ff @( posedge clk, negedge rst_n ) begin : Counter
        if(!rst_n)begin
            bit_cnt <= '0;
        end else if (clr_cnt) begin
            bit_cnt <= 0;
        end else if (cnt) begin
            bit_cnt <= bit_cnt + 1'b1;
        end
    end

    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            cur_state <= RAMP_UP;
        end else if (accum) begin 
            cur_state <= nxt_state;
        end
    end

    // --- Combinational Logic (State Machine) ----
    always_comb begin : State_Logic
        nxt_state = cur_state;
        ramp_direction = 0; // Default to UP
        cnt = 0;
        clr_cnt = 0;
        accum_stage = 0;
        do_wave_accum = 0;
        clr_nco = 0;

        case (cur_state)
            RAMP_UP : begin
                accum_stage = 1;
                do_wave_accum = 1;
                ramp_direction = 0;
                // MODIFIED: Check against internal_max_limit instead of fixed max
                if (effective_wave_accum >= internal_max_limit) begin
                    nxt_state = ON_CNT;
                end
            end

            ON_CNT : begin
                cnt = 1;
                if(bit_cnt == on_cnt)begin
                    accum_stage = 1;
                    ramp_direction = 1; // Start going down
                    clr_cnt = 1;
                    nxt_state = RAMP_DOWN;
                end
            end
            
            RAMP_DOWN : begin
                accum_stage = 1;
                do_wave_accum = 1;
                ramp_direction = 1;
                if (effective_wave_accum == '0) begin
                    do_wave_accum = 1;
                    nxt_state = OFF_CNT;
                end
            end 

            OFF_CNT : begin
                cnt = 1;
                if(bit_cnt == off_cnt)begin
                    clr_nco = 1;
                    ramp_direction = 0;
                    accum_stage = 1;
                    clr_cnt = 1;
                    nxt_state = RAMP_UP;
                end
            end
            
            default: begin
                nxt_state = RAMP_UP;
                ramp_direction = 0;
            end 
        endcase
    end

    // --- Sequential Logic (Output Handshake) ---
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            req <= 1'b0;
        end else begin
            if (do_accum) begin
                req <= 1'b1;
            end else if (ack) begin
                req <= 1'b0;
            end
        end
    end

    // --- Output Assignment ---
    // Takes the top bits (MSBs). 
    // Since we saturated the internal accumulator based on the MSBs of max_amp,
    // this slice will output exactly the range 0 to max_amp.
    assign wave_out = wave_accum[INPUT_BITS-1 : INPUT_BITS-OUTPUT_BITS];

endmodule