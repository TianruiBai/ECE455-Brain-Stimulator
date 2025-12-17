/**
 * @brief Calculates a feedback current based on the peak of an offset input sine wave.
 *
 * This module accepts an *unsigned* 16-bit sine wave 'sample_dat' which
 * is offset by 16'd22900.
 *
 * 1. It subtracts the offset to get the true AC amplitude ('sample_ac').
 * 2. It detects the peak of this AC amplitude by tracking the maximum value
 * between an upward offset-crossing and a downward offset-crossing.
 * 3. It uses this latched peak amplitude in the fixed-point calculation:
 * fb_current = 6.602 * peak_amplitude + 42.89
 */
module peak_proc (
    input  logic             clk,         // System clock
    input  logic             rst_n,       // Active-low asynchronous reset
    input  logic [15:0]      sample_dat,  // 16-bit *unsigned* sine wave input
    output logic [15:0]       fb_current   // 16-bit calculated feedback current
);

    // --- System Parameters ---
    localparam logic [15:0] DC_OFFSET = 16'd22900; // NEW: DC offset of the input

    // --- Fixed-Point Parameters ---
    // Using 8 fractional bits (Q.8 format)
    // C1 = 6.602 * 2^8 = 1690.112 -> 1690
    localparam logic signed [15:0] C1_Q8 = 16'sd1690; 
    // C2 = 42.89 * 2^8 = 10979.84 -> 10980
    localparam logic signed [15:0] C2_Q8 = 16'sd10980;

    // --- Peak Detection Logic ---
    
    // NEW: Internal signal for the AC component (Amplitude)
    // Needs 17 bits: (0 to 65535) - 22900 = (-22900 to +42635)
    logic signed [16:0] sample_ac;
    assign sample_ac = $signed(sample_dat) - $signed(DC_OFFSET);

    // NEW: Widen peak storage to 17 bits for the AC amplitude
    logic signed [16:0] current_peak; // Tracks peak of the *current* half-cycle
    logic signed [16:0] last_peak;    // Stores the peak of the *previous* half-cycle

    logic               is_above_offset;  // Current sample is above offset
    logic               was_above_offset; // Previous sample was above offset

    // Register to store the previous state to detect offset-crossings
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            was_above_offset <= 1'b0;
        end else begin
            was_above_offset <= is_above_offset;
        end
    end

    // NEW: Combinational logic to determine current position relative to offset
    assign is_above_offset = (sample_dat >= DC_OFFSET);

    // NEW: Detect offset crossings
    wire offset_up_crossing = (is_above_offset && !was_above_offset); // Went from below to above
    wire offset_down_crossing = (!is_above_offset && was_above_offset); // Went from above to below

    // Peak tracking and latching
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_peak <= 17'sd0;
            last_peak    <= 17'sd0;
        end else begin
            if (offset_up_crossing) begin
                // Start of a new "positive" (above-offset) half-cycle.
                // Store the current AC amplitude.
                current_peak <= sample_ac; 
            end 
            else if (is_above_offset) begin
                // We are in the "positive" half-cycle, track the max AC amplitude.
                if (sample_ac > current_peak) begin
                    current_peak <= sample_ac;
                end
            end 
            else if (offset_down_crossing) begin
                // We just finished the "positive" half-cycle.
                // Latch the peak AC amplitude we found.
                last_peak <= current_peak << 1;
            end
        end
    end

    // --- Current Calculation Logic ---
    // This logic is based on the latched 'last_peak' (the AC amplitude).

    // Intermediate signals for calculation
    // NEW: Widen peak_q8 to 25 bits (17-bit peak + 8 frac bits)
    logic signed [24:0] peak_q8;           // S(16,0) -> S(24,8)
    logic signed [39:0] mult_result_q16;   // S(24,8) * S(15,8) -> S(39,16)
    logic signed [23:0] c2_q16;            // S(15,8) -> S(23,16)
    logic signed [39:0] add_result_q16;    // S(39,16) + S(23,16) -> S(39,16)
    logic signed [23:0] result_int;        // S(39,16) >> 16 -> S(23,0) (Integer)

    // 1. Convert latched peak (S(16,0)) to Q(24,8) format by left-shifting
    assign peak_q8 = {last_peak, 8'b0};

    // 2. Perform multiplication: (peak * 6.602)
    //    S(24,8) * S(15,8) -> S(39,16)
    assign mult_result_q16 = peak_q8 * C1_Q8;

    // 3. Convert C2 (42.89) from Q(15,8) to Q(23,16) to align for addition
    assign c2_q16 = {C2_Q8, 8'b0};

    // 4. Perform addition: (product + 42.89)
    //    S(39,16) + S(23,16) -> S(39,16) (adder sign-extends c2_q16)
    /* verilator lint_off WIDTHEXPAND */
    assign add_result_q16 = mult_result_q16 + c2_q16;
    /* verilator lint_on WIDTHEXPAND */

    // 5. Convert final Q(39,16) result back to an integer by shifting
    //    This truncates the 16 fractional bits.
    /* verilator lint_off WIDTHTRUNC */
    assign result_int = add_result_q16 >> 16;
    /* verilator lint_on WIDTHTRUNC */


    // --- Output Register ---
    // Register the final calculated value with saturation.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fb_current <= 16'h0000;
        end else begin
            // Saturate the result to fit in 16 bits (0 to 65535)
            // The formula should result in a positive current.
            if (result_int < 0) begin
                fb_current <= 16'h0000; // Saturate at 0
            end else if (result_int > 24'hFFFF) begin
                fb_current <= 16'hFFFF; // Saturate at max (65535)
            end else begin
                fb_current <= result_int[15:0];
            end
        end
    end

endmodule