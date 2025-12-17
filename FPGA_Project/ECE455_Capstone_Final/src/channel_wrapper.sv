module channel_wrapper #(
    parameter CHANNEL_ID = 0,
    parameter int CLK_FREQ_HZ = 50_000_000 // DEFAULT: 50MHz.
)(
    input  logic        clk,
    input  logic        rst_n,

    // --- Interface with cmd_proc ---
    input  logic [15:0] i_channel_sel,
    input  logic [31:0] i_payload32_a,
    input  logic [31:0] i_payload32_b,
    input  logic [63:0] i_payload64,
    input  logic [15:0] i_payload16,
    input  logic [1:0]  i_wave_type,
    input  logic [15:0] i_dat_type,
    input  logic        i_cmd_valid,

    // --- External Hardware I/O ---
    output logic        dac_sclk,
    output logic        dac_ss_n,
    output logic        dac_mosi,
    input  logic        dac_miso,
    
    output logic        adc_sclk,
    output logic        adc_ss_n,
    output logic        adc_mosi,
    input  logic        adc_miso,

    output logic        led0,
    output logic        led1
);

    // --- Ramp Speed Calculation ---
    // Target: 0.05mA/s over 0-5mA range (16-bit resolution)
    localparam real TARGET_RAMP_RATE_MA_S = 0.075;
    localparam real MAX_CURRENT_MA        = 6;
    localparam int  DAC_RES_MAX           = 65535; // 16-bit
    
    // Calculate cycles to wait before changing amplitude by 1 LSB
    localparam int CYCLES_PER_STEP = int'(CLK_FREQ_HZ / ((TARGET_RAMP_RATE_MA_S / MAX_CURRENT_MA) * DAC_RES_MAX));

    // --- Configuration Registers ---
    logic [31:0] reg_freq_up;
    logic [31:0] reg_freq_down;
    logic [31:0] reg_phase;
    logic [15:0] reg_target_amp;
    logic [1:0]  reg_wave_type;
    logic [63:0] reg_ramp_up_cnt; 
    logic [63:0] reg_ramp_down_cnt;
    logic        reg_enable;

    // --- Internal Signals ---
    logic [31:0] nco_data_out;
    logic        nco_valid;
    logic [15:0] saw_data_out;
    logic        saw_req;
    logic [15:0] amp_to_wavegen;
    
    // AFE
    logic [15:0] afe_corrected_amp;
    logic        afe_req_o;
    
    // SPI
    logic [15:0] adc_sample_val;
    logic        adc_done;
    logic        dac_done;
    logic        reset_dac;
    logic        output_en;
    logic [15:0] dac_data_to_send;

    // --- 1. Configuration Logic ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_freq_up       <= 32'b0;
            reg_freq_down     <= 32'b0;
            reg_phase         <= 32'b0;
            reg_target_amp    <= 16'b0;
            reg_wave_type     <= 2'b0;
            reg_ramp_up_cnt   <= 64'b0;
            reg_ramp_down_cnt <= 64'b0;
            reg_enable        <= 1'b0;
        end else begin
            if (i_cmd_valid && i_channel_sel[CHANNEL_ID]) begin
                case (i_dat_type[3:0])
                    4'h0: begin 
                        reg_freq_up   <= i_payload32_a;
                        reg_freq_down <= i_payload32_b;
                    end
                    4'h1: begin 
                        reg_phase      <= i_payload32_a;
                        reg_wave_type  <= i_wave_type;
                        reg_target_amp <= i_payload16;
                    end
                    4'h2: reg_ramp_up_cnt   <= i_payload64;
                    4'h3: reg_ramp_down_cnt <= i_payload64;
                    4'h4: reg_enable        <= 1'b1;
                    4'h5: reg_enable        <= 1'b0;
                endcase
            end
        end
    end

    // --- 2. Amplitude Ramp State Machine ---
    typedef enum logic [2:0] {RESET, IDLE, RAMP_UP, ACTIVE, RAMP_DOWN} state_t;
    state_t state;
    logic [15:0] current_amp_envelope;
    logic [31:0] step_timer; 
    
    // Logic to determine where we stop ramping down
    // If disabled: Stop at 0. If enabled (user lowered target): Stop at target.
    logic [15:0] ramp_down_floor;
    assign ramp_down_floor = (reg_enable) ? reg_target_amp : 16'b0;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            current_amp_envelope <= 16'b0;
            step_timer <= 32'b0;
            amp_to_wavegen <= 16'b0;
            output_en <= 1'b0;
            led0 <= 1'b0;
            led1 <= 1'b0;
            reset_dac <= 1'b0;
        end else begin
            
            // Default Assignment
            amp_to_wavegen <= current_amp_envelope; 

            case (state)
                RESET: begin
                    reset_dac <= 1'b1;
                    if(dac_done) state <= IDLE;
                end

                IDLE: begin
                    reset_dac <= 1'b0;
                    led0 <= 1'b0;
                    led1 <= 1'b0;
                    output_en <= 1'b0;
                    current_amp_envelope <= 16'b0;
                    step_timer <= 32'b0;
                    
                    if (reg_enable) begin
                        state <= RAMP_UP;
                    end
                end

                RAMP_UP: begin
                    led0 <= 1'b1;
                    led1 <= 1'b0; // Indicate Ramping
                    output_en <= 1'b1;

                    // 1. Check for Disable
                    if (!reg_enable) begin
                        state <= RAMP_DOWN;
                    end 
                    // 2. Dynamic Check: If target lowered below current while ramping up
                    else if (current_amp_envelope > reg_target_amp) begin
                        state <= RAMP_DOWN;
                    end
                    // 3. Ramp Logic
                    else if (current_amp_envelope == reg_target_amp) begin
                        state <= ACTIVE;
                    end 
                    else begin
                        if (step_timer >= CYCLES_PER_STEP) begin
                            step_timer <= 0;
                            current_amp_envelope <= current_amp_envelope + 1;
                        end else begin
                            step_timer <= step_timer + 1;
                        end
                    end
                end

                ACTIVE: begin
                    led0 <= 1'b1;
                    led1 <= 1'b1; // Indicate Active/Stable
                    output_en <= 1'b1;
                    
                    // In Active mode, we use the AFE corrected value for the waveform,
                    // but we keep current_amp_envelope at the target for reference.
                    amp_to_wavegen <= afe_corrected_amp; 

                    // 1. Check for Disable
                    if (!reg_enable) begin
                        state <= RAMP_DOWN;
                    end 
                    // 2. Dynamic Update: User Increased Target
                    else if (reg_target_amp > current_amp_envelope) begin
                        // Reset timer to ensure smooth start of ramp
                        step_timer <= 0; 
                        state <= RAMP_UP;
                    end
                    // 3. Dynamic Update: User Decreased Target
                    else if (reg_target_amp < current_amp_envelope) begin
                        step_timer <= 0;
                        state <= RAMP_DOWN;
                    end
                end

                RAMP_DOWN: begin
                    led0 <= 1'b0;
                    led1 <= 1'b1; 
                    output_en <= 1'b1;

                    // 1. Dynamic Check: If target raised above current while ramping down
                    if (reg_enable && (current_amp_envelope < reg_target_amp)) begin
                        state <= RAMP_UP;
                    end
                    // 2. Ramp Logic
                    else if (current_amp_envelope <= ramp_down_floor) begin
                        // Enforce floor to avoid undershoot
                        current_amp_envelope <= ramp_down_floor;
                        
                        // Decide next state based on why we were ramping down
                        if (reg_enable) begin
                            // We ramped down to a new valid non-zero target
                            state <= ACTIVE;
                        end else begin
                            // We ramped down to 0 because of disable
                            amp_to_wavegen <= 16'b0;
                            state <= IDLE;
                        end
                    end 
                    else begin
                        if (step_timer >= CYCLES_PER_STEP) begin
                            step_timer <= 0;
                            current_amp_envelope <= current_amp_envelope - 1;
                        end else begin
                            step_timer <= step_timer + 1;
                        end
                    end
                end
            endcase
        end
    end

    // --- 3. Instantiations ---

    analog_front_end u_afe (
        .clk          (clk),
        .rst_n        (rst_n),
        .current_val  (current_amp_envelope),
        .sample_dat   (adc_sample_val),
        .o_nco_amp_val(afe_corrected_amp),
        .req_o        (afe_req_o),
        .ack_i        (adc_done)
    );

    nco_generator u_nco (
        .clk           (clk),
        .reset_n       (rst_n),
        .i_freq_word   ((reg_freq_up)), 
        .i_phase_offset(reg_phase),
        .i_amp_val     (amp_to_wavegen),
        .o_nco_dat     (nco_data_out),
        .o_nco_dat_req (nco_valid),
        .i_nco_dat_ack (dac_done)
    );

    tri_sawtooth_gen #(
        .INPUT_BITS(16),
        .OUTPUT_BITS(16)
    ) u_saw (
        .clk           (clk),
        .rst_n         (rst_n),
        .accum         (output_en),
        .ramp_up_freq  (reg_freq_up[20:5]),
        .ramp_down_freq(reg_freq_down[20:5]),
        .on_cnt        (1'b0),
        .off_cnt       (1'b0),
        .max_amp       (amp_to_wavegen),
        .wave_out      (saw_data_out),
        .req           (saw_req),
        .ack           (dac_done)
    );

    // --- 4. Data Multiplexing & DAC Formatting ---
    // DAC Spec: 0x8000 = 0V, 0xFFFF = +5V, 0x0000 = -5V
    
    always_comb begin
        case (reg_wave_type)
            2'b00: begin 
                // NCO Output: Signed 2's Complement.
                dac_data_to_send = { ~nco_data_out[31], nco_data_out[30:16] };
            end
            
            2'b01: begin 
                // Sawtooth Output: Unsigned.
                // Assuming saw_data_out is 0 to MAX_AMP (unsigned)
                // We likely want to center this or map it appropriately.
                // Standard offset mapping:
                dac_data_to_send = saw_data_out - 17'sd32768; 
            end
            
            default: dac_data_to_send = 16'h8000;
        endcase
    end
    
    

    // --- 5. SPI Control ---

    logic trigger_dac;
    assign trigger_dac = reset_dac | ((output_en) & ((reg_wave_type == 2'b01) ? saw_req : nco_valid));

    SPI_monrch u_spi_dac (
        .clk (clk),
        .rst_n(rst_n),
        .snd (trigger_dac), 
        .cmd (dac_data_to_send),
        .done(dac_done),
        .resp(),
        .SS_n(dac_ss_n),
        .SCLK(dac_sclk),
        .MOSI(dac_mosi),
        .MISO(dac_miso)
    );

    // Trigger ADC in sync with DAC update
    logic trigger_adc;
    assign trigger_adc = trigger_dac;

    SPI_monrch u_spi_adc (
        .clk (clk),
        .rst_n(rst_n),
        .snd (trigger_adc),
        .cmd (16'h0000), 
        .done(adc_done),
        .resp(adc_sample_val),
        .SS_n(adc_ss_n),
        .SCLK(adc_sclk),
        .MOSI(adc_mosi),
        .MISO(adc_miso)
    );

endmodule