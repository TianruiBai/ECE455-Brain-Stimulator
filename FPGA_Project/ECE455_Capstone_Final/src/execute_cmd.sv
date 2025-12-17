module cmd_exec (
    input  logic        clk,
    input  logic        rst,

    input  logic        pkt_ready,
    input  logic [3:0]  pkt_cmd,
    input  logic [3:0]  channels,
    input  logic [20:0] phase,
    input  logic [1:0]  wave_type,
    input  logic [19:0] frequency,
    input  logic [20:0] currents,

    output logic        cmd_executed,

    // --- SPI Interface (4 Channels) ---
    // DAC Interface
    output logic [3:0]  dac_sclk,
    output logic [3:0]  dac_ss_n,
    output logic [3:0]  dac_mosi,
    input  logic [3:0]  dac_miso,

    // ADC Interface
    output logic [3:0]  adc_sclk,
    output logic [3:0]  adc_ss_n,
    output logic [3:0]  adc_mosi,
    input  logic [3:0]  adc_miso,

    output logic [3:0]  led0,
    output logic [3:0]  led1
);

    // --- Internal Signals ---
    logic        rst_n;
    logic [15:0] wrapper_channel_sel;
    logic [31:0] wrapper_payload32_a;
    logic [31:0] wrapper_payload32_b;
    logic [63:0] wrapper_payload64;
    logic [15:0] wrapper_payload16;
    logic [1:0]  wrapper_wave_type;
    logic [15:0] wrapper_dat_type;
    logic        wrapper_cmd_valid;         

    // --- Signal Mapping & Logic ---
    
    // Invert active-high rst to active-low rst_n for the wrappers
    assign rst_n = ~rst; 

    // 1. Channel Selection
    // Map 4-bit 'channels' input to the lower 4 bits of the 16-bit selector
    // The channel_wrapper checks i_channel_sel[CHANNEL_ID]
    assign wrapper_channel_sel = {12'b0, channels};

    // 2. Data Type / Command
    assign wrapper_dat_type = {12'b0, pkt_cmd};

    // 3. Command Valid Logic
    assign wrapper_cmd_valid = pkt_ready;
    
    // Simple acknowledgment: Register the ready signal to indicate execution
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            cmd_executed <= 1'b0;
        end else begin
            cmd_executed <= pkt_ready;
        end
    end

    // 4. Payload Mapping
    always_comb begin
        // Defaults
        wrapper_payload32_a = 32'b0;
        wrapper_payload32_b = 32'b0;
        wrapper_payload16   = 16'b0;
        wrapper_wave_type   = 2'b0;
        wrapper_payload64   = 64'b0;

        // Construct 64-bit payload from all inputs (22 + 21 + 21 = 64 bits)
        // Order: Frequency (MSB) -> Phase -> Currents (LSB)
        wrapper_payload64 = {frequency, phase, currents};

        case (pkt_cmd)
            // Command 0: Set Frequencies
            // Wrapper expects freq_up in payload32_a, freq_down in payload32_b
            4'h0: begin
                // Frequency is 22-bit. Map to high 32-bit (MSB aligned).
                wrapper_payload32_a = frequency * 32'd37625;
                // Assuming symmetric up/down freq if only one is provided
                wrapper_payload32_b = frequency * 32'd37625; 
            end

            // Command 1: Set Phase, Wave Type, Amp
            // Wrapper expects Phase in payload32_a, Amp in payload16
            4'h1: begin
                // Phase is 21-bit. Map to high 32-bit (MSB aligned).
                wrapper_payload32_a = {phase, 11'b0};
                
                // Currents is 21-bit. Target is 16-bit. "Use low 16-bit".
                wrapper_payload16   = currents[15:0];

                // Wave Type is 2-bit. 
                // We extract this from unused bits [17:16] of the 'currents' input.
                wrapper_wave_type   = wave_type; 
            end

            // Command 2 & 3: Ramp Counts
            // Wrapper expects 64-bit payload
            4'h2, 4'h3: begin
                wrapper_payload64 = {frequency, phase, currents};
            end

            default: begin
                // Default behavior (zeros)
            end
        endcase
    end

    // --- Instantiation of 4 Channel Wrappers ---
    genvar i;
    generate
        for (i = 0; i < 4; i++) begin : gen_channels
            channel_wrapper #(
                .CHANNEL_ID(i)
            ) u_channel (
                .clk            (clk),
                .rst_n          (rst_n),
                
                // Cmd Proc Interface
                .i_channel_sel  (wrapper_channel_sel),
                .i_payload32_a  (wrapper_payload32_a),
                .i_payload32_b  (wrapper_payload32_b),
                .i_payload64    (wrapper_payload64),
                .i_payload16    (wrapper_payload16),
                .i_wave_type    (wrapper_wave_type),
                .i_dat_type     (wrapper_dat_type),
                .i_cmd_valid    (wrapper_cmd_valid),

                // External Hardware I/O
                .dac_sclk       (dac_sclk[i]),
                .dac_ss_n       (dac_ss_n[i]),
                .dac_mosi       (dac_mosi[i]),
                .dac_miso       (dac_miso[i]),
                
                .adc_sclk       (adc_sclk[i]),
                .adc_ss_n       (adc_ss_n[i]),
                .adc_mosi       (adc_mosi[i]),
                .adc_miso       (adc_miso[i]),
                
                .led0           (led0[i]),
                .led1           (led1[i])
            );
        end
    endgenerate

endmodule