module SPI_monrch (
    input wire clk,
    input wire rst_n,
    input wire snd,
    input wire [15:0] cmd,
    output reg done,
    output wire [15:0] resp,
    output reg SS_n,
    output wire SCLK,
    output reg MOSI,
    input wire MISO
);

typedef enum { IDLE, TRANCEIVE, DONE } state_t;
state_t state, next_state;
reg init, ld_SCLK, set_done;
wire done16, shft, full;
reg [15:0] SPI_shift_reg;
reg [3:0] SCLK_div_reg;
reg [4:0] bit_cntr_reg;
wire [15:0] next_shift_reg;

assign next_shift_reg = {SPI_shift_reg[14:0],MISO};

always_ff @( posedge clk ) begin : SHIFT_REG
    if(init)begin
        SPI_shift_reg <= cmd;
    end else if (shft) begin
        SPI_shift_reg <= next_shift_reg;
    end
end

assign MOSI = SPI_shift_reg[15];
assign resp = SPI_shift_reg;

always_ff @( posedge clk ) begin : SCLK_div
    if(ld_SCLK)begin
        SCLK_div_reg <= 4'b1011;
    end else begin
        SCLK_div_reg <= SCLK_div_reg + 1;
    end
end

assign shft = SCLK_div_reg == 4'b1001;
assign full = SCLK_div_reg == 4'b1111;

assign SCLK = SCLK_div_reg[3];

always_ff @( posedge clk ) begin : bit_cntr
    if(init)begin
        bit_cntr_reg <= 5'b00000;
    end else if (shft) begin
        bit_cntr_reg <= bit_cntr_reg + 1;
    end
end

assign done16 = bit_cntr_reg == 5'h10;

always_ff @( posedge clk, negedge rst_n ) begin : StateMachine_FF
    if(!rst_n)begin
        state <= IDLE;
    end else begin
        state <= next_state;
    end
end

always_comb begin
    init = 1'b0;
    ld_SCLK = 1'b0;
    set_done = 1'b0;
    next_state = state;
    case (state)
        IDLE : begin
            ld_SCLK = 1'b1;
            if(snd)begin
                init = 1'b1;
                next_state = TRANCEIVE;
            end
        end
        TRANCEIVE : begin
            if(done16)begin
                next_state = DONE;
            end
        end
        DONE : begin
            if(full) begin
                set_done = 1'b1;
                next_state = IDLE;
            end
        end
        default: 
            next_state <= IDLE;
    endcase
end

always_ff @( posedge clk, negedge rst_n ) begin : SS_SRFF
    if(!rst_n)begin
        SS_n <= 1'b1;
    end
    else begin
        if(set_done & ~init)begin
            SS_n <= 1'b1;
        end
        else if(~set_done & init)begin
            SS_n <= 1'b0;
        end
    end
end

always_ff @( posedge clk, negedge rst_n ) begin : done_SRFF
    if(!rst_n)begin
        done <= 1'b0;
    end
    else begin
        if(set_done & ~init)begin
            done <= 1'b1;
        end
        else if(~set_done & init)begin
            done <= 1'b0;
        end
    end
end

endmodule