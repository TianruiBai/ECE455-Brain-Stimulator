module UART_tx (
    input wire clk,
    input wire rst_n,
    input wire trmt,
    input wire [7:0] tx_data,
    output wire TX,
    output reg tx_done
);
typedef enum {
    IDLE,
    TRANSMIT
  } state_t;

  state_t state, next_state;
  reg shift, init, transmitting, set_done;

  reg [3:0] bit_cnt;

  always_ff @( posedge clk ) begin : bit_counter
    if(init) begin
        bit_cnt <= 4'b0;
    end
    else if (shift)begin
        bit_cnt <= bit_cnt + 4'b1;
    end
  end

  reg [11:0] baud_cnt;

  always_ff @( posedge clk ) begin : baud_counter
    if(init | shift) begin
        baud_cnt = '0;
    end
    else if (transmitting) begin
        baud_cnt <= baud_cnt + 12'b1;
    end
  end

  assign shift = (baud_cnt == 12'hA2C) ? 1'b1 : 1'b0;

  reg [8:0] tx_shift_reg;
  wire [8:0] next_tx_shift;

  assign next_tx_shift = init ? (trmt ? {tx_data, 1'b0} : 9'h1FF) :
                        shift ? {{1'b1}, {tx_shift_reg[8:1]}} :
                        tx_shift_reg;

    always_ff @( posedge clk, negedge rst_n ) begin : tx_shifter
        if(!rst_n) begin
            tx_shift_reg <= '1;
        end
        else begin
            tx_shift_reg <= next_tx_shift;
        end
    end
    
    assign TX = tx_shift_reg[0];

    always_ff @( posedge clk, negedge rst_n ) begin : StateMachine_FF
        if(!rst_n) begin
            state <= IDLE;
        end
        else begin
            state <= next_state;
        end
    end

    always_comb begin : StateMachine
        init = 1'b1;
        set_done = 1'b0;
        transmitting = 1'b0;
        next_state = state;
        case(state)
        IDLE: begin
            if(trmt)begin
                transmitting = 1'b1;
                next_state = TRANSMIT;
            end 
        end
        TRANSMIT: begin
            init = 1'b0;
            transmitting = 1'b1;
            if(bit_cnt == 4'hA)begin
                set_done = 1'b1;
                next_state = IDLE;
            end
        end
        default: begin 
            next_state = IDLE;
        end
    endcase
    end

    always_ff @( posedge clk, negedge rst_n ) begin : SRFF
        if(!rst_n)begin
            tx_done <= 1'b0;
        end
        else begin
            if(set_done & ~init)begin
                tx_done <= 1'b1;
            end
            else if(~set_done & init)begin
                tx_done <= 1'b0;
            end
        end
    end
endmodule