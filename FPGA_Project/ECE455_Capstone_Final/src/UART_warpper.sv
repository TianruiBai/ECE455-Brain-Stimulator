module UART_warpper (
    input wire clk,
    input wire rst_n,
    input wire clr_cmd_rdy,
    input wire trmt,
    input wire [7:0] resp, // tx_data
    input wire RX,
    output reg cmd_rdy,
    output wire [15:0] cmd,
    output wire tx_done,
    output wire TX
);

typedef enum {
    IDLE,
    LSB_Stage
  } state_t;

state_t state, next_state;
wire rx_rdy;
reg clr_rdy, set_cmd_rdy, next_cmd_data;
reg [7:0] MSB_cmd;
wire [7:0] rx_data;

UART iUART_Receive (.clk(clk),.rst_n(rst_n),.RX(RX),.TX(TX),.rx_rdy(rx_rdy),.clr_rx_rdy(clr_rdy),.rx_data(rx_data),.trmt(trmt),.tx_data(resp),.tx_done(tx_done));

always_ff @( posedge clk, negedge rst_n ) begin : StateMachine_FF
    if(!rst_n) begin
        state <= IDLE;
    end else begin
        state <= next_state;
    end
end

always_comb begin : StateMachine
        clr_rdy = 1'b0;
        next_cmd_data = 1'b0;
        set_cmd_rdy = 1'b0;
        next_state = state;
    case (state)
        IDLE: begin
            if(rx_rdy)begin
                next_cmd_data = 1'b1;
                clr_rdy = 1'b1;
                next_state <= LSB_Stage;
            end
        end
        LSB_Stage : begin
            if(rx_rdy)begin
                set_cmd_rdy = 1'b1;
                next_state <= IDLE;
            end
        end
        default: begin
            next_state = IDLE;
        end
    endcase
end

always_ff @( posedge clk, negedge rst_n ) begin : SRFF
            if(!rst_n)begin
                cmd_rdy <= 1'b0;
            end
            else if (clr_cmd_rdy) begin
                cmd_rdy <= 1'b0;
            end
            else begin
                if(set_cmd_rdy & ~next_cmd_data)begin
                    cmd_rdy <= 1'b1;
                end
                else if(~set_cmd_rdy & next_cmd_data)begin
                    cmd_rdy <= 1'b0;
                end
            end
        end

always_ff @( posedge clk) begin : UART_DATA
    if(next_cmd_data)begin
        MSB_cmd <= rx_data;
    end
end

assign cmd = {MSB_cmd, rx_data};

endmodule