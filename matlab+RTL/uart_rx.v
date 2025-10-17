// Code your design here
module uart_rx #(
    parameter CLOCKS_PER_PULSE = 4 , //50_000_000 / 115200 = 434
              BITS_PER_WORD = 8, // Number of bits in each word
              W_OUT = 24 // Width of output data bus
) (
    input clk,rst_n,rx,
    output reg m_valid,
    output reg [W_OUT-1:0] m_data// Output data bus
);
    localparam  IDLE    = 2'b00, 
                START   = 2'b01, 
                DATA    = 2'b10, 
                STOP    = 2'b11;
  	reg [1:0] state; 
    localparam NUM_WORDS = W_OUT / BITS_PER_WORD;

    //counters 
    reg[$clog2(CLOCKS_PER_PULSE)-1:0] c_clocks;
    reg[$clog2(BITS_PER_WORD)-1:0] c_bits;
    reg[$clog2(NUM_WORDS)-1:0] c_words;

    //state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {c_words,c_bits,c_clocks,m_valid,m_data} <= 1'b0;
            state <= IDLE;
        end else begin
            m_valid <= 0;
            case(state)
                IDLE: if(rx == 0)
                    state <= START;
                
                START: if(c_clocks == CLOCKS_PER_PULSE/2 - 1) begin
                        c_clocks <= 0;
                        state <= DATA;
                    end else begin
                        c_clocks <= c_clocks + 1;
                    end
              DATA:   if(c_clocks == CLOCKS_PER_PULSE-1)begin
                            c_clocks <= 0;
                            m_data <= {rx, m_data[W_OUT-1:1]}; // Shift in the received bit
                            if(c_bits == BITS_PER_WORD-1) begin
                                c_bits <= 0;
                                state <= STOP; // Move to stop state after all bits are received
                                if(c_words == NUM_WORDS - 1) begin
                                    m_valid <= 1; // Indicate that a complete word has been received
                                    c_words <= 0; // Reset word counter

                                end else c_words <= c_words + 1; // Move to next word
                            end else c_bits <= c_bits + 1; // Move to next bit
                        end else c_clocks <= c_clocks + 1; // Increment clock counter
                STOP: if(c_clocks == CLOCKS_PER_PULSE - 1) begin
                        c_clocks <= 0;
                        state <= IDLE; // Return to idle state after stop bit
                    end else c_clocks <= c_clocks + 1;
                        
            endcase
        end
    end        
                                                      
endmodule