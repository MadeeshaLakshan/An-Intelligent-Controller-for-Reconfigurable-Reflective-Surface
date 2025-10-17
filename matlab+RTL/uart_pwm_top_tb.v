`timescale 1ns/1ps
`include "uart_pwm_top.v"
`include "uart_rx.v"
`include "pwm_9ch.v"
`include "sev_seg_decorder.v"

module uart_pwm_top_tb;

    // Parameters
    localparam CLK_FREQ      = 50_000_000;
    localparam BAUD_RATE     = 115200;
    localparam CLKS_PER_BIT  = CLK_FREQ / BAUD_RATE;  // ~434
    localparam BIT_PERIOD    = CLKS_PER_BIT * 20;     // in ns (20 ns per clock cycle)

    // DUT signals
    reg        clk;
    reg        rst_n;
    reg        uart_rx;
    wire [8:0] pwm_out;

    // Seven-segment outputs
    wire [6:0] seg0, seg1, seg2, seg3;
    wire [3:0] pkt_count;

    // Instantiate DUT
    uart_pwm_top dut (
        .clk(clk),
        .rst_n(rst_n),
        .uart_rx(uart_rx),
        .pwm_out(pwm_out),
        .seg0(seg0),
        .seg1(seg1),
        .seg2(seg2),
        .seg3(seg3),
        .pkt_count(pkt_count)
    );

    // Clock generation (50 MHz)
    initial clk = 0;
    always #10 clk = ~clk;  // 20 ns period -> 50 MHz


    // UART send task (8-bit word, LSB first)
    task uart_send_byte(input [7:0] data);
        integer i;
        begin
            // Start bit
            uart_rx <= 0;
            #(BIT_PERIOD);

            // Data bits (LSB first)
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx <= data[i];
                #(BIT_PERIOD);
            end

            // Stop bit
            uart_rx <= 1;
            #(BIT_PERIOD);
        end
    endtask

    // UART send task (16-bit word -> 2 UART bytes, MSB first)
    task uart_send_word(input [15:0] data);
        begin
            uart_send_byte(data[15:8]); // High byte first
            uart_send_byte(data[7:0]);  // Low byte
        end
    endtask


    // Test sequence
    initial begin
        rst_n = 0;
        uart_rx = 1;  // Idle state
        #200;
        rst_n = 1;
        #1000;

        // Send 9 words = 9 duty cycles
        uart_send_word(16'h0001);  // PWM0 ~ 1/16 duty
        uart_send_word(16'h0002);  // PWM1 ~ 1/8 duty
        uart_send_word(16'h0003);  // PWM2 ~ 1/4 duty
        uart_send_word(16'h6666);  // PWM3 ~ 3/8 duty
        uart_send_word(16'h8888);  // PWM4 ~ 1/2 duty
        uart_send_word(16'hAAAA);  // PWM5 ~ 5/8 duty
        uart_send_word(16'hCCCC);  // PWM6 ~ 3/4 duty
        uart_send_word(16'hEEEE);  // PWM7 ~ 7/8 duty
        uart_send_word(16'hFFFF);  // PWM8 ~ full duty

        // Wait some time to observe PWM and seven-seg changes
        #20000000;

        $stop;
    end

    // Dump waves for GTKWave
    initial begin
        $dumpfile("uart_pwm_top_tb.vcd");
        $dumpvars(0, uart_pwm_top_tb);
    end

endmodule
