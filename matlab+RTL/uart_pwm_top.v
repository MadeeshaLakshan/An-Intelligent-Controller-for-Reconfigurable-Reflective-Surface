// =====================================================
// Data Store Module for PWM duty registers
// =====================================================
module uart_pwm_data_store #(
    parameter WIDTH = 16,
    parameter CHANNELS = 9
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              rx_valid,
    input  wire [WIDTH-1:0]  rx_data,
    output reg  [3:0]        pkt_count,

    output reg [WIDTH-1:0] duty0,
    output reg [WIDTH-1:0] duty1,
    output reg [WIDTH-1:0] duty2,
    output reg [WIDTH-1:0] duty3,
    output reg [WIDTH-1:0] duty4,
    output reg [WIDTH-1:0] duty5,
    output reg [WIDTH-1:0] duty6,
    output reg [WIDTH-1:0] duty7,
    output reg [WIDTH-1:0] duty8
);

    // Duty register array (internal only)
    reg [WIDTH-1:0] duty_reg [0:CHANNELS-1];

    // Update duty registers
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pkt_count <= 4'd0;
            duty0 <= 0; duty1 <= 0; duty2 <= 0;
            duty3 <= 0; duty4 <= 0; duty5 <= 0;
            duty6 <= 0; duty7 <= 0; duty8 <= 0;
        end else if (rx_valid) begin
            duty_reg[pkt_count] <= rx_data;

            // update unpacked outputs
            duty0 <= duty_reg[0];
            duty1 <= duty_reg[1];
            duty2 <= duty_reg[2];
            duty3 <= duty_reg[3];
            duty4 <= duty_reg[4];
            duty5 <= duty_reg[5];
            duty6 <= duty_reg[6];
            duty7 <= duty_reg[7];
            duty8 <= duty_reg[8];

            // increment or wrap count
            if (pkt_count == CHANNELS-1)
                pkt_count <= 4'd0;
            else
                pkt_count <= pkt_count + 1;
        end
    end

endmodule


// =====================================================
// Top Module
// =====================================================
module uart_pwm_top (
    input  wire        clk,        // System clock
    input  wire        rst_n,      // Active low reset
    input  wire        uart_rx,    // UART RX input
    output wire [8:0]  pwm_out,    // 9 PWM outputs

    // === Added for 4 seven-segment displays ===
    output wire [6:0] seg0,
    output wire [6:0] seg1,
    output wire [6:0] seg2,
    output wire [6:0] seg3,
    output wire [3:0] pkt_count
);

    // UART receiver signals
    wire        rx_valid;
    wire [15:0] rx_data;

    // Instantiate UART receiver
    uart_rx #(.CLOCKS_PER_PULSE(434), // Assuming 50MHz clock and 115200 baud rate
              .BITS_PER_WORD(8), 
              .W_OUT(16) 
    ) uart_rx_inst (
        .clk(clk),
        .rst_n(rst_n),
        .rx(uart_rx),
        .m_data(rx_data),
        .m_valid(rx_valid)
    );

    // Duty signals (individual wires)
    wire [15:0] duty0, duty1, duty2, duty3, duty4, duty5, duty6, duty7, duty8;

    // Instantiate Data Store
    uart_pwm_data_store #(
        .WIDTH(16),
        .CHANNELS(9)
    ) data_store_inst (
        .clk(clk),
        .rst_n(rst_n),
        .rx_valid(rx_valid),
        .rx_data(rx_data),
        .pkt_count(pkt_count),
        .duty0(duty0), .duty1(duty1), .duty2(duty2),
        .duty3(duty3), .duty4(duty4), .duty5(duty5),
        .duty6(duty6), .duty7(duty7), .duty8(duty8)
    );

    // Instantiate PWM generator (9 channels)
    pwm_9ch #(
        .RESOLUTION(16)
    ) pwm_inst (
        .clk(clk),
        .rst_n(rst_n),
        .duty0(duty0),
        .duty1(duty1),
        .duty2(duty2),
        .duty3(duty3),
        .duty4(duty4),
        .duty5(duty5),
        .duty6(duty6),
        .duty7(duty7),
        .duty8(duty8),
        .pwm_out(pwm_out)
    );

    // === Added: Display last received rx_data on 4 seven-segment displays ===
    wire [3:0] digit0 = rx_data[3:0];    // Least significant nibble
    wire [3:0] digit1 = rx_data[7:4];
    wire [3:0] digit2 = rx_data[11:8];
    wire [3:0] digit3 = rx_data[15:12];  // Most significant nibble

    sev_seg_decorder seg_inst0 (.digit(digit0), .seg(seg0));
    sev_seg_decorder seg_inst1 (.digit(digit1), .seg(seg1));
    sev_seg_decorder seg_inst2 (.digit(digit2), .seg(seg2));
    sev_seg_decorder seg_inst3 (.digit(digit3), .seg(seg3));

endmodule
