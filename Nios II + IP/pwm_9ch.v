`timescale 1ns/1ps
module pwm_9ch #(
    parameter RESOLUTION = 16
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [RESOLUTION-1:0] duty0,
    input  wire [RESOLUTION-1:0] duty1,
    input  wire [RESOLUTION-1:0] duty2,
    input  wire [RESOLUTION-1:0] duty3,
    input  wire [RESOLUTION-1:0] duty4,
    input  wire [RESOLUTION-1:0] duty5,
    input  wire [RESOLUTION-1:0] duty6,
    input  wire [RESOLUTION-1:0] duty7,
    input  wire [RESOLUTION-1:0] duty8,
    output reg  [8:0]           pwm_out
);

    reg [RESOLUTION-1:0] counter;

    // Shared counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            counter <= {RESOLUTION{1'b0}};
        else
            counter <= counter + 1;
    end

    // Compare against duty values
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_out <= 9'b0;
        end else begin
            pwm_out[0] <= (counter < duty0);
            pwm_out[1] <= (counter < duty1);
            pwm_out[2] <= (counter < duty2);
            pwm_out[3] <= (counter < duty3);
            pwm_out[4] <= (counter < duty4);
            pwm_out[5] <= (counter < duty5);
            pwm_out[6] <= (counter < duty6);
            pwm_out[7] <= (counter < duty7);
            pwm_out[8] <= (counter < duty8);
        end
    end

endmodule
