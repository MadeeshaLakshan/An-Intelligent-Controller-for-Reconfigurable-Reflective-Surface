`timescale 1ns/1ps
module pwm_avalon_wrapper #(
    parameter RESOLUTION = 16
)(
    // Avalon-MM slave-like interface (simple)
    input  wire             clk,
    input  wire             reset_n,
    input  wire [3:0]       address,    // register select (0..8)
    input  wire             write,      // write strobe (asserted when addressing this slave)
    input  wire [31:0]      writedata,  // write data
    input  wire             read,       // read strobe (asserted when master reads)
    output reg  [31:0]      readdata,   // read data

    // PWM outputs
    output wire [8:0]       pwm_out
);

    // Duty cycle registers (9 channels)
    reg [RESOLUTION-1:0] duty [0:8];
    integer i;

    // ---------------------------
    // Write logic (synchronous)
    // ---------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            for (i = 0; i < 9; i = i + 1)
                duty[i] <= {RESOLUTION{1'b0}};
        end else if (write) begin
            case (address)
                4'd0: duty[0] <= writedata[RESOLUTION-1:0];
                4'd1: duty[1] <= writedata[RESOLUTION-1:0];
                4'd2: duty[2] <= writedata[RESOLUTION-1:0];
                4'd3: duty[3] <= writedata[RESOLUTION-1:0];
                4'd4: duty[4] <= writedata[RESOLUTION-1:0];
                4'd5: duty[5] <= writedata[RESOLUTION-1:0];
                4'd6: duty[6] <= writedata[RESOLUTION-1:0];
                4'd7: duty[7] <= writedata[RESOLUTION-1:0];
                4'd8: duty[8] <= writedata[RESOLUTION-1:0];
                default: ; // ignore out-of-range writes
            endcase
        end
    end

    // ---------------------------
    // Read logic (synchronous, gated by 'read')
    // - readdata is updated only when 'read' is asserted.
    // ---------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            readdata <= 32'h0;
        end else if (read) begin
            case (address)
                4'd0: readdata <= {{(32-RESOLUTION){1'b0}}, duty[0]};
                4'd1: readdata <= {{(32-RESOLUTION){1'b0}}, duty[1]};
                4'd2: readdata <= {{(32-RESOLUTION){1'b0}}, duty[2]};
                4'd3: readdata <= {{(32-RESOLUTION){1'b0}}, duty[3]};
                4'd4: readdata <= {{(32-RESOLUTION){1'b0}}, duty[4]};
                4'd5: readdata <= {{(32-RESOLUTION){1'b0}}, duty[5]};
                4'd6: readdata <= {{(32-RESOLUTION){1'b0}}, duty[6]};
                4'd7: readdata <= {{(32-RESOLUTION){1'b0}}, duty[7]};
                4'd8: readdata <= {{(32-RESOLUTION){1'b0}}, duty[8]};
                default: readdata <= 32'h0;
            endcase
        end
        // else: keep previous readdata value (no invalid transient)
    end

    // ---------------------------
    // Instantiate PWM core
    // ---------------------------
    pwm_9ch #(.RESOLUTION(RESOLUTION)) pwm_inst (
        .clk(clk),
        .rst_n(reset_n),
        .duty0(duty[0]),
        .duty1(duty[1]),
        .duty2(duty[2]),
        .duty3(duty[3]),
        .duty4(duty[4]),
        .duty5(duty[5]),
        .duty6(duty[6]),
        .duty7(duty[7]),
        .duty8(duty[8]),
        .pwm_out(pwm_out)
    );

endmodule


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
