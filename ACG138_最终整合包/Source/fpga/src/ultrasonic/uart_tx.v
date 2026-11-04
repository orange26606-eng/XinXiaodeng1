`timescale 1ns/1ps
module uart_tx #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200
) (
    input wire clk, input wire rst,
    input wire [7:0] data, input wire valid,
    output reg busy, output reg serial_out
);
    localparam integer BIT_CLKS = (CLK_HZ + BAUD/2) / BAUD;
    reg [31:0] count;
    reg [3:0] bit_index;
    reg [9:0] frame;
    always @(posedge clk) begin
        if (rst) begin
            busy <= 0; serial_out <= 1; count <= 0; bit_index <= 0; frame <= 10'h3ff;
        end else if (!busy) begin
            serial_out <= 1;
            if (valid) begin
                frame <= {1'b1, data, 1'b0};
                serial_out <= 0; busy <= 1; count <= BIT_CLKS-1; bit_index <= 0;
            end
        end else if (count != 0) count <= count-1;
        else if (bit_index == 9) begin busy <= 0; serial_out <= 1; end
        else begin
            bit_index <= bit_index+1'b1;
            serial_out <= frame[bit_index+1'b1]; count <= BIT_CLKS-1;
        end
    end
endmodule
