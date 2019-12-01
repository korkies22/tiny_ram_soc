/*
 * Clock Module
 */

module clock (
	input clk,
    input resetn,
	output [31:0] clock_out,
);
	reg [31:0] counterI=0;

    reg [31:0] counterO=32'h00000000;

    assign clock_out = counterO;

	always @(posedge clk) begin
		counterI <= counterI + 1;
		if(counterI == 7)
		begin
			counterI<=0;
			counterO <= counterO+1;
		end
	end


endmodule