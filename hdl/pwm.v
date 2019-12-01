/*
 * PWM Module
 */

module pwm #(
	parameter DURATION_CYCLE=32
)(
	input clk,
    input resetn,
	input [31:0] pwm_in,
	output pwm_out,
);
	reg [31:0] counterI=0;

    reg pwm_counter=0;

	reg [9:0] count_temp=0;

	reg state= 0;

    assign pwm_out = pwm_counter;

	always @(posedge clk) begin
        if (!resetn) begin
			counterI <= 0;
            pwm_counter <= 1;
			state<=0;
			count_temp<= 0;
		end else begin
			counterI<= counterI+1;
            if (counterI[6] == 1) begin
				count_temp<=count_temp+1;
				counterI<=0;
			end
			if (state== 1'b0 && count_temp >= pwm_in) begin
				if (pwm_in<255) begin
					pwm_counter<=0;
				end
				state= 1'b1;
			end
			if (state== 1'b1 && count_temp[8] == 1) begin
				if (pwm_in>0) begin
					pwm_counter<=1;
				end
				state= 1'b0;
				count_temp<=0;
			end
		end
		
	end


endmodule
