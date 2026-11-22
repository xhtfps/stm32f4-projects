module div_250(
	clk,rst,out
);
	input clk;
	input rst;
	
	output reg out;
	reg [7:0] cnt;
	
	parameter div=49;
	
	always @ (posedge clk,negedge rst)
	begin 
		if(!rst)
			cnt<=0;
		else if(cnt==div)
			cnt<=0;
		else 
			cnt<=cnt+1;
	end
	
	always @(posedge clk,negedge rst)
	begin
		if(!rst)
			out<=0;
		else if(cnt==div)
			out<=~out;
	end
endmodule
		