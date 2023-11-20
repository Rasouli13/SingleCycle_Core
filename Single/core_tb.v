`timescale 1 ns/10 ps  // time-unit = 1 ns, precision = 10 ps
`include "single.v"
module core_tb;

	reg clk;
	reg rst;
	
	core core(clk,rst);
	initial
	begin
	clk = 0;
		rst = 1;
		#11
		rst = 0;
	end

	always #5 clk = ~clk;
endmodule