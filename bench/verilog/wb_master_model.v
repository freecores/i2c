//
// Wishbone master model
//

`include "timescale.v"

module wb_master_model(clk, rst, adr, din, dout, cyc, stb, we, ack, err, rty);

input         clk, rst;
output [31:0]	adr;
input  [ 7:0]	din;
output [ 7:0]	dout;
output        cyc, stb;
output	      	we;
input		       ack, err, rty;

////////////////////////////////////////////////////////////////////
//
// Local Wires
//

reg	[31:0]	adr;
reg	[ 7:0]	dout;
reg		      cyc, stb;
reg		      we;

////////////////////////////////////////////////////////////////////
//
// Memory Logic
//

initial
	begin
		//adr = 32'hxxxx_xxxx;
		//adr = 0;
		adr  = 32'hxxxx_xxxx;
		dout = 8'hxx;
		cyc  = 1'b0;
		stb  = 1'bx;
		we   = 1'hx;
		#1;
		$display("\nINFO: WISHBONE MASTER MODEL INSTANTIATED (%m)\n");
	end

////////////////////////////////////////////////////////////////////
//
// Wishbone write cycle
//

task wb_write;
	input        delay;
	integer delay;

	input	[31:0]	a;
	input	[ 7:0]	d;

	begin

		repeat(delay) @(posedge clk);
		#1;
		adr  = a;
		dout = d;
		cyc  = 1'b1;
		stb  = 1'b1;
		we   = 1'b1;

		@(posedge clk);
		while(~ack)	@(posedge clk);
		#1;
		cyc  = 1'b0;
		stb  = 1'bx;
		adr  = 32'hxxxx_xxxx;
		dout = 8'hxx;
		we   = 1'hx;

	end
endtask

////////////////////////////////////////////////////////////////////
//
// Wishbone read cycle
//

task wb_read;
	input        delay;
	integer delay;

	input	 [31:0]	a;
	output	[ 7:0]	d;

	begin

		repeat(delay) @(posedge clk);
		#1;
		adr  = a;
		dout = 8'hxx;
		cyc  = 1'b1;
		stb  = 1'b1;
		we   = 1'b0;

		@(posedge clk);
		while(~ack)	@(posedge clk);
		#1;
		cyc  = 1'b0;
		stb  = 1'bx;
		adr  = 32'hxxxx_xxxx;
		dout = 8'hxx;
		we   = 1'hx;
		d    = din;

	end
endtask

endmodule
