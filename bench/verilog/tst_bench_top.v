//
// Testbench for wishbone i2c master module
//

`include "timescale.v"

module tst_bench_top();

	//
	// wires && regs
	//
	reg  clk;
	reg  rstn;

	wire [31:0] adr;
	wire [ 7:0] dat_i, dat_o;
	wire we;
	wire stb;
	wire cyc;
	wire ack;
	wire inta;

	reg [7:0] q;

	wire scl, scl_o, scl_oen;
	wire sda, sda_o, sda_oen;

	parameter PRER_LO = 3'b000;
	parameter PRER_HI = 3'b001;
	parameter CTR     = 3'b010;
	parameter RXR     = 3'b011;
	parameter TXR     = 3'b011;
	parameter CR      = 3'b100;
	parameter SR      = 3'b100;

	parameter TXR_R   = 3'b101; // undocumented / reserved output
	parameter CR_R    = 3'b110; // undocumented / reserved output

	//
	// Module body
	//

	// generate clock
	always #5 clk = ~clk;

	// hookup wishbone master model
	wb_master_model u0 (
		.clk(clk),
		.rst(rstn),
		.adr(adr),
		.din(dat_i),
		.dout(dat_o),
		.cyc(cyc),
		.stb(stb),
		.we(we),
		.ack(ack),
		.err(1'b0),
		.rty(1'b0)
	);

	// hookup wishbone_i2c_master core
	i2c_master_top i2c_top (

		// wishbone interface
		.wb_clk_i(clk), 
		.wb_rst_i(1'b0), 
		.arst_i(rstn), 
		.wb_adr_i(adr[2:0]), 
		.wb_dat_i(dat_o), 
		.wb_dat_o(dat_i), 
		.wb_we_i(we), 
		.wb_stb_i(stb), 
		.wb_cyc_i(cyc), 
		.wb_ack_o(ack), 
		.wb_inta_o(inta),

		// i2c signals
		.scl_pad_i(scl), 
		.scl_pad_o(scl_o), 
		.scl_padoen_o(scl_oen), 
		.sda_pad_i(sda), 
		.sda_pad_o(sda_o), 
		.sda_padoen_o(sda_oen)
	);

	// hookup i2c slave model
	i2c_slave_model #(7'b1010_000) i2c_slave (
		.scl(scl),
		.sda(sda)
	);

	// create i2c lines
	assign scl = scl_oen ? 1'bz : scl_o; // create tri-state buffer for i2c_master scl line
	assign sda = sda_oen ? 1'bz : sda_o; // create tri-state buffer for i2c_master sda line

	pullup p1(scl); // pullup scl line
	pullup p2(sda); // pullup sda line

	initial
		begin
			// initially values
			clk = 0;

			// reset system
			rstn = 1'b1; // negate reset
			#2;
			rstn = 1'b0; // assert reset
			repeat(20) @(posedge clk);
			rstn = 1'b1; // negate reset
			
			@(posedge clk);

			//
			// program core
			//

			// program internal registers
			u0.wb_write(1, PRER_LO, 8'hfa); // load prescaler lo-byte
			u0.wb_write(1, PRER_HI, 8'h00); // load prescaler hi-byte

			u0.wb_write(1, CTR,     8'h80); // enable core

			//
			// access slave (write)
			//

			// drive slave address
			u0.wb_write(1, TXR,     8'ha0); // present slave address, set write-bit (== !read)
			u0.wb_write(0, CR,      8'h90); // set command (start, write)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(0, SR, q); // poll it until it is zero

			// send memory address
			u0.wb_write(1, TXR,     8'h01); // present slave's memory address
			u0.wb_write(0, CR,      8'h10); // set command (write)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(0, SR, q); // poll it until it is zero

			// send memory contents
			u0.wb_write(1, TXR,     8'ha5); // present data
			u0.wb_write(0, CR,      8'h10); // set command (stop, write)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(1, SR, q); // poll it until it is zero

			// send memory contents for next memory address (auto_inc)
			u0.wb_write(1, TXR,     8'h5a); // present data
			u0.wb_write(0, CR,      8'h50); // set command (stop, write)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(1, SR, q); // poll it until it is zero


			//
			// delay
			//
			#100000; // wait for 10us.

			//
			// access slave (read)
			//

			// drive slave address
			u0.wb_write(1, TXR,     8'ha0); // present slave address, set write-bit (== !read)
			u0.wb_write(0, CR,      8'h90); // set command (start, write)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(1, SR, q); // poll it until it is zero

			// send memory address
			u0.wb_write(1, TXR,     8'h00); // present slave's memory address
			u0.wb_write(0, CR,      8'h10); // set command (write)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(1, SR, q); // poll it until it is zero

			// drive slave address
			u0.wb_write(1, TXR,     8'ha1); // present slave's address, set read-bit
			u0.wb_write(0, CR,      8'h90); // set command (start, write)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(1, SR, q); // poll it until it is zero

			// read data from slave
			u0.wb_write(1, CR,      8'h20); // set command (read, ack_read)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(1, SR, q); // poll it until it is zero

			// read data from slave
			u0.wb_write(1, CR,      8'h20); // set command (read, ack_read)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(1, SR, q); // poll it until it is zero

			// read data from slave
			u0.wb_write(1, CR,      8'h20); // set command (read, ack_read)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(1, SR, q); // poll it until it is zero

			// read data from slave
			u0.wb_write(1, CR,      8'h28); // set command (read, nack_read)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(1, SR, q); // poll it until it is zero

			//
			// check invalid slave memory address
			//

			// drive slave address
			u0.wb_write(1, TXR,     8'ha0); // present slave address, set write-bit (== !read)
			u0.wb_write(0, CR,      8'h90); // set command (start, write)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(1, SR, q); // poll it until it is zero

			// send memory address
			u0.wb_write(1, TXR,     8'h10); // present slave's memory address
			u0.wb_write(0, CR,      8'h10); // set command (write)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(1, SR, q); // poll it until it is zero

			// slave should have send NACK
			if (!q[7])
				$display("\nERROR: Expected NACK, received ACK\n");

			// read data from slave
			u0.wb_write(1, CR,      8'h40); // set command (stop)

			// check tip bit
			u0.wb_read(1, SR, q);
			while (q[1])
				u0.wb_read(1, SR, q); // poll it until it is zero

		end

endmodule


