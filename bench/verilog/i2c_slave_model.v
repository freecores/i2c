//
// I2C slave model
//

`include "timescale.v"

module i2c_slave_model (scl, sda);

	//
	// parameters
	//
	parameter I2C_ADR = 7'b001_0000;

	//
	// input && outpus
	//
	input scl;
	inout sda;

	//
	// Variable declaration
	//
	reg [7:0] mem [3:0]; // initiate memory
	reg [7:0] mem_adr;   // memory address
	reg [7:0] mem_do;    // memory data output

	reg sta, d_sta;
	reg sto, d_sto;

	reg [7:0] sr;        // 8bit shift register
	reg       rw;        // read/write direction

	wire      my_adr;    // my address called ??
	wire      i2c_reset; // i2c-statemachine reset
	reg [2:0] bit_cnt;   // 3bit downcounter
	wire      acc_done;  // 8bits transfered
	reg       ld;        // load downcounter

	reg       sda_o;     // sda-drive level

	// statemachine declaration
	parameter idle        = 3'b000;
	parameter slave_ack   = 3'b001;
	parameter get_mem_adr = 3'b010;
	parameter gma_ack     = 3'b011;
	parameter data        = 3'b100;
	parameter data_ack    = 3'b101;

	reg [2:0] state; // synopsys enum_state

	//
	// module body
	//

	initial
		begin
			sda_o = 1'b1;
			state = idle;
		end

	// generate shift register
	always@(posedge scl)
		sr <= #1 {sr[6:0],sda};

	//detect my_address
	assign my_adr = (sr[7:1] == I2C_ADR);

	//generate bit-counter
	always@(posedge scl)
		if (ld)
			bit_cnt <= #1 3'b111;
		else
			bit_cnt <= #1 bit_cnt - 3'h1;
	
	//generate access done signal
	assign acc_done = !(|bit_cnt);

	//detect start condition
	always@(negedge sda)
		if (scl)
			sta <= #1 1'b1;
		else
			sta <= #1 1'b0;

	always@(posedge scl)
		d_sta <= #1 sta;

	// detect stop condition
	always@(posedge sda)
		if (scl)
			sto <= #1 1'b1;
		else
			sto <= #1 1'b0;

	//generate i2c_reset signal
	assign i2c_reset = sta || sto;

	// generate statemachine
	always@(negedge scl or posedge sto)
		if (sto || (sta && !d_sta) )
			begin
				state <= #1 idle; // reset statemachine

				sda_o <= #1 1'b1;
				ld    <= #1 1'b1;
			end
		else
			begin
				// initial settings
				sda_o <= #1 1'b1;
				ld    <= #1 1'b0;
		
				case (state) // synopsys full_case parallel_case
					idle: // idle state
						if (acc_done && my_adr)
								begin
									state <= #1 slave_ack;
									rw <= #1 sr[0];

									sda_o <= #1 1'b0; // generate i2c_ack
								end

					slave_ack:
						begin
							if (rw)
								begin
									state <= #1 data;
									sda_o <= #1 mem_do[7];
								end
							else
								state <= #1 get_mem_adr;

							ld    <= #1 1'b1;
						end

					get_mem_adr: // wait for memory address
						if (acc_done)
							begin
								state <= #1 gma_ack;
								mem_adr <= #1 sr; // store memory address

								sda_o <= #1 !(sr <= 15); // generate i2c_ack, for valid address
							end

					gma_ack:
						begin
							state <= #1 data;
							ld    <= #1 1'b1;
						end

					data: // receive or drive data
						begin
							if (rw)
								sda_o <= #1 mem_do[7];

							if (acc_done)
								begin
									state <= #1 data_ack;

									mem_adr <= #1 mem_adr + 8'h1;

									if (!rw)
										mem[ mem_adr[3:0] ] <= #1 sr; // store data in memory
		
									sda_o <= #1 (rw && (mem_adr <= 15) ); // send ack on write, receive ack on read
								end
						end

					data_ack:
						begin
							ld    <= #1 1'b1;

							if (rw)
								if (sda) // read operation && master send NACK
									begin
										state <= #1 idle;
										sda_o <= #1 1'b1;
									end
								else
									begin
										state <= #1 data;
										sda_o <= #1 mem_do[7];
									end
							else
								begin
									state <= #1 data;
									sda_o <= #1 1'b1;
								end
						end

				endcase
			end

	// read data from memory
	always@(posedge scl)
		if (acc_done)
			mem_do <= #1 mem[mem_adr];
		else
			mem_do <= #1 {mem_do[6:0], 1'b1}; // insert 1'b1 for host ack generation

	// generate tri-states
	assign sda = sda_o ? 1'bz : 1'b0;
	
endmodule





