/////////////////////////////////////////////////////////////////////
////                                                             ////
////  WISHBONE rev.B2 compliant I2C Master bit-controller        ////
////                                                             ////
////                                                             ////
////  Author: Richard Herveille                                  ////
////          richard@asics.ws                                   ////
////          www.asics.ws                                       ////
////                                                             ////
////  Downloaded from: http://www.opencores.org/projects/i2c/    ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
////                                                             ////
//// Copyright (C) 2001 Richard Herveille                        ////
////                    richard@asics.ws                         ////
////                                                             ////
//// This source file may be used and distributed without        ////
//// restriction provided that this copyright statement is not   ////
//// removed from the file and that any derivative work contains ////
//// the original copyright notice and the associated disclaimer.////
////                                                             ////
////     THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY     ////
//// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   ////
//// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   ////
//// FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR      ////
//// OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         ////
//// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ////
//// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ////
//// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ////
//// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF  ////
//// LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT  ////
//// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  ////
//// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         ////
//// POSSIBILITY OF SUCH DAMAGE.                                 ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

//  CVS Log
//
//  $Id: i2c_master_bit_ctrl.v,v 1.4 2002-10-30 18:10:07 rherveille Exp $
//
//  $Date: 2002-10-30 18:10:07 $
//  $Revision: 1.4 $
//  $Author: rherveille $
//  $Locker:  $
//  $State: Exp $
//
// Change History:
//               $Log: not supported by cvs2svn $
//               Revision 1.3  2002/06/15 07:37:03  rherveille
//               Fixed a small timing bug in the bit controller.\nAdded verilog simulation environment.
//
//               Revision 1.2  2001/11/05 11:59:25  rherveille
//               Fixed wb_ack_o generation bug.
//               Fixed bug in the byte_controller statemachine.
//               Added headers.
//

//
/////////////////////////////////////
// Bit controller section
/////////////////////////////////////
//
// Translate simple commands into SCL/SDA transitions
// Each command has 5 states, A/B/C/D/idle
//
// start:	SCL	~~~~~~~~~~\____
//	SDA	~~~~~~~~\______
//		 x | A | B | C | D | i
//
// repstart	SCL	____/~~~~\___
//	SDA	__/~~~\______
//		 x | A | B | C | D | i
//
// stop	SCL	____/~~~~~~~~
//	SDA	==\____/~~~~~
//		 x | A | B | C | D | i
//
//- write	SCL	____/~~~~\____
//	SDA	==X=========X=
//		 x | A | B | C | D | i
//
//- read	SCL	____/~~~~\____
//	SDA	XXXX=====XXXX
//		 x | A | B | C | D | i
//

// Timing:     Normal mode      Fast mode
///////////////////////////////////////////////////////////////////////
// Fscl        100KHz           400KHz
// Th_scl      4.0us            0.6us   High period of SCL
// Tl_scl      4.7us            1.3us   Low period of SCL
// Tsu:sta     4.7us            0.6us   setup time for a repeated start condition
// Tsu:sto     4.0us            0.6us   setup time for a stop conditon
// Tbuf        4.7us            1.3us   Bus free time between a stop and start condition
//

`include "timescale.v"
`include "i2c_master_defines.v"

module i2c_master_bit_ctrl(clk, rst, nReset, clk_cnt, ena, cmd, cmd_ack, busy, din, dout, scl_i, scl_o, scl_oen, sda_i, sda_o, sda_oen);

	//
	// inputs & outputs
	//
	input clk;
	input rst;
	input nReset;
	input ena;            // core enable signal

	input [15:0] clk_cnt; // clock prescale value

	input  [3:0] cmd;
	output       cmd_ack;
	reg cmd_ack;
	output       busy;
	reg busy;

	input  din;
	output dout;
	reg dout;

	// I2C lines
	input  scl_i;    // i2c clock line input
	output scl_o;    // i2c clock line output
	output scl_oen;  // i2c clock line output enable (active low)
	reg scl_oen;
	input  sda_i;    // i2c data line input
	output sda_o;    // i2c data line output
	output sda_oen;  // i2c data line output enable (active low)
	reg sda_oen;


	//
	// variable declarations
	//

	reg sSCL, sSDA;             // synchronized SCL and SDA inputs
	reg dscl_oen;               // delayed scl_oen
	reg clk_en;                 // clock generation signals
	wire slave_wait;
//	reg [15:0] cnt = clk_cnt;   // clock divider counter (simulation)
	reg [15:0] cnt;             // clock divider counter (synthesis)

	//
	// module body
	//

	// synchronize SCL and SDA inputs
	always @(posedge clk)
	  begin
	      sSCL <= #1 scl_i;
	      sSDA <= #1 sda_i;
	  end

	// delay scl_oen
	always @(posedge clk)
	  dscl_oen <= #1 scl_oen;

	// whenever the slave is not ready it can delay the cycle by pulling SCL low
	assign slave_wait = dscl_oen && !sSCL;

	// generate clk enable signal
	always @(posedge clk or negedge nReset)
	  if(~nReset)
	    begin
	        cnt    <= #1 16'h0;
	        clk_en <= #1 1'b1;
	    end
	  else if (rst)
	    begin
	        cnt    <= #1 16'h0;
	        clk_en <= #1 1'b1;
	    end
	  else if ( !(|cnt) || !ena)
	    begin
	        cnt    <= #1 clk_cnt;
	        clk_en <= #1 1'b1;
	    end
	  else
	    begin
	        if(!slave_wait)
	          cnt <= #1 cnt - 16'h1;

	        clk_en <= #1 1'b0;
	    end


	// generate bus status controller
	reg dSDA;
	reg sta_condition;
	reg sto_condition;

	// detect start condition => detect falling edge on SDA while SCL is high
	// detect stop condition => detect rising edge on SDA while SCL is high
	always @(posedge clk)
	  begin
	      dSDA <= #1 sSDA; // generate a delayed version of sSDA

	      sta_condition <= #1 !sSDA &&  dSDA && sSCL;
	      sto_condition <= #1  sSDA && !dSDA && sSCL;
	  end

	// generate bus busy signal
	always @(posedge clk or negedge nReset)
	  if(!nReset)
	    busy <= #1 1'b0;
	  else if (rst)
	    busy <= #1 1'b0;
	  else
	    busy <= #1 (sta_condition || busy) && !sto_condition;


	// generate statemachine

	// nxt_state decoder
	parameter [16:0] idle    = 17'b0_0000_0000_0000_0000;
	parameter [16:0] start_a = 17'b0_0000_0000_0000_0001;
	parameter [16:0] start_b = 17'b0_0000_0000_0000_0010;
	parameter [16:0] start_c = 17'b0_0000_0000_0000_0100;
	parameter [16:0] start_d = 17'b0_0000_0000_0000_1000;
	parameter [16:0] start_e = 17'b0_0000_0000_0001_0000;
	parameter [16:0] stop_a  = 17'b0_0000_0000_0010_0000;
	parameter [16:0] stop_b  = 17'b0_0000_0000_0100_0000;
	parameter [16:0] stop_c  = 17'b0_0000_0000_1000_0000;
	parameter [16:0] stop_d  = 17'b0_0000_0001_0000_0000;
	parameter [16:0] rd_a    = 17'b0_0000_0010_0000_0000;
	parameter [16:0] rd_b    = 17'b0_0000_0100_0000_0000;
	parameter [16:0] rd_c    = 17'b0_0000_1000_0000_0000;
	parameter [16:0] rd_d    = 17'b0_0001_0000_0000_0000;
	parameter [16:0] wr_a    = 17'b0_0010_0000_0000_0000;
	parameter [16:0] wr_b    = 17'b0_0100_0000_0000_0000;
	parameter [16:0] wr_c    = 17'b0_1000_0000_0000_0000;
	parameter [16:0] wr_d    = 17'b1_0000_0000_0000_0000;

	reg [16:0] c_state, nxt_state; // synopsis enum_state
	reg icmd_ack, store_sda;

	always @(c_state or cmd)
	  begin
	      nxt_state  = c_state;
	      icmd_ack   = 1'b0; // default no command acknowledge
	      store_sda  = 1'b0;

	      case (c_state) // synopsis full_case parallel_case
	        // idle state
	        idle:
	          case (cmd) // synopsis full_case parallel_case
	            `I2C_CMD_START:
	               nxt_state = start_a;

	            `I2C_CMD_STOP:
	               nxt_state = stop_a;

	            `I2C_CMD_WRITE:
	               nxt_state = wr_a;

	            `I2C_CMD_READ:
	               nxt_state = rd_a;

	            default:
	              nxt_state = idle;

	          endcase

	        // start
	        start_a:
	          nxt_state = start_b;

	        start_b:
	          nxt_state = start_c;

	        start_c:
	          nxt_state = start_d;

	        start_d:
	          nxt_state = start_e;

	        start_e:
	          begin
	              nxt_state = idle;
	              icmd_ack  = 1'b1;
	          end

	        // stop
	        stop_a:
	          nxt_state = stop_b;

	        stop_b:
	          nxt_state = stop_c;

	        stop_c:
	          nxt_state = stop_d;

	        stop_d:
	          begin
	              nxt_state = idle;
	              icmd_ack  = 1'b1;
	          end

	        // read
	        rd_a:
	          nxt_state = rd_b;

	        rd_b:
	          nxt_state = rd_c;

	        rd_c:
	          begin
	              nxt_state = rd_d;
	              store_sda = 1'b1;
	          end

	        rd_d:
	          begin
	              nxt_state = idle;
	              icmd_ack  = 1'b1;
	          end

	        // write
	        wr_a:
	          nxt_state = wr_b;

	        wr_b:
	          nxt_state = wr_c;

	        wr_c:
	          nxt_state = wr_d;

	        wr_d:
	          begin
	              nxt_state = idle;
	              icmd_ack  = 1'b1;
	          end

	      endcase
	  end


	// generate registers
	always @(posedge clk or negedge nReset)
	  if (!nReset)
	    begin
	        c_state <= #1 idle;
	        cmd_ack <= #1 1'b0;
	        dout    <= #1 1'b0;
	    end
	  else if (rst)
	    begin
	        c_state <= #1 idle;
	        cmd_ack <= #1 1'b0;
	        dout    <= #1 1'b0;
	    end
	  else
	    begin
	        if (clk_en)
	          begin
	              c_state <= #1 nxt_state;
	              if (store_sda)
	                dout <= #1 sSDA;
	          end

	          cmd_ack <= #1 icmd_ack && clk_en;
	    end

	//
	// convert states to SCL and SDA signals
	//

	// assign scl and sda output (always gnd)
	assign scl_o = 1'b0;
	assign sda_o = 1'b0;

	// assign scl and sda output_enables
	always @(posedge clk or negedge nReset)
	  if (!nReset)
	    begin
	        scl_oen <= #1 1'b1;
	        sda_oen <= #1 1'b1;
	    end
	  else if (rst)
	    begin
	        scl_oen <= #1 1'b1;
	        sda_oen <= #1 1'b1;
	    end
	  else if (clk_en)
	    case (c_state) // synopsis full_case parallel_case

	      // idle state
	      idle:
	        begin
	            scl_oen <= #1 scl_oen; // keep SCL in same state
	            sda_oen <= #1 sda_oen; // keep SDA in same state
	        end

	      // start
	      start_a:
	        begin
	            scl_oen <= #1 scl_oen; // keep SCL in same state
	            sda_oen <= #1 1'b1;    // set SDA high
	        end

	      start_b:
	        begin
	            scl_oen <= #1 1'b1; // set SCL high
	            sda_oen <= #1 1'b1; // keep SDA high
	        end

	      start_c:
	        begin
	            scl_oen <= #1 1'b1; // keep SCL high
	            sda_oen <= #1 1'b0; // set SDA low
	        end

	      start_d:
	        begin
	            scl_oen <= #1 1'b1; // keep SCL high
	            sda_oen <= #1 1'b0; // keep SDA low
	        end

	      start_e:
	        begin
	            scl_oen <= #1 1'b0; // set SCL low
	            sda_oen <= #1 1'b0; // keep SDA low
	        end

	      // stop
	      stop_a:
	        begin
	            scl_oen <= #1 1'b0; // keep SCL low
	            sda_oen <= #1 1'b0; // set SDA low
	        end

	      stop_b:
	        begin
	            scl_oen <= #1 1'b1; // set SCL high
	            sda_oen <= #1 1'b0; // keep SDA low
	        end

	      stop_c:
	        begin
	            scl_oen <= #1 1'b1; // keep SCL high
	            sda_oen <= #1 1'b0; // keep SDA low
	        end

	      stop_d:
	        begin
	            scl_oen <= #1 1'b1; // keep SCL high
	            sda_oen <= #1 1'b1; // set SDA high
	        end

	      //write
	      wr_a:
	        begin
	            scl_oen <= #1 1'b0; // keep SCL low
	            sda_oen <= #1 din;  // set SDA
	        end

	      wr_b:
	        begin
	            scl_oen <= #1 1'b1; // set SCL high
	            sda_oen <= #1 din;  // keep SDA
	        end

	      wr_c:
	        begin
	            scl_oen <= #1 1'b1; // keep SCL high
	            sda_oen <= #1 din;
	        end

	      wr_d:
	        begin
	            scl_oen <= #1 1'b0; // set SCL low
	            sda_oen <= #1 din;
	        end

	      // read
	      rd_a:
	        begin
	            scl_oen <= #1 1'b0; // keep SCL low
	            sda_oen <= #1 1'b1; // tri-state SDA
	        end

	      rd_b:
	        begin
	            scl_oen <= #1 1'b1; // set SCL high
	            sda_oen <= #1 1'b1; // keep SDA tri-stated
	        end

	      rd_c:
	        begin
	            scl_oen <= #1 1'b1; // keep SCL high
	            sda_oen <= #1 1'b1;
	        end

	      rd_d:
	        begin
	            scl_oen <= #1 1'b0; // set SCL low
	            sda_oen <= #1 1'b1;
	        end

	endcase

endmodule
