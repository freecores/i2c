---------------------------------------------------------------------
----                                                             ----
----  WISHBONE revB2 compl. I2C Master Core; bit-controller      ----
----                                                             ----
----                                                             ----
----  Author: Richard Herveille                                  ----
----          richard@asics.ws                                   ----
----          www.asics.ws                                       ----
----                                                             ----
----  Downloaded from: http://www.opencores.org/projects/i2c/    ----
----                                                             ----
---------------------------------------------------------------------
----                                                             ----
---- Copyright (C) 2000 Richard Herveille                        ----
----                    richard@asics.ws                         ----
----                                                             ----
---- This source file may be used and distributed without        ----
---- restriction provided that this copyright statement is not   ----
---- removed from the file and that any derivative work contains ----
---- the original copyright notice and the associated disclaimer.----
----                                                             ----
----     THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY     ----
---- EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   ----
---- TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   ----
---- FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR      ----
---- OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         ----
---- INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ----
---- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ----
---- GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ----
---- BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF  ----
---- LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT  ----
---- (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  ----
---- OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         ----
---- POSSIBILITY OF SUCH DAMAGE.                                 ----
----                                                             ----
---------------------------------------------------------------------

--  CVS Log
--
--  $Id: i2c_master_bit_ctrl.vhd,v 1.3 2002-10-30 18:09:53 rherveille Exp $
--
--  $Date: 2002-10-30 18:09:53 $
--  $Revision: 1.3 $
--  $Author: rherveille $
--  $Locker:  $
--  $State: Exp $
--
-- Change History:
--               $Log: not supported by cvs2svn $
--               Revision 1.2  2002/06/15 07:37:04  rherveille
--               Fixed a small timing bug in the bit controller.\nAdded verilog simulation environment.
--
--               Revision 1.1  2001/11/05 12:02:33  rherveille
--               Split i2c_master_core.vhd into separate files for each entity; same layout as verilog version.
--               Code updated, is now up-to-date to doc. rev.0.4.
--               Added headers.
--


--
-------------------------------------
-- Bit controller section
------------------------------------
--
-- Translate simple commands into SCL/SDA transitions
-- Each command has 5 states, A/B/C/D/idle
--
-- start:	SCL	~~~~~~~~~~\____
--	SDA	~~~~~~~~\______
--		 x | A | B | C | D | i
--
-- repstart	SCL	____/~~~~\___
--	SDA	__/~~~\______
--		 x | A | B | C | D | i
--
-- stop	SCL	____/~~~~~~~~
--	SDA	==\____/~~~~~
--		 x | A | B | C | D | i
--
--- write	SCL	____/~~~~\____
--	SDA	==X=========X=
--		 x | A | B | C | D | i
--
--- read	SCL	____/~~~~\____
--	SDA	XXXX=====XXXX
--		 x | A | B | C | D | i
--

-- Timing:      Normal mode     Fast mode
-----------------------------------------------------------------
-- Fscl         100KHz          400KHz
-- Th_scl       4.0us           0.6us   High period of SCL
-- Tl_scl       4.7us           1.3us   Low period of SCL
-- Tsu:sta      4.7us           0.6us   setup time for a repeated start condition
-- Tsu:sto      4.0us           0.6us   setup time for a stop conditon
-- Tbuf         4.7us           1.3us   Bus free time between a stop and start condition
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

entity i2c_master_bit_ctrl is
	generic(
		Tcq : time := 1 ns
	);
	port (
		clk    : in std_logic;
		rst    : in std_logic;
		nReset : in std_logic;
		ena    : in std_logic;				-- core enable signal

		clk_cnt : in unsigned(15 downto 0);		-- clock prescale value

		cmd     : in std_logic_vector(3 downto 0);
		cmd_ack : out std_logic;
		busy    : out std_logic;

		din  : in std_logic;
		dout : out std_logic;

		-- i2c lines
		scl_i   : in std_logic;  -- i2c clock line input
		scl_o   : out std_logic; -- i2c clock line output
		scl_oen : out std_logic; -- i2c clock line output enable, active low
		sda_i   : in std_logic;  -- i2c data line input
		sda_o   : out std_logic; -- i2c data line output
		sda_oen : out std_logic  -- i2c data line output enable, active low
	);
end entity i2c_master_bit_ctrl;

architecture structural of i2c_master_bit_ctrl is
	constant I2C_CMD_NOP    : std_logic_vector(3 downto 0) := "0000";
	constant I2C_CMD_START  : std_logic_vector(3 downto 0) := "0001";
	constant I2C_CMD_STOP   : std_logic_vector(3 downto 0) := "0010";
	constant I2C_CMD_READ   : std_logic_vector(3 downto 0) := "0100";
	constant I2C_CMD_WRITE  : std_logic_vector(3 downto 0) := "1000";

	type states is (idle, start_a, start_b, start_c, start_d, start_e, 
	                stop_a, stop_b, stop_c, stop_d, rd_a, rd_b, rd_c, rd_d, wr_a, wr_b, wr_c, wr_d);
	signal c_state : states;

	signal iscl_oen, isda_oen : std_logic;          -- internal I2C lines
	signal sSCL, sSDA         : std_logic;          -- synchronized SCL and SDA inputs
	signal dscl_oen           : std_logic;          -- delayed scl_oen signals

	signal clk_en, slave_wait :std_logic;           -- clock generation signals
--	signal cnt : unsigned(15 downto 0) := clk_cnt;  -- clock divider counter (simulation)
	signal cnt : unsigned(15 downto 0);             -- clock divider counter (synthesis)

begin
	-- synchronize SCL and SDA inputs
	synch_scl_sda: process(clk)
	begin
	    if (clk'event and clk = '1') then
	      sSCL <= scl_i after Tcq;
	      sSDA <= sda_i after Tcq;
	    end if;
	end process synch_SCL_SDA;

	-- delay scl_oen
	process (clk)
	begin
	    if (clk'event and clk = '1') then
	      dscl_oen <= iscl_oen after Tcq;
	    end if;
	end process;

	-- whenever the slave is not ready it can delay the cycle by pulling SCL low
	slave_wait <= dscl_oen and not sSCL;

	-- generate clk enable signal
	gen_clken: process(clk, nReset)
	begin
	    if (nReset = '0') then
	      cnt    <= (others => '0') after Tcq;
	      clk_en <= '1' after Tcq;
	    elsif (clk'event and clk = '1') then
	      if (rst = '1') then
	        cnt    <= (others => '0') after Tcq;
	        clk_en <= '1' after Tcq;
	      else
	        if ( (cnt = 0) or (ena = '0') ) then
	          clk_en <= '1' after Tcq;
	          cnt    <= clk_cnt after Tcq;
	        else
	          if (slave_wait = '0') then
	            cnt <= cnt -1 after Tcq;
	          end if;
	          clk_en <= '0' after Tcq;
	        end if;
	      end if;
	    end if;
	end process gen_clken;


	-- generate bus status controller
	bus_status_ctrl: block
	  signal dSDA : std_logic;
	  signal sta_condition : std_logic;
	  signal sto_condition : std_logic;

	  signal ibusy : std_logic;
	begin
	    -- detect start condition => detect falling edge on SDA while SCL is high
	    -- detect stop condition  => detect rising edge on SDA while SCL is high
	    detect_sta_sto: process(clk)
	    begin
	        if (clk'event and clk = '1') then
	          dSDA <= sSDA; -- generate a delayed version of sSDA

	          sta_condition <= (not sSDA and dSDA) and sSCL;
	          sto_condition <= (sSDA and not dSDA) and sSCL;
	        end if;
	    end process detect_sta_sto;

	    -- generate bus busy signal
	    gen_busy: process(clk, nReset)
	    begin
	        if (nReset = '0') then
	          ibusy <= '0' after Tcq;
	        elsif (clk'event and clk = '1') then
	          if (rst = '1') then
	            ibusy <= '0' after Tcq;
	          else
	            ibusy <= (sta_condition or ibusy) and not sto_condition after Tcq;
	          end if;
	        end if;
	    end process gen_busy;

	    -- assign output
	    busy <= ibusy;
	end block bus_status_ctrl;


	-- generate statemachine
	nxt_state_decoder : process (clk, nReset, c_state, cmd)
	  variable nxt_state : states;
	  variable icmd_ack, store_sda : std_logic;
	begin
	    nxt_state := c_state;

	    icmd_ack := '0'; -- default no acknowledge

	    store_sda := '0';

	    case (c_state) is
	      -- idle
	      when idle =>
	        case cmd is
	          when I2C_CMD_START =>
	            nxt_state := start_a;

	          when I2C_CMD_STOP =>
	            nxt_state := stop_a;

	          when I2C_CMD_WRITE =>
	            nxt_state := wr_a;

	          when I2C_CMD_READ =>
	            nxt_state := rd_a;

	          when others =>  -- NOP command
	            nxt_state := idle;
	        end case;

	      -- start
	      when start_a =>
	        nxt_state := start_b;

	      when start_b =>
	        nxt_state := start_c;

	      when start_c =>
	        nxt_state := start_d;

	      when start_d =>
	        nxt_state := start_e;

	      when start_e =>
	        nxt_state := idle;
	        icmd_ack := '1'; -- command completed

	      -- stop
	      when stop_a =>
	        nxt_state := stop_b;

	      when stop_b =>
	        nxt_state := stop_c;

	      when stop_c =>
	        nxt_state := stop_d;

	      when stop_d =>
	        nxt_state := idle;
	        icmd_ack := '1'; -- command completed

	      -- read
	      when rd_a =>
	        nxt_state := rd_b;

	      when rd_b =>
	        nxt_state := rd_c;

	      when rd_c =>
	        nxt_state := rd_d;
	        store_sda := '1';

	      when rd_d =>
	        nxt_state := idle;
	        icmd_ack := '1'; -- command completed

	      -- write
	      when wr_a =>
	        nxt_state := wr_b;

	      when wr_b =>
	        nxt_state := wr_c;

	      when wr_c =>
	        nxt_state := wr_d;

	      when wr_d =>
	        nxt_state := idle;
	        icmd_ack := '1'; -- command completed

	    end case;

	    -- generate regs
	    if (nReset = '0') then
	      c_state <= idle after Tcq;
	      cmd_ack <= '0' after Tcq;
	      Dout    <= '0' after Tcq;
	    elsif (clk'event and clk = '1') then
	      if (rst = '1') then
	        c_state <= idle after Tcq;
	        cmd_ack <= '0' after Tcq;
	        Dout    <= '0' after Tcq;
	      elsif (clk_en = '1') then
	        c_state <= nxt_state after Tcq;

	        if (store_sda = '1') then
	          dout <= sSDA after Tcq;
	        end if;
	      end if;

	      cmd_ack <= icmd_ack and clk_en;
	    end if;
	end process nxt_state_decoder;

	--
	-- convert states to SCL and SDA signals
	--
	output_decoder: process (clk, nReset, c_state, iscl_oen, isda_oen, din)
	  variable iscl, isda : std_logic;
	begin
	    case (c_state) is
	      -- idle
	      when idle =>
	        iscl := iscl_oen; -- keep SCL in same state
	        isda := isda_oen; -- keep SDA in same state

	      -- start
	      when start_a =>
	        iscl := iscl_oen; -- keep SCL in same state (for repeated start)
	        isda := '1';      -- set SDA high

	      when start_b =>
	        iscl := '1'; -- set SCL high
	        isda := '1'; -- keep SDA high

	      when start_c =>
	        iscl := '1'; -- keep SCL high
	        isda := '0'; -- set SDA low

	      when start_d =>
	        iscl := '1'; -- keep SCL high
	        isda := '0'; -- keep SDA low

	      when start_e =>
	        iscl := '0'; -- set SCL low
	        isda := '0'; -- keep SDA low

	      -- stop
	      when stop_a =>
	        iscl := '0'; -- keep SCL disabled
	        isda := '0'; -- set SDA low

	      when stop_b =>
	         iscl := '1'; -- set SCL high
	         isda := '0'; -- keep SDA low

	      when stop_c =>
	         iscl := '1'; -- keep SCL high
	         isda := '0'; -- keep SDA low

	      when stop_d =>
	        iscl := '1'; -- keep SCL high
	        isda := '1'; -- set SDA high

	      -- write
	      when wr_a =>
	        iscl := '0'; -- keep SCL low
	        isda := din; -- set SDA

	      when wr_b =>
	        iscl := '1'; -- set SCL high
	        isda := din; -- keep SDA

	      when wr_c =>
	        iscl := '1'; -- keep SCL high
	        isda := din; -- keep SDA

	      when wr_d =>
	        iscl := '0'; -- set SCL low
	        isda := din; -- keep SDA

	      -- read
	      when rd_a =>
	        iscl := '0'; -- keep SCL low
	        isda := '1'; -- tri-state SDA

	      when rd_b =>
	        iscl := '1'; -- set SCL high
	        isda := '1'; -- tri-state SDA

	      when rd_c =>
	        iscl := '1'; -- keep SCL high
	        isda := '1'; -- tri-state SDA

	      when rd_d =>
	        iscl := '0'; -- set SCL low
	        isda := '1'; -- tri-state SDA
	    end case;

	    -- generate registers
	    if (nReset = '0') then
	      iscl_oen <= '1' after Tcq;
	      isda_oen <= '1' after Tcq;
	    elsif (clk'event and clk = '1') then
	      if (rst = '1') then
	        iscl_oen <= '1' after Tcq;
	        isda_oen <= '1' after Tcq;
	      elsif (clk_en = '1') then
	        iscl_oen <= iscl after Tcq;
	        isda_oen <= isda after Tcq;
	      end if;
	    end if;
	end process output_decoder;

	-- assign outputs
	scl_o   <= '0';
	scl_oen <= iscl_oen;
	sda_o   <= '0';
	sda_oen <= isda_oen;
end architecture structural;
