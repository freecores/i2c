--
-- WISHBONE revB2 compiant I2C master core
--
-- author: Richard Herveille
-- rev. 0.1 based on simple_i2c
-- rev. 0.2 april 27th 2001, fixed incomplete sensitivity list on assign_dato process (thanks to Matt Oseman)
-- rev. 0.3 may 4th 2001, fixed typo rev.0.2 txt -> txr
--
-- Changes compared to simple_i2c
-- 1) WISHBONE interface
-- 2) added start/stop detection
-- 3) added busy bit
-- 4) removed automatic tri-state buffer insertion (for ASIC support)
--



library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

entity wishbone_i2c_master is
	port (
		-- wishbone signals
		CLK_I : in std_logic;				-- master clock input
		RST_I : in std_logic := '0';			-- synchronous active high reset
		nRESET: in std_logic := '1';			-- asynchronous active low reset
		ADR_I : in unsigned(1 downto 0);		-- lower address bits
		DAT_I : in std_logic_vector(15 downto 0);	-- Databus input
		DAT_O : out std_logic_vector(15 downto 0);	-- Databus output
		SEL_I : in std_logic_vector(1 downto 0);		-- Byte select signals
		WE_I : in std_logic;				-- Write enable input
		STB_I : in std_logic;				-- Strobe signals / core select signal
		CYC_I : in std_logic;				-- Valid bus cycle input
		ACK_O : out std_logic;			-- Bus cycle acknowledge output
		INTA_O : out std_logic;			-- interrupt request output signal

		-- I2C signals
		SCLi : in std_logic;				-- I2C clock line
		SCLo : out std_logic;
		SDAi : in std_logic;				-- I2C data line
		SDAo : out std_logic
	);
end entity wishbone_i2c_master;

architecture structural of wishbone_i2c_master is
	component byte_ctrl is
	port (
		clk : in std_logic;
		rst : in std_logic;		-- synchronous active high reset (WISHBONE compatible)
		nReset : in std_logic;	-- asynchornous active low reset (FPGA compatible)

		clk_cnt : in unsigned(15 downto 0);	-- 4x SCL 

		-- input signals
		ena,
		start,
		stop,
		read,
		write,
		ack_in : std_logic;
		Din : in std_logic_vector(7 downto 0);

		-- output signals
		cmd_ack : out std_logic;
		ack_out : out std_logic;
		Dout : out std_logic_vector(7 downto 0);
		i2c_busy : out std_logic;

		-- i2c signals
		SCLi : in std_logic;				-- I2C clock line
		SCLo : out std_logic;
		SDAi : in std_logic;				-- I2C data line
		SDAo : out std_logic
	);
	end component byte_ctrl;

	-- registers
	signal prer : unsigned(15 downto 0);			-- clock prescale register
	signal ctr : std_logic_vector(7 downto 0);			-- control register
	signal txr : std_logic_vector(7 downto 0);			-- transmit register
	signal rxr : std_logic_vector(7 downto 0);			-- receive register
	signal cr : std_logic_vector(7 downto 0);			-- command register
	signal sr : std_logic_vector(7 downto 0);			-- status register

	-- done signal: command completed, clear command register
	signal done : std_logic;

	-- command register signals
	signal sta, sto, rd, wr, ack, iack : std_logic;

	-- core enable signal
	signal core_en : std_logic;

	-- status register signals
	signal irxack, rxack : std_logic; 			-- received aknowledge from slave
	signal tip : std_logic;					-- transfer in progress
	signal irq_flag : std_logic;				-- interrupt pending flag
	signal i2c_busy : std_logic;				-- bus busy (start signal detected)

begin
	-- generate acknowledge output signal
	ACK_O <= STB_I;	-- since timing is always honored 


	-- assign DAT_O
	assign_dato : process(ADR_I, prer, ctr, txr, cr, rxr, sr)
	begin
		case ADR_I is
			when "00" =>
				DAT_O <= std_logic_vector(prer);

			when "01" =>
				DAT_O <= (x"00" & ctr);

			when "10" =>
				DAT_O <= (txr & cr);

			when "11" =>
				DAT_O <= (rxr & sr);

			when others =>
				DAT_O <= (others => 'X');	-- for simulation only
		end case;
	end process assign_dato;


	-- registers block
	regs_block: block
		-- address decode signals
		signal we_a0, we_a1, we_a2, we_a3 : std_logic;
	begin
		-- decode address lines
		we_a0 <= CYC_I and STB_I and WE_I and not ADR_I(1) and not ADR_I(0);
		we_a1 <= CYC_I and STB_I and WE_I and not ADR_I(1) and       ADR_I(0);
		we_a2 <= CYC_I and STB_I and WE_I and       ADR_I(1) and not ADR_I(0);
		we_a3 <= CYC_I and STB_I and WE_I and       ADR_I(1) and       ADR_I(0);

		-- store data in writeable registers
		-- prescale register
		write_prer: process(nRESET, CLK_I)
		begin
			if (nRESET = '0') then
					prer <= (others => '1');
			elsif (CLK_I'event and CLK_I = '1') then
				if (RST_I = '1') then
					prer <= (others => '1');
				else
					if ( (we_a0 and SEL_I(1)) = '1') then
						prer(15 downto 8) <= unsigned(DAT_I(15 downto 8));
					end if;

					if ( (we_a0 and SEL_I(0)) = '1') then
						prer(7 downto 0) <= unsigned(DAT_I(7 downto 0));
					end if;
				end if;
			end if;
		end process write_prer;


		-- control register
		write_ctr: process(nRESET, CLK_I)
		begin
			if (nRESET = '0') then
					ctr <= (others => '0');
			elsif (CLK_I'event and CLK_I = '1') then
				if (RST_I = '1') then
					ctr <= (others => '0');
				else
					if ( (we_a1 and SEL_I(0)) = '1') then
						ctr <= DAT_I(7 downto 0);
					end if;
				end if;
			end if;
		end process write_ctr;

		-- transmit register
		write_txr: process(nRESET, CLK_I)
		begin
			if (nRESET = '0') then
					txr <= (others => '0');
			elsif (CLK_I'event and CLK_I = '1') then
				if (RST_I = '1') then
					txr <= (others => '0');
				else
					if ( (we_a2 and SEL_I(1)) = '1') then
						txr <= DAT_I(15 downto 8);
					end if;
				end if;
			end if;
		end process write_txr;


		-- command register
		write_cr: process(nRESET, CLK_I)
		begin
			if (nRESET = '0') then
					cr <= (others => '0');					-- asynchronous clear
			elsif (CLK_I'event and CLK_I = '1') then
				if (RST_I = '1') then
					cr <= (others => '0');					-- synchronous clear
				else

					if ( (we_a2 and SEL_I(0)) = '1') then
						if (core_en = '1') then
							cr <= DAT_I(7 downto 0);		-- only take new commands when I2C core is enabled, pending commands are finished
						end if;
					else
						if (done = '0') then
							cr(7 downto 4) <= cr(7 downto 4);
						else
							cr(7 downto 0) <= (others => '0'); 	-- clear command_bits when command completed
						end if;
						cr(2 downto 1) <= cr(2 downto 1);
						cr(0) <= cr(0) and irq_flag; 			-- automatically clear when irq_flag is cleared
					end if;
				end if;
			end if;
		end process write_cr;
	end block regs_block;

		
	-- decode command register
	sta <= cr(7);
	sto <= cr(6);
	rd <= cr(5);
	wr <= cr(4);
	ack <= cr(3);
	iack <= cr(0);

	-- decode control register
	core_en <= ctr(7);

	-- hookup byte controller block
	u1: byte_ctrl port map (clk => CLK_I, rst => RST_I, nReset => nRESET,  clk_cnt => prer, ena => core_en, 
		start => sta,  stop => sto, read => rd, write => wr, ack_in => ack, i2c_busy => i2c_busy,
		Din => txr, cmd_ack => done, ack_out => irxack, Dout => rxr, 			-- note: maybe store rxr in registers ??
		SCLi => SCLi, SCLo => SCLo, SDAi => SDAi, SDAo => SDAo);


	-- status register block + interrupt request signal
	st_block : block
	begin
		-- generate status register bits
		gen_sr_bits: process (CLK_I, nRESET)
		begin
			if (nRESET = '0') then
				rxack <= '0';
				tip <= '0';
				irq_flag <= '0';
			elsif (CLK_I'event and CLK_I = '1') then
				if (RST_I = '1') then
					rxack <= '0';
					tip <= '0';
					irq_flag <= '0';
				else
					rxack <= irxack;
					tip <= ( rd or wr );
					irq_flag <= (done or irq_flag) and not iack;	-- interrupt request flag is always generated
				end if;
			end if;
		end process gen_sr_bits;

		-- generate interrupt request signals
		gen_irq: process (CLK_I, nRESET)
		begin
			if (nRESET = '0') then
				INTA_O <= '0';
			elsif (CLK_I'event and CLK_I = '1') then
				if (RST_I = '1') then
					INTA_O <= '0';
				else
					INTA_O <= irq_flag and ctr(6);			-- interrupt signal is only generated when IEN (interrupt enable bit) is set
				end if;
			end if;
		end process gen_irq;

		-- assign status register bits
		sr(7) <= rxack;
		sr(6) <= i2c_busy;
		sr(5 downto 2) <= (others => '0'); -- reserved
		sr(1) <= tip;
		sr(0) <= irq_flag;
	end block;

end architecture structural;


--
------------------------------------------
-- Byte controller section
------------------------------------------
--
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

entity byte_ctrl is
	port (
		clk : in std_logic;
		rst : in std_logic;		-- synchronous active high reset (WISHBONE compatible)
		nReset : in std_logic;	-- asynchornous active low reset (FPGA compatible)

		clk_cnt : in unsigned(15 downto 0);	-- 4x SCL 

		-- input signals
		ena,
		start,
		stop,
		read,
		write,
		ack_in : std_logic;
		Din : in std_logic_vector(7 downto 0);

		-- output signals
		cmd_ack : out std_logic;
		ack_out : out std_logic;
		Dout : out std_logic_vector(7 downto 0);
		i2c_busy : out std_logic;

		-- i2c signals
		SCLi : in std_logic;				-- I2C clock line
		SCLo : out std_logic;
		SDAi : in std_logic;				-- I2C data line
		SDAo : out std_logic
	);
end entity byte_ctrl;

architecture structural of byte_ctrl is
	component bit_ctrl is
	port (
		clk : in std_logic;
		rst : in std_logic;
		nReset : in std_logic;

		clk_cnt : in unsigned(15 downto 0);		-- clock prescale value

		ena : in std_logic;				-- core enable signal
		cmd : in std_logic_vector(2 downto 0);
		cmd_ack : out std_logic;
		busy : out std_logic;

		Din : in std_logic;
		Dout : out std_logic;

		SCLin : in std_logic;				-- I2C clock line
		SCLout : out std_logic;
		SDAin : in std_logic;				-- I2C data line
		SDAout : out std_logic
	);
	end component bit_ctrl;

	-- commands for i2c_core
	constant CMD_NOP	: std_logic_vector(2 downto 0) := "000";
	constant CMD_START	: std_logic_vector(2 downto 0) := "010";
	constant CMD_STOP	: std_logic_vector(2 downto 0) := "011";
	constant CMD_READ	: std_logic_vector(2 downto 0) := "100";
	constant CMD_WRITE	: std_logic_vector(2 downto 0) := "101";

	-- signals for bit_controller
	signal core_cmd : std_logic_vector(2 downto 0);
	signal core_ack, core_txd, core_rxd : std_logic;

	-- signals for shift register
	signal sr : std_logic_vector(7 downto 0); -- 8bit shift register
	signal shift, ld : std_logic;

	-- signals for state machine
	signal go, host_ack : std_logic;
begin
	-- hookup bit_controller
	u1: bit_ctrl port map (clk, rst, nReset, clk_cnt, ena, core_cmd, core_ack, i2c_busy, core_txd, core_rxd, SCLi, SCLo, SDAi, SDAo);

	-- generate host-command-acknowledge
	cmd_ack <= host_ack;
	
	-- generate go-signal
	go <= (read or write) and not host_ack;

	-- assign Dout output to shift-register
	Dout <= sr;

	-- assign ack_out output to core_rxd (contains last received bit)
	ack_out <= core_rxd;

	-- generate shift register
	shift_register: process(clk)
	begin
		if (clk'event and clk = '1') then
			if (ld = '1') then
				sr <= din;
			elsif (shift = '1') then
				sr <= (sr(6 downto 0) & core_rxd);
			end if;
		end if;
	end process shift_register;

	--
	-- state machine
	--
	statemachine : block
		type states is (st_idle, st_start, st_read, st_write, st_ack, st_stop);
		signal state : states;
		signal dcnt : unsigned(2 downto 0);
	begin
		--
		-- command interpreter, translate complex commands into simpler I2C commands
		--
		nxt_state_decoder: process(clk, nReset, state)
			variable nxt_state : states;
			variable idcnt : unsigned(2 downto 0);
			variable ihost_ack : std_logic;
			variable icore_cmd : std_logic_vector(2 downto 0);
			variable icore_txd : std_logic;
			variable ishift, iload : std_logic;
		begin
			-- 8 databits (1byte) of data to shift-in/out
			idcnt := dcnt;

			-- no acknowledge (until command complete)
			ihost_ack := '0';

			icore_txd := core_txd;

			-- keep current command to bit_controller
			icore_cmd := core_cmd;

			-- no shifting or loading of shift-register
			ishift := '0';
			iload := '0';

			-- keep current state;
			nxt_state := state;
			case state is
				when st_idle =>
					if (go = '1') then
						if (start = '1') then
							nxt_state := st_start;	
							icore_cmd := CMD_START;
						elsif (read = '1') then
							nxt_state := st_read;
							icore_cmd := CMD_READ;
							idcnt := "111";
						else
							nxt_state := st_write;
							icore_cmd := CMD_WRITE;
							idcnt := "111";
							iload := '1';
						end if;
					end if;

				when st_start =>
					if (core_ack = '1') then
						if (read = '1') then
							nxt_state := st_read;
							icore_cmd := CMD_READ;
							idcnt := "111";
						else
							nxt_state := st_write;
							icore_cmd := CMD_WRITE;
							idcnt := "111";
							iload := '1';
						end if;
					end if;

				when st_write =>
					if (core_ack = '1') then
						idcnt := dcnt -1;	-- count down Data_counter
						icore_txd := sr(7);
						if (dcnt = 0) then
							nxt_state := st_ack;
							icore_cmd := CMD_READ;
						else
							ishift := '1';
--							icore_txd := sr(7);
						end if;
					end if;			

				when st_read =>
					if (core_ack = '1') then
						idcnt := dcnt -1;	-- count down Data_counter
						ishift := '1';
						if (dcnt = 0) then
							nxt_state := st_ack;
							icore_cmd := CMD_WRITE;
							icore_txd := ack_in;
						end if;
					end if;			

				when st_ack =>
					if (core_ack = '1') then
						-- generate command acknowledge signal
						ihost_ack := '1';

						-- Perform an additional shift, needed for 'read' (store last received bit in shift register)
						ishift := '1';

						-- check for stop; Should a STOP command be generated ?
						if (stop = '1') then
							nxt_state := st_stop;
							icore_cmd := CMD_STOP;
						else
							nxt_state := st_idle;
							icore_cmd := CMD_NOP;
						end if;
					end if;

				when st_stop =>
					if (core_ack = '1') then
						nxt_state := st_idle;
						icore_cmd := CMD_NOP;
					end if;

				when others => -- illegal states
					nxt_state := st_idle;
					icore_cmd := CMD_NOP;
			end case;

			-- generate registers
			if (nReset = '0') then
				core_cmd <= CMD_NOP;
				core_txd <= '0';
				
				shift <= '0';
				ld <= '0';

				dcnt <= "111";
				host_ack <= '0';

				state <= st_idle;
			elsif (clk'event and clk = '1') then
				if (rst = '1') then
					core_cmd <= CMD_NOP;
					core_txd <= '0';
				
					shift <= '0';
					ld <= '0';

					dcnt <= "111";
					host_ack <= '0';

					state <= st_idle;
				else
					state <= nxt_state;

					dcnt <= idcnt;
					shift <= ishift;
					ld <= iload;

					core_cmd <= icore_cmd;
					core_txd <= icore_txd;

					host_ack <= ihost_ack;
				end if;
			end if;
		end process nxt_state_decoder;

	end block statemachine;

end architecture structural;


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

-- Timing:		Normal mode	Fast mode
-----------------------------------------------------------------
-- Fscl		100KHz		400KHz
-- Th_scl		4.0us		0.6us	High period of SCL
-- Tl_scl		4.7us		1.3us	Low period of SCL
-- Tsu:sta		4.7us		0.6us	setup time for a repeated start condition
-- Tsu:sto		4.0us		0.6us	setup time for a stop conditon
-- Tbuf		4.7us		1.3us	Bus free time between a stop and start condition
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

entity bit_ctrl is
	port (
		clk : in std_logic;
		rst : in std_logic;
		nReset : in std_logic;

		clk_cnt : in unsigned(15 downto 0);		-- clock prescale value

		ena : in std_logic;				-- core enable signal
		cmd : in std_logic_vector(2 downto 0);
		cmd_ack : out std_logic;
		busy : out std_logic;

		Din : in std_logic;
		Dout : out std_logic;

		-- i2c lines
		SCLin : in std_logic;				-- I2C clock line
		SCLout : out std_logic;
		SDAin : in std_logic;				-- I2C data line
		SDAout : out std_logic
	);
end entity bit_ctrl;

architecture structural of bit_ctrl is
	constant CMD_NOP	: std_logic_vector(2 downto 0) := "000";
	constant CMD_START	: std_logic_vector(2 downto 0) := "010";
	constant CMD_STOP	: std_logic_vector(2 downto 0) := "011";
	constant CMD_READ	: std_logic_vector(2 downto 0) := "100";
	constant CMD_WRITE	: std_logic_vector(2 downto 0) := "101";

	type cmds is (idle, start_a, start_b, start_c, start_d, stop_a, stop_b, stop_c, rd_a, rd_b, rd_c, rd_d, wr_a, wr_b, wr_c, wr_d);
	signal state : cmds;

	signal SCLo, SDAo : std_logic;		-- internal I2C lines
	signal sSCL, sSDA : std_logic;			-- synchronized SCL and SDA inputs

	signal txd : std_logic;			-- transmit bit
	signal clk_en, slave_wait :std_logic;		-- clock generation signals
	signal cnt : unsigned(15 downto 0) := clk_cnt;	-- clock divider counter
begin
	-- synchronize SCL and SDA inputs
	synch_SCL_SDA: process(clk)
	begin
		if (clk'event and clk = '1') then
			sSCL <= SCLin;
			sSDA <= SDAin;
		end if;
	end process synch_SCL_SDA;
	
	-- whenever the slave is not ready it can delay the cycle by pulling SCL low
	slave_wait <= '1' when ( (SCLo = '1') and (sSCL = '0') ) else '0';

	-- generate clk enable signal
	gen_clken: process(clk, nReset)
	begin
		if (nReset = '0') then
			cnt <= (others => '0');
			clk_en <= '1';
		elsif (clk'event and clk = '1') then
			if (rst = '1') then
				cnt <= (others => '0');
				clk_en <= '1';
			else
				if ( (cnt = 0) or (ena = '0') ) then
					clk_en <= '1';
					cnt <= clk_cnt;
				else
					if (slave_wait = '0') then
						cnt <= cnt -1;
					end if;
					clk_en <= '0';
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
		-- detect stop condition => detect rising edge on SDA while SCL is high
		det_sta_sto: process(clk)
		begin
			if (clk'event and clk = '1') then
				dSDA <= sSDA;	-- generate a delayed version of sSDA

				sta_condition <= (not sSDA and dSDA) and sSCL;
				sto_condition <= (sSDA and not dSDA) and sSCL;
			end if;
		end process det_sta_sto;

		-- generate bus busy signal
		gen_busy: process(clk)
		begin
			if (nReset = '0') then
				ibusy <= '0';
			elsif (clk'event and clk = '1') then
				if (rst = '1') then
					ibusy <= '0';
				else
					ibusy <= (sta_condition or ibusy) and not sto_condition;
				end if;
			end if;
		end process gen_busy;

		-- assign output
		busy <= ibusy;
	end block bus_status_ctrl;


	-- generate statemachine
	nxt_state_decoder : process (clk, nReset, state, cmd, SDAin)
		variable nxt_state : cmds;
		variable icmd_ack, store_sda : std_logic;
		variable itxd : std_logic;
	begin

		nxt_state := state;

		icmd_ack := '0'; -- default no acknowledge

		store_sda := '0';

		itxd := txd;

		case (state) is
			-- idle
			when idle =>
				case cmd is
					when CMD_START =>
						nxt_state := start_a;
						icmd_ack := '1'; -- command completed

					when CMD_STOP =>
						nxt_state := stop_a;
						icmd_ack := '1'; -- command completed

					when CMD_WRITE =>
						nxt_state := wr_a;
						icmd_ack := '1'; -- command completed
						itxd := Din;

					when CMD_READ =>
						nxt_state := rd_a;
						icmd_ack := '1'; -- command completed

					when others =>
						nxt_state := idle;
-- don't acknowledge NOP command						icmd_ack := '1'; -- command completed
				end case;

			-- start
			when start_a =>
				nxt_state := start_b;

			when start_b =>
				nxt_state := start_c;

			when start_c =>
				nxt_state := start_d;

			when start_d =>
				nxt_state := idle;

			-- stop
			when stop_a =>
				nxt_state := stop_b;

			when stop_b =>
				nxt_state := stop_c;

			when stop_c =>
				nxt_state := idle;

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

			-- write
			when wr_a =>
				nxt_state := wr_b;

			when wr_b =>
				nxt_state := wr_c;

			when wr_c =>
				nxt_state := wr_d;

			when wr_d =>
				nxt_state := idle;

		end case;

		-- generate regs
		if (nReset = '0') then
			state <= idle;
			cmd_ack <= '0';
			txd <= '0';
			Dout <= '0';
		elsif (clk'event and clk = '1') then
			if (rst = '1') then
				state <= idle;
				cmd_ack <= '0';
				txd <= '0';
				Dout <= '0';
			else
				if (clk_en = '1') then
					state <= nxt_state;

					txd <= itxd;
					if (store_sda = '1') then
						Dout <= sSDA;
					end if;
				end if;

				cmd_ack <= icmd_ack and clk_en;
			end if;
		end if;
	end process nxt_state_decoder;

	--
	-- convert states to SCL and SDA signals
	--
	output_decoder: process (clk, nReset, state)
		variable iscl, isda : std_logic;
	begin
		case (state) is
			when idle =>
				iscl := SCLo; -- keep SCL in same state
				isda := sSDA; -- keep SDA in same state

			-- start
			when start_a =>
				iscl := SCLo; -- keep SCL in same state (for repeated start)
				isda := '1'; -- set SDA high

			when start_b =>
				iscl := '1';	-- set SCL high
				isda := '1'; -- keep SDA high

			when start_c =>
				iscl := '1';	-- keep SCL high
				isda := '0'; -- sel SDA low

			when start_d =>
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
				isda := '1'; -- set SDA high

			-- write
			when wr_a =>
				iscl := '0';	-- keep SCL low
				isda := Din;

			when wr_b =>
				iscl := '1';	-- set SCL high
				isda := Din;

			when wr_c =>
				iscl := '1';	-- keep SCL high
				isda := Din;

			when wr_d =>
				iscl := '0'; -- set SCL low
				isda := Din;

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
			SCLo <= '1';
			SDAo <= '1';
		elsif (clk'event and clk = '1') then
			if (rst = '1') then
				SCLo <= '1';
				SDAo <= '1';
			else
				if (clk_en = '1') then
					SCLo <= iscl;
					SDAo <= isda;
				end if;
			end if;
		end if;
	end process output_decoder;

	-- assign outputs
	SCLout <= SCLo;
	SDAout <= SDAo;
end architecture structural;

