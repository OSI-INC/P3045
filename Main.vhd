-- <pre> Fiber Controller (A3045) Firmware, Toplevel Unit

-- V1.1 [19-JUL-22] Based upon P3041 V1.6. We run the OSR8 exclusively off the
-- clock input, wich we drive with amicropower 32.768 kHz oscillator. We run 
-- the ring oscillator continuously to generate DAC Clock (DCK). We enable and 
-- disable the ring oscillator with ENDCK from the CPU.

library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		CK, -- Clock
		SDI -- Serial Data In
		: in std_logic; 
		ND, -- North Digital
		SD, -- South Digital
		ED, -- East Digital
		WD, -- West Digital
		SDO, -- Serial Data Out
		TP1, -- Test Point One (TMS)
		TP2, -- Test Point Two (TDI)
		TP3, -- Test Point Three (TDO)
		TP4  -- Test Point Four (TCK)
		: out std_logic;
		IN1,IN2,IN3,IN4,IN5 : in std_logic -- Dummy Inputs for Routing
	);

-- Configuration of OSR8 CPU.
	constant prog_addr_len : integer := 12;
	constant cpu_addr_len : integer := 12;
	constant start_pc : integer := 0;
	constant interrupt_pc : integer := 3;
	constant ram_addr_len : integer := 10;
	constant cmd_addr_len : integer := 10;

-- Memory Map Constants, sizes and base addresses in units of 512 Bytes.
	constant ram_base : integer := 0;
	constant ram_range : integer := 2;
	constant cmd_base : integer := 2;
	constant cmd_range : integer := 2;
	constant ctrl_base : integer := 4;
	constant ctrl_range : integer := 2;
	
-- Memory Map Constants, low nibble addresses in units of bytes;
	constant mmu_irqb : integer := 1;  -- Interrupt Request Bits
	constant mmu_imsk : integer := 2;  -- Interrupt Mask Bits
	constant mmu_irst : integer := 3;  -- Interrupt Reset Bits
	constant mmu_rst  : integer := 4;  -- System Reset
	constant mmu_dfr  : integer := 5;  -- Diagnostic Flag Register
	constant mmu_sr   : integer := 6;  -- Status Register
	constant mmu_it1p : integer := 7;  -- Interrupt Timer One Period
	constant mmu_it2p : integer := 8;  -- Interrupt Timer Two Period
	constant mmu_it3p : integer := 9;  -- Interrupt Timer Three Period
	constant mmu_it4p : integer := 10; -- Interrupt Timer Four Period
	constant mmu_cch  : integer := 11; -- Command Count HI Byte
	constant mmu_ccl  : integer := 12; -- Command Count LO Byte
	constant mmu_crst : integer := 13; -- Command Processor Reset
	constant mmu_edc  : integer := 14; -- Enable DAC Clock
	
-- Calibration of DAC Clock
	constant fck_divisor : integer := 15; 
end;

architecture behavior of main is

-- Attributes to guide the compiler.
	attribute syn_keep : boolean;
	attribute nomerge : string;

-- Power Controller
	signal USERSTDBY, CLRFLAG, SFLAG, STDBY, RESET : std_logic;
	attribute syn_keep of RESET : signal is true;
	attribute nomerge of RESET : signal is "";
	signal SWRST : boolean := false; -- Software Reset
		
-- Diagnostic Flag Register
	signal df_reg : std_logic_vector(3 downto 0) := (others => '0');

-- Program Memory Signals
	signal prog_data : std_logic_vector(7 downto 0); -- ROM Data
	signal prog_addr : std_logic_vector(prog_addr_len-1 downto 0); -- ROM Address
	
-- Process Memory Signals
	signal ram_addr : std_logic_vector(ram_addr_len-1 downto 0); -- RAM Address
	signal ram_out, ram_in : std_logic_vector(7 downto 0); -- RAM Data In and Out
	signal RAMWR : std_logic; -- Command Memory Write
	
-- Central Processing Unit Signals
	signal cpu_data_out, cpu_data_in : std_logic_vector(7 downto 0); 
	signal cpu_addr : std_logic_vector(cpu_addr_len-1 downto 0);
	attribute syn_keep of cpu_addr : signal is true;
	attribute nomerge of cpu_addr : signal is "";  
	signal CPUWR, -- Write (Not Read)
		CPUDS, -- Data Strobe
		CPUIRQ -- Interrupt Request
		: boolean; 
	signal CPUSIG : std_logic_vector(2 downto 0); -- Signals for debugging.

-- Interrupt Handler signals.
	signal int_mask, int_bits, int_rst, int_set : std_logic_vector(7 downto 0);
	signal int_period_1, int_period_2, int_period_3, int_period_4 : std_logic_vector(7 downto 0);
	signal INTZ1, INTZ2, INTZ3, INTZ4 : boolean; -- Interrupt Counter Zero Flag
	
-- Byte Receiver
	signal SDIS, -- Serial Data In Synchronized
		ICMD, -- Initiate Command Reception
		TCMD, -- Terminate Command Reception
		RCMD, -- Receive Command
		RBI, -- Receive Command Byte Initiate
		RBD, -- Receive Command Byte Done
		CRCERR, -- Cyclic Redundancy Checksum Error
		BYTERR, -- Byte Error
		BYTS, -- Command Byte Strobe
		BITS -- Command Bit Strobe
		: boolean := false; 
	
-- Command Memory
	constant cmd_addr_max : integer := (2 ** cmd_addr_len) - 1;
	signal cmd_wr_addr : std_logic_vector(cmd_addr_len-1 downto 0); -- Command Memory Write Address
	signal cmd_rd_addr : std_logic_vector(cmd_addr_len-1 downto 0); -- Command Memory Read Address
	signal cmd_in : std_logic_vector(7 downto 0); -- Command Memory Data In
	signal cmd_out : std_logic_vector(7 downto 0); -- Command Memory Data Out
	signal BYTSEL, -- Command Memory Select
		CMWR  -- Command Memory Write
		: std_logic; 

-- Command Processor
	signal CPA, -- Command Processor Active
		CMDRDY, -- Command Ready
		CPRST -- Command Processor Reset
		: boolean := false;
		
-- Digital to Analog Converters
	signal ENDCK : boolean; -- Enable the DAC Clock
	signal FCK : std_logic; -- Fast Clock (approx 8 MHz)
	signal DCK : std_logic; -- DAC Clock (approx 1 MHz)
	attribute syn_keep of FCK, DCK : signal is true;
	attribute nomerge of FCK, DCK : signal is "";  

-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;

begin

-- We turn off the logic chip bandgap references and other power-hungry
-- circuits with the power controller unit. Within a few milliseconds
-- of power-up, the chip is fully operational, but consuming several 
-- milliamps. We move the chip into standby mode by first clearing the 
-- standby flag with CLRFLAG, then asserting the USERSTDBY control signal
-- that begins the transition to standby mode. The PCU has two outputs: 
-- STDBY and SFLAG. The STDBY signal is intended as a command to put 
-- circuits to sleep, while SFLAG is intended as a signal that the system
-- is in standby mode, which must be cleared after returning to full-power
-- mode. We return to full-power mode when we program the chip. The OND
-- signal keeps the power turned on to the chip.
	Power_Controller: entity PCU port map (
		CLRFLAG => CLRFLAG,
		USERSTDBY => USERSTDBY, 
		STDBY => STDBY,
		SFLAG => SFLAG);	

	PowerUp: process (CK) is
		constant end_state : integer := 255;
		variable state : integer range 0 to end_state := 0;
	begin
		if rising_edge(CK) then
			CLRFLAG <= to_std_logic(state = 1);
			USERSTDBY <= to_std_logic(state >= 3);
			RESET <= to_std_logic((state < end_state) or SWRST);

			if (state = 0) then state := 1;
			elsif (state = 1) then state := 2;
			elsif (state = 2) then state := 3;
			elsif (SFLAG = '0') then state := 3;
			elsif (state < end_state) then state := state + 1; 
			else state := end_state; end if;
		end if;
	end process;
	
-- User memory and configuration code for the CPU. This RAM will be initialized at
-- start-up with a configuration file, and so may be read after power up to configure
-- sensor. The configuration data will begin at address zero.
	RAM : entity RAM port map (
		Clock => not CK,
		ClockEn => '1',
        Reset => RESET,
		WE => RAMWR,
		Address => ram_addr, 
		Data => ram_in,
		Q => ram_out);

-- Instruction Memory for CPU. This read-only memory will be initialized with the
-- CPU program, the first instruction of the program being stored at address zero.
-- The CPU reads the instruction memory with a separate address bus, which we call
-- the program counter.
	ROM : entity ROM port map (
		Address => prog_addr,
        OutClock => not CK,
        OutClockEn => '1',
        Reset => RESET,	
        Q => prog_data);

-- The processor itself, and eight-bit microprocessor with thirteen-bit address bus.
	CPU : entity OSR8_CPU 
		generic map (
			prog_addr_len => prog_addr_len,
			cpu_addr_len => cpu_addr_len,
			start_pc => start_pc,
			interrupt_pc => interrupt_pc
		)
		port map (
			prog_data => prog_data,
			prog_addr => prog_addr,
			cpu_data_out => cpu_data_out,
			cpu_data_in => cpu_data_in,
			cpu_addr => cpu_addr,
			WR => CPUWR,
			DS => CPUDS,
			IRQ => CPUIRQ,
			SIG => CPUSIG,
			RESET => RESET,
			CK => CK
		);
		
-- The Memory Manager maps eight-bit read and write access to the Sensor Controller, Sample 
-- Transmitter, Random Access Memory, and Interrupt Handler. Byte ordering is big-endian 
-- (most significant byte at lower address). 
	MMU : process (CK,RESET) is
		variable top_bits : integer range 0 to 7;
		variable bottom_bits : integer range 0 to 63;
	begin
	
		-- Some variables for brevity.
		top_bits := to_integer(unsigned(cpu_addr(cpu_addr_len-1 downto 9)));
		bottom_bits := to_integer(unsigned(cpu_addr(5 downto 0)));
		
		-- We want the following signals to be combinatorial functions
		-- of the address. Here we define their default values.
		RAMWR <= '0';
		ram_in <= cpu_data_out;
		ram_addr <= cpu_addr(ram_addr_len-1 downto 0);
		cmd_rd_addr <= cpu_addr(cmd_addr_len-1 downto 0);
		cpu_data_in <= (others => '0');	
		
		-- These signals develop after the CPU assserts a new address
		-- along with CPU Write and CPU Sixteen-Bit Access. They will
		-- be ready before the falling edge of the CPU clock.
		case top_bits is
		when ram_base to (ram_base+ram_range-1) => 
			if not CPUWR then
				cpu_data_in <= ram_out;
			else
				RAMWR <= to_std_logic(CPUDS);
			end if;
		when cmd_base to (cmd_base+cmd_range-1) => 
			if not CPUWR then
				cpu_data_in <= cmd_out;
			end if;
		when ctrl_base to (ctrl_base+ctrl_range-1) =>
			if not CPUWR then 
				case bottom_bits is
					when mmu_irqb => cpu_data_in <= int_bits;
					when mmu_imsk => cpu_data_in <= int_mask;
					when mmu_dfr => cpu_data_in(3 downto 0) <= df_reg;
					when mmu_sr => 
						cpu_data_in(0) <= to_std_logic(CMDRDY); -- Command Ready Flag
						cpu_data_in(1) <= to_std_logic(ENDCK);  -- DAC Clock Enabled
						cpu_data_in(4) <= to_std_logic(CPA);    -- Command Processor Active Flag
					when mmu_cch => 
						cpu_data_in(cmd_addr_len-9 downto 0) <= cmd_wr_addr(cmd_addr_len-1 downto 8);
					when mmu_ccl =>
						cpu_data_in <= cmd_wr_addr(7 downto 0);
				end case;
			end if;
		end case;
		
		-- We use RESET to clear some registers and signals, but not all. We do not clear the
		-- software reset signal, SWRST, on RESET, since we want SWRST to assert RESET for one
		-- CK period. After a reset, the cpu address will not select the SWRST location, so
		-- SWRST will be cleared on the next falling edge of CK.
		if (RESET = '1') then
			ENDCK <= false;
			int_period_1 <= (others => '0');
			int_period_2 <= (others => '0');
			int_period_3 <= (others => '0');
			int_period_4 <= (others => '0');
			df_reg <= (others => '0');
			int_mask <= (others => '0');
			CPRST <= true;
		-- We use the falling edge of CK to write to registers and to initiate sensor 
		-- and transmit activity. Some signals we assert only for one CK period, and 
		-- these we assertS as false by default.
		elsif falling_edge(CK) then
			CPRST <= false;
			SWRST <= false;
			int_rst <= (others => '0');
			if CPUDS and CPUWR then 
				if (top_bits >= ctrl_base) and (top_bits <= ctrl_base+ctrl_range-1) then
					case bottom_bits is
						when mmu_imsk => int_mask <= cpu_data_out;
						when mmu_irst => int_rst <= cpu_data_out;
						when mmu_rst => SWRST <= (cpu_data_out(0) = '1');
						when mmu_dfr => df_reg <= cpu_data_out(3 downto 0);
						when mmu_crst => CPRST <= true;
						-- Disable one or more of the eight-bit interrupt timers, and have
						-- their resources freed, by commenting out lines below.
						when mmu_it1p => int_period_1 <= cpu_data_out;
						when mmu_it2p => int_period_2 <= cpu_data_out;
						when mmu_it3p => int_period_3 <= cpu_data_out;
						when mmu_it4p => int_period_4 <= cpu_data_out;
						when mmu_edc => ENDCK <= (cpu_data_out(0) = '1');
					end case;
				end if;
			end if;
		end if;
	end process;

	-- The Interrupt_Controller provides the interrupt signal to the CPU in response to
	-- sensor and timer events. By default, at power-up, all interrupts are masked.
	Interrupt_Controller : process (CK,RESET) is
	variable counter_1, counter_2, counter_3, counter_4 : integer range 0 to 255;
	begin
	
		-- The interrupt timers, counting down from their interrupt period to zero 
		-- running off CK. We stop a timer by writing a zero to its interrupt period
		-- register. Otherwise, they never stop counting down, reloading the period
		-- value and counting down again. The period register should be loaded with 
		-- the desired interrupt period minus one, because the count-down includes 
		-- zero. So 0xFF (255) for the register gives a period of 256 CK periods. 
		-- We use the falling edge of CK to count down.
		if falling_edge(CK) then
			if (counter_1 = 0) then
				counter_1 := to_integer(unsigned(int_period_1));
			else
				counter_1 := counter_1 - 1;
			end if;
			if (counter_2 = 0) then
				counter_2 := to_integer(unsigned(int_period_2));
			else
				counter_2 := counter_2 - 1;
			end if;
			if (counter_3 = 0) then
				counter_3 := to_integer(unsigned(int_period_3));
			else
				counter_3 := counter_3 - 1;
			end if;
			if (counter_4 = 0) then
				counter_4 := to_integer(unsigned(int_period_4));
			else
				counter_4 := counter_4 - 1;
			end if;
		end if;

		-- The interrupt management runs off CK. On reset, we clear the interrupt 
		-- request line and the interrupt bits. We clear the delayed counter zero lines.
		if (RESET = '1') then
			CPUIRQ <= false;
			int_bits <= (others => '0');
			INTZ1 <= false;
			INTZ2 <= false;
			INTZ3 <= false;
			INTZ4 <= false;
		elsif rising_edge(CK) then
		
			-- The timer one interrupt is set when counter_1 goes from value
			-- one to value zero, and at no other time. We reset when we write 
			-- 1 to int_rst(0). The timer generates an interrupt on bit zero.
			INTZ1 <= (counter_1 = 0);
			if (int_rst(0) = '1') then
				int_bits(0) <= '0';
			elsif ((counter_1 = 0) and (not INTZ1)) then
				int_bits(0) <= '1';
			end if;
			
			-- The timer two interrupt, interrupt bit one.
			INTZ2 <= (counter_2 = 0);
			if (int_rst(1) = '1') then
				int_bits(1) <= '0';
			elsif ((counter_2 = 0) and (not INTZ2)) then
				int_bits(1) <= '1';
			end if;
			
			-- The timer three interrupt, interrupt bit two.
			INTZ3 <= (counter_3 = 0);
			if (int_rst(2) = '1') then
				int_bits(2) <= '0';
			elsif ((counter_3 = 0) and (not INTZ3)) then
				int_bits(2) <= '1';
			end if;
			
			-- The timer four interrupt, interrupt bit three.
			INTZ4 <= (counter_4 = 0);
			if (int_rst(3) = '1') then
				int_bits(3) <= '0';
			elsif ((counter_4 = 0) and (not INTZ4)) then
				int_bits(3) <= '1';
			end if;
			
			-- We disable the remaining interrupt lines.
			for i in 4 to 7 loop
				int_bits(i) <= '0';
			end loop;			
		end if;

		-- We generate an interrupt if any one interrupt bit is 
		-- set and unmasked.
		CPUIRQ <= (int_bits and int_mask) /= "00000000";
	end process;
	
-- The Receive Power signal must be synchronized with the CK clock.
	Synchronize_SDI: process is 
	begin
		wait until (CK = '0');
		SDIS <= (SDI = '1');
	end process;
	
-- We detect a long enough burst of command power to initiate
-- command reception, and set the ICMD signal.
	Initiate_Command: process is 
		constant endcount : integer := 63;
		variable counter : integer range 0 to endcount := 0;
	begin
		wait until (CK = '1');
		if SDIS then 
			if (counter = endcount) then 
				counter := endcount;
				ICMD <= true;
			else 
				counter := counter + 1;
				ICMD <= false;
			end if;
		else
			counter := 0;
			ICMD <= false;
		end if;
	end process;
	
-- We detect a long enough period without command power to 
-- terminate command reception, and set the TCMD signal.
	Terminate_Command: process is 
		constant endcount : integer := 255;
		variable counter : integer range 0 to endcount := 0;
	begin
		wait until (CK = '1');
		if not SDIS then 
			if (counter = endcount) then 
				counter := endcount;
				TCMD <= true;
			else 
				counter := counter + 1;
				TCMD <= false;
			end if;
		else
			counter := 0;
			TCMD <=  false;
		end if;
	end process;
	
-- The Receive Command (RCMD) signal indicates that a command is being 
-- received. We set RCMD when Initiate Command (ICMD) occurs, and we clear
-- RCMD when Terminate Command (TCMD) occurs.
	Receive_Command: process is
	begin
		wait until (CK = '1');
		if not RCMD then
			RCMD <= ICMD;
		else 
			RCMD <= not TCMD;
		end if;
	end process;

-- We watch for a start bit and receive serial bytes when instructed
-- to do so by the Command Processor with the RBI signal.
	Byte_Receiver: process is
		variable state, next_state : integer range 0 to 63 := 0;
		variable no_stop_bit : boolean := false;
	begin
		wait until (CK = '1');
		
		-- Idle state, waiting for Receive Byte Initiate.
		if (state = 0) then
			if RBI and (not SDIS) then 
				next_state := 1;
			else 
				next_state := 0;
			end if;
		end if;
		
		-- Wait for a start bit. If we wait long enough, we will see the 
		-- termination signal, in which case we abort and wait for not RBI.
		-- We clear no stop bit variable, which clears the global BYTERR 
		-- signal.
		if (state = 1) then
			if TCMD then 
				next_state := 63; 
			else 
				if SDIS then 
					next_state := 2;
				else 
					next_state := 1; 
				end if;
			end if;
			no_stop_bit := false;
		end if;
		BYTERR <= no_stop_bit;
		
		-- Once we have a start bit, we proceed through the eight bits of
		-- a command byte, each bit taking four states. The first bit occurs
		-- at state 7 and the stop bit at state 39.
		if (state >= 2) and (state <= 38) then 
			next_state := state + 1; 
		end if;
		
		-- If the stop bit is present, we go to our end state. If it's missing,
		-- we go to our byte error state. The stop bit is zero, so SDIS should 
		-- at this point be false.
		if (state = 39) then
			if not SDIS then 
				next_state := 63;
			else 
				next_state := 62;
			end if;
		end if;
		
		-- Here we deal with unused states by directing them towards the byte
		-- error state.
		if (state > 39) and (state < 62) then 
			next_state := 62; 
		end if;
		
		-- In the byte error state, we set the "no stop bit" flag, which asserts the 
		-- global BYTERR signal. We will not reset this flag until the Byte Receiver
		-- starts a new byte reception. This flag tells the Command Processor to ignore
		-- the entire command. We wait in the byte error state until RBI is unasserted. 
		-- Because we do not assert RBD, the un-assertion of RBI will occur only when
		-- the Command Receiver encounters a Terminate Command signal.
		if (state = 62) then
			if not RBI then 
				next_state := 0;
			else 
				next_state := 62;
			end if;
			no_stop_bit := true;
		end if;
		
		-- In the end state, we assert Receive Byte Done and we wait for the command
		-- processor to un-assert Receive Byte Initiate. When we see not RBI, we return
		-- to the idle state. We stop asserting RBD, or refrain from asserting it, when
		-- we have Terminate Command.
		if (state = 63) then 
			if not RBI then 
				next_state := 0; 
			else 
				next_state := 63; 
			end if;
		end if;
		RBD <= (state = 63) and (not TCMD);
				
		-- The eight bits of the command are set every four states during
		-- the command reception.
		for i in 0 to 7 loop
			if (state = 35 - i * 4) then 
				if SDIS then 
					cmd_in(i) <= '1'; 
				else 
					cmd_in(i) <= '0'; 
				end if;
			else 
				cmd_in(i) <= cmd_in(i); 
			end if;
		end loop;
		
		-- We assert Command Bit Strobe one CK period before the best moment
		-- to sample each bit value.
		if (state = 34) or (state = 30) or (state = 26) or (state = 22) 
			or (state = 18) or (state = 14) or (state = 10) or (state = 6) then
			BITS <= true;
		else 
			BITS <= false;
		end if;
		
		-- The Byte Strobe signal indicates that we have a start bit and is 
		-- useful as a test point trigger. It provides a pulse of two CK 
		-- periods.
		BYTS <= (state = 2) or (state = 3);
		
		-- Assert the new state.
		state := next_state;
	end process;

-- This process runs all the bits of a command through a sixteen-bit linear 
-- feedback shift register, with local name "crc" for "cyclic redundancy check". 
-- We preset crc to all ones. The final sixteen bits of every command are chosen 
-- so that they reset the crc register to all zeros. If crc is not zero at the 
-- end of a command, there was some error during reception. We use the Bit Strobe 
-- (BITS) signal to clock crc, because BITS is asserted only when a command data 
-- bit is receive, not when we receive a start or stop bit.
	Error_Check : process is
		variable crc, next_crc : std_logic_vector(15 downto 0) := (others => '1');
	begin
		wait until (CK = '1');
		
		if ICMD then
			-- When a new command transmission starts, we preload the cyclic redundancy
			-- check register to all ones.
			crc := (others => '1');
		else
			-- We use Command Bit Strobe to clock each command bit into the CRC.
			-- The transmitter calculates the checksum with zeros in the last
			-- sixteen bits, reverses the order of these checksum bits, and sends
			-- them as the last two bytes of the actual transmission, instead of the
			-- zeros it used when it calculated its own checksum. These last sixteen
			-- bits, thus obtained, will reset the receiver CRC to zero, provided there
			-- has been no corruption of the data on the way.
			if BITS then
				for i in 0 to 9 loop next_crc(i) := crc(i+1); end loop;
				next_crc(10) := crc(11) xor crc(0);
				next_crc(11) := crc(12);
				next_crc(12) := crc(13) xor crc(0);
				next_crc(13) := crc(14) xor crc(0);
				next_crc(14) := crc(15);
				next_crc(15) := to_std_logic(SDIS) xor crc(0);	
				crc := next_crc;
			end if;		
		end if;
		
		-- The CRCERR flag tells us when the CRC is not zero. It will be zero when it
		-- has been reset by the two bytes of a correct checksum.
		CRCERR <= (crc /= "0000000000000000");
	end process;

-- Command Memory
	Command_Memory : entity CMD_RAM port map (
		Reset => '0', 
		WrClock => not CK,
		WrClockEn => '1',
		WE => CMWR,
		WrAddress => cmd_wr_addr, 
		Data => cmd_in,
		RdAddress => cmd_rd_addr,
		RdClock => not CK,
		RdClockEn => '1',
		Q => cmd_out);
	
-- This Command Processor detects Inititiate Command (ICMD) and activates the Byte Receiver. 
-- It stores command bytes in the Command Memory until it detects Terminate Command (TCMD). If
-- the Error Check reports no error, the Command Processor asserts Command Ready (CMDRDY) and
-- waits until the CPU asserts Command Processor Reset (CPRST) before returning to its rest
-- state. When the command is ready, the CPU can read all bytes out of the Command Memory. 
-- The Command Processor runs on the reference clock, which is 32.768 kHz, and proceeds to a 
-- new state every clock cycle. 
	Command_Processor: process (CK, RESET, CPRST) is
		
		-- General-purpose state names for the Command Processor
		constant idle_s : integer := 0;
		constant receive_cmd_s : integer := 1;
		constant store_cmd_s : integer := 2;
		constant inc_addr_s : integer := 3;
		constant check_cmd_s : integer := 4;
		constant complete_s : integer := 5;
		
		-- Variables for the Command Processor
		variable state, next_state : integer range 0 to 31 := 0;
		variable addr : integer range 0 to cmd_addr_max := 0;
		
	begin
		-- We reset to the idle state on global RESET or the Command Processor
		-- Reset (CPRST).
		if (RESET = '1') or CPRST then
			state := idle_s;
			addr := 0;
			
		-- The Command Processor state machine runs off CK, which allows it to
		-- work with the Byte Receiver.
		elsif rising_edge(CK) then
			-- Default next state.
			next_state := idle_s;
		
			-- Idle State.
			if (state = idle_s) then
				if ICMD then 
					next_state := receive_cmd_s; 
				else 
					next_state := idle_s;
				end if;
				addr := 0;
			end if;
			
			-- Receive a command byte. We assert RBI and wait for RBD. If we see 
			-- Terminate Command (TCMD), we look at the number of command bytes we have 
			-- received so far. If we have less than three, we have only the checksum,
			-- so we go back to the idle state. If we have three or more, we move on.
			-- Note that the Byte Receiver aborts on TCMD also.
			if (state = receive_cmd_s) then 
				if TCMD then 
					if (addr <= 2) then 
						next_state := idle_s;
					else 
						next_state := check_cmd_s;
					end if;
				else 
					if RBD then 
						next_state := store_cmd_s;
					else 
						next_state := receive_cmd_s;
					end if;
				end if;
			end if;
			RBI <= (state = receive_cmd_s);
			
			-- Store the new command byte in the command memory. We assert CMWR.
			if (state = store_cmd_s) then 
				if not RBD then 
					next_state := inc_addr_s;
				else 
					next_state := store_cmd_s;
				end if;
			end if;
			CMWR <= to_std_logic(state = store_cmd_s);
			
			-- Increment the command address. If we have run out of space in the
			-- Command Memory, we abort our attempt to process the command, and wait
			-- for the next command.
			if (state = inc_addr_s) then
				addr := addr + 1;
				if (addr = cmd_addr_max) then
					next_state := idle_s;
				else
					next_state := receive_cmd_s;
				end if;
			end if;		
			
			-- There are two possible sources of error: a failure in the cyclic redundancy
			-- check (CRCERR) or an error in the structure of a command byte (BYTERR). If either
			-- is asserted, we go back to idle.
			if (state = check_cmd_s) then
				if CRCERR or BYTERR then 
					next_state := idle_s;
				else 
					next_state := complete_s;
				end if;
			end if;

			-- We have a completed command in memory, waiting for the CPU to read it out.
			-- We assert CMDRDY and wait until the CPU asserts CPRST. The command processor
			-- will ignore any further command transmission.
			if (state = complete_s) then
				next_state := complete_s;
			end if;
			
			-- Advance the state variable.
			state := next_state;
		end if;
		
		-- Command Ready tells the CPU that a command is available.
		CMDRDY <= (state = complete_s);
			
		-- Command Processor Active is true whenever the state is not idle.
		CPA <= (state /= idle_s);
			
		-- The Command Memory Write Address is always equal to the Command Processor's
		-- addr variable.
		cmd_wr_addr <= std_logic_vector(to_unsigned(addr,cmd_addr_len));
	end process;
	
-- Ring Oscillator. This oscillator generates FCK, which is around 8 MHz. We enable
-- the oscilator with Enable DAC Clock (ENDCK) under control of the CPU.
	Fast_CK : entity ring_oscillator port map (
		ENABLE => to_std_logic(ENDCK), 
		calib => fck_divisor,
		CK => FCK);
		
-- The Digital to Analog Converter Clock (DCK) should be around 1 MHz, so we divide
-- FCK by eight.
	DCK_Generator : process (RESET, FCK) is
		variable count : integer range 0 to 7 := 0;
	begin
		if (RESET = '1') then
			count := 0;
			DCK <= '0';
		elsif rising_edge(FCK) then
			count := count + 1;
			if (count >= 4) then
				DCK <= '1';
			else
				DCK <= '0';
			end if;
		end if;
	end process;

-- Test Point One appears on P1-6.
	TP1 <= df_reg(0);
	
-- Test Point Two appears on P1-3.
	TP2 <= df_reg(1);
	
-- Test Point Three appears on P1-2.
	TP3 <= FCK;

-- Test Point Four appears on P1-8.
	TP4 <= to_std_logic((DCK = '1') or (IN1 = '1') or (IN2 = '1') 
		or (IN3 = '1') or (IN4 = '1') or (IN5 = '1'));
	
-- We have no implementation of SDO yet. It is an open-drain output
-- so we drive it to logic HI to release it.
	SDO <= '1';

end behavior;