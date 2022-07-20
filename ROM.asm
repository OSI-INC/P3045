; Implantable Stimulator-Transponder (IST) Program
; ------------------------------------------------

; This code runs in the OSR8V3 microprocessor of the A3041A.

; V1: Provides a ramp value for transmission at 128 SPS. Sets
; TP1 is set during the transmit interrupt. TP2 is set during the
; main loop. RCK is connected directly to the reference clock. The
; TCK pin is connected to FHI.

; Calibration Constants.
const device_id  0xA123 ; Bottom niblle 1-14.
const tx_calib        5 ; Transmit frequency calibration

; Address Map Boundary Constants
const mmu_vmem 0x0000 ; Base of Variable Memory
const mmu_cmem 0x0400 ; Base of Command Memory
const mmu_ctrl 0x0800 ; Base of Control Space
const mmu_sba  0x0300 ; Stack Base Address

; Address Map Locations
const mmu_sdb  0x0800 ; Sensor Data Byte
const mmu_scr  0x0801 ; Sensor Control Register
const mmu_irqb 0x0802 ; Interrupt Request Bits
const mmu_imsk 0x0803 ; Interrupt Mask Bits
const mmu_irst 0x0804 ; Interrupt Reset Bits
const mmu_dva  0x0805 ; Device Active
const mmu_stc  0x0806 ; Stimulus Current
const mmu_rst  0x0807 ; System Reset
const mmu_xhb  0x0808 ; Transmit HI Byte
const mmu_xlb  0x0809 ; Transmit LO Byte
const mmu_xcn  0x080A ; Transmit Channel Number
const mmu_xcr  0x080B ; Transmit Control Register
const mmu_xfc  0x080C ; Transmit Frequency Calibration
const mmu_etc  0x080D ; Enable Transmit Clock
const mmu_tcf  0x080E ; Transmit Clock Frequency
const mmu_tcd  0x080F ; Transmit Clock Divider
const mmu_bcc  0x0810 ; Boost CPU Clock
const mmu_dfr  0x0811 ; Diagnostic Flag Register
const mmu_sr   0x0812 ; Status Register
const mmu_cch  0x0813 ; Command Count HI
const mmu_ccl  0x0814 ; Command Count LO
const mmu_cpr  0x0815 ; Command Processor Reset
const mmu_it1p 0x0816 ; Interrupt Timer One Period
const mmu_it2p 0x0817 ; Interrupt Timer Two Period
const mmu_it3p 0x0818 ; Interrupt Timer Three Period
const mmu_it4p 0x0819 ; Interrupt Timer Four Period

; Status Bit Masks, for use with status register.
const sr_cmdrdy  0x01 ; Command Ready Flag
const sr_entck   0x02 ; Transmit Clock Enabled
const sr_saa     0x04 ; Sensor Access Active Flag
const sr_txa     0x08 ; Transmit Active Flag
const sr_cpa     0x10 ; Command Processor Active
const sr_boost   0x20 ; Boost Flag

; Transmit Control Masks, for use with tansmit control register.
const tx_txi     0x01 ; Assert transmit initiate.
const tx_txwp    0x02 ; Assert transmit warm-up.
const id_at         0 ; Auxiliary type for identification.
const ack_at        1 ; Auxiliary type for acknowledgements.
const batt_at       2 ; Auxiliary type for battery measurement.

; Bit Masks
const bit0_mask  0x01 ; Bit Zero Mask
const bit1_mask  0x02 ; Bit One Mask
const bit2_mask  0x04 ; Bit Two Mask
const bit3_mask  0x08 ; Bit Three Mask
const bit0_clr   0xFE ; Bit Zero Clear
const bit1_clr   0xFD ; Bit One Clear
const bit2_clr   0xFB ; Bit Two Clear
const bit3_clr   0xF7 ; Bit Three Clear

; Timing Constants.
const min_tcf       72  ; Minimum TCK periods per half RCK period.
const tx_delay      50  ; Wait time for sample transmission, TCK periods.
const sa_delay      30  ; Wait time for sensor access, TCK periods.
const wp_delay     255  ; Warm-up delay for auxiliary messages.
const num_vars      40  ; Number of vars to clear at start.
const initial_tcd   15  ; Max possible value of TCK divisor.
const stim_tick     33  ; Stimulus interrupt period.
const xx_delay   32767  ; Transmit Extinguish Delay
const id_delay      33  ; To pad id delay to 50 TCK periods.

; Stimulus Control Variables
const Scurrent    0x0000 ; Stimulus Current
const Spulse_1    0x0001 ; Pulse Length, HI
const Spulse_0    0x0002 ; Pulse Length, LO
const Sinterval_2 0x0003 ; Interval Length, HI
const Sinterval_1 0x0004 ; Interval Length, MID
const Sinterval_0 0x0005 ; Interval Length, LO
const Slength_1   0x0006 ; Stimulus Length, HI
const Slength_0   0x0007 ; Stimulus Length, LO
const Srandomize  0x0008 ; Randomise
const Srun        0x0009 ; Run stimulus
const Sprun       0x000A ; Stimulus Pulse Run Flag
const Sack_key    0x000B ; Acknowledgement key
const Spcnt1      0x000C ; Stimulus Pulse Counter Byte One
const Spcnt0      0x000D ; Stimulus Pulse Counter Byte Zero
const Sicnt2      0x000E ; Stimulus Interval Counter Byte Two
const Sicnt1      0x000F ; Stimulus Interval Counter Byte One
const Sicnt0      0x0010 ; Stimulus Interval Counter Byte Zero
const Sistart     0x0011 ; Stimulus Interval Start
const Sdly2       0x0012 ; Stimulus Delay Byte Two
const Sdly1       0x0013 ; Stimulus Delay Byte One
const Sdly0       0x0014 ; Stimulus Delay Byte Zero
const Sdrun       0x0015 ; Stimulus Delay Run Flag

; Command Execution Variables
const cmd_cnt_h   0x0020 ; Command Count, HI
const cmd_cnt_l   0x0021 ; Command Count, LO

; Random Number Variabls
const Rand1       0x0030 ; Random Number Byte One
const Rand0       0x0031 ; Random Number Byte Zero

; Transmission Control Variables
const xmit_T      0x0040 ; Transmit Period
const xmit_pcn    0x0041 ; Primary Channel Number
const xxcnt1      0x0042 ; Transmit Extinguish Counter Byte One
const xxcnt0      0x0043 ; Transmit Extinguish Counter Byte Zero

; Global Scratch Registers Variables.
const scratch1    0x0050 ; Scratchpad Variable 1
const scratch2    0x0051 ; Scratchpad Variable 2
const scratch3    0x0052 ; Scratchpad Variable 3
const scratch4    0x0053 ; Scratchpad Variable 4
const scratch5    0x0054 ; Scratchpad Variable 5
const scratch6    0x0055 ; Scratchpad Variable 6

; Operation Codes
const op_stop_stim   0 ; 0 operands
const op_start_stim  1 ; 8 operands
const op_xmit        2 ; 1 operand
const op_ack         3 ; 1 operand
const op_battery     4 ; 0 operand
const op_identify    5 ; 0 operands
const op_setpcn      6 ; 1 operand

; Synchronization.
const synch_nostim  32 ; 
const synch_stim    96 ;

; Random Number Generator.
const rand_taps   0xB4 ; Determines which taps to XOR.

; ------------------------------------------------------------
; The CPU reserves two locations 0x0000 for the start of program
; execution, and 0x0003 for interrupt execution. We put jumps at
; both locations. A jump takes exactly three bytes.

start:

jp main
jp interrupt

; ---------------------------------------------------------------
; Eight-bit multiplier. Load two eight-bit operands into B and C
; and the sixteen-bit result will be returned in B (HI) and C (LO). 
; Takes 200 to 300 clock cycles depending upon the operand C, an 
; average of 250 (50 us at 5 MHz or 7.6 ms at 32.768 kHz).

multiply:

; Save registers and flags on the stack.

push F
push A
push D
push H
push L

; We use D to count down from eight to zero.

ld A,8
push A
pop D

; Clear HL.

ld A,0
push A
pop H
push A
pop L

; Shift C left and check the bit that comes out the top end, now in our
; carry bit. If carry is not set, jump forward to shift HL.

mult_start:
push C
pop A
sla A
push A
pop C
jp nc,mult_check_done

; Carry bit set means we add B to HL.

push L
pop A
add A,B
push A
pop L
push H
pop A
adc A,0
push A
pop H

; Decrement D. If zero, we have added eight times and
; there is no need to shift HL again, we are done.

mult_check_done:
dec D
jp z,mult_done

; Shift HL to the left, filling in bit zero with a zero. We are
; going repeat our addition loop.

push L
pop A
sla A
push A
pop L
push H
pop A
rl A
push A
pop H
jp mult_start

; Multiplication is complete and the result is in HL. Move the 
; result to BC so that this routine affects only BC.

mult_done:
push H
pop B
push L
pop C

; Recover registers and flags.

pop L
pop H
pop D
pop A
pop F
ret

; ------------------------------------------------------------
; Calibrate the transmit clock frequency. We take the CPU out
; of boost, turn off the transmit clock, and repeat a cycle of
; setting the transmit clock divisor and running the transmit
; clock to measure its frequency. Eventually we get a diviso
; that provides a transmit period in the range 195-215 ns. We
; leave the transmit clock off at the end.

calibrate_tck:

; Push flags and registers, disable interrupts.

push F
push A           
push B           

ld A,0x00        ; Clear bit zero of A
ld (mmu_bcc),A   ; Disable CPU Clock Boost
ld (mmu_etc),A   ; Disable Transmit Clock
ld A,initial_tcd ; The initial value of transmit clock divisor
push A           ; Push divisor onto the stack
pop B            ; Store divisor in B
cal_tck_1:
dec B            ; Decrement the divisor.
push B           ; Push divisor onto stack.
pop A            ; Pop divisor into A.
ld (mmu_tcd),A   ; Write divisor to transmit clock generator.
ld A,0x01        ; Set bit zero of A.
ld (mmu_etc),A   ; Enable the transmit clock.
ld A,(mmu_tcf)   ; Read the transmit clock frequency.
sub A,min_tcf    ; Subtract the minimum frequency.
ld A,0x00        ; Clear bit zero of A.
ld (mmu_etc),A   ; Disable Transmit Clock.
jp np,cal_tck_1  ; Try smaller divisor.

; Pop registers and return.

pop B           
pop A           
pop F
ret             

; ------------------------------------------------------------
; The interrupt routine. Handles data transmission and clock.
; Runs with CPU in boost to save time.

interrupt:

; Push A onto the stack, boost CPU, push F.

push A              ; Save A on stack
ld A,0x01           ; Set bit zero to one.
ld (mmu_etc),A      ; Enable the transmit clock, TCK.
ld (mmu_bcc),A      ; Boost the CPU clock to TCK.
push F              ; Save the flags onto the stack.
ld A,(mmu_dfr)      ; Load the diagnostic flag register.
or A,bit0_mask      ; set bit zero and
ld (mmu_dfr),A      ; write to diagnostic flag register.
push H
push L

; Handle the stimulus interrupt, if it exists. We decrement the stimulus
; interval counter and the stimulus pulse counter and set the stimulus 
; interval start and stimulus pulse run flags for the main program to use.

int_stim:

ld A,(mmu_irqb)     ; Read the interrupt request bits
and A,bit1_mask     ; and test bit one,
jp z,int_stim_done  ; skip this is not the stimulus interrupt.

ld A,bit1_mask      ; Reset this interrupt
ld (mmu_irst),A     ; with the bit one mask.

ld A,(Sicnt0)       ; Load stimulus count byte zero,
sub A,stim_tick     ; decremement by the stimulus tick,
ld (Sicnt0),A       ; and write to memory.
ld A,(Sicnt1)       ; Load byte one,
sbc A,0             ; continue subtraction with carry,
ld (Sicnt1),A       ; and write to memory.
ld A,(Sicnt2)       ; Load byt two,
sbc A,0             ; continue subtraction with carry,
ld (Sicnt2),A       ; and write to memory.
jp nc,int_Spulse    ; If the result is >=0, jump,
ld A,1              ; but if <0,
ld (Sistart),A      ; set Sistart flag.

int_Spulse:
ld A,(Sprun)        ; Check the stimulus pulse flag
add A,0             ; and if it is zero,
jp z,int_Sdly       ; jump forward to stimulus delay check.
ld A,(Spcnt0)       ; Load stimulus pulse counter byte zero,
sub A,stim_tick     ; subtract the stimulus tick,
ld (Spcnt0),A       ; and write the result to memory.
ld A,(Spcnt1)       ; Load counter byte one,
sbc A,0             ; continue subtraction with carry,
ld (Spcnt1),A       ; and write to memory.
jp nc,int_stim_done ; If result >=0, done with interrupt,
ld A,0              ; but if <0,
ld (mmu_stc),A      ; turn off the lamp,
ld (Sprun),A        ; clear the pulse flag and now
ld A,(mmu_dfr)     
and A,bit3_clr  
ld (mmu_dfr),A    
jp int_stim_done    ; we are done with this interrupt.

int_Sdly:
ld A,(Sdrun)        ; If the stimulus delay flag is
add A,0             ; set, we decrement delay counter,
jp z,int_stim_done  ; otherwise done with this interrupt.
ld A,(Sdly0)        ; Load the counter zero byte,
sub A,stim_tick     ; subtract the stimulus tick
ld (Sdly0),A        ; and write result to memory.
ld A,(Sdly1)        ; Extend the 
sbc A,0             ; subtraction
ld (Sdly1),A        ; all the
ld A,(Sdly2)        ; way up
sbc A,0             ; through the second
ld (Sdly2),A        ; delay byte.
jp nc,int_stim_done ; If counter positive, done with this interrupt.
ld A,0              ; But if negative,
ld (Sdrun),A        ; clear the delay flag,
ld A,1              ; set the
ld (Sprun),A        ; pulse flag, 
ld A,(Scurrent)     ; load the pulse stimulus current and 
ld (mmu_stc),A      ; start the pulse. Now done with this interrupt.
ld A,(mmu_dfr)      
or A,bit3_mask     
ld (mmu_dfr),A     
int_stim_done:

; Handle the transmit interrupt, if it exists. We won't wait for the transmission
; to complete because we are certain to follow our transmission with at least one
; RCK period when we move out of boost.

int_xmit:

ld A,(mmu_irqb)     ; Read the interrupt request bits
and A,bit0_mask     ; and test bit zero,
jp z,int_xmit_done  ; skip transmit if not set.

ld A,bit0_mask      ; Reset this interrupt
ld (mmu_irst),A     ; with the bit zero mask.

ld A,(xmit_pcn)     ; Load A with primary channel number
ld (mmu_xcn),A      ; and write the transmit channel register.

; If a not Srun, we will transmit synch_nostim. If Srun but not Sprun,
; we transmit synch_stim. If Srun we transmit synch_stim + 8*Scurrent.
; Regardless, the lower byte we transmit will be zero.

ld A,0              ; Load A with zero
ld (mmu_xlb),A      ; write to transmit LO register.

ld A,(Srun)         ; Load A with Srun
add A,0             ; check value
jp nz,int_xmit_stim ; jump if set.

ld A,synch_nostim   ; Load A with synch_nostim and
ld (mmu_xhb),A      ; write to transmit HI register.
jp int_xmit_rdy   

int_xmit_stim:
ld A,(Sprun)        ; Load A with Sprun
add A,0             ; check value, jump if set.
jp nz,int_xmit_pulse

ld A,synch_stim     ; Load A with synch_stim and
ld (mmu_xhb),A      ; write to transmit HI register.
jp int_xmit_rdy   

int_xmit_pulse:
ld A,(Scurrent)     ; Load A with Scurrent and
sla A               ; shift left
sla A               ; three times to
sla A               ; multiply by eight
add A,synch_stim    ; then add synch_stim.
ld (mmu_xhb),A      ; Write to transmit HI register.

int_xmit_rdy:
ld A,tx_txi         ; Load transmit initiate bit
ld (mmu_xcr),A      ; and write to transmit control register.

ld A,(Srun)         ; Load A with Srun
add A,0             ; and check value
jp z,int_xmit_xx    ; jump if zero.

ld HL,xx_delay      ; Load the transmit exitinguish
push H              ; delay into HL
pop A               ; and store
ld (xxcnt1),A       ; in the
push L              ; transmit extinguish
pop A               ; counter locations
ld (xxcnt0), A      ; ready for when Srun is cleared.
jp int_xmit_done

int_xmit_xx:        
ld A,(xxcnt0)       ; Decrement the
sub A,1             ; extinguish
ld (xxcnt0),A       ; counter.
ld A,(xxcnt1)       ; When it gets below zero,
sbc A,0             ; we set the transmit period to
ld (xxcnt1),A       ; zero in memory, which will
jp nc,int_xmit_done ; allow the main loop to turn
ld A,0              ; off power to the
ld (xmit_T),A       ; device, preserving our battery.

int_xmit_done:

; Turn off the transmit clock, move out of boost, restore registers and return 
; from interrupt.

int_done:
pop L
pop H
ld A,(mmu_dfr)      ; Load the diagnostic flag register.
and A,bit0_clr      ; Clear bit zero and
ld (mmu_dfr),A      ; write to diagnostic flag register.
ld A,0x00           ; Clear bit zero and use it to
ld (mmu_bcc),A      ; move CPU back to slow RCK
ld (mmu_etc),A      ; and stop the transmit clock.
pop F               ; Restore the flags.
pop A               ; Restore A.

rti                 ; Return from interrupt.

; -----------------------------------------------------------
; Decrement the command count. The decrement does not allow
; the count to drop below zero. The routine leaves all registers
; intact.

dec_cmd_cnt:

push F
push A

ld A,(cmd_cnt_l)
sub A,1
ld (cmd_cnt_l),A
ld A,(cmd_cnt_h)
sbc A,0
ld (cmd_cnt_h),A
jp nc,dec_cmd_cnt_p
ld A,0
ld (cmd_cnt_h),A
ld (cmd_cnt_l),A
dec_cmd_cnt_p:

pop A
pop F

ret

; ------------------------------------------------------------
; Transmit an acknowledgement. We have to warm up the VCO before
; the transmit, or its frequency will be wrong. The routine assumes
; we are running in boost with the interrupts disabled.

xmit_ack:

push F
push A

; Prepare the VCO for message transmission.

ld A,tx_txwp        ; Turn on the VCO by writing the 
ld (mmu_xcr),A      ; warm-up bit to the transmit control register.
ld A,wp_delay       ; Wait for a number of TCK periods while 
dly A               ; the VCO warms up.
ld A,0              ; Turn off the VCO and
ld (mmu_xcr),A      ; let the battery recover
ld A,wp_delay       ; before we 
dly A               ; transmit.

; Prepare the auxiliary message: auxiliary channel number, top four bits of
; primary channel number, auxiliary type, and acknowledgement key.

ld A,(xmit_pcn)     ; Load A with primary channel number
or A,0x0F           ; set lower four bits to one
ld (mmu_xcn),A      ; and write the transmit channel register.
ld A,(Sack_key)     ; Load the acknowledgement key
ld (mmu_xlb),A      ; and write to transmit LO register.
ld A,(xmit_pcn)     ; Load A with primary channel number again
sla A               ; Shift A 
sla A               ; left
sla A               ; four
sla A               ; times.
or A,ack_at         ; Set the auxiliary type to acknowledgement.
ld (mmu_xhb),A      ; Write to transmit HI register.

; Transmit the message.

ld A,tx_txi         ; Initiate transmission 
ld (mmu_xcr),A      ; with another write to control register.
ld A,tx_delay       ; Wait for a number of TCK periods while 
dly A               ; the transmit completes.

pop A
pop F

ret

; ------------------------------------------------------------
; Transmit a battery measurement. We assume interrupts are disabled and
; the CPU is boosted. The battery  measurement is inversely proportional 
; to the battery voltage. We have: VBAT = 1.2 V * 256 / batt_meas. We 
; must access twice to acquire and convert.

xmit_batt:

push F
push A

; Prepare the VCO for a transmission.

ld A,tx_txwp        ; Turn on the VCO by writing the 
ld (mmu_xcr),A      ; warm-up bit to the transmit control register.
ld A,wp_delay       ; Wait for a number of TCK periods while 
dly A               ; the VCO warms up.
ld A,0              ; Turn off the VCO and
ld (mmu_xcr),A      ; let the battery recover
ld A,wp_delay       ; before we 
dly A               ; transmit.

; Load the most recent battery measurement from memory and use as
; the auxiliary message data byte. 

ld (mmu_scr),A      ; Initiate conversion of battery voltage.
ld A,sa_delay       ; Load sensor delay,
dly A               ; Wait,
ld (mmu_scr),A      ; conver again,
ld A,sa_delay       ; wait
dly A               ; again
ld A,(mmu_sdb)      ; and get battery measurement.
ld (mmu_xlb),A      ; Write the to transmit LO register.

; Prepare the auiliary message: auxiliary channel number, top four bits of
; primary channel number, and auxiliary type.

ld A,(xmit_pcn)     ; Load A with primary channel number
or A,0x0F           ; set lower four bits to one
ld (mmu_xcn),A      ; and write the transmit channel register.
ld A,(xmit_pcn)     ; Load A with primary channel number again
sla A               ; Shift A 
sla A               ; left
sla A               ; four
sla A               ; times.
or A,batt_at        ; The battery type code for auxiliary message.
ld (mmu_xhb),A      ; Write to transmit HI register.

; Transit the message and wait until complete.

ld A,tx_txi         ; Initiate transmission 
ld (mmu_xcr),A      ; with another write to control register.
ld A,tx_delay       ; Wait for a number of TCK periods while 
dly A               ; the transmit completes.

pop A
pop F

ret

; ------------------------------------------------------------
; Transmit an identification message. We assume interrupts are 
; disabled and the CPU is running on the boost clock. 

xmit_identify:

push F
push A
push H
push L

; Delay for 50 clcok cycles multiplied by numeric value of 
; the device id. By this means, each device transmits its
; identifying message at a different time, up to 656 ms from
; the time of the command.

ld HL,device_id 
identify_delay:
ld A,id_delay
dly A
push L
pop A
sub A,1
push A
pop L
push H
pop A
sbc A,0
push A
pop H
jp nc,identify_delay

; Prepare the VCO for a transmission.

ld A,tx_txwp        ; Turn on the VCO by writing the 
ld (mmu_xcr),A      ; warm-up bit to the transmit control register.
ld A,wp_delay       ; Wait for a number of TCK periods while 
dly A               ; the VCO warms up.
ld A,0              ; Turn off the VCO and
ld (mmu_xcr),A      ; let the battery recover
ld A,wp_delay       ; before we 
dly A               ; transmit.

; Load the top byte of the device_id into the transmit LO byte.

ld HL,device_id     ; Load device_id into HL
push H              ; move to
pop A               ; A and 
ld (mmu_xlb),A      ; write the battery measurement to transmit LO register.

; Prepare the auiliary message. We use the lower byte of the device_id
; as the primary channel number for an auxiliary message. 

push L              ; Move the lower byte of device_id
pop A               ; into A,
or A,0x0F           ; set lower four bits to one
ld (mmu_xcn),A      ; and write the transmit channel register.
push L              ; Load A with primary
pop A               ; channel number again
sla A               ; Shift A 
sla A               ; left
sla A               ; four
sla A               ; times.
or A,id_at          ; The identify type code for auxiliary message.
ld (mmu_xhb),A      ; Write to transmit HI register.

; Transit the message and wait until complete.

ld A,tx_txi         ; Initiate transmission 
ld (mmu_xcr),A      ; with another write to control register.
ld A,tx_delay       ; Wait for a number of TCK periods while 
dly A               ; the transmit completes.

pop L
pop H
pop A
pop F

ret

; ------------------------------------------------------------
; Read out, interpret, and execute comands. Uses the global command
; count variable, stimulus and configuration locations, and starts
; and stops stimuli, transmission, battery measurement and
; acknowledgements.

cmd_execute:

; Push the flags onto the stack and disable interrupts. Allowing interrupts
; while we are configuring a stimulus or a transmission is more challenging
; than simply turning them off and making sure everything is set up properly
; before returning from this routine and popping the flags off the stack 
; again, restoring the interrupt flag (I) to its prior state. 

push F              ; Push flags.
seti                ; Disable interrupts.

; Now we push A, turn on the transmit clock and go into boost, then push all 
; the remaining registers we plan to use.

push A              ; Save A.
ld A,0x01           ; Set bit zero to one.
ld (mmu_etc),A      ; Enable the transmit clock, TCK.
ld (mmu_bcc),A      ; Boost the CPU clock to TCK.
push B
push C
push D
push E
push H
push L
push IX

ld A,(mmu_dfr)       ; A pulse to show that we 
or A,bit1_mask       ; are starting the command
ld (mmu_dfr),A       ; execution.

; Calculate and store the command count in memory. We read the wr_cmd_addr and subtract
; two. We will use the dec_cmd_cnt routine to decrement as we increment through the command
; memory. If the command count is less than zero, we abort.

ld A,(mmu_ccl)
sub A,2
ld (cmd_cnt_l),A
ld A,(mmu_cch)
sbc A,0
ld (cmd_cnt_h),A
jp c,cmd_done

; Load IX with the base of the command memory to start reading bytes.

ld IX,mmu_cmem

; Load the device id into HL and the command's device id into DE.

ld HL,device_id
ld A,(IX)
push A
pop D
inc IX
call dec_cmd_cnt
ld A,(IX)
push A
pop E
inc IX
call dec_cmd_cnt

; Check to see if HL = DE. If so, we'll process this command.

push L
pop B
push E
pop A
sub A,B
jp nz,cmd_no_match
push H
pop B
push D
pop A
sub A,B
jp nz,cmd_no_match
jp cmd_loop_start

; If HL is the wildcard identifier 0xFFFF, we'll process this command.

cmd_no_match:
push E
pop A
sub A,0xFF
jp nz,cmd_done
push D
pop A
sub A,0xFF
jp nz,cmd_done

; Every time we execute this loop, IX should be pointed to the next
; command byte we want to process.

cmd_loop_start:

; The stimulus stop instruction.

check_stop_stim:
ld A,(IX)
sub A,op_stop_stim
jp nz,check_start_stim
inc IX
call dec_cmd_cnt
ld A,0
ld (Srun),A
ld (mmu_stc),A
ld A,(mmu_imsk)
and A,bit1_clr
ld (mmu_imsk),A
jp cmd_loop_end

; The stimulus start instruction.

check_start_stim:
ld A,(IX)
sub A,op_start_stim
jp nz,check_xmit
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read stimulus current.
ld (Scurrent),A
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read pulse length byte one.
ld (Spulse_1),A
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read pulse length byte zero.
ld (Spulse_0),A
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read interval length byte two.
ld (Sinterval_2),A
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read interval length byte one.
ld (Sinterval_1),A
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read interval length byte zero.
ld (Sinterval_0),A
inc IX
call dec_cmd_cnt
ld A,(IX)            ; Read stimulus length byte one.
ld (Slength_1),A
inc IX
call dec_cmd_cnt
ld A,(IX)            ; Read stimulus length byte zero.
ld (Slength_0),A
inc IX
call dec_cmd_cnt
ld A,(IX)            ; Read randomization state
ld (Srandomize),A    ; and write to memory.
inc IX
call dec_cmd_cnt
ld A,0x01            ; Set the
ld (Srun),A          ; stimulus run and
ld (Sistart),A       ; stimulus start flags.
ld A,0               ; Load zero so we can
ld (Sicnt0),A        ; set the stimulus interval
ld (Sicnt1),A        ; counter to
ld (Sicnt2),A        ; zero.
ld (Sprun),A         ; Clear the pulse run flag
ld (Sdrun),A         ; and the delay flag.
ld A,(mmu_dfr)     
and A,bit3_clr    
ld (mmu_dfr),A    
ld A,stim_tick       ; Set stimulus interrupt period by loading
dec A                ; the period, subtracting one, and writing
ld (mmu_it2p),A      ; to the timer register.
ld A,(mmu_imsk)      ; Load the interrupt mask and
or A,bit1_mask       ; set bit one to enable the
ld (mmu_imsk),A      ; stimulus interrupt.
jp cmd_loop_end

; Start data transmission.

check_xmit:
ld A,(IX)
sub A,op_xmit
jp nz,check_ack
inc IX
call dec_cmd_cnt
ld A,(IX)            ; Read transmit period minus one. 
ld (xmit_T),A        ; Save to memory and
ld (mmu_it1p),A      ; write to interrupt timer period.
inc IX
call dec_cmd_cnt
add A,0              ; If period = 0 jump forwards
jp z,xmit_disable    ; to disable.
ld A,(mmu_imsk)      ; If period > 0 enable xmit
or A,bit0_mask       ; interrupt
ld (mmu_imsk),A      ; with mask.
ld HL,xx_delay       ; Load the transmit exitinguish
push H               ; delay into HL
pop A                ; and store
ld (xxcnt1),A        ; in the
push L               ; transmit extinguish
pop A                ; counter locations
ld (xxcnt0), A       ; so we can count them down.
jp cmd_loop_end      
xmit_disable:      
ld A,(mmu_imsk)      ; When period = 0 we disable
and A,bit0_clr       ; the xmit interrupt
ld (mmu_imsk),A      ; with mask.
jp cmd_loop_end

; Acknowledgement request instruction. We read the acknowledgement
; key that will serve as a verification code and call the acknowledgement
; transmit routine, which will take about fifty microseconds.

check_ack:
ld A,(IX)
sub A,op_ack
jp nz,check_battery
inc IX
call dec_cmd_cnt
ld A,(IX)           
ld (Sack_key),A      
inc IX             
call dec_cmd_cnt     
call xmit_ack        
jp cmd_loop_end

; Battery voltage measurement request instruction. This instruction
; takes no parameters. We call the battery measurement routine
; immediately, which will take about fifty microseconds.

check_battery:
ld A,(IX)
sub A,op_battery
jp nz,check_identify
inc IX                
call dec_cmd_cnt     
call xmit_batt       
jp cmd_loop_end

; Identification request instruction. This instruction takes no
; operands. We call the identification transmission routine, which
; will occupy the CPU for up to 650 ms before transmitting a single
; message that gives the device id to any listeners.

check_identify:
ld A,(IX)
sub A,op_identify
jp nz,check_setpcn
inc IX                
call dec_cmd_cnt      
call xmit_identify    
jp cmd_loop_end

; Set the primary channel number for acknowledgements, battery
; measurements and synchronizing signal transmission. We read
; the primary channel number and write to memory.

check_setpcn:
ld A,(IX)
sub A,op_setpcn
jp nz,cmd_done
inc IX 
call dec_cmd_cnt
ld A,(IX)           
ld (xmit_pcn),A      
inc IX               
call dec_cmd_cnt    
jp cmd_loop_end     

; Check the number of bytes remaining to be read. If greater
; than zero, jump back to start of loop, otherwise we are done.

cmd_loop_end:
ld A,(cmd_cnt_h)
add A,0
jp nz,cmd_loop_start
ld A,(cmd_cnt_l)
add A,0
jp nz,cmd_loop_start

; Now that we are done with command processing, we turn
; on device power. It's up to the main loop to turn
; the device off. We reset the command processor too.

cmd_done:
ld A,0x01
ld (mmu_dva),A
ld (mmu_cpr),A

ld A,(mmu_dfr)     
and A,bit1_clr    
ld (mmu_dfr),A    

; Restore most registers.

pop IX
pop L
pop H
pop E
pop D
pop C
pop B

; Un-boost the CPU and exit.

ld A,0x00           
ld (mmu_bcc),A      
ld (mmu_etc),A      
pop A               
pop F               
ret                 

; ------------------------------------------------------------
; The random number generator updates Rand0 and Rand1 with a 
; sixteen-bit linear feedback shift register.

random:

push F
push A

ld A,(Rand1)      ; Rotate Rand1 to the right,
srl A             ; filling top bit with zero,
ld (Rand1),A      ; and placing bottom bit in carry.
ld A,(Rand0)      ; Rotate Rand0 to the right,
rr A              ; filling top bit with carry,
ld (Rand0),A      ; and placing bottom bit in carry.

ld A,(Rand1)      ; Load A with Rand1 again.
jp nc,rand_tz     ; If carry is set, perform the XOR
xor A,rand_taps   ; operation on tap bits and
ld (Rand1),A      ; save to memory.
rand_tz:

pop A
pop F

ret

; ------------------------------------------------------------
; Initiate a pulse, leaving the termination of the pulse to the
; interrupt routine. A pulse consists of a delay, light turning
; on, another delay, and the light turning off.

start_pulse:

push F
push A

; Disable interrupts so we can boost the CPU and read the
; interval counter without risking the interrupt routine
; changing its value while we are reading.
seti

ld A,0x01           ; Set bit zero to one.
ld (mmu_etc),A      ; Enable the transmit clock, TCK.
ld (mmu_bcc),A      ; Boost the CPU clock to TCK.
push B
push C
push D
push E
push H
push L

; Generate a pulse on diagnostic flag two.

ld A,(mmu_dfr)      
or A,bit2_mask     
ld (mmu_dfr),A     

; Add the stimulus interval to the stimulus interval counter.

ld A,(Sinterval_0)  ; Add the stimulus interval
push A              ; to the stimulus counter.
pop B               ; Start with byte zero.
ld A,(Sicnt0)       
add A,B             
ld (Sicnt0),A      
 
ld A,(Sinterval_1)  ; Then byte one. We add with carry
push A              ; to account for overflow from the
pop B               ; first byte.
ld A,(Sicnt1)
adc A,B
ld (Sicnt1),A

ld A,(Sinterval_2)  ; And byte two. If this addition 
push A              ; overflows, that's because we are
pop B               ; adding to a negative number and
ld A,(Sicnt2)       ; restoring a positive number.
adc A,B
ld (Sicnt2),A

ld A,0x00           ; Clear the stimulus start
ld (Sistart),A      ; flag.

ld A,(Spulse_0)     ; Refresh the pulse counter by reading
ld (Spcnt0),A       ; both bytes from memory
ld A,(Spulse_1)     ; and writing to the pulse
ld (Spcnt1),A       ; counter bytes.

; Check the randomize flag.

ld A,(Srandomize)
add A,0
jp z,stp_nr

; Copy the stimulus interval length into scratch variables
; and subtract the pulse length. The result is our maximum 
; delay for randomized pulses.

ld A,(Spulse_0)
push A
pop B
ld A,(Sinterval_0)
sub A,B
ld (scratch1),A
ld A,(Spulse_1)
push A
pop B
ld A,(Sinterval_1)
sbc A,B
ld (scratch2),A
ld A,(Sinterval_2)
sbc A,0
ld (scratch3),A

; Get a random number and stash it in D. This is one
; of our product terms, the other is the stimulus interval.

ld A,(Rand0)        ; Load the random number.
push A              ; and move to B
pop B               ; for multiplication.
push A              ; Also store in D for 
pop D               ; later.

; Multiply the stimulus interval by the random number and
; store that top three bytes of the thirty-two bit product
; in the stimulus delay register.

ld A,(scratch1)     ; Load LO byte of max delay
push A              ; and place in C for
pop C               ; multiplication.
call multiply       ; Let BC := B * C.
push B              ; Store B (first carry byte) in
pop E               ; E for later.
push D              ; Bring back our random
pop B               ; number
ld A,(scratch2)     ; and put MID byte of max delay
push A              ; in
pop C               ; C for multiplication
call multiply       ; Let BC := B * C.
push B              ; Store B (second carry byte) in
pop H               ; H for later.
push C              ; Move C (product byte two)          
pop B               ; to B.
push E              ; Bring first carry out
pop A               ; of E and
add A,B             ; add to obtain delay byte zero,
ld (Sdly0),A        ; this addition never generates a carry.
push D              ; Bring back our random
pop B               ; number
ld A,(scratch3)      ; and put HI byte of max delay
push A              ; in
pop C               ; C for multiplication
call multiply       ; Let BC := B * C.
push B              ; Store B (third carry byte) in
pop E               ; E for later.
push C              ; Move C (product byte three)
pop B               ; to B.
push H              ; Bring second carry out
pop A               ; of H and
add A,B             ; add to obtain delay byte one,
ld (Sdly1),A        ; this addition never generates a carry.
push E              ; Bring back third carry byte
pop A               ; and use as
ld (Sdly2),A        ; delay byte two.

; Set the delay run flag, make sure pulse run is clear.

ld A,1
ld (Sdrun),A
ld A,0
ld (Sprun),A
ld A,(mmu_dfr)     
and A,bit3_clr    
ld (mmu_dfr),A    
jp stp_done

; Without randomization, we start the pulse right away.

stp_nr:
ld A,(Scurrent)     ; Load the pulse stimulus current and 
ld (mmu_stc),A      ; start the pulse.
ld A,1              ; Set the
ld (Sprun),A        ; pulse run flag.
ld A,(mmu_dfr)     
or A,bit3_mask   
ld (mmu_dfr),A    
ld A,0              ; Clear the
ld (Sdrun),A        ; delay run flag.

stp_done:

; End pulse on diagnostic flag two.

ld A,(mmu_dfr)     
and A,bit2_clr    
ld (mmu_dfr),A    

pop L
pop H
pop E
pop D
pop C
pop B
ld A,0x00           ; Load a zero and use it to
ld (mmu_bcc),A      ; move CPU out of boost and
ld (mmu_etc),A      ; stop the transmit clock.
pop A 
pop F
ret                 ; return.

; ------------------------------------------------------------
; The main program. We begin by initializing the device, which
; includes initializing the stack pointer, variables, and interrupts.

main:

; Initialize the stack pointer.
ld HL,mmu_sba
ld SP,HL

; Initialize variable locations to zero. This activity also serves
; as a boot-up delay to let the power supply settle before we
; calibrate the transmit clock. We are clearing all flags.

ld IX,mmu_vmem
ld A,num_vars
push A
pop B
ld A,0
main_vclr:
ld (IX),A
inc IX
dec B
jp nz,main_vclr

; Configure some registers.

ld A,0             ; Make sure the stimulus
ld (mmu_stc),A     ; current is zero.
ld HL,device_id    ; Set the primary channel number
push L             ; equal to the low byte
pop A              ; of the
ld (xmit_pcn),A    ; device identifier.
ld (Rand0),A       ; And seed the random number generator
push H             ; with the
pop A              ; device
ld (Rand1),A       ; identifier.
ld A,tx_calib      ; Set the radio frequency for
ld (mmu_xfc),A     ; transmission to calibration value.

; Calibrate the transmit clock.

call calibrate_tck

; Reset and disable all interrupts.

ld A,0xFF            ; Load A with ones
ld (mmu_irst),A      ; and reset all interrupts.
ld A,0x00            ; Load zeros
ld (mmu_imsk),A      ; and disable all interrupts.

; The main event loop.

main_loop:

; Deal with any pending commands.

ld A,(mmu_sr)       ; Fetch status register.
and A,sr_cmdrdy     ; Check the command ready bit.
jp z,main_nocmd     ; Jump if it's clear,
call cmd_execute    ; execute command if it's set.
main_nocmd:

; Update the random number.

call random

; Check to see if the stimulus is running, and if we are at the
; start of an interval we will decrement the stimulus length 
; counter and see if we are done with our stimulus.

ld A,(Srun)
and A,0x01
jp z,main_nostim
ld A,(Sistart)
and A,0x01
jp z,main_nostim

; Decrement the stimulus length counter, which is the number of pulses
; that remain in the stimulus. Jump forwards if it is still positive.

ld A,(Slength_0)
sub A,1
ld (Slength_0),A
ld A,(Slength_1)
sbc A,0
ld (Slength_1),A
jp nc,main_pulse

; The stimulus is complete. We disable the stimulus clock interrupt,
; we set Srun to zero.

ld A,(mmu_imsk)      
and A,bit1_clr      
ld (mmu_imsk),A      
ld A,0
ld (mmu_it2p),A   
ld (Srun),A
jp main_nostim

; Start a new stimulus pulse.

main_pulse:
call start_pulse

; Check to see if we should still be running. If so, repeat the
; main loop. 

main_nostim:
ld A,(xmit_T)
add A,0
jp nz,main_loop
ld A,(Srun)
add A,0
jp nz,main_loop

; Switch off. We could wait here, but jumping back to the start
; of the loop makes the code more robust: if something goes wrong
; with the turn-off, the device is still watching for commands.

ld A,0
ld (mmu_dva),A
jp main_loop

; ---------------------------------------------------------------
