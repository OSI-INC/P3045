; Fiber Controller (P3045) Softare
; --------------------------------

; This code runs in the OSR8V3 microprocessor of the A3045A.

; V1: Based upon P3041 v1.6. Change address map to match firmware.
; Remove uneccessary code. Ramp up North, South, East, West values.

; Calibration Constants.
const device_id  0xA123 ; Bottom nibble must be 1-14.

; Address Map Boundary Constants
const mmu_vmem 0x0000 ; Base of Variable Memory
const mmu_cmem 0x0400 ; Base of Command Memory
const mmu_ctrl 0x0800 ; Base of Control Space
const mmu_sba  0x0300 ; Stack Base Address

; Address Map Locations
const mmu_irqb 0x0801 ; Interrupt Request Bits
const mmu_imsk 0x0802 ; Interrupt Mask Bits
const mmu_irst 0x0803 ; Interrupt Reset Bits
const mmu_rst  0x0804 ; System Reset
const mmu_dfr  0x0805 ; Diagnostic Flag Register
const mmu_sr   0x0806 ; Status Register
const mmu_it1p 0x0807 ; Interrupt Timer One Period
const mmu_it2p 0x0808 ; Interrupt Timer Two Period
const mmu_it3p 0x0809 ; Interrupt Timer Three Period
const mmu_it4p 0x080A ; Interrupt Timer Four Period
const mmu_cch  0x080B ; Command Count HI
const mmu_ccl  0x080C ; Command Count LO
const mmu_cpr  0x080D ; Command Processor Reset
const mmu_edc  0x080E ; Enable DAC Clock
const mmu_dcc  0x080F ; DAC Clock Calibration
const mmu_ndh  0x0820 ; North Digital HI Byte
const mmu_ndl  0x0821 ; North Digital LO Byte
const mmu_sdh  0x0822 ; South Digital HI Byte
const mmu_sdl  0x0823 ; South Digital LO Byte
const mmu_edh  0x0824 ; East Digital HI Byte
const mmu_edl  0x0825 ; East Digital LO Byte
const mmu_wdh  0x0826 ; West Digital HI Byte
const mmu_wdl  0x0827 ; West Digital LO Byte
const mmu_ds   0x0828 ; DAC Counter Step

; Status Bit Masks, for use with status register.
const sr_cmdrdy  0x01 ; Command Ready Flag
const sr_endck   0x02 ; DAC Clock Enabled
const sr_cpa     0x10 ; Command Processor Active

; Bit Masks
const bit0_mask  0x01 ; Bit Zero Mask
const bit1_mask  0x02 ; Bit One Mask
const bit2_mask  0x04 ; Bit Two Mask
const bit3_mask  0x08 ; Bit Three Mask
const bit0_clr   0xFE ; Bit Zero Clear
const bit1_clr   0xFD ; Bit One Clear
const bit2_clr   0xFB ; Bit Two Clear
const bit3_clr   0xF7 ; Bit Three Clear

; Counter constants
const num_vars   200 ; Number of vars to clear at start.
const dck_divisor  8 ; Divisor for the ring oscillator. 
const dac_step     8 ; Increment for sixteen-bit DAC counter.

; DAC Shadow Variables.
const Ndh         0x0000 ; North HI Byte
const Ndl         0x0001 ; North LO Byte
const Sdh         0x0002 ; South HI Byte
const Sdl         0x0003 ; South LO Byte
const Edh         0x0004 ; East HI Byte
const Edl         0x0005 ; East LO Byte
const Wdh         0x0006 ; West HI Byte
const Wdl         0x0007 ; West LO Byte

; Command Execution Variables
const cmd_cnt_h   0x0020 ; Command Count, HI
const cmd_cnt_l   0x0021 ; Command Count, LO

; Random Number Variabls
const Rand1       0x0030 ; Random Number Byte One
const Rand0       0x0031 ; Random Number Byte Zero

; Other Flags
const selftest    0x0032 ; Selftest Flag

; Global Scratch Registers Variables.
const scratch1    0x0050 ; Scratchpad Variable 1
const scratch2    0x0051 ; Scratchpad Variable 2
const scratch3    0x0052 ; Scratchpad Variable 3
const scratch4    0x0053 ; Scratchpad Variable 4
const scratch5    0x0054 ; Scratchpad Variable 5
const scratch6    0x0055 ; Scratchpad Variable 6

; Command constants.
const op_shutdown    0 ; 0 operands
const op_set_north   1 ; 2 operands
const op_set_south   2 ; 2 operands
const op_set_east    3 ; 2 operands
const op_set_west    4 ; 2 operands
const op_ack         5 ; 1 operand
const op_identify    6 ; 0 operands
const op_diagnostic  7 ; 0 operands
const op_selftest    8 ; 0 operands
const wildcard_h  0xFF ; Wildcard Identifier, HI
const wildcard_l  0xFF ; Wildcard Identifier, LO

; Random Number Generator.
const rand_taps   0xB4 ; Determines which taps to XOR.

; ------------------------------------------------------------
; The CPU reserves two locations 0x0000 for the start of program
; execution, and 0x0003 for interrupt execution. We put jumps at
; both locations. A jump takes exactly three bytes.
; ------------------------------------------------------------

start:

jp initialize
jp interrupt

; ---------------------------------------------------------------
; Eight-bit multiplier. Load two eight-bit operands into B and C
; and the sixteen-bit result will be returned in B (HI) and C (LO). 
; Takes 200 to 300 clock cycles depending upon the operand C, an 
; average of 250, which is 50 us at 5 MHz or 7.6 ms at 32.768 kHz.
; ------------------------------------------------------------

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
; Interrupt routine. Right now this is a dummy routine that
; marks its execution with a diagnostic flag and clears the
; interrupt bits.
; ------------------------------------------------------------

interrupt:

; Push A and F, set diagnostic flag bit.

push F              ; Save flags on stack
push A              ; Save A on stack.
ld A,(mmu_dfr)      ; Load the diagnostic flag register.
or A,bit2_mask      ; set bit zero and
ld (mmu_dfr),A      ; write to diagnostic flag register.

; Reset interrupts.

ld A,0xFF            ; Load A with ones
ld (mmu_irst),A      ; and reset all interrupts.

; Clear diagnostic flag, pop F and A and return from interrupt.

ld A,(mmu_dfr)      ; Load the diagnostic flag register.
and A,bit2_clr      ; Clear bit zero and
ld (mmu_dfr),A      ; write to diagnostic flag register.
pop A               ; Restore the A.
pop F               ; Restore flags.
rti                 ; Return from interrupt.

; -----------------------------------------------------------
; Subroutine: Decrement the Command Count. The decrement does 
; not allow the count to drop below zero. The routine leaves 
; all registers intact.
; -----------------------------------------------------------

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
; Subroutine: Transmit an Acknowledgement. A dummy routine for
; now, until we figure out the transmit protocol.
; -----------------------------------------------------------

xmit_ack:

push F
push A

pop A
pop F

ret

; ------------------------------------------------------------
; Subroutine: Transmit Identifier. A dummy routine for now,
; until we figure out the transmit protocol.
; -----------------------------------------------------------

xmit_identify:

push F
push A

pop A
pop F

ret

; ------------------------------------------------------------
; Subroutine: Comamnd Execute. We read out, interpret, and 
; execute comands stored in the command buffer. The routine
; presupposes that a command is available in the buffer. All
; Commands begin with an two-byte identifier, one or more
; instructions and a two-byte checksum. The checksum has 
; already been compared to the value we obtain from the 
; preceding bytes, so we do not check it here. We instead
; examine the identifier and proceed through the instructions 
; only if the identifier matches this device's identifier or
; is the wildcard identifier. Instructions consist of a sigle-
; byte opcode and one or more bytes of operands.
; ------------------------------------------------------------


cmd_execute:

; Push the flags and registers onto the stack and disable interrupts. Allowing 
; interrupts while we are decyphering a command creates too many potential 
; problems to be worth bothering with. We'll restore the interrupt disable flag
; to its original state when we pop the flags off the stack upon return.

push F              
push A             
push B
push C
push D
push E
push H
push L
push IX
push IY
seti 

; Set a diagnostic flag.

ld A,(mmu_dfr)       
or A,bit1_mask      
ld (mmu_dfr),A   

; Clear the self-test flag.

ld A,0x00
ld (selftest),A  

; Calculate and store the command count in memory. We read the wr_cmd_addr and 
; subtract two. The result is the number of commands without the two checksum 
; bytes at the end. We will use the dec_cmd_cnt routine to decrement as we increment 
; through the command memory. If the command count is less than zero, we abort.

ld A,(mmu_ccl)
sub A,2
ld (cmd_cnt_l),A
ld A,(mmu_cch)
sbc A,0
ld (cmd_cnt_h),A
jp c,cmd_done

; Load IX with the base of the command memory to start reading bytes.

ld IX,mmu_cmem

; Load the device identifier into HL and the command's identifier 
; into DE. We leave IX pointing to the first instruction opcode.

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

; If HL is the wildcard identifier, we'll process this command.

cmd_no_match:
push E
pop A
sub A,wildcard_l
jp nz,cmd_done
push D
pop A
sub A,wildcard_h
jp nz,cmd_done

; Every time we execute this loop, IX should be pointing to the next
; instruction opcode.

cmd_loop_start:

; The shutdown command we don't yet know what to do with.

check_op_shutdown:
ld A,(IX)
sub A,op_shutdown
jp nz,check_set_north
inc IX
call dec_cmd_cnt
; Shutdown code will go here.
jp cmd_loop_end

; Set north takes a two-byte operand, writes the first byte to the
; North DAC HI byte, and the second byte to the North DAC LO byte.
; We also write to the shadow registers, which allows us to read back
; the DAC values in other routines.

check_set_north:
ld A,(IX)
sub A,op_set_north
jp nz,check_set_south
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read HI byte of North
ld (mmu_ndh),A      ; and write to DAC
ld (Ndh),A          ; and shadow register.        
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read LO byte of North
ld (mmu_ndl),A      ; and write to DAC
ld (Ndl),A          ; and shadow register.
inc IX
call dec_cmd_cnt 
jp cmd_loop_end

; Set south sets the South DAC value.

check_set_south:
ld A,(IX)
sub A,op_set_south
jp nz,check_set_east
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read HI byte of South
ld (mmu_sdh),A      ; and write to DAC
ld (Sdh),A          ; and shadow register.
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read LO byte of South
ld (mmu_sdl),A      ; and write to DAC
ld (Sdl),A          ; and shadow register.
inc IX
call dec_cmd_cnt 
jp cmd_loop_end

; Set east sets the East DAC value.

check_set_east:
ld A,(IX)
sub A,op_set_east
jp nz,check_set_west
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read HI byte of East
ld (mmu_edh),A      ; and write to DAC
ld (Edh),A          ; and shadow register.
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read LO byte of East
ld (mmu_edl),A      ; and write to DAC
ld (Edl),A          ; and shadow register.
inc IX
call dec_cmd_cnt 
jp cmd_loop_end

; Set west sets the West DAC value.

check_set_west:
ld A,(IX)
sub A,op_set_west
jp nz,check_ack
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read HI byte of West
ld (mmu_wdh),A      ; and write to DAC
ld (Wdh),A          ; and shadow register
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read LO byte of West
ld (mmu_wdl),A      ; and write to DAC
ld (Wdl),A          ; and shadow register.
inc IX
call dec_cmd_cnt 
jp cmd_loop_end

; Acknowledge calls the transmit acknowledgement routine.

check_ack:
ld A,(IX)
sub A,op_ack
jp nz,check_identify
inc IX
call dec_cmd_cnt
call xmit_ack
jp cmd_loop_end

; The identify instruction calls the transmit identifier routine.

check_identify:
ld A,(IX)
sub A,op_identify
jp nz,check_selftest
inc IX
call dec_cmd_cnt
call xmit_identify
jp cmd_loop_end

; The self-test instruction sets the self-test flag and the main loop
; will execute the self-test routine. Note that we clear this flag at
; the start of the command interpreter, so the absence of a self-test
; instruction ends the self-test behavior. This is the last opcode in 
; our list. If we don't get match, we abort by jumping to command done.

check_selftest:
ld A,(IX)
sub A,op_selftest
jp nz,cmd_done
inc IX
call dec_cmd_cnt
ld A,0xFF
ld (selftest),A
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

; Now that we are done with command processing, or we are aborting
; processing due to an unrecognised opcode, we reset the command 
; processor and end our pulse on diagnostic flag.

cmd_done:
ld A,0x01
ld (mmu_cpr),A
ld A,(mmu_dfr)     
and A,bit1_clr    
ld (mmu_dfr),A    

; Restore registers and return from subroutine. Note that when
; we pop the flags off the stack, we will pop the previous state
; of the interrupt disable flag into the flag register.

pop IY
pop IX
pop L
pop H
pop E
pop D
pop C
pop B
pop A
pop F
ret                 

; ------------------------------------------------------------
; The random number generator updates Rand0 and Rand1 with a 
; sixteen-bit linear feedback shift register.
; ------------------------------------------------------------

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
; The initialization routine. We set the stack pointer, zero 
; variables, and configure interrupts.
; ------------------------------------------------------------

initialize:

; Initialize the stack pointer.
ld HL,mmu_sba
ld SP,HL

; Initialize variable locations to zero. This sets all flags to
; zero as well.

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

; Reset and disable all interrupts.

ld A,0xFF            ; Load A with ones
ld (mmu_irst),A      ; and reset all interrupts.
ld A,0x00            ; Load zeros
ld (mmu_imsk),A      ; and disable all interrupts.

; Configure the DAC clock and the digital to analog converters.

ld A,dck_divisor   ; Set the DAC clock divisor,
ld (mmu_dcc),A     ; which determines the DAC Clock frequency.
ld A,1             ; Enable the fast clock and therefore
ld (mmu_edc),A     ; the DAC Clock as well.
ld A,dac_step      ; Set the resolution and frequency of
ld (mmu_ds),A      ; of the duty-cycle DACs.

; Jump to the main event loop.

jp main

; ------------------------------------------------------------
; Main event loop. 
; ------------------------------------------------------------

main:

; Mark start of execution with a pulse.

ld A,(mmu_dfr)     
or A,bit0_mask    
ld (mmu_dfr),A  
and A,bit0_clr
ld (mmu_dfr),A  

; Deal with any pending commands.

ld A,(mmu_sr)       ; Fetch status register.
and A,sr_cmdrdy     ; Check the command ready bit.
jp z,main_nocmd     ; Jump if it's clear,
call cmd_execute    ; execute command if it's set.
main_nocmd:

; Update the random number.

call random

; If the selftest flag is set, call the selftest routine.

ld A,(selftest)
add A,0
jp z,main_no_selftest
call selftest
main_no_selftest:

; Jump back to start of main loop.

jp main

; ---------------------------------------------------------------
; Subroutine: Self-Test. We increment two DAC values and decrement 
; the other two, at two different rates, so we can watch the DAC and 
; amplifier outputs over their entire range for diagnostics.
; ---------------------------------------------------------------

selftest:

; Push accumulator and flags.

push F
push A

; Increment the North DAC value.

ld A,(Ndl)
add A,8
ld (Ndl),A
ld (mmu_ndl),A
ld A,(Ndh)
adc A,0
ld (Ndh),A
ld (mmu_ndh),A

; Decrement the South DAC value.

ld A,(Sdl)
sub A,8
ld (Sdl),A
ld (mmu_sdl),A
ld A,(Sdh)
sbc A,0
ld (Sdh),A
ld (mmu_sdh),A

; Increment the East DAC value.

ld A,(Edl)
add A,32
ld (Edl),A
ld (mmu_edl),A
ld A,(Edh)
adc A,0
ld (Edh),A
ld (mmu_edh),A

; Decrement the West DAC value.

inc_Wd1:
ld A,(Wdl)
sub A,32
ld (Wdl),A
ld (mmu_wdl),A
ld A,(Wdh)
sbc A,0
ld (Wdh),A
ld (mmu_wdh),A

; Pop and return.

pop A
pop F
ret
