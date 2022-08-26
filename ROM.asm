; Fiber Controller (P3045) Softare
; --------------------------------

; This code runs in the OSR8V3 microprocessor of the A3045A.

; V1: Based upon P3041 v1.6. Change address map to match firmware.
; Remove uneccessary code. Ramp up North, South, East, West values.

; Calibration Constants.
const device_id  0xA123 ; Bottom niblle 1-14.

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
const fck_divisor  8 ; Divisor for the ring oscillator. 
const dac_step     8 ; Increment for sixteen-bit DAC counter.

; DAC Shadow Variables
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

; Global Scratch Registers Variables.
const scratch1    0x0050 ; Scratchpad Variable 1
const scratch2    0x0051 ; Scratchpad Variable 2
const scratch3    0x0052 ; Scratchpad Variable 3
const scratch4    0x0053 ; Scratchpad Variable 4
const scratch5    0x0054 ; Scratchpad Variable 5
const scratch6    0x0055 ; Scratchpad Variable 6

; Operation Codes. Operands are bytes. A two-byte value
; passed with an instruction is broken into hi and lo
; bytes, so is two operands.
const op_shutdown    0 ; 0 operands
const op_set_north   1 ; 2 operands
const op_set_south   2 ; 2 operands
const op_set_east    3 ; 2 operands
const op_set_west    4 ; 2 operands
const op_ack         5 ; 1 operand
const op_identify    6 ; 0 operands

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
; The interrupt routine. 

interrupt:

; Push A and F, set diagnostic flag bit.

push F              ; Save flags on stack
push A              ; Save A on stack.
ld A,(mmu_dfr)      ; Load the diagnostic flag register.
or A,bit2_mask      ; set bit zero and
ld (mmu_dfr),A      ; write to diagnostic flag register.


; Clear diagnostic flag, pop F and A and return from interrupt.

ld A,(mmu_dfr)      ; Load the diagnostic flag register.
and A,bit2_clr      ; Clear bit zero and
ld (mmu_dfr),A      ; write to diagnostic flag register.
pop A               ; Restore the A.
pop F               ; Restore flags.
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
; Transmit an acknowledgement. 

xmit_ack:

push F
push A

pop A
pop F

ret

; ------------------------------------------------------------
; Transmit an identification message. 

xmit_identify:

push F
push A

pop A
pop F

ret

; ------------------------------------------------------------
; Read out, interpret, and execute comands.

cmd_execute:

; Push the flags and registers onto the stack and disable interrupts. Allowing 
; interrupts while we are decyphering a command creates too many potential 
; problems to be worth bothering with. We'll restore the interrupt disable flag
; to its original state when we pop the flags off the stack before we return.

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

check_op_shutdown:
ld A,(IX)
sub A,op_shutdown
jp nz,check_set_north
inc IX
call dec_cmd_cnt
jp cmd_loop_end

check_set_north:
ld A,(IX)
sub A,op_set_north
jp nz,cmd_done
inc IX
call dec_cmd_cnt
ld A,(IX)           ; Read HI byte of North
inc IX
ld (Ndh),A
call dec_cmd_cnt
ld A,(IX)           ; Read LO byte of North
inc IX
ld (Ndl),A
call dec_cmd_cnt 

; Check the number of bytes remaining to be read. If greater
; than zero, jump back to start of loop, otherwise we are done.

cmd_loop_end:
ld A,(cmd_cnt_h)
add A,0
jp nz,cmd_loop_start
ld A,(cmd_cnt_l)
add A,0
jp nz,cmd_loop_start

; Now that we are done with command processing, we reset the 
; command processor and end our pulse on diagnostic flag.

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

; Reset and disable all interrupts.

ld A,0xFF            ; Load A with ones
ld (mmu_irst),A      ; and reset all interrupts.
ld A,0x00            ; Load zeros
ld (mmu_imsk),A      ; and disable all interrupts.

; Configure the firmware.

ld A,fck_divisor   ; Set the fast clock divisor,
ld (mmu_dcc),A     ; which determines the DAC Clock frequency.
ld A,1             ; Enable the fast clock and therefore
ld (mmu_edc),A     ; the DAC Clock as well.
ld A,dac_step      ; Set the resolution and frequency of
ld (mmu_ds),A      ; of the duty-cycle DACs.

; The main event loop.

main_loop:

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

; Increment the North DAC value.

ld A,(Ndl)
add A,1
ld (Ndl),A
ld (mmu_ndl),A
ld A,(Ndh)
adc A,0
ld (Ndh),A
ld (mmu_ndh),A

; Increment the South DAC value.

ld A,(Sdl)
add A,8
ld (Sdl),A
ld (mmu_sdl),A
ld A,(Sdh)
adc A,0
ld (Sdh),A
ld (mmu_sdh),A

; Increment the East DAC value.

ld A,(Edl)
add A,64
ld (Edl),A
ld (mmu_edl),A
ld A,(Edh)
adc A,0
ld (Edh),A
ld (mmu_edh),A

; Increment the West DAC value.

ld A,(Wdl)
add A,0
ld (Wdl),A
ld (mmu_wdl),A
ld A,(Wdh)
adc A,2
ld (Wdh),A
ld (mmu_wdh),A

; Jump back to start of main loop.

jp main_loop

; ---------------------------------------------------------------
