.include "m2560def.inc"

.def temp = r16
.def temp1=r17
.def temp2=r18

.dseg
    .org 0x200 ; set origin of data segment to 0x200
    get_on: .byte 1
    get_off: .byte 1
.cseg 

.org 0x00 
jmp RESET 

.org INT0addr ; INT0addr is the address of EXT_INT0 (External Interrupt 0)
jmp EXT_INT0 ; interrupt vector for External Interrupt 0
.org INT1addr ; INT1addr is the address of EXT_INT1 (External Interrupt 1)
jmp EXT_INT1 ; interrupt vector for External Interrupt 1

RESET:
    ldi r16, low(RAMEND)
    out SPL, r16
    ldi r16, high(RAMEND)
    out SPH, r16
    sei ; Enable the global interrupt
    jmp main


; set get_off to 1 if the button triggered
EXT_INT0: ; Interrupt handler for External Interrupt 0
    push temp
    in temp, SREG
    push temp

    ldi xl, low(get_off) ; Let x pointer point tp get_off
    ldi xh, high(get_off)
    ldi temp,1
    st x,temp

    pop temp
    out SREG, temp
    pop temp
    reti


; set get_on to 1 if the button triggered
EXT_INT1: ; Interrupt handler for External Interrupt 1
    push temp
    in temp, SREG
    push temp

    ldi xl, low(get_on) ; Let x pointer point tp get_on
    ldi xh, high(get_on)
    ldi temp,1
    st x,temp

    pop temp
    out SREG, temp
    pop temp
    reti
