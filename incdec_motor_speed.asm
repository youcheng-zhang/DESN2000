;
; Project.asm
;
; Created: 2020/7/26 12:26:18
; Author : youcheng
;

; A function which will print the name of next station

.include "m2560def.inc"

.def temp = r16
.def temp1=r17
.def temp2=r18
.dseg

.cseg 

.org 0x00 
jmp RESET 


RESET:
ldi r16, low(RAMEND)
out SPL, r16
ldi r16, high(RAMEND)
out SPH, r16

ser temp
out DDRE, temp; set PORTE as output
;ldi temp, 0b00000100; this value and the operation mode determines the PWM duty cycle
ldi temp, 0
sts OCR3BL, temp	; OC3B low register
clr temp
sts OCR3BH, temp	; 0C3B high register
ldi temp, (1<<CS30) ; CS30 = 1: no prescaling
sts TCCR3B, temp	; set the prescaling value
ldi temp, (1<<WGM30)|(1<<COM3B1)
; WGM30=1: phase correct PWM, 8 bits
; COM3B1=1: make OC3B override the normal port functionality of the I/O pin PE2
sts TCCR3A, temp
sei

ser temp 
clr temp 
out DDRD, temp 
out PORTD, temp 
ldi temp, (2 << ISC10) | (2 << ISC00)	; set for falling edge 
sts EICRA, temp 
in temp, EIMSK 
ori temp, (1<<INT0) | (1<<INT1) 
out EIMSK, temp 
sei 
	
jmp main 


main:
    call decrease_speed
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
rjmp main

increase_speed:
    push temp
    in temp, SREG
    push temp
    push temp1
    push temp2

    ldi temp1,10
    ldi temp,0
    ldi temp2,10
increase_speed_loop:
    sts OCR3BL, temp1
    sts OCR3BH, temp
    cpi temp1,250
    brlt end_increase_speed_loop
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    add temp1,temp2
    rjmp increase_speed_loop
end_increase_speed_loop:

    pop temp2
    pop temp1
    pop temp
    out SREG, temp
    pop temp
    ret



decrease_speed:
    push temp
    in temp, SREG
    push temp
    push temp1
    push temp2

    ldi temp1,250
    ldi temp,0
    ldi temp2,10
decrease_speed_loop:
    sts OCR3BL, temp1
    sts OCR3BH, temp
    cpi temp1,0
    brge end_decrease_speed_loop
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    call delay
    sub temp1,temp2
    rjmp decrease_speed_loop
end_decrease_speed_loop:
    ldi temp1,0
    ldi temp,0
    sts OCR3BL, temp1
    sts OCR3BH, temp

    pop temp2
    pop temp1
    pop temp
    out SREG, temp
    pop temp
    ret
	

delay: ; for debouncing switches
    push temp
    in temp, SREG
    push temp
    push temp1
    push temp2
delay1: 
    clr temp1
    inc temp
    cpi temp, 200
    breq leave
delay2:
    clr temp2
    inc temp1
    cpi temp1, 100
    breq delay1
delay3:
    cpi temp2, 50
    breq delay2
    nop
    inc temp2
    rjmp delay3
leave:
    clr temp
    clr temp1
    clr temp2
    pop temp2
    pop temp1
    pop temp
    out SREG, temp
    pop temp
ret