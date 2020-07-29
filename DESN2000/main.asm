;
; DESN2000.asm
;
; Created: 14/07/2020 1:41:26 PM
; Author : Group 8 
;

; A function which will print the name of next station

.include "m2560def.inc"

.def temp=r16
.def temp1=r17
.def temp2=r18
.equ PATTERN = 0b11  ; define a pattern for 8 LEDs
.def  leds = r19   ; r17 stores a LED pattern

; macros
.macro do_lcd_command
	ldi r16, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro
.macro do_lcd_data
	mov r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro

.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

.macro lcd_set
	sbi PORTA, @0
.endmacro
.macro lcd_clr
	cbi PORTA, @0
.endmacro

.macro clear
ldi YL, low(@0)     ; load the memory address to Y pointer
ldi YH, high(@0)
 clr temp            ; set temp to 0 
 st Y+, temp       ; clear the two bytes at @0 in SRAM
 st Y, temp
.endmacro


.dseg
    .org 0x200 ; set origin of data segment to 0x200
    current_station: .byte 1
    get_on: .byte 1
    get_off: .byte 1
	SecondCounter: .byte 2     ; two-byte counter for counting seconds.
	TempCounter: .byte 2       ; temporary counter used to determine if one second has passed



.cseg

	.org 0x0000
	.org INT0addr ; INT0addr is the address of EXT_INT0 (External Interrupt 0)
		jmp EXT_INT0 ; interrupt vector for External Interrupt 0
	.org INT1addr ; INT1addr is the address of EXT_INT1 (External Interrupt 1)
		jmp EXT_INT1 ; interrupt vector for External Interrupt 1
	.org OVF0addr      ; OVF0addr is the address of Timer0 Overflow Interrupt Vector
		jmp Timer0OVF   ; jump to the interrupt handler for Timer0 overflow.
		jmp DEFAULT     ; default service for all other interrupts.
	DEFAULT: reti     ; no interrupt handling 
    ; every station take 20bytes, end with "!"
    stations: .db "Daring Harbour!     City Marine!        White Bay!          Simmons Point!      SYD Observatory!    SYD Aquarium! "

rjmp RESET

RESET:

    ; initialize the LCD
    ldi r16, low(RAMEND)
	out SPL, r16
	ldi r16, high(RAMEND)
	out SPH, r16

	ser r16
	out DDRF, r16
	out DDRA, r16
	clr r16
	out PORTF, r16
	out PORTA, r16

	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off?
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001110 ; Cursor on, bar, no blink
	
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

	ser temp       ; set Port C as output
	out DDRC, temp
	
	rjmp main



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




Timer0OVF:     ; interrupt subroutine to Timer0
       in temp, SREG
       push temp      ; prologue starts
       push YH        ; save all conflicting registers in the prologue
       push YL
       push r25
       push r24        ; prologue ends
       ; Load the value of the temporary counter
       lds r24, TempCounter
       lds r25, TempCounter+1
       adiw r25:r24, 1      ; increase the temporary counter by one
       cpi r24, low(1302)         ; check if (r25:r24) = 1302
       ldi temp, high(1302)      ; 7812 = 106/128
       cpc r25, temp
       brne NotSecond
       cpi leds,pattern
       breq intr_led_off
       ldi leds,PATTERN
       rjmp end_intr_led_off
intr_led_off:
       ldi leds,0
end_intr_led_off:
       out PORTC, leds
       clear TempCounter      ; reset the temporary counter
       ; Load the value of the second counter
       lds r24, SecondCounter
       lds r25, SecondCounter+1
       adiw r25:r24, 1            ; increase the second counter by one
       sts SecondCounter, r24
       sts SecondCounter+1, r25
       rjmp EndIF
NotSecond:   ; store the new value of the temporary counter
       sts TempCounter, r24
       sts TempCounter+1, r25
EndIF: pop r24        ; epilogue starts
       pop r25        ; restore all conflicting registers from the stack
       pop YL
       pop YH
       pop temp
       out SREG, temp
       reti               ; return from the interrupt





main:

    ; initialize current station
    ldi xl, low(current_station) ; Let x pointer point tp current_station
    ldi xh, high(current_station)
    ldi temp, 3
    st x,temp
    call print_next_station
	call LDE_on
halt:
    rjmp halt







print_next_station:  
    push temp
    in temp, SREG
    push temp
    push temp1
    ;current station +1 (=0 if currentstation=5)
    ldi xl, low(current_station) ; Let x pointer point tp current_station
    ldi xh, high(current_station)
    ld temp, x
    cpi temp,5
    brge reset_current_station
    ldi temp1,1
    add temp,temp1
    st x,temp
    rjmp end_reset_current_station
reset_current_station:
    ldi temp,0
    st x,temp
end_reset_current_station:
    ; temp now contains the index of the next station
	do_lcd_command 0b00000001 ; clear display
    ldi zl, low(stations<<1) ; Let z pointer point
    ldi zh, high(stations<<1) ; to the start of stations
    ldi temp1,20
    ld temp, x
    mul temp,temp1
    add zl,r0
    adc zh,r1
print_loop:
    lpm temp1, z+
    cpi temp1, 33 ; if temp1 == "!"
    breq end_print
    do_lcd_data temp1
    rjmp print_loop
end_print:
    pop temp1
    pop temp
    out SREG, temp
    pop temp
    ret



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
	



LDE_on:
    push temp
    in temp, SREG
    push temp
    push leds

	ldi leds, 0xFF          ; main program starts here
	out PORTC, leds     ; set all LEDs on at the beginning
	ldi leds, PATTERN 
	clear TempCounter         ; initialize the temporary counter to 0
	clear SecondCounter      ; initialize the second counter to 0
	ldi temp, 0b00000000
	out TCCR0A, temp
	ldi temp, 0b00000010
	out TCCR0B, temp          ; set prescalar value to 8
	ldi temp, 1<<TOIE0        ; TOIE0 is the bit number of TOIE0 which is 0   
	sts TIMSK0, temp           ; enable Timer0 Overflow Interrupt
	sei                                    ; enable global interrupt

    pop leds
    pop temp
    out SREG, temp
    pop temp
    ret





LED_off:
    push temp
    in temp, SREG
    push temp

	ldi temp, 0
	sts TIMSK0, temp           ; disable Timer0 Overflow Interrupt

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


    
;
; Send a command to the LCD (r16)
;

lcd_command:
	out PORTF, r16
	nop
	lcd_set LCD_E
	nop
	nop
	nop
	lcd_clr LCD_E
	nop
	nop
	nop
	ret

lcd_data:
	out PORTF, r16
	lcd_set LCD_RS
	nop
	nop
	nop
	lcd_set LCD_E
	nop
	nop
	nop
	lcd_clr LCD_E
	nop
	nop
	nop
	lcd_clr LCD_RS
	ret

lcd_wait:
	push r16
	clr r16
	out DDRF, r16
	out PORTF, r16
	lcd_set LCD_RW
lcd_wait_loop:
	nop
	lcd_set LCD_E
	nop
	nop
        nop
	in r16, PINF
	lcd_clr LCD_E
	sbrc r16, 7
	rjmp lcd_wait_loop
	lcd_clr LCD_RW
	ser r16
	out DDRF, r16
	pop r16
	ret

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
	push r24
	push r25
	ldi r25, high(DELAY_1MS)
	ldi r24, low(DELAY_1MS)
delayloop_1ms:
	sbiw r25:r24, 1
	brne delayloop_1ms
	pop r25
	pop r24
	ret

sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	ret