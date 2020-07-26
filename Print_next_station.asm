;
; Project.asm
;
; Created: 2020/7/26 12:26:18
; Author : youcheng
;

; A function which will print the name of next station

.include "m2560def.inc"

.def temp=r16
.def temp1=r18
.def temp2=r19

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



.dseg
    .org 0x200 ; set origin of data segment to 0x200
    current_station: .byte 1

.cseg
.org 0x0000
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


loop_main:

    ; initialize current station
    ldi xl, low(current_station) ; Let x pointer point tp current_station
    ldi xh, high(current_station)
    ldi temp, 3
    st x,temp
    call print_next_station
    rjmp loop_main


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