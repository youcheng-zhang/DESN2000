;
; Project.asm
;
; Created: 2020/7/26 12:26:18
; Author : youcheng
;

; A function which will print the name of next station

.include "m2560def.inc"

.def temp=r16
.def temp1=r17
.def temp2=r18
.def  leds = r19   ; r17 stores a LED pattern
.def row =r20
.def col =r21
.def mask =r22
.equ PATTERN = 0b11  ; define a pattern for 8 LEDs
.equ PORTLDIR = 0xF0
.equ INITCOLMASK = 0xEF
.equ INITROWMASK = 0x01
.equ ROWMASK = 0x0F

.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

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
.macro do_lcd_data1
	ldi r17, @0
	mov r16, r17
	rcall lcd_data
	rcall lcd_wait
.endmacro

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
    SpeedCounter: .byte 2     ; two-byte counter for counting seconds.
    interrupt_mode: .byte 1
    LastKeyPressed: .byte 1		; stores the last key pressed
    LastKeyPressedAscii: .byte 1 ; stores the last key pressed ascii value
    NumberStations: .byte 1 	; stores the number of stations
    TravelTime:		.byte 1 	; stores the travel time 
    StopTime:		.byte 1 	; stores the stop time

.cseg

	.org 0x0000
jmp RESET

	.org INT0addr ; INT0addr is the address of EXT_INT0 (External Interrupt 0)
		jmp EXT_INT0 ; interrupt vector for External Interrupt 0
	.org INT1addr ; INT1addr is the address of EXT_INT1 (External Interrupt 1)
		jmp EXT_INT1 ; interrupt vector for External Interrupt 1
    .org INT2addr    ; INT2addr is the address of EXT_INT2 (External Interrupt 0)
        jmp EXT_INT2 
	.org OVF0addr      ; OVF0addr is the address of Timer0 Overflow Interrupt Vector
		jmp Timer0OVF   ; jump to the interrupt handler for Timer0 overflow.
		jmp DEFAULT     ; default service for all other interrupts.
	DEFAULT: reti     ; no interrupt handling 
    ; every station take 20bytes, end with "!"
    stations: .db "Daring Harbour!     City Marine!        White Bay!          Simmons Point!      SYD Observatory!    SYD Aquarium! "
    emergency_message: .db "EMERGENCY!"
    INVALID_INPUT: .db "INVALID INPUT!"

RESET:

    ; initialize the LCD
    ldi r16, low(RAMEND)
	out SPL, r16
	ldi r16, high(RAMEND)
	out SPH, r16

    ldi temp, PORTLDIR ; columns are outputs, rows are inputs
    STS DDRL, temp     ; cannot use out
    ser temp
    out DDRC, temp ; Make PORTC all outputs
    out PORTC, temp ; Turn on all the LEDs

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
    rjmp setup
	
; transition to the setup phase
setup:
    do_lcd_data1 'N'
    do_lcd_data1 'U'
    do_lcd_data1 'M'
    do_lcd_data1 ' '
    do_lcd_data1 'S'
    do_lcd_data1 'T'
    do_lcd_data1 'A'
    do_lcd_data1 'T'
    do_lcd_data1 'I'
    do_lcd_data1 'O'
    do_lcd_data1 'N'
    do_lcd_data1 'S'
    do_lcd_data1 ':'
    do_lcd_data1 ' '
    call keypad			; calls keypad, after this code a valid input is stored in LastKeyPressed 
                        ; and LastKeyPressedAscii contains the ascii version

    ; check if key press is valid, LastKeyPressedAscii >= '1'
    lds temp, LastKeyPressedAscii
    ldi temp2, '1'
    cp temp, temp2
    brlo incorrect_output1
    ; check if key press is valid, LastKeyPressedAscii <= '9'
    lds temp, LastKeyPressedAscii
    ldi temp2, '9'
    cp temp2, temp
    brlo incorrect_output1

    ; save num stations, then move onto next stage
    lds temp, LastKeyPressed
    sts NumberStations, temp
    jmp setup_time

; handle incorrect output
incorrect_output1:
    call print_invalid
    call lcd_delay
    do_lcd_command 0b00000001 ; clear display
    jmp setup

print_invalid:
    do_lcd_command 0b00000001 ; clear display
    ldi zl, low(INVALID_INPUT<<1) ; Let z point to start
    ldi zh, high(INVALID_INPUT<<1) ; to the start

print_invalid_message:
    ldi temp,33
    lpm temp1, z+
    cp temp, temp1					; if temp1 == "!"
    breq invalid_end
    do_lcd_data temp1
    rjmp print_invalid_message

invalid_end:
    ret

setup_time:
    do_lcd_command 0b00000001 ; clear display
    do_lcd_data1 'T'
    do_lcd_data1 'R'
    do_lcd_data1 'A'
    do_lcd_data1 'V'
    do_lcd_data1 'E'
    do_lcd_data1 'L'
    do_lcd_data1 ' '
    do_lcd_data1 'T'
    do_lcd_data1 'I'
    do_lcd_data1 'M'
    do_lcd_data1 'E'
    do_lcd_data1 ':'
    do_lcd_data1 ' '
    call keypad			; calls keypad, after this code a valid input is stored in LastKeyPressed 
                        ; and LastKeyPressedAscii contains the ascii version

    ; check if key press is valid, LastKeyPressedAscii >= '1'
    lds temp, LastKeyPressedAscii
    ldi temp2, '1'
    cp temp, temp2
    brlo incorrect_output2
    ; check if key press is valid, LastKeyPressedAscii <= '9'
    lds temp, LastKeyPressedAscii
    ldi temp2, '9'
    cp temp2, temp
    brlo incorrect_output2

    ; save travel time, then move onto next stage
    lds temp, LastKeyPressed
    sts TravelTime, temp
    jmp setup_stop_time

incorrect_output2:
    call print_invalid
    call lcd_delay
    do_lcd_command 0b00000001 ; clear display
    jmp setup_time

setup_stop_time:
    do_lcd_command 0b00000001 ; clear display
    do_lcd_data1 'S'
    do_lcd_data1 'T'
    do_lcd_data1 'O'
    do_lcd_data1 'P'
    do_lcd_data1 ' '
    do_lcd_data1 'T'
    do_lcd_data1 'I'
    do_lcd_data1 'M'
    do_lcd_data1 'E'
    do_lcd_data1 ':'
    do_lcd_data1 ' '
    call keypad			; calls keypad, after this code a valid input is stored in LastKeyPressed 
                        ; and LastKeyPressedAscii contains the ascii version

    ; check if key press is valid, LastKeyPressedAscii >= '1'
    lds temp, LastKeyPressedAscii
    ldi temp2, '1'
    cp temp, temp2
    brlo incorrect_output3
    ; check if key press is valid, LastKeyPressedAscii <= '9'
    lds temp, LastKeyPressedAscii
    ldi temp2, '9'
    cp temp2, temp
    brlo incorrect_output3

    ; save stop time, then move to main
    lds temp, LastKeyPressed
    sts StopTime, temp
    do_lcd_command 0b00000001 ; final clear display ends setup
    jmp main

incorrect_output3:
    call print_invalid
    call lcd_delay
    do_lcd_command 0b00000001 ; clear display
    jmp setup_stop_time


; keypad keeps scanning the keypad to find which key is pressed.
keypad:
    ldi mask, INITCOLMASK ; initial column mask
    clr col ; initial column
colloop:
    STS PORTL, mask ; set column to mask value
    ; (sets column 0 off)
    ldi temp, 0xFF ; implement a delay so the
    ; hardware can stabilize
delay_simple:
    dec temp
    brne delay_simple
    LDS temp, PINL ; read PORTL. Cannot use in 
    andi temp, ROWMASK ; read only the row bits
    cpi temp, 0xF ; check if any rows are grounded
    breq nextcol ; if not go to the next column
    ldi mask, INITROWMASK ; initialise row check
    clr row ; initial row
rowloop:      
    mov temp2, temp
    and temp2, mask ; check masked bit
    brne skipconv ; if the result is non-zero,
    ; we need to look again


    rcall convert ; if bit is clear, convert the bitcode
    call keypad_delay
    lds temp, LastKeyPressedAscii
    do_lcd_data temp			; print out the pressed key
    lds temp, LastKeyPressed
    out PORTC, temp
    ret

    jmp keypad ; and start again
skipconv:
    inc row ; else move to the next row
    lsl mask ; shift the mask to the next bit
    jmp rowloop          
nextcol:     
    cpi col, 3 ; check if we^�re on the last column
    breq keypad ; if so, no buttons were pushed,
    ; so start again.

    sec ; else shift the column mask:
    ; We must set the carry bit
    rol mask ; and then rotate left by a bit,
    ; shifting the carry into
    ; bit zero. We need this to make
    ; sure all the rows have
    ; pull-up resistors
    inc col ; increment column value
    jmp colloop ; and check the next column
    ; convert function converts the row and column given to a
    ; binary number and also outputs the value to PORTC.
    ; Inputs come from registers row and col and output is in
    ; temp.


convert:   ; converts a VALID number into the right hex
    cpi col, 3 ; we return
    breq convert_end
    cpi row, 3 ; we return
    breq convert_end
    mov temp, row ; otherwise we have a number (1-9)
    lsl temp ; temp = row * 2
    add temp, row ; temp = row * 3
    add temp, col ; add the column address
    ; to get the offset from 1
    ldi temp2, '1'
    add temp, temp2  ; row*3 + col + '1'

convert_end:
    sts LastKeyPressedAscii, temp		; store the ascii value of the last key pressed
    ldi temp2, '1'
    sub temp, temp2
    inc temp							; stores the bit vlaue of the last key pressed
    sts LastKeyPressed, temp
    ret ; return to caller


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



EXT_INT2:
    push temp
    push r25
    push r24

    lds r24, SpeedCounter
    lds r25, SpeedCounter+1
    adiw r25:r24, 1            ; increase the second counter by one
    sts SpeedCounter, r24
    sts SpeedCounter+1, r25

    pop r24        ; epilogue starts
    pop r25        ; restore all conflicting registers from the stack
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
        ldi xl, low(interrupt_mode) ; Let x pointer point tp current_station
        ldi xh, high(interrupt_mode)
        ld temp, x
        cpi temp,0
        breq LED_mode
        cpi temp,1
        breq Speed_monitor_mode

LED_mode:
       cpi leds,pattern
       breq intr_led_off
       ldi leds,PATTERN
       jmp end_intr_led_off
intr_led_off:
       ldi leds,0
end_intr_led_off:
       out PORTC, leds
       rjmp end_mode

Speed_monitor_mode:
        ldi xl, low(SpeedCounter) ; Let x pointer point tp SpeedCounter
        ldi xh, high(SpeedCounter)
        ld temp, x
        cpi temp,10
        brge emergency
        rjmp back_to_normal
emergency:
print_emergency:
	    do_lcd_command 0b00000001 ; clear display
    ldi zl, low(emergency_message<<1) ; Let x pointer point
    ldi zh, high(emergency_message<<1) ; to the start
print_emergency_message:
    ldi temp,33
    lpm temp1, z+
    cp temp, temp1 ; if temp1 == "!"
    breq end_emergency
    do_lcd_data temp1
    rjmp print_emergency_message
back_to_normal:
    call print_curr_station
end_emergency:
    clear SpeedCounter
    rjmp end_mode


end_mode:
       clear TempCounter      ; reset the temporary counter
       ; Load the value of the second counter
       lds r24, SecondCounter
       lds r25, SecondCounter+1
       adiw r25:r24, 1            ; increase the second counter by one
       sts SecondCounter, r24
       sts SecondCounter+1, r25
       jmp EndIF
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
    call increase_speed
    call Speed_detect_on
leave_station:
	call print_next_station
    call sleep_5s
    
    ldi xl, low(get_on) ; Let x pointer point tp get_on
    ldi xh, high(get_on)
    ld temp,x
    cpi temp,1
    breq stop_at_next_station

    ldi xl, low(get_off) ; Let x pointer point tp get_off
    ldi xh, high(get_off)
    ld temp,x
    cpi temp,1
    breq stop_at_next_station

    rjmp leave_station

stop_at_next_station:
    call Speed_detect_off
    ldi xl, low(get_on) ; Let x pointer point tp get_on
    ldi xh, high(get_on)
    ldi temp,0
    st x, temp
    ldi xl, low(get_off) ; Let x pointer point tp get_off
    ldi xh, high(get_off)
    ldi temp,0
    st x, temp
    call decrease_speed
    call LED_on
    call sleep_5s
    call LED_off
    call increase_speed
    call Speed_detect_on

    rjmp leave_station






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
    jmp end_reset_current_station
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
    jmp print_loop
end_print:
    pop temp1
    pop temp
    out SREG, temp
    pop temp
    ret




print_curr_station:  
    push temp
    in temp, SREG
    push temp
    push temp1
    ;current station
    ldi xl, low(current_station) ; Let x pointer point tp current_station
    ldi xh, high(current_station)
    ld temp, x
    ; temp now contains the index of the next station
	do_lcd_command 0b00000001 ; clear display
    ldi zl, low(stations<<1) ; Let z pointer point
    ldi zh, high(stations<<1) ; to the start of stations
    ldi temp1,20
    ld temp, x
    mul temp,temp1
    add zl,r0
    adc zh,r1
print_curr_loop:
    lpm temp1, z+
    cpi temp1, 33 ; if temp1 == "!"
    breq end_curr_print
    do_lcd_data temp1
    jmp print_curr_loop
end_curr_print:
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
    call sleep_500ms
    add temp1,temp2
    jmp increase_speed_loop
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

    ldi temp1,150
    ldi temp,0
    ldi temp2,10
decrease_speed_loop:
    sts OCR3BL, temp1
    sts OCR3BH, temp
    cpi temp1,0
    brge end_decrease_speed_loop
    call sleep_500ms
    call sleep_500ms
    call sleep_500ms
    sub temp1,temp2
    jmp decrease_speed_loop
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
	



LED_on:
    push temp
    in temp, SREG
    push temp
    push leds

    ;mode:0 LED
    ldi xl, low(interrupt_mode) ; Let x pointer point tp interrupt_mode
    ldi xh, high(interrupt_mode)
    ldi temp,0
    st x,temp

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

	ldi leds, 0
	out PORTC, leds
    ldi temp, 0
    sts TIMSK0, temp           ; disable Timer0 Overflow Interrupt
    sei

	pop temp
    out SREG, temp
    pop temp
    ret






Speed_detect_on:
    push temp
    in temp, SREG
    push temp

    ldi temp, (2 << ISC20)
    sts EICRA, temp   ; temp=0b00001010, so both interrupts are configured as falling edge triggered interrupts     
    in temp, EIMSK
    ori temp, (1<<INT2)  ; INT2=0
    out EIMSK, temp  ; Enable External Interrupts 0 and 1

    ;mode:1 Speed Detect
    ldi xl, low(interrupt_mode) ; Let x pointer point tp interrupt_mode
    ldi xh, high(interrupt_mode)
    ldi temp,1
    st x,temp

	clear TempCounter         ; initialize the temporary counter to 0
	clear SecondCounter      ; initialize the second counter to 0
	ldi temp, 0b00000000
	out TCCR0A, temp
	ldi temp, 0b00000010
	out TCCR0B, temp          ; set prescalar value to 8
	ldi temp, 1<<TOIE0        ; TOIE0 is the bit number of TOIE0 which is 0   
	sts TIMSK0, temp           ; enable Timer0 Overflow Interrupt
	sei                                    ; enable global interrupt

    pop temp
    out SREG, temp
    pop temp
    ret




Speed_detect_off:
    push temp
    in temp, SREG
    push temp

    ldi temp, 0
    sts TIMSK0, temp           ; disable Timer1 Overflow Interrupt
    sei

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
    jmp delay3
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
	jmp lcd_wait_loop
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

sleep_50ms:
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	ret

sleep_500ms:
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_50ms
	rcall sleep_50ms
	ret

sleep_5s:
	rcall sleep_500ms
	rcall sleep_500ms
	rcall sleep_500ms
	rcall sleep_500ms
	rcall sleep_500ms
	rcall sleep_500ms
	rcall sleep_500ms
	rcall sleep_500ms
	rcall sleep_500ms
	rcall sleep_500ms
    ret

