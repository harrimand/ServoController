;
; ServoController.asm
;
; Created: 4/10/2018 5:43:31 PM
; Author : Darrell
;

/*
Dual Servo Motor Control

Updates to Interrupt handling:
During testing in class we determined that the update rate on the servo position
was much too frequent which would limit controllability of the servo motor.  
The pulse to the servo motor happens every 20 mS.  There is no advantage to 
updating the servo motor position more frequent than the motor is pulsed.  It
makes sense to synchronize the updates with the pulse every 20 mS.  We are using
the Non PWM CTC (Clear Timer on Compare Match) mode (Mode 12) on Timer 1
See [Pg. 106 WGM13:0] [Pg. 109 ICIE1] [Pg. 110 ICF1] in the DataSheet.
The Input Capture Interrupt is triggered when Timer 1 matches the Input Capture
Register.  We can test the pushbuttons and update the servo positions on each 
Input Capture Interrupt to get a 20 mS update period.  

Calculating the step size for each position update.
Choose a desired time to travel from full left to full right.
Divide TravelTime by update rate (.020 Seconds) to get number of updates in 
chosen travel time.  
Example:  
4 second travel time 
Update Period = 20 mS (.020 Sec)
Max Pulse Width = 2000 uS. (.002 Sec)
Min Pulse Width = 1000 uS  (.001 Sec)
Updates for full Travel = 4 Sec / Update Period = number of updates.
4 Sec / .020 Sec = 200 Updates

If we want the pulse to change from 1 mS (Full Left) to 2 mS (Full Right) 
in 200 steps calculate (2 mS - 1 mS) / 200 steps.  (.002 - .001) / 200 = .000005
Increasing the position value by 5 microseconds every 20 milliseconds will 
cause the motor to change from 1000 microSeconds to 2000 microSeconds in 4 Sec.

Adjust the Travel Time, Max Pulse Width and Min Pulse Width to suit your needs.
PINB 2 22
*/

.nolist
.include "tn2313def.inc"
.list


.equ	MinPulse = 450
.equ	MaxPulse = 2350
.equ	MidPulse = (MinPulse + MaxPulse) / 2
.equ	PulsePeriod = 20000
.equ	StepSize = $000A    ;**

.def	TEMP = R16
.def	M1_H = R21
.def 	M1_L = R20
.def	M2_H = R19
.def 	M2_L = R18
.def	StepH = R15
.def	StepL = R14
.def	MaxH = R13
.def	MaxL = R12
.def	MinH = R11
.def	MinL = R10


.ORG	$0000
		rjmp	RESET
.ORG	INT0addr
		rjmp	TBD1
.ORG	INT1addr
		rjmp	TBD2
.ORG    ICP1addr                ;**
        rjmp    SERVOcontrol    ;**
.ORG	OC1Aaddr
		reti
.ORG	PCIaddr
        reti                       ;**
;		rjmp	SERVOcontrol        ;**
.ORG	OC1Baddr
		reti

.ORG	INT_VECTORS_SIZE
RESET:

		ldi 	TEMP, low(RAMEND)
		out 	SPL, TEMP

		ldi 	TEMP, high(StepSize)
		mov 	StepH, TEMP
		ldi 	TEMP, low(StepSize)
		mov 	StepL, TEMP

		ldi 	TEMP, high(MaxPulse)
		mov 	MaxH, TEMP
		ldi 	TEMP, low(MaxPulse)
		mov 	MaxL, TEMP
		
		ldi 	TEMP, high(MinPulse)
		mov 	MinH, TEMP
		ldi 	TEMP, low(MinPulse)
		mov 	MinL, TEMP

		ldi 	TEMP, (1<<PB4)|(1<<PB3)
		out 	DDRB, TEMP

		ldi 	TEMP, (1<<PB5)|(1<<PB2)|(1<<PB1)|(1<<PB0)
		out 	PORTB, TEMP

		cbi 	DDRD, PD3
		sbi 	PORTD, PD3

		cbi 	DDRD, PD2
		sbi 	PORTD, PD2

;		ldi 	TEMP, (1<<PCINT5)|(1<<PCINT2)|(1<<PCINT1)|(1<<PCINT0)
;		out 	PCMSK, TEMP

		ldi 	TEMP, (1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00)
		out 	MCUCR, TEMP

		ldi 	TEMP, (1<<INT1)|(1<<INT0)|(0<<PCIE) ;**
		out 	GIMSK, TEMP
		
		ldi 	TEMP, (1<<OCIE1A)|(1<<OCIE1B)|(1<<ICIE1)    ;**
		out 	TIMSK, TEMP

		ldi 	TEMP, high(PulsePeriod)
		out 	ICR1H, TEMP
		ldi 	TEMP, low(PulsePeriod)
		out 	ICR1L, TEMP

		ldi 	M1_H, high(MidPulse)
		out 	OCR1AH, M1_H

		ldi 	M1_L, low(MidPulse)
		out 	OCR1AL, M1_L

		ldi 	M2_H, high(MidPulse)
		out 	OCR1BH, M2_H

		ldi 	M2_L, low(MidPulse)
		out 	OCR1BL, M2_L

		ldi 	TEMP, (1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM10)
		out 	TCCR1A, TEMP

		ldi 	TEMP, (1<<WGM13)|(1<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10)
		out 	TCCR1B, TEMP

		sei

MAIN:
		nop
		nop
		nop
		nop
		rjmp	MAIN

SERVOcontrol:
		sbis	PINB, 0
		rcall	M1_LEFT
		sbis	PINB, 1
		rcall	M1_RIGHT
		sbis	PINB, 2
		rcall	M2_LEFT
		sbis	PINB, 5
		rcall	M2_RIGHT
		reti

M1_RIGHT:
		sub 	M1_L, StepL
		sbc 	M1_H, StepH
		cp  	M1_L, MinL
		cpc 	M1_H, MinH
		brpl	M1notMin
		mov 	M1_H, MinH
		mov 	M1_L, MinL
M1notMin:                       ;**
		out 	OCR1AH, M1_H
		out 	OCR1AL, M1_L
		ret

M1_LEFT:
		add 	M1_L, StepL
		adc 	M1_H, StepH
		cp  	M1_L, MaxL
		cpc 	M1_H, MaxH
		brmi	M1notMax
		mov 	M1_H, MaxH
		mov 	M1_L, MaxL
M1notMax:                       ;**
		out 	OCR1AH, M1_H
		out 	OCR1AL, M1_L
		ret

M2_RIGHT:
		sub 	M2_L, StepL
		sbc 	M2_H, StepH
		cp  	M2_L, MinL
		cpc 	M2_H, MinH
		brpl	M2notMin
		mov 	M2_H, MinH
		mov 	M2_L, MinL
M2notMin:                       ;**
		out 	OCR1BH, M2_H
		out 	OCR1BL, M2_L
		ret

M2_LEFT:
		add 	M2_L, StepL
		adc 	M2_H, StepH
		cp  	M2_L, MaxL
		cpc 	M2_H, MaxH
		brmi	M2notMax
		mov 	M2_H, MaxH
		mov 	M2_L, MaxL
M2notMax:                       ;**
		out 	OCR1BH, M2_H
		out 	OCR1BL, M2_L
		ret

TBD1:
		nop
		reti
		
TBD2:
		nop
		reti


