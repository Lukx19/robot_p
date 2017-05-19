/*
 * pwm.h
 *
 * Created: 19-May-17 10:23:15 AM
 *  Author: petrg
 */ 
#ifndef pwm_h_
#define pwm_h_

#ifndef F_CPU
# warning "F_CPU macro is not defined, setting up default 1MHz"
#define F_CPU 1000000LU
#endif

#include <avr/io.h>

void pwm_16_init()
{
	//using 16 bit timer 1 p. 170
	//1A left wheel
	//1B right wheel

	//f_pwm = f_clk/(N*(1+TOP))    ....for N=8 prescaler, f_clk = 16MHz
	//TOP = f_clk/(N * f_pwm) - 1  ....our desired f_pwm = 1000Hz
	const uint16_t timer_top = F_CPU/(8*1000) - 1;
	
	//setting up: Clear OCnA/OCnB/OCnC on compare match,
	// set OCnA/OCnB/OCnC at BOTTOM,(non-inverting mode)
	TCCR1A |= (1 << COM1A1) | (0 << COM1A0);
	TCCR1B |= (1 << COM1B1) | (0 << COM1B0);
	
	//Fast PWM - with TOP = ICRn
	TCCR1A |= (1 << WGM11) | (0 << WGM10);
	TCCR1B |= (1 << WGM13) | (1 << WGM12);
	
	
	//setup I/O clock with no prescaler N=8
	TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
	
	//setup timer TOP value
	ICR1H = (uint8_t)(timer_top >> 8);
	ICR1L = (uint8_t)timer_top;
	
	//setting up initial duty cycle for left wheel
	OCR1AH = 0;
	OCR1AL = 0;
	
	//setting up initial duty cycle for right wheel
	OCR1BH = 0;
	OCR1BL = 0;
	
	//setting up (OC1A)PB1 - left wheel and (OC1B)PB2 - right wheel
	DDRB |= (1 << PB1) | (1 << PB2); //pin 11 resp. 12
}
#endif
