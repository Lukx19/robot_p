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
#include <stdlib.h>

//using 16 bit timer 1 p. 170
//1A left wheel
//1B right wheel

//f_pwm = f_clk/(N*(1+TOP))    ....for N=8 prescaler, f_clk = 16MHz
//TOP = f_clk/(N * f_pwm) - 1  ....our desired f_pwm = 1000Hz
#define TIMER_TOP  (F_CPU/(8*1000) - 1)

void pwm_16_init(void)
{
	//setting up (OC1A)PBjdkk1 - left wheel and (OC1B)PB2 - right wheel
	DDRB |= (1 << DDB1) | (1 << DDB2); //pin 11 resp. 12
	
	//setting up: Clear OCnA/OCnB/OCnC on compare match,
	// set OCnA/OCnB/OCnC at BOTTOM,(non-inverting mode)
	TCCR1A |= (1 << COM1A1)|(1 << COM1B1);
	
	//Fast PWM - with TOP = ICRn
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM13) | (1 << WGM12);
	
	//setup timer TOP value
	ICR1H = (uint8_t)(TIMER_TOP >> 8);
	ICR1L = (uint8_t)TIMER_TOP;
	
	//setting up initial duty cycle for left wheel
	OCR1AH = 0;
	OCR1AL = 0;
	
	//setting up initial duty cycle for right wheel
	OCR1BH = 0;
	OCR1BL = 0;	
}

void pwm_16_start(void)
{
	//setup I/O clock with prescaler N=8
	TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
	//TODO reset timer
}

void pwm_16_stop(void)
{
	//No clock source (Timer/Counter stopped).
	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
}

//driver has resolution 1..75 thus 100 steps should be far enough
void pwm_set_left_dutycycle(uint8_t percentage)
{
	if(percentage > 100)
	{
		//TODO mark somehow error
		exit(1);
	}
	
	uint16_t value = (TIMER_TOP * percentage)/100;
	
	OCR1AH = (uint8_t)(value >> 8);
	OCR1AL = (uint8_t)value;	
}

//driver has resolution 1..75 thus 100 steps should be far enough
void pwm_set_right_dutycycle(uint8_t percentage)
{
	if(percentage > 100)
	{
		//TODO mark somehow error
		exit(1);
	}
		
	uint16_t value = (TIMER_TOP * percentage)/100;
	
	OCR1BH = (uint8_t)(value >> 8);
	OCR1BL = (uint8_t)value;
}

#endif
