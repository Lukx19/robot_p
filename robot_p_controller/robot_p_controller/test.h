/*
 * test.h
 *
 * Created: 22-May-17 7:02:06 PM
 *  Author: petrg
 */ 
#ifndef test_h_
#define test_h_

#include <util/delay.h>
#include "pwm.h"
#include "usart.h"

void test_pwm(void)
{
	uint16_t i = 0;
	usart_init();
	pwm_16_init();
	pwm_16_start();
	sei();
	
	

	//uint16_t z = TIMER_TOP - 100;//1900;
	//OCR1AH = (uint8_t)(z >>8);
	//OCR1AL = (uint8_t)z;		

	while(1)
	{
		//continue;
		//OCR1AH = (uint8_t)(i >> 8);
		//OCR1AL = (uint8_t)i;
		//
		//uint16_t j = 500 - i;
		//OCR1BH = (uint8_t)(j >> 8);
		//OCR1BL = (uint8_t)j;

	
		pwm_set_left_dutycycle(i);

		_delay_ms(500);
		
		i++;
		i %= 100;
	}	
}

#endif