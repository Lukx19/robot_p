/*
 * robot_p_controller.c
 *
 * Created: 18-May-17 4:16:35 PM
 * Author : petrg
 */ 

#define F_CPU 16000000LU
#define USART_BAUDRATE 9600
#include <util/delay.h>
#include "pwm.h"
#include "uart.h"

//full datasheet -> http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf
//summary -> http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Summary.pdf
//arduino nano pinout -> http://www.pighixxx.com/test/pinouts/boards/nano.pdf
//flashing .hex file -> https://forum.arduino.cc/index.php?topic=410618.0

int main (void)
{
	uint16_t i = 0;
	USART0Init();
	pwm_16_init();
	//DDRB |= (1 << PB1);
	while(1)
	{
		//PORTB ^=  1 << PORTB1;
		OCR1AH = OCR1BH= (uint8_t)(i >> 8);
		OCR1AL = OCR1BL= (uint8_t)i;
		
		//uint16_t j = 500 - i;
		//OCR1BH = (uint8_t)(j >> 8);
		//OCR1BL = (uint8_t)j;
		
		_delay_ms(50);
		
		i += 5;
		i %= 500;
		
		// Receive data
		//u8TempData = USART0ReceiveByte();
		// Increment received data

		//Send back to terminal
		//USART0SendByte(u8TempData);
	} 
}

