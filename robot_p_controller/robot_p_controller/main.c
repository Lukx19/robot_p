/*
 * robot_p_controller.c
 *
 * Created: 18-May-17 4:16:35 PM
 * Author : petrg
 */ 

#define F_CPU 16000000LU
#define USART_BAUDRATE 57600
#include <util/delay.h>
#include "pwm.h"
#include "usart.h"
#include "pid.h"
#include "test.h"
#include <string.h>
#include <stdlib.h>

//#define TICK_TEST
#define PID_TEST

#include <avr/interrupt.h> 

//full data sheet -> http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf
//summary -> http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Summary.pdf
//arduino nano pinout -> http://www.pighixxx.com/test/pinouts/boards/nano.pdf
//flashing .hex file -> https://forum.arduino.cc/index.php?topic=410618.0
//NOTE: base unit for atmel328P is 1 byte thus processor does not deal atomically for example with int type (there can be race condition between main loop and interrupt)
//NOTE: by default interrupt can not be interrupted by another one

pid_t left_pid, right_pid;

void pins_init(void)
{
	//input:
	//SPEED P2(INT0)(D2)
	//SPEED P3(INT1)(D3)
	//setup pull-ups
	PORTD |= (1 << PORTD2) | (1<<PORTD3);
	MCUCR &= ~(1 << PUD);
	
	//interrupts on rising edge
	EICRA |= (1 << ISC11) | (1 << ISC01); 
	EIMSK |= (1 << INT1) | (1 << INT0);
	
	//output ENBL L/R:	
	DDRD |= (1 << DDD4) | (1 << DDD5);
	//output F/R L/R:
	DDRD |= (1 << DDD6) | (1 << DDD7);
	PORTD |= (1 << PORTD4) | (1 << PORTD5);//init to stop

	//output BRK L
	DDRB |= (1 << DDB0);
	//output BRK R
	DDRC |= (1 << DDC6);
	//output B1/2 pwm

	PORTB |= (1 << PORTB0);//init to not break
	PORTC |= (1 << PORTC6);//init to not break
	
	//input ALM L/R
	PORTB |= (1 << PORTB3) | (1 << PORTB4);
	PCICR |= (1 << PCIE0);
	PCMSK0 |= (1 << PCINT4) | (1 << PCINT3);
}

void set_left_direction(uint8_t direction)
{
	left_pid.direction = direction;
	switch(direction)
	{
		case DIRECTION_FORWARD:
			PORTD &= ~(1 << PORTD6);
			break;
		case DIRECTION_BACKWARD:
			PORTD |= (1 << PORTD6);
			break;		
	}
}

void set_right_direction(uint8_t direction)
{
	right_pid.direction = direction;
	switch(direction)
	{
		case DIRECTION_FORWARD:
			PORTD &= ~(1 << PORTD7);
			break;
		case DIRECTION_BACKWARD:
			PORTD |= (1 << PORTD7);
			break;
	}
}

void loop_timer_init(void)
{
	//set top OCR0A register
	TCCR0A |= (1 << WGM01);	
	
	//enable interrupt
	TIMSK0 |= (1 << OCIE0A);
	//setup interrupts to 125Hz
	//16Mhz / 1024 prescaler / 125 = 125 MHz
	OCR0A = (uint8_t)(F_CPU / 1024LU / 125LU);
}

void send_steps(void)
{
	uint8_t s = 6;
	uint8_t buffer[s];
	usart_init_buffer(buffer, s);
	buffer[0] = 'T';
	*((speed_t*)&buffer[1]) = left_pid.current_steps;
	*((speed_t*)&buffer[3]) = right_pid.current_steps;
	//TODO calculate CRC
	
	usart_send_buffer(buffer, s);
}

void loop_timer_start(void)
{
	//1024 prescaler
	TCCR0B |= (1 << CS02)|(0 << CS01)|(1 << CS00);
	//TODO reset timer
}

void loop_timer_stop(void)
{
	//1024 prescaler
	TCCR0B &= ~((1 << CS02)|(1 << CS01)|(1 << CS00));	
}

uint8_t started = 0;

void start(void)
{
	if(started)
		return;
		
	//set ENBL to LOW -> enabled
	PORTD &= ~((1 << PORTD4) | (1 << PORTD5));

	pwm_16_start();
	loop_timer_start();
	started = 1;	

	left_pid.current_steps = 0;
	right_pid.current_steps = 0;

#ifdef TICK_TEST
	set_left_direction(DIRECTION_BACKWARD);
	set_right_direction(DIRECTION_FORWARD);
	pwm_set_left_dutycycle(50);
	pwm_set_right_dutycycle(50);
#endif
}

void stop(void)
{
	if(!started)
		return;
	
	pwm_16_stop();
	loop_timer_stop();	

	//set ENBL to HIGH-> disabled 
	PORTD |= (1 << PORTD4) | (1 << PORTD5);
	started = 0;

#ifdef TICK_TEST
	 send_steps();
#endif
}

ISR(PCINT0_vect)
{
	uint8_t s = 6;
	uint8_t text[s];
	usart_init_buffer(text, s);

	if((PINB & (1 << PINB3)) == 0)
	{
		stop();
		
		strcpy((char*)text, "ALM_L");					
		usart_send_buffer(text, s);		
	}

	if((PINB & (1 << PINB4)) == 0)
	{
		stop();

		strcpy((char*)text, "ALM_R");					
		usart_send_buffer(text, s);
	}
}

ISR(INT0_vect)
{
	pid_add_speed_tick(&left_pid);
}

ISR(INT1_vect)
{
	pid_add_speed_tick(&right_pid);
}

ISR(USART_RX_vect)
{	
	static uint8_t buffer[6];
	static uint8_t i = 0;
	
	buffer[i++] = usart_receive_byte();
	
	if(i == 6)//!!!!!!!!!!!
	{		
		i = 0;
	
		//TODO calculate CRC
	
		switch(buffer[0])
		{
			case 'P':
				pid_p = *((uint32_t*)&buffer[1]);
				break;
			case 'I':
				pid_i = *((uint32_t*)&buffer[1]);
				break;
			case 'D':
				pid_d = *((uint32_t*)&buffer[1]);
				break;
			case 'V':
				left_pid.desired_speed = *((int16_t*)&buffer[1]);
				right_pid.desired_speed = *((int16_t*)&buffer[3]);
				break;
			case 'S':
				if(strncmp((char*)&buffer,"START", 5) == 0)
				{
					start();
				}
				else if(strncmp((char*)buffer,"STOP", 4) == 0)
				{				
					stop();
				}
				
				break;
		}
	}
}

ISR(TIMER0_COMPA_vect)
{
#ifndef TICK_TEST

	static uint8_t ticks = 0;
	ticks++;

	//effective frequency 125/5 = 25Hz because timer runs at 125Hz
	if(ticks == 5)
	{
#ifndef PID_TEST
		send_steps();
#else

		usart_send_byte('S');
		usart_send_byte('L');
		usart_send_byte(' ');
		usart_send_num_str(left_pid.current_steps);
		usart_write_line();

		usart_send_byte('S');
		usart_send_byte('R');
		usart_send_byte(' ');
		usart_send_num_str(right_pid.current_steps);
		usart_write_line();
#endif

		speed_t left_power = pid_update(&left_pid);
		speed_t right_power = pid_update(&right_pid);
		
		set_left_direction(left_power < 0 ? DIRECTION_BACKWARD : DIRECTION_FORWARD);
		set_right_direction(right_power < 0 ? DIRECTION_BACKWARD : DIRECTION_FORWARD);
		
		pwm_set_left_dutycycle(abs(left_power));
		pwm_set_right_dutycycle(abs(right_power));

#ifdef PID_TEST
		usart_send_num_str((int32_t)left_power);
		usart_send_byte(' ');
		usart_send_num_str((int32_t)right_power);
		usart_write_line();
#endif
		
		ticks = 0;
	}
#endif
}

void main_release(void)
{
	usart_init();
	pwm_16_init();
	pins_init();
	loop_timer_init();	
	pid_init(&left_pid, DIRECTION_BACKWARD, -100, 100, -30);
	pid_init(&right_pid, DIRECTION_FORWARD, -100, 100, 30);
	
	sei();	
	while(1)
	{

	}
}

int main (void)
{
	main_release();
	//test_pwm();
}