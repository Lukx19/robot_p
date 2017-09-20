/*
 * robot_p_controller.c
 *
 * full data sheet -> http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf
 * summary -> http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Summary.pdf
 * arduino nano pinout -> http://www.pighixxx.com/test/pinouts/boards/nano.pdf
 * flashing .hex file -> https://forum.arduino.cc/index.php?topic=410618.0
 * NOTE: base unit for atmel328P is 1 byte thus processor does not deal atomically for example with int type (there can be race condition between main loop and interrupt)
 * NOTE: by default interrupt can not be interrupted by another one (but some interrupts can be lost when there is raised to much interrupts)
 * Created: 18-May-17 4:16:35 PM
 * Author : Petr Geiger 
 */ 

 //TESTS
//#define PID_TEST
//#define TURN_TEST

#define F_CPU 16000000LU
#define USART_BAUDRATE 57600

#include <util/delay.h>
#include "pwm.h"
#include "usart.h"
#include "pid.h"
#include "test.h"
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h> 

volatile pid_t left_pid, right_pid;
volatile uint8_t started = 0;

//these fileds are required for skipping fake instant ALM signals (instatnt overvoltage etc.)
//if the value is -1 alm is not up thus do not measure time
volatile int16_t almL_time = -1;
volatile int16_t almR_time = -1;

volatile uint8_t control = 0;

#ifdef TURN_TEST
volatile int16_t xxx = 0;
#endif

#ifdef PID_TEST
volatile uint16_t pid_test_t = 0;
#endif



//initializes pins function 
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

void set_left_break(uint8_t active)
{
	if(active)
		PORTB &= ~(1 << PORTB0);
	else
		PORTB |= (1 << PORTB0);
}

void set_right_break(uint8_t active)
{
	if(active)
		PORTC &= ~(1 << PORTC6);
	else
		PORTC |= (1 << PORTC6);
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
	*((speed_t*)&buffer[1]) = -right_pid.current_steps;
	*((speed_t*)&buffer[3]) = left_pid.current_steps;
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

//sets default pin values, starts pwm generation and starts loop for handling commands
void start(void)
{
	if(started)
		return;

	almL_time = -1;
	almR_time = -1;
		
	//set ENBL to LOW -> enabled
	PORTD &= ~((1 << PORTD4) | (1 << PORTD5));

	pwm_16_start();
	loop_timer_start();
	started = 1;	

	left_pid.current_steps = 0;
	right_pid.current_steps = 0;
	set_left_break(1);
	set_right_break(1);
}


//stops pwm generation and stops loop for handling commands
void stop(void)
{
	if(!started)
		return;
	
	pwm_16_stop();
	loop_timer_stop();	

	//set ENBL to HIGH-> disabled 
	PORTD |= (1 << PORTD4) | (1 << PORTD5);
	started = 0;

	almL_time = -1;
	almR_time = -1;

	set_left_break(0);
	set_right_break(0);
}

//recieves and executes commands from usart interface
void usart_recv_message(void)
{
	volatile static uint8_t buffer[6];
	volatile static uint8_t i = 0;
	
	buffer[i++] = usart_receive_byte();
	
	if(i == 6)//!!!!!!!!!!!
	{		
		i = 0;
	
		//TODO calculate CRC
	
		switch(buffer[0])
		{
			case 'P':
				pid_p = *((speed_t*)&buffer[1]);
				break;
			case 'I':
				pid_i = *((speed_t*)&buffer[1]);
				break;
			case 'D':
				pid_d = *((speed_t*)&buffer[1]);
				break;
			case 'V':
				right_pid.desired_speed = *((speed_t*)&buffer[1]);
				left_pid.desired_speed = -*((speed_t*)&buffer[3]);
				break;
			case 'S':
				start();
				break;
			case 'E':
				stop();
				break;
		}
	}
}

//check wheather alm signal is generated by driver
void check_alm(int16_t* alm_time, char* alm_message)
{
	if(*alm_time >= 0)//alm timer is up
	{
		(*alm_time)++;

		if(*alm_time >= 12) // alm is up roughly half a 100 milisecond
		{
			uint8_t s = 6;
			uint8_t text[s];
			usart_init_buffer(text, s);
			stop();
			strcpy((char*)text, alm_message);					
			usart_send_buffer(text, s);		
		}
	}
}

//updates pwm powers and using PID controller to match desired speed
void control_update(void)
{
#ifndef PID_TEST
#ifndef TURN_TEST
		send_steps();
#endif
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

		if(left_power == 0)
			set_left_break(1);
		else
			set_left_break(0);

		if(right_power == 0)
			set_right_break(1);
		else
			set_right_break(0);
		
		//if power is zero let direction as it is, because ticks will probably probably in the same direction as it was
		if(left_power != 0)
			set_left_direction(left_power < 0 ? DIRECTION_BACKWARD : DIRECTION_FORWARD);

		if(right_power != 0)
			set_right_direction(right_power < 0 ? DIRECTION_BACKWARD : DIRECTION_FORWARD);
		
		pwm_set_left_dutycycle(abs(left_power));
		pwm_set_right_dutycycle(abs(right_power));

#ifdef PID_TEST
		usart_send_num_str((int32_t)left_power);
		usart_send_byte(' ');
		usart_send_num_str((int32_t)right_power);
		usart_write_line();
#endif
		
}

ISR(PCINT0_vect)
{
	if(!started)
		return;

	//start measure alm up time - if it is too long stop is called in timer interrupt
	//if alm is changed to down -> stop timer by setting it to -1
	//NOTE: "false" alm is not up even 1/125 of second => still alm up time out is set to 100 ms
	if((PINB & (1 << PINB3)) == 0)
	{
		//usart_send_byte('L');
		almL_time = 0;
	}
	else 
	{
		//usart_send_byte('X');
		almL_time = -1;
	}

	if((PINB & (1 << PINB4)) == 0)
	{
		//usart_send_byte('R');
		almR_time = 0;

	}
	else 
	{
		//usart_send_byte('Y');
		almR_time = -1;
	}
}

ISR(INT0_vect)
{
	pid_add_speed_tick(&left_pid);

#ifdef TURN_TEST

	xxx++;
	if(xxx > 257)
		left_pid.desired_speed = 0;

#endif
}

ISR(INT1_vect)
{
	pid_add_speed_tick(&right_pid);
}

ISR(USART_RX_vect)
{	
	usart_recv_message();
}

ISR(TIMER0_COMPA_vect)
{
	check_alm(&almL_time, "ALM_L");
	check_alm(&almR_time, "ALM_R");

#ifdef PID_TEST
	pid_test_t++;
	if(pid_test_t >= 125*3) //3 s
	{
		if(left_pid.desired_speed != -30)
			left_pid.desired_speed = -30;
		else
			left_pid.desired_speed = -5;

		if(right_pid.desired_speed != 30)
			right_pid.desired_speed = 30;
		else
			right_pid.desired_speed = 5;

		pid_test_t = 0;
	}
#endif

	static uint8_t ticks = 0;
	ticks++;

	//effective frequency 125/5 = 25Hz because timer runs at 125Hz
	if(ticks >= 5)
	{
		control = 1;
		ticks = 0;
	}
}

void main_release(void)
{
	usart_init();
	pwm_16_init();
	pins_init();
	loop_timer_init();	
	int16_t t = 0;

#ifdef TURN_TEST
	t = 7;
#endif

	pid_init(&left_pid, DIRECTION_BACKWARD, -100, 100, -t);
	pid_init(&right_pid, DIRECTION_FORWARD, -100, 100, t);
	
	sei();	
	//start();
	while(1)
	{
		if(control == 1)
		{
			control = 0;
			control_update();
		}

	}
}

int main (void)
{
	main_release();
	//test_pwm();
}
