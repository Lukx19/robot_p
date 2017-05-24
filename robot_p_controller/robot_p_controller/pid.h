/*
 * pid.h
 *
 * Created: 22-May-17 4:26:01 PM
 *  Author: petrg
 */ 
#ifndef pid_h_
#define pid_h_

#include <avr/interrupt.h> 
#include <math.h>

typedef int32_t speed_t;

#define DIRECTION_BACKWARD		0
#define DIRECTION_FORWARD		1

speed_t pid_p = 1;
speed_t pid_i = 1;
speed_t pid_d = 1;

typedef struct PID 
{
	uint8_t direction;
	speed_t previous_error;
	speed_t integral;
	speed_t current_steps;
	speed_t desired_speed;
	speed_t min_output;
	speed_t max_output;
} pid_t;

speed_t min(speed_t v1, speed_t v2)
{
	return v1 < v2 ? v1 : v2;
}

speed_t max(speed_t v1, speed_t v2)
{
	return v1 > v2 ? v1 : v2;
}


void pid_init(pid_t* data, uint8_t initial_direction, speed_t min_output, speed_t max_output)
{
	data->direction = initial_direction;
	data->previous_error = 0;
	data->integral = 0;
	data->current_steps = 0;
	data->desired_speed = 0;
	data->min_output = min_output;
	data->max_output = max_output;
}

void pid_add_speed_tick(pid_t* data)
{
	switch(data->direction)
	{
		case DIRECTION_FORWARD:
			data->current_steps++;
			break;
		case DIRECTION_BACKWARD:
			data->current_steps--;
			break;	
	}	
}

speed_t pid_update(pid_t* data, speed_t dt)
{		
	//TODO find out right value for multiplying flowing variables in order to have sufficient precision
	speed_t current_speed = data->current_steps / dt;
	speed_t error = data->desired_speed - current_speed;
	
	data->integral += error * dt;
	
	//Reset Windup - do not add to integral if it is not in pwm bounds
	//http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup/
	data->integral = max(min(data->integral, data->max_output),data->min_output);
	
	
	speed_t derivative = (error - data->previous_error)/dt;
	speed_t output = (pid_p*error + pid_i*data->integral + pid_d*derivative)/100000; //TODO dividing because p,i,d are multiplied by 100000
	
	//Reset Windup for output
	output = max(min(output, data->max_output),data->min_output);
	
	data->previous_error = error;
	data->current_steps = 0;
	
	return output;
}

#endif