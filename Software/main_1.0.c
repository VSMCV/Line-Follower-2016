/*
 * main.c
 *
 *  Created on: 2016 Jan 14 19:42:41
 *  Author: Vlad
 */

#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

#include<defines_1.0.h>
#include<rc_driver.h>

/**

 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point. It is invoked by the device startup code. It is responsible for
 * invoking the APP initialization dispatcher routine - DAVE_Init() and hosting the place-holder for user application
 * code.
 */

volatile uint32_t color_flag = 0;
uint8_t mag1_state = 0; //state of left magnet sensor
uint8_t mag2_state = 0; //state of right magnet sensor
int16_t speed;
int16_t sensor_counter;
int16_t i;
int16_t d;
int16_t last_p;
uint32_t samples_since_out = 0;

uint32_t initial_delay_timer, initial_delay_timer_status = 1;
uint32_t avoid_wheelie_timer, avoid_wheelie_timer_status = 1;
uint32_t sampling_timer, sampling_timer_status = 1;

int16_t error(void)
{
	uint32_t port0_in = *(volatile uint32_t*)(port0_in_location);
	port0_in = port0_in ^ color_flag;

	sensor_counter = ((port0_in >> p1) & 1) + ((port0_in >> p2) & 1) + ((port0_in >> p3) & 1) + ((port0_in >> p4) & 1) +
					 ((port0_in >> p5) & 1) + ((port0_in >> p6) & 1) + ((port0_in >> p7) & 1) + ((port0_in >> p8) & 1);

	if(sensor_counter != 0)
	{
		samples_since_out = 0;

		int32_t err = ((port0_in >> p1) & 1) * i1 + ((port0_in >> p2) & 1) * i2 + ((port0_in >> p3) & 1) * i3 + ((port0_in >> p4) & 1) * i4 +
					  ((port0_in >> p5) & 1) * i5 + ((port0_in >> p6) & 1) * i6 + ((port0_in >> p7) & 1) * i7 + ((port0_in >> p8) & 1) * i8;

		err = err * 1000 / sensor_counter;

		return err;
	}
	else
	{
		samples_since_out ++;

		return last_p;
	}
}

int16_t pid(int16_t p)
{
	if((i + p) < maxi && (i + p) > (0 - maxi))
	{
		i = i + p;
	}

	d = p - last_p;
	last_p = p;

	int32_t sum = (p * kp + i * ki + d * kd) / 1000;

	return (int16_t)sum;
}

void motors(int16_t sum, int16_t v)
{
	if(sensor_counter == 0)
	{
		if(v > decrease_vref)
		{
			v = v - decrease_vref;
		}
	}

	if(sum < 0)
	{
		DIGITAL_IO_SetOutputLow(&Digital_R);
		PWM_SetDutyCycle(&PWM_R, v);

		if((v + sum) < 0)
		{
			DIGITAL_IO_SetOutputHigh(&Digital_L);
			PWM_SetDutyCycle(&PWM_L, (10000 + v + sum));
		}
		else
		{
			DIGITAL_IO_SetOutputLow(&Digital_L);
			PWM_SetDutyCycle(&PWM_L, (v + sum));
		}
	}
	else
	{
		DIGITAL_IO_SetOutputLow(&Digital_L);
		PWM_SetDutyCycle(&PWM_L, v);

		if((v - sum) < 0)
		{
			DIGITAL_IO_SetOutputHigh(&Digital_R);
			PWM_SetDutyCycle(&PWM_R, (10000 + v - sum));
		}
		else
		{
			DIGITAL_IO_SetOutputLow(&Digital_R);
			PWM_SetDutyCycle(&PWM_R, (v - sum));
		}
	}
}

void updateMotors(void)
{
	motors(pid(error()), speed);
}

void switchColor(void)
{
	color_flag = ~color_flag;
}

void straightAhead(void)
{
	sampling_timer_status = SYSTIMER_StopTimer(sampling_timer);
	motors(0, max_vref);
}

void bridge(void)
{
	speed = bridge_vref;
}

void bump(void)
{
	speed = bump_vref;
}

void maxPower(void)
{
	speed = max_vref;
}

void avoidWheelie(void)
{
	avoid_wheelie_timer_status = SYSTIMER_StartTimer(avoid_wheelie_timer);

	speed = initial_vref;

	sampling_timer_status = SYSTIMER_StartTimer(sampling_timer);
}

void initialize(void)
{
	error();
	if(sensor_counter > 4) //initial color recognition. delete for manual setting
	{
		color_flag = ~color_flag;
	}

	avoidWheelie();
}

void magoneDetect(void) //callback function for left magnet sensor interrupt
{
	mag1_state = 0; //set to 0 to disable left magnet sensor
}

void magtwoDetect(void) //callback function for right magnet sensor interrupt
{
	mag2_state = 0; //set to 0 to disable right magnet sensor
}

int main(void)
{
	DAVE_STATUS_t status;

	uint8_t mag_case = 1; //magnet input processing case
	uint32_t previous_time = 0;
	uint32_t current_time = 0;

	status = DAVE_Init();           /* Initialization of DAVE APPs  */

	if(status == DAVE_STATUS_FAILURE)
	{
		/* Placeholder for error handler code. The while loop below can be replaced with an user error handler. */
		XMC_DEBUG("DAVE APPs initialization failed\n");

		while(1U)
		{

		}
	}

	motors(0, 0);

	initialize_rc_driver();

	while(get_received_rc_code() != rc_power) //decomment for remote start
	{

	}

	clear_received_rc_code();

	initial_delay_timer = SYSTIMER_CreateTimer(initial_delay, SYSTIMER_MODE_ONE_SHOT, (void*)initialize, NULL);
	avoid_wheelie_timer = SYSTIMER_CreateTimer(avoid_wheelie_time, SYSTIMER_MODE_ONE_SHOT, (void*)maxPower, NULL);
	sampling_timer = SYSTIMER_CreateTimer(sampling_time, SYSTIMER_MODE_PERIODIC, (void*)updateMotors, NULL);

	initial_delay_timer_status = SYSTIMER_StartTimer(initial_delay_timer);

	while(1U)
 	{
		if(sensor_counter > 4) //blind color switch - decomment if magnet sensor fails to start color switch
		{
			switchColor();
		}

		if(samples_since_out > samples_out_limit)
		{
			sampling_timer_status = SYSTIMER_StopTimer(sampling_timer);

			motors(0, 0);
		}

		if(get_received_rc_code() == rc_power) //remote stop. delete if robot stops for no apparent reason
		{
			sampling_timer_status = SYSTIMER_StopTimer(sampling_timer);

			motors(0, 0);
		}
		clear_received_rc_code();

		current_time = SYSTIMER_GetTime();

		switch(mag_case) //magnet sensor input processing
		{
			case 1: //standby

				if((mag1_state != 0) || (mag2_state != 0))
				{
					previous_time = current_time;
					mag_case = 2;
				}

				break;

			case 2: //one or both magnet sensors activated

				if((mag1_state != 0) && (mag2_state != 0)) //both sensors activated entry
				{
					bridge();

					previous_time = current_time;
					mag_case = 3;
				}
				else
				{
					if(current_time - previous_time > 300000)
					{
						if(mag1_state != 0) //left sensor activated entry
						{
							bump();

							previous_time = current_time;
							mag_case = 5;
						}
						/*if(mag2_state != 0) //right sensor activated entry
						{
							straightAhead();

							previous_time = current_time;
							mag_case = 6;
						}*/
					}
				}

				break;

			case 3: //debounce if both sensors activated

				if(current_time - previous_time > 300000)
				{
					previous_time = current_time;
					mag_case = 4;
				}

				break;

			case 4: //both sensors activated

				if(current_time - previous_time > bridge_time)
				{
					maxPower();

					mag1_state = 0;
					mag2_state = 0;
					mag_case = 1;
				}

				break;

			case 5: //left sensor activated

				if(current_time - previous_time > bump_time)
				{
					maxPower();

					mag1_state = 0;
					mag2_state = 0;
					mag_case = 1;
				}

				break;

			case 6: //right sensor activated

				if(current_time - previous_time > color_switch_time)
				{
					switchColor();
					sampling_timer_status = SYSTIMER_StartTimer(sampling_timer);

					mag1_state = 0;
					mag2_state = 0;
					mag_case = 1;
				}

			default:

				break;
		}
 	}

	return 0;
}
