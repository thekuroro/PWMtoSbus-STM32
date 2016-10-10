/*
 * main.h
 *
 *  Created on: 6 Oct 2016
 *      Author: Kuroro
 */

#ifndef MAIN_H_
#define MAIN_H_

#define RAISING  		0
#define FALLING  		1
#define RC_CH_NB 		3
#define RC_CH_PER_TIMER 3
#define PWM_Ratio		23

typedef struct CH_Pwm_Val_s
{
	uint32_t Current_Edge;
	uint32_t Rising;
	uint32_t Falling;
	uint32_t Delta;
	// test only
	uint32_t Prev_Delta;

}CH_Pwm_Val_t;



#endif /* MAIN_H_ */
