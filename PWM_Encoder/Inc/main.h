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
#define MAX_RC_CH_NB 	8
#define USED_RC_CH_NB 	6
#define TIMER_1_CH_NB 	3
#define TIMER_2_CH_NB 	4
#define PWM_Ratio		24 // 72Mhz/3(Prescaler)

typedef enum {
CH_NOT_DETECTED = 0,
CH_DETECTED
}CH_Detection_TypeDef;

//#define CH_DETECTED		1
//#define CH_NOT_DETECTED	0

typedef struct CH_Pwm_Val_s
{
	uint32_t 				Current_Edge;
	uint32_t 				Rising;
	uint32_t 				Falling;
	uint32_t 				Delta;
	CH_Detection_TypeDef 	CH_Detected;
	// test only
	uint32_t Prev_Delta;

}CH_Pwm_Val_t;



#endif /* MAIN_H_ */
