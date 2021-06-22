/*
 * AO.h
 *
 *  Created on: Jun 3, 2020
 *      Author: test
 */

#ifndef _FUNC_AO_H_
#define _FUNC_AO_H_

#define AOMODE +20


#include "FUNC.h"
#include "stdio.h"
#include "math.h"



extern struct Adaptive_Oscillators hip1,hip2;
extern uint64_t Aoindex;

void AO_Init(float w0);
void AO(float d,uint8_t node);

float switch_task(struct Adaptive_Oscillators * AO, float d, float w,uint8_t node);
float assive_torque(struct Adaptive_Oscillators * AO, float d);



void test_AO(void);






#include "win.h"

uint8_t findpeak(WIN win);
#endif /* 1_FUNC_AO_H_ */
