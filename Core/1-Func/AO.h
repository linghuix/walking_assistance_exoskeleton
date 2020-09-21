/*
 * AO.h
 *
 *  Created on: Jun 3, 2020
 *      Author: test
 */

#ifndef _FUNC_AO_H_
#define _FUNC_AO_H_


#include "FUNC.h"
#include <stdio.h>
#include <math.h>


extern struct Adaptive_Oscillators hip1,hip2;
extern uint64_t index;

void AO_Init(void);
void AO(float d,uint8_t node);

float switch_task(struct Adaptive_Oscillators * AO, float d, float w,uint8_t node);
float assive_torque(struct Adaptive_Oscillators * AO, float d);
	
void test_AO(void);

#endif /* 1_FUNC_AO_H_ */
