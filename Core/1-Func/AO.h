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


#define MaxSize 20
struct Adaptive_Oscillators{
    float va;					// learning parameter
    float vw;
    float vph;

    int order;					// order+1 AOs
    float dt;					// sample interval
    float phase[20];			// phase
    int step;					// predict step
    
    float aa[20],pphh[20],ww;	// amplitude, phase, basic frequency
    float init_aa[20],init_ww,init_pphh[20];
    float w_real[20];			// frequency of all the AOs
	
    float output;				// Aos output
    float predict;				// Aos predict based on the step

    /*用于检测预测的准确性*/
    float predictedSaveData[MaxSize];	// save predict input curve
	float predictedBasicPhase;
    float outputSave[MaxSize];
    int8_t index;		//指向要填充的下一个空间
};


#define PI 3.14159

extern struct Adaptive_Oscillators hip1,hip2;
extern uint64_t Aoindex;

void AO_Init(float w0);
void AO(float d,uint8_t node);

float switch_task(struct Adaptive_Oscillators * AO, float d, float w,uint8_t node);
float assive_torque(struct Adaptive_Oscillators * AO, float d);



void test_AO(void);
void test_AOs(void);



#include "win.h"

uint8_t findpeak(WINp win);
#endif /* 1_FUNC_AO_H_ */
