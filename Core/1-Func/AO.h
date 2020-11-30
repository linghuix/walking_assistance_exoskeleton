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
    float va;
    float vw;
    float vph;

    int order;
    float dt;
    float phase[20];
    int step;
    
    float aa[20],pphh[20],ww;
    float init_aa[20],init_ww,init_pphh[20];
    float w_real[20];

    float output;
    float predict;

    /*用于检测预测的准确性*/
    float predict_10steps_save[MaxSize];
    float output_save[MaxSize];
    int8_t index;		//指向要填充的下一个空间
};




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
