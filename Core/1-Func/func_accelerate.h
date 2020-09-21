/*
 * accelerate.h
 *
 *  Created on: May 25, 2020
 *      Author: test
 */

#ifndef _FUNC_ACCELERATE_H_
#define _FUNC_ACCELERATE_H_

#include "conf_usart.h"


#define acc1_huart huart3
#define acc2_huart huart2

#define acc1_uart USART3
#define acc2_uart USART2


extern uint8_t imu_2_flag;
extern uint8_t flag_1,flag_2,flag_3;
extern uint8_t flag_11,flag_22,flag_33;

extern uint8_t acc2[11];
extern int16_t a2[3];
extern int16_t w2[3];
extern int16_t angle2[3];

extern uint8_t acc1[11];
extern int16_t a1[3];
extern int16_t w1[3];
extern int16_t angle1[3];
extern float T;

extern uint8_t acc1[11];


void Acc1_Init(void);
void Acc2_Init(void);

void Acc2_Start(void);
void Acc1_Start(void);

#endif /* 1_FUNC_ACCELERATE_H_ */
