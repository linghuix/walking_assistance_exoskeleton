/*
 * func_can.h
 *
 *  Created on: Mar 14, 2020
 *      Author: test
 */

#ifndef __ADC_H_
#define __ADC_H_

#include "BSP.h"

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);


TEST getData(void);

TEST getSingleChannelData(void);
#endif /* BSP_FUNC_CAN_H_ */
