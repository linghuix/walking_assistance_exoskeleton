/*
 * PO.h
 *
 *  Created on: Jun 3, 2020
 *      Author: test
 */

#ifndef _FUNC_PO_H_
#define _FUNC_PO_H_


#define POMODE -20


#include "FUNC.h"
#include "math.h"



float PO(float d, float w,uint8_t node);
float APOPhase1(float d, float w);
float APOPhase2(float d, float w);
extern int stopflag2, stopflag3;
#endif /* 1_FUNC_PO_H_ */
