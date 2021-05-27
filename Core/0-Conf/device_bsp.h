/*
 * device_bsp.h
 *
 *  Created on: Mar 2, 2020
 *      Author: test
 */

#ifndef __DEVICE_BSP_H_
#define __DEVICE_BSP_H_


#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "debug.h"

#define TEST void
#define TEST_ON


#define MSG_deviceTest(...) //printf (__VA_ARGS__)


#define Error_Handler() ERROR(333, "error");//while(1);

#endif /* BSP_DEVICE_BSP_H_ */
