/*
 * ECON_controller_I.h
 *
 *  Created on: May 24, 2020
 *      Author: test
 */

#ifndef _FUNC_ECON_CONTROLLER_I_H_
#define _FUNC_ECON_CONTROLLER_I_H_


#include "conf_tim.h"
#include "conf_gpio.h"
#include "Odrive.h"


#define GPIO2_1 GPIOC,GPIO_PIN_2
#define GPIO2_2 GPIOA,GPIO_PIN_1
#define GPIO1_1 GPIOA,GPIO_PIN_0
#define GPIO1_2 GPIOC,GPIO_PIN_1



#define CLK1	HAL_GPIO_WritePin(GPIO1_1, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO1_2, GPIO_PIN_SET);
#define UNCLK1	HAL_GPIO_WritePin(GPIO1_1, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIO1_2, GPIO_PIN_RESET);
#define RESET1	HAL_GPIO_WritePin(GPIO1_1, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO1_2, GPIO_PIN_RESET)

#define CLK2	HAL_GPIO_WritePin(GPIO2_1, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO2_2, GPIO_PIN_SET);
#define UNCLK2	HAL_GPIO_WritePin(GPIO2_1, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIO2_2, GPIO_PIN_RESET);
#define RESET2	HAL_GPIO_WritePin(GPIO2_1, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIO2_2, GPIO_PIN_RESET)


void ECON_action(void);
void ECON_I_init(void);
void set_I_direction(uint8_t node, float I);



TEST test_ECON_I_controller(void);
#endif /* 1_FUNC_ECON_CONTROLLER_I_H_ */
