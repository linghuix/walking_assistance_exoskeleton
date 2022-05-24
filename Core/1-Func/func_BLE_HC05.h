/*
 * func_BLE_HC05.h
 *
 *  Created on: May 25, 2020
 *      Author: test
 */

#ifndef _FUNC_FUNC_BLE_HC05_H_
#define _FUNC_FUNC_BLE_HC05_H_

#include "conf_usart.h"
#include "debug.h"

#define HC05_huart huart1

void HC05_Init(void);
void HC05_config(void);

extern uint32_t inc;


void HC05_send(uint8_t data[], uint8_t size);
TEST test_HC05_communication(void);
TEST test_USART1_communication(void);
void HC05_RcvCmd(void);

TEST IDLE_UART_IRQHandler (UART_HandleTypeDef *huart);


#endif /* 1_FUNC_FUNC_BLE_HC05_H_ */

