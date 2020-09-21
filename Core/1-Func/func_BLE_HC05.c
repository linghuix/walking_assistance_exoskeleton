/*
 * func_BLE.c
 *
 *  Created on: May 25, 2020
 *      Author: test
 */
#include "func_BLE_HC05.h"

void HC05_Init(void)
{
	MX_USART1_UART_Init();
	printf("beginning...\r\n");
}


void HC05_config(void)
{
	uint8_t test_data[] = "AT+UART\r\n";
	HAL_UART_Transmit(&HC05_huart, test_data, sizeof(test_data),500);
}

void HC05_send(uint8_t data[], uint8_t size)
{
	HAL_UART_Transmit_IT(&HC05_huart, data, size);
}

uint32_t inc=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4){
		inc++;
	}
}

TEST test_HC05_communication(void)
{
	HC05_Init();
	uint8_t test_data[5] = {0x55,0x66,0x22,0x33,0x44};
	HAL_UART_Transmit_IT(&HC05_huart, test_data, 3);
}
