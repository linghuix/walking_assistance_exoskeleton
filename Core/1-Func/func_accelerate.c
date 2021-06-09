/*
 * accelerate.c
 *
 *  Created on: May 25, 2020
 *      Author: test
 */
#include "func_accelerate.h"

uint8_t imu_2_flag = 0;
uint8_t flag_1,flag_2,flag_3;
uint8_t flag_11,flag_22,flag_33;
uint8_t acc2[11];
int16_t a2[3], w2[3], angle2[3];
int16_t a1[3], w1[3], angle1[3];
float T;

uint8_t acc1[11];


void Acc1_Init(void)
{
	MX_USART3_UART_Init();
}


void Acc2_Init(void)
{
	MX_USART2_UART_Init();
}

//uint8_t test_data[5] = {0x55,0x00,0x00,0x00,0x00};

void Acc2_Start(void)
{
	//HAL_UART_Transmit_IT(&acc2_huart, test_data, 5);
	HAL_UART_Receive_IT(&acc2_huart, acc2, 1);
}



void Acc1_Start(void)
{
	//HAL_UART_Transmit_IT(&acc1_huart, test_data, 5);
	HAL_UART_Receive_IT(&acc1_huart, acc1, 1);
}


uint8_t state1 = 0,state2 = 0;
extern uint8_t CommandReceive[20], receivebyte, length;
extern uint8_t hardtest_CommandReceive[20], hardtest_receivebyte, hardtest_length;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1){
		INF("command arrive");
		hardtest_CommandReceive[hardtest_length] = hardtest_receivebyte;
		hardtest_length++;
		HAL_UART_Receive_IT(&huart1, &hardtest_receivebyte, 1);
	}
	if(huart->Instance == acc1_uart){
		INF("acc1,%d", state1);
		switch(state1){
			case 0:
				if(acc1[0]==0x55){state1 = 1;HAL_UART_Receive_IT(&acc1_huart, &acc1[1], 10);}
				else HAL_UART_Receive_IT(&acc1_huart, acc1, 1);
				break;
			case 1:
				switch(acc1[1]){
				  case 0x51:
					a1[0] = ((int16_t)(acc1[3]<<8| acc1[2]));
					a1[1] = ((int16_t)(acc1[5]<<8| acc1[4]));
					a1[2] = ((int16_t)(acc1[7]<<8| acc1[6]));
					T = ((int16_t)(acc1[9]<<8| acc1[8]));
					flag_1 = 1;
					break;
				  case 0x52:
					w1[0] = ((int16_t)(acc1[3]<<8| acc1[2]));
					w1[1] = ((int16_t)(acc1[5]<<8| acc1[4]));//hip flex z
					w1[2] = ((int16_t)(acc1[7]<<8| acc1[6]));
					T = ((int16_t)(acc1[9]<<8| acc1[8]));
					flag_2 = 1;
					break;
				  case 0x53:
					angle1[0] = ((int16_t)(acc1[3]<<8| acc1[2]));
					angle1[1] = ((int16_t)(acc1[5]<<8| acc1[4]));//hip felx
					angle1[2] = ((int16_t)(acc1[7]<<8| acc1[6]));
					T = ((int16_t)(acc1[9]<<8| acc1[8]));
					flag_3 = 1;
					break;
				}
				state1 = 0;HAL_UART_Receive_IT(&acc1_huart, acc1, 1);
				break;
		}
	}
	if(huart->Instance == acc2_uart){
		INF("acc2,%d ", state2);
		switch(state2){
			case 0:
				if(acc2[0]==0x55){state2 = 1;HAL_UART_Receive_IT(&acc2_huart, &acc2[1], 10);}
				else HAL_UART_Receive_IT(&acc2_huart, acc2, 1);
				break;
			case 1:
				switch(acc2[1]){
				  case 0x51:
					a2[0] = ((int16_t)(acc2[3]<<8| acc2[2]));
					a2[1] = ((int16_t)(acc2[5]<<8| acc2[4]));
					a2[2] = ((int16_t)(acc2[7]<<8| acc2[6]));
					T = ((int16_t)(acc2[9]<<8| acc2[8]));
					flag_11 = 1;
					break;
				  case 0x52:
					w2[0] = ((int16_t)(acc2[3]<<8| acc2[2]));
					w2[1] = ((int16_t)(acc2[5]<<8| acc2[4]));//hip flex z
					w2[2] = ((int16_t)(acc2[7]<<8| acc2[6]));
					T = ((int16_t)(acc2[9]<<8| acc2[8]));
					flag_22 = 1;
					break;
				  case 0x53:
					angle2[0] = ((int16_t)(acc2[3]<<8| acc2[2]));
					angle2[1] = ((int16_t)(acc2[5]<<8| acc2[4]));//hip felx  -90~90
					angle2[2] = ((int16_t)(acc2[7]<<8| acc2[6]));
					T = ((int16_t)(acc2[9]<<8| acc2[8]));
					flag_33 = 1;
					break;
				}
				state2 = 0;HAL_UART_Receive_IT(&acc2_huart, acc2, 1);
				break;
		}
	}
}



/* TEST

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1){
	    //imu_2_flag = 1;
		HAL_UART_Receive_IT(&huart1, acc1, 6);
		HAL_UART_Transmit_IT(&huart1, acc1, 6);
	}
	if(huart->Instance == acc1_uart){
	    //imu_2_flag = 1;
		HAL_UART_Receive_IT(&acc1_huart, acc1, 6);
		HAL_UART_Transmit_IT(&acc1_huart, acc1, 6);
	}
	if(huart->Instance == acc2_uart){
	    //imu_2_flag = 1;
		HAL_UART_Receive_IT(&acc2_huart, acc2, 6);
		HAL_UART_Transmit_IT(&acc2_huart, acc2, 6);
	}
}
*/

/*
 * author lhx
 * May 25, 2020
 *
 * @brief : 通讯测试
 * 			对于stm32f103rbt6,未开启AFIO，映射。
 * 			串口２　-　PA2-RX  PA3-TX
 * 			串口３　-  PB10-TX  PB11-RX
 * Window > Preferences > C/C++ > Editor > Templates.
 */

TEST test_acc_communication(void)
{
	Acc1_Init();
	Acc2_Init();

	uint8_t test_data[5] = {0x55,0x66,0x22,0x33,0x44};
	HAL_UART_Transmit_IT(&acc1_huart, test_data, 1);
	HAL_UART_Transmit_IT(&acc2_huart, test_data, 5);
}
