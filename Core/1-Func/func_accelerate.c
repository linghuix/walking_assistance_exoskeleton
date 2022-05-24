/*
 * accelerate.c
 *
 *  Created on: May 25, 2020
 *      Author: test
 */
#include "func_accelerate.h"

uint8_t flag_1, flag_2, flag_3;  //左侧IMU角加速度/角速度/角度获取到最新数据的标志
uint8_t flag_11, flag_22, flag_33;  //右侧IMU角加速度，角速度，角度获取到最新数据的标志
uint8_t acc1[11];                   //存储左侧IMU通讯原始数据
uint8_t acc2[11];                   //存储右侧IMU通讯原始数据
int16_t a2[3], w2[3], angle2[3];  //右侧IMU检测到的角加速度/角速度/角度
int16_t a1[3], w1[3], angle1[3];  //左侧IMU检测到的角加速度/角速度/角度
float   T;                        // IMU检测到的温度


/**
 * @date   2022/5/22
 * @author lhx
 * @brief  初始化左侧加速度计
 */
void Acc1_Init(void) { MX_UART4_UART_Init(); }

/**
 * @date   2022/5/22
 * @author lhx
 * @brief  初始化右侧加速度计
 */
void Acc2_Init(void) { MX_USART2_UART_Init(); }


/**
 * @date   2022/5/22
 * @author lhx
 * @brief  启动右侧加速度计,启动串口接收中断
 */
void Acc2_Start(void)
{
  // HAL_UART_Transmit_IT(&acc2_huart, test_data, 5);
  HAL_UART_Receive_IT(&acc2_huart, acc2, 1);
}

/**
 * @date   2022/5/22
 * @author lhx
 * @brief  启动左侧加速度计,启动串口接收中断
 */
void Acc1_Start(void)
{
  // HAL_UART_Transmit_IT(&acc1_huart, test_data, 5);
  HAL_UART_Receive_IT(&acc1_huart, acc1, 1);
}


/**
 * @date   2022/5/22
 * @author lhx
 * @brief  加速度计接收中断回调函数，解析通讯协议，提取角度，角速度，角加速度和温度
 * @param  huart  存储串口信息的结构指针
 */
uint8_t        state1 = 0, state2 = 0;
extern uint8_t CommandReceive[20], receivebyte, length;
extern uint8_t hardtest_CommandReceive[200], hardtest_receivebyte, hardtest_length;
void           HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // HC05 receive
  if (huart->Instance == USART1) {
    INF("command arrive");
    hardtest_CommandReceive[hardtest_length] = hardtest_receivebyte;
    hardtest_length++;
    HAL_UART_Receive_IT(&huart1, &hardtest_receivebyte, 1);
  }
  if (huart->Instance == acc1_uart) {
    //		INF("acc1-%d-%d-%d-%d ", state1, flag_1, flag_2, flag_3);
    switch (state1) {
      case 0:
        if (acc1[0] == 0x55) {
          state1 = 1;
          HAL_UART_Receive_IT(&acc1_huart, &acc1[1], 10);
        }
        else
          HAL_UART_Receive_IT(&acc1_huart, acc1, 1);
        break;
      case 1:
        switch (acc1[1]) {
          case 0x51:  //角加速度，温度
            a1[0]  = ((int16_t)(acc1[3] << 8 | acc1[2]));
            a1[1]  = ((int16_t)(acc1[5] << 8 | acc1[4]));
            a1[2]  = ((int16_t)(acc1[7] << 8 | acc1[6]));
            T      = ((int16_t)(acc1[9] << 8 | acc1[8]));
            flag_1 = 1;
            break;
          case 0x52:  //角速度，温度
            w1[0]  = ((int16_t)(acc1[3] << 8 | acc1[2]));
            w1[1]  = ((int16_t)(acc1[5] << 8 | acc1[4]));  // hip flex z
            w1[2]  = ((int16_t)(acc1[7] << 8 | acc1[6]));
            T      = ((int16_t)(acc1[9] << 8 | acc1[8]));
            flag_2 = 1;
            break;
          case 0x53:  //角度，温度
            angle1[0] = ((int16_t)(acc1[3] << 8 | acc1[2]));
            angle1[1] = ((int16_t)(acc1[5] << 8 | acc1[4]));  // hip felx
            angle1[2] = ((int16_t)(acc1[7] << 8 | acc1[6]));
            T         = ((int16_t)(acc1[9] << 8 | acc1[8]));
            flag_3    = 1;
            break;
        }
        state1 = 0;
        HAL_UART_Receive_IT(&acc1_huart, acc1, 1);
        break;
    }
  }
  if (huart->Instance == acc2_uart) {
    //		INF("acc2-%d-%d-%d-%d ", state2, flag_11, flag_22, flag_33);
    switch (state2) {
      case 0:
        if (acc2[0] == 0x55) {
          state2 = 1;
          HAL_UART_Receive_IT(&acc2_huart, &acc2[1], 10);
        }
        else
          HAL_UART_Receive_IT(&acc2_huart, acc2, 1);
        break;
      case 1:
        switch (acc2[1]) {
          case 0x51:
            a2[0]   = ((int16_t)(acc2[3] << 8 | acc2[2]));
            a2[1]   = ((int16_t)(acc2[5] << 8 | acc2[4]));
            a2[2]   = ((int16_t)(acc2[7] << 8 | acc2[6]));
            T       = ((int16_t)(acc2[9] << 8 | acc2[8]));
            flag_11 = 1;
            break;
          case 0x52:
            w2[0]   = ((int16_t)(acc2[3] << 8 | acc2[2]));
            w2[1]   = ((int16_t)(acc2[5] << 8 | acc2[4]));  // hip flex z
            w2[2]   = ((int16_t)(acc2[7] << 8 | acc2[6]));
            T       = ((int16_t)(acc2[9] << 8 | acc2[8]));
            flag_22 = 1;
            break;
          case 0x53:
            angle2[0] = ((int16_t)(acc2[3] << 8 | acc2[2]));
            angle2[1] = ((int16_t)(acc2[5] << 8 | acc2[4]));  // hip felx -90~90
            angle2[2] = ((int16_t)(acc2[7] << 8 | acc2[6]));
            T         = ((int16_t)(acc2[9] << 8 | acc2[8]));
            flag_33   = 1;
            break;
        }
        state2 = 0;
        HAL_UART_Receive_IT(&acc2_huart, acc2, 1);
        break;
    }
  }
}


// -------------------*---------------------------*---------- TEST
// ---------*----------------------*-----------------//
// -------------------*---------------------------*---------- TEST
// ---------*----------------------*-----------------//

#ifdef ACC_TEST

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

/**
 * @author lhx
 * @date May 25, 2020
 *
 * @brief : 通讯测试
 * 			对于stm32f103rbt6,未开启AFIO，映射。
 * 			串口２　-　PA2-RX  PA3-TX
 * 			串口３　-  PB10-TX  PB11-RX
 */
TEST test_acc_communication(void)
{
  Acc1_Init();
  Acc2_Init();

  uint8_t test_data[5] = {0x55, 0x66, 0x22, 0x33, 0x44};
  HAL_UART_Transmit_IT(&acc1_huart, test_data, 1);
  HAL_UART_Transmit_IT(&acc2_huart, test_data, 5);
}
#endif
