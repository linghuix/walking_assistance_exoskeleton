/**
 * func_BLE.c
 *
 *  Created on: May 25, 2020
 *      Author: test
 */
#include "func_BLE_HC05.h"

/**
 * @date   2022/5/24
 * @author lhx
 * @brief  串口初始化
 * @note
 */
// void HC05_Init(void)
// {
// MX_USART1_UART_Init();
// MSG("beginning...\r\n");
// }


// void HC05_config(void)
// {
// uint8_t test_data[] = "AT+UART\r\n";
// HAL_UART_Transmit(&HC05_huart, test_data, sizeof(test_data), 500);
// }


/**
 * @date   2022/5/24
 * @author lhx
 * @brief  蓝牙串口打印
 * @param  parameters definition
 * @note   usually the special usage need to be noticed
 */
// void HC05_send(uint8_t data[], uint8_t size) { HAL_UART_Transmit_IT(&HC05_huart, data, size); }


uint8_t hardtest_CommandReceive[200], hardtest_receivebyte, hardtest_length;
uint8_t receivebyte, CommandReceive[20], length = 0;

/**
 * @date   2022/5/24
 * @author lhx
 * @brief  串口接收命令的初始化启动函数
 * @note   需要解析串口接收的命令时，需要先调用该函数
 */
void HC05_RcvCmd(void)
{
  MX_USART1_UART_Init();

  HAL_UART_Receive_IT(&huart1, &hardtest_receivebyte, 1);  // 接收中断
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);             // 空闲中断
  //	HAL_UART_Receive_IT(&huart1, &receivebyte, 1);
  MSG("RcvCmd start\r\n");
}


/***********************************************************************************************************************/

int idleflag = 0;  //串口接收进入空闲 标志
/**
 * @date   2022/5/24
 * @author lhx
 * @brief  串口空闲中断回调函数
 * @retval None
 */
TEST USAR_UART1_IDLECallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    // HAL_UART_Transmit(&huart1, hardtest_CommandReceive,
    // hardtest_length, 500);
    idleflag = 1;
  }
}


extern int        PREDICT_TIME;
extern float      tao_Ep, fai_Ep, fai_Er, fai_Ef, a[3], b[3], tao_Fp, fai_Fp, fai_Fr, fai_Ff;
struct parasturct para = {0};  //串口接收命令存储 结构体
void              inputPara(struct parasturct *parasturct, uint8_t *paradata, uint8_t length);
/**
 * @date   2022/5/24
 * @author lhx
 * @brief  hardtest_CommandReceive 中的串口数据解析
 * @note 第一个数字必为170
 */
void commandPrase(void)
{
  CMD("command : %s", hardtest_CommandReceive);
  CMD("command length : %d\r\n", hardtest_length);

  inputPara(&para, hardtest_CommandReceive, hardtest_length);

  int flag = 1;
  if (para.para[0] == 170) {  // 第一个数字必为170 二进制位 0x10101010，便于判断错误
    flag = 0;
  }
  if (flag == 0) {
    printf("get number int %d\r\n", para.paranum);
    if (para.paranum >= 11) {
      /* assistive torque curve*/
      tao_Ep = (float)para.para[1] / 100.0;   // 5-10 Nm
      fai_Ep = (float)para.para[2] / 1000.0;  // 0.2-0.3
      fai_Er = (float)para.para[3] / 1000.0;  // 0.1-0.2
      fai_Ef = (float)para.para[4] / 1000.0;  // 0.1-0.2

      PREDICT_TIME = para.para[5];

      a[0] = (float)para.para[6] / 1000.0;
      a[1] = (float)para.para[7] / 1000.0;
      a[2] = (float)para.para[8] / 1000.0;
      b[0] = (float)para.para[9] / 1000.0;
      b[1] = (float)para.para[10] / 1000.0;
      b[2] = (float)para.para[11] / 1000.0;

      tao_Fp = tao_Ep;
      fai_Fp = 0.5 + fai_Ep;
      fai_Fr = fai_Er;  // 0.1-0.2
      fai_Ff = fai_Ef;  // 0.1-0.2

      // 毕竟是在中断内部，可能会发生死机情况
      //			float tmp1 = (fai_Er*fai_Er);
      //			float tmp2 = (fai_Ep*fai_Ep);
      //			float tmp3 = (fai_Ef*fai_Ef);
      //			a[0] = -tao_Ep/tmp1;
      //			a[1] = 2*tao_Ep*fai_Ep/tmp1;
      //			a[2] = tao_Ep - tao_Ep*tmp2/tmp1;
      //			b[0] = -tao_Ep/tmp3;
      //			b[1] = 2*tao_Ep*fai_Ep/tmp3;
      //			b[2] = tao_Ep - tao_Ep*tmp2/tmp3;

      /* AO */

      //			CMD("para change : PREDICT_TIME=%d\r\n",
      // PREDICT_TIME);
      CMD("para change : "
          "[tao_Ep,fai_Ep,fai_Er,fai_Ef]=[%.3f,%.3f,%.3f,%.3f]\r\n",
          tao_Ep, fai_Ep, fai_Er, fai_Ef);
      //			CMD("para change :
      //[a,b]=[[%.3f,%.3f,%.3f],[%.3f,%.3f,%.3f]]\r\n",
      // a[0],a[1],a[2],b[0],b[1],b[2]);
      printf("\r\n#####\r\n");
      hardtest_length = 0;
    }
  }
}


/**
 * @date   2022/5/24
 * @author lhx
 * @brief
 * 串口输入字符串"5/6/7/8"，解读出所有整数数字。支持正负数字，串口输入需要换行
 * @note
 * 	 对于输入不全的，不会更新对应参数； “/70” - 不更新  “89/”不更新
         但是输入过多的，会更新对应参数；“90/80” 更新
         输入过少但是全的，会更新。
 * @param parasturct:	存储输入数据结构体
 * @param paradata:		串口输入的命令字符串
 * @param length:		串口输入的命令字符串字节数
 */
#include <stdlib.h>
void inputPara(struct parasturct *parasturct, uint8_t *paradata, uint8_t length)
{
  uint8_t para[50];  // paradata 读取串口输入字符，para 读取数字

  int index           = 0;  // index 光标
  parasturct->paranum = 0;  // paranum 参数数量

  // 遍历所有字符串
  for (int i = 0; i < length; i++) {
    if (paradata[i] == '/') {
      para[index]                             = '\0';                // 补全字符串结尾标志
      parasturct->para[parasturct->paranum++] = atoi((char *)para);  // 字符串转整型并存储
      index                                   = 0;
    }
    else if (paradata[i] == '\n') {  // 命令字符串结尾
      para[index]                             = '\0';
      parasturct->para[parasturct->paranum++] = atoi((char *)para);
      break;
    }
    else {
      para[index++] = paradata[i];
    }
  }

  //	uint8_t output[2]; //串口输出 	for(int i=0;i<parasturct->paranum;i++){

  //		HAL_UART_Transmit(&huart1, (uint8_t *)&parasturct->para[i],
  // sizeof(output), 500); 		output[0] = 0xFF;
  // HAL_UART_Transmit(&huart1, output, 1, 500);
  //	}
}

/**
 * @date   2022/5/24
 * @author lhx
 * @brief	串口空闲中断的服务函数
 * @note 	1. need to be put into stm32f10x_it.c
                        2. 调用 USAR_UART1_IDLECallback
 */
void IDLE_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    if (RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))  // IDLE
    {
      __HAL_UART_CLEAR_IDLEFLAG(huart);
      USAR_UART1_IDLECallback(huart);
    }
  }
}
