/*
        串口打印调试代码
*/
#include "debug.h"

//#define DEBUG_TEST

// G T R Vt


extern UART_HandleTypeDef huart1;
uint8_t                   ch_print;
uint8_t                   ch_scanf;
struct Buffer             debugBuffer = {"inital", 0, 0};


/**
 * @author lhx
 * @date May 13, 2020
 *
 * @brief : put data to buffer, buffer is FULL return 0.
 * @param c - 需要存入的数据, string to be inputting to buffer
 */
uint8_t addDebugBuffer(char c)
{
  if ((debugBuffer.in + 2) % BufferSize == debugBuffer.out) {
    return 0;
  }
  else {
    debugBuffer.data[debugBuffer.in] = c;
    debugBuffer.in                   = (debugBuffer.in + 1) % BufferSize;
    return 1;
  }
}

/**
 * @author lhx
 * @Date May 13, 2020
 *
 * @brief : 获取buff中的数据，buffer为空则返回0
 * @return c - 取出的字符 get string
 *
 */
char getDebugBuffer(void)
{
  char c;
  if (debugBuffer.in == debugBuffer.out) {
    return 0;
  }
  else {
    c               = debugBuffer.data[debugBuffer.out];
    debugBuffer.out = (debugBuffer.out + 1) % BufferSize;
    return c;
  }
}

/**
 * @author lhx
 * @date May 14, 2020
 *
 * @brief : STM32CubeIDE printf 重定位到 PORT
 *
 */
#ifdef Cube
#ifdef BUFF_Printf
int _write(int file, char *ptr, int len)
{
  int DataIndex;

  for (DataIndex = 0; DataIndex < len; DataIndex++) {
    addDebugBuffer(*ptr++);
  }
  __HAL_UART_ENABLE_IT(&PORT, UART_IT_TXE);
  return len;
}
#endif
#endif


/**
 * @author lhx
 * @date May 29, 2020
 *
 * @brief : 在 keil 中重定向 printf 与 scanf.
            定义了BUFF_Printf表明串口输出有配置缓冲区
 * @Note 注意使用前需要开启 microLib
 *
 */
#ifdef KEIL
#ifdef BUFF_Printf
int fputc(int ch, FILE *f)  // Keil
{
  if (addDebugBuffer(ch) != 0) {
    if (huart1.Instance == USART1) {  // waiting for usart1 initalization. so that printf can be
                                      // used before usart really work.
      __HAL_UART_ENABLE_IT(&PORT, UART_IT_TXE);
    }
    return ch;
  }
  return 0;
}

int fgetc(FILE *F)
{
  HAL_UART_Receive(&PORT, &ch_scanf, 1, 0xffff);  //����
  return ch_scanf;
}
#endif


#ifdef NO_BUFF_Printf
int fputc(int ch, FILE *f)  // Keil
{
  while ((USART1->SR & 0X40) == 0) {
  };  //循环发送,直到发送完毕
  USART1->DR = (uint8_t)ch;
  return ch;
}

int fgetc(FILE *F)  // Keil
{
  HAL_UART_Receive(&PORT, &ch_scanf, 1, 0xffff);  //����
  return ch_scanf;
}
#endif

#endif

/**
 * @author  lhx
 * @date    May 13, 2020
 *
 * @brief : USART initalization. Enable serial IT
 *			Set serial property
 *
 */
void debug_init(void)
{
  MX_USART1_UART_Init();

  MSG("debug initing ... \r\n");
}


/**
 * @author lhx
 * @date May 13, 2020
 *
 * @brief : usart1 interrupt service function
 */
void debug_IRQ(void)
{
  uint32_t isrflags = READ_REG(huart1.Instance->SR);
  uint32_t cr1its   = READ_REG(huart1.Instance->CR1);
  char     c;

  /* UART in mode Transmitter -----------TXE
   * ------------------------------------*/
  if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET)) {
    c = getDebugBuffer();

    /* If buffer is not empty, then get a string and send it to TX_USART_buffer */
    if (c != 0) {
      huart1.Instance->DR = (uint16_t)(c & (uint16_t)0x01FF);
    }
    /* If buffer is empty, Disable the UART Transmit Complete Interrupt*/
    else {
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
      //__HAL_UART_DISABLE_IT(huart, UART_IT_TC);
    }
  }

  /* UART in mode Receiver
   * -------------------RXNE------------------------------*/
  /*if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) !=
  RESET)){ UART_Receive_IT(huart1); return;
  }*/
}

/**
 ******************************************************************************
 * @section    Test
 * @author  xlh
 * @brief
 ******************************************************************************
 */
//@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ test
#ifdef DEBUG_TEST

#include "main.h"

void test_SpeedOfBuffer_printf(void)
{
  Core_Config();
  tick_init(1000000);
  debug_init();

  int i;
  while (1) {
    i = 50;
    while (i--)
      TESTOUT(
          "%d , %d - send buffer test for skipping error possibility and "
          "speeding: in-%d-out-%d\r\n",
          i, HAL_GetTick(), debugBuffer.in, debugBuffer.out);
    TESTOUT("delay 2000ms\r\n");
    i = 100000000;
    while (i--)
      ;
  }
}

/**
 * @brief  Test the function of the uart receive and transmit.
 * @note pay attention to that
 *		  debug must define NO_BUFF_Printf in debug.h
 *		  AND CANNOT USE MSG for it can only used in BUFF_Printf
 * 	  USART1 interrupt must be use HAL_UART_IRQHandler(&huart1);
 *       In theory, we can use scanf() but we haven't achieve this.
 *
 */

uint8_t test_buffRX[20];
uint8_t test_buffTX[100];
void    test_printf(void)
{
  //		Core_Config();
  //		debug_init();

  uint8_t test_data[] = "hello word!\r\n";
  HAL_UART_Transmit_IT(&huart1, test_data, sizeof(test_data));
  HAL_Delay(200);  // wait for transmit finish.


  /*scanf printf*/

  uint8_t test_data1[] = "test scanf ... \r\nplease enter a num \r\n";
  HAL_UART_Transmit_IT(&huart1, test_data1, sizeof(test_data1));
  int x = 365;
  HAL_Delay(1000);  // wait for transmit finish.
  //		scanf("%d",&x);

  sprintf((char *)test_buffTX, "receive : %d\r\n", x);
  HAL_UART_Transmit_IT(&huart1, test_buffTX, sizeof(test_buffTX));
  HAL_Delay(200);


  /*IT*/
  sprintf((char *)test_buffTX, "please input one hex data\r\n");
  HAL_UART_Transmit_IT(&huart1, test_buffTX, sizeof(test_buffTX));
  HAL_Delay(200);
  sprintf((char *)test_buffTX,
          "it will return 0x0A, that is the last char when you input to scanf "
          "function\r\n");
  HAL_UART_Transmit_IT(&huart1, test_buffTX, sizeof(test_buffTX));
  HAL_Delay(200);

  for (int i; i < 10; i++) {
    test_buffRX[i] = i;
  }
  HAL_UART_Receive_IT(&huart1, test_buffRX, 2);
}


void test_RXTX_callback(void)
{
  static uint8_t msg[2];
  for (int i = 0; i < 2; i++) {
    msg[i] = test_buffRX[i];
  }
  HAL_UART_Transmit_IT(&huart1, msg, 2);
  HAL_UART_Receive_IT(&huart1, test_buffRX, 2);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    test_RXTX_callback();
  }
}
#endif
