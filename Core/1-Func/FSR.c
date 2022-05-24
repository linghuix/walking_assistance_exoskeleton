#include "FSR.h"


/**
 *****************************************************************************************************************
 * @file    FSR.c
 * @author  xlh
 * @data	 20210123
 * @brief   FSR reader module through UART connector corresponding to self-made
 *FSR detector.
 *
 *           + Initialization and de-initialization functions
 *           + uart read functions
 */


#define FSR_TEST


/**
 * @brief  Init Force collector beforce measure force data.
 *         1. initalize adc1 device
 */

void FSR_Init(void)
{
  MX_USART1_UART_Init();
  INF("fsr initing ... \r\n");
}


/**
 * @brief  Get raw force information. It is not calibrated and handled.
 */
uint8_t  cmd    = 0x80;
uint8_t  rev[2] = {0};
uint16_t GetFSRForce(void)
{
  uint16_t force = 0;
  HAL_UART_Transmit_IT(&huart1, &cmd, 1);
  if (HAL_OK == HAL_UART_Receive(&huart1, rev, 2, 1)) {
    force = rev[1] << 8 | rev[0];
  }

  return force;
}


float FSROffset = 0;  // Offset value at zero external force
/**
 * @brief  Calcuate the offset at n = OffsetWindowsPosition
 * @retval Offset
 */
#define OffsetWindowsPosition 1000
float GetFSROffset(void)
{
  if (OffsetWindowsPosition) {
    TESTOUT("------------------ offset get --------------------\r\n");
  }

  return FSROffset;
}


#ifdef FSR_TEST

uint16_t FSR_force = 0;

TEST FSRCollectExperiment(void)
{
  FSR_Init();
  while (1) {
    FSR_force = GetFSRForce();
    TESTOUT("%d\r\n", FSR_force);
    HAL_Delay(20);
  }
}

#endif
