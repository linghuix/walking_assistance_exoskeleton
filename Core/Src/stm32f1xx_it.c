/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "func_BLE_HC05.h"

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
//  MSG("NMI_error\r\n");
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  //MSG("hardware error\r\n");
  while (1)
  {

  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{

  while (1)
  {

  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{

  while (1)
  {

  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{

  while (1)
  {

  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{

}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{

}


/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{

}


/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{

  HAL_IncTick();

}

/**
* @brief This function handles USART1 global interrupt.
*/
#include "debug.h"
void USART1_IRQHandler(void)
{

	debug_IRQ();
	
	HAL_UART_IRQHandler(&huart1);
	IDLE_UART_IRQHandler(&huart1);

}


void USART2_IRQHandler(void)
{
//printf("IRQ uart4\r\n");
  HAL_UART_IRQHandler(&huart2);
}


void USART3_IRQHandler(void)
{

  HAL_UART_IRQHandler(&huart3);
}

void TIM4_IRQHandler(void)
{

	HAL_TIM_IRQHandler(&htim4);
}

void TIM1_UP_IRQHandler(void)
{

	HAL_TIM_IRQHandler(&htim1);
}

//void TIM1_TRG_COM_IRQHandler(void)
//{

//	HAL_TIM_IRQHandler(&htim1);
//}

//void TIM1_CC_IRQHandler(void)
//{

//	HAL_TIM_IRQHandler(&htim1);
//}

//void TIM1_BRK_IRQHandler(void)
//{

//	HAL_TIM_IRQHandler(&htim1);
//}

void UART4_IRQHandler(void)
{
//	printf("4\r\n");
  HAL_UART_IRQHandler(&huart4);
}


void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart5);
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
