/**
  ******************************************************************************
  * @file    delay.c
  * @author  Linghui Xu
  * @version V1.0
  * @date    2019.4.27
  * @brief   用3.5.0版本库建的工程模板
  ******************************************************************************
  * @attention
  * 1. 本程序主要存放引脚的配置，针对GPIO以及其他复用功能TIME，ADC，USART，CAN...
  * 2. “事件控制”，“重映射”，“调试IO配置”，“外部中断”，时需要开启AFIO
  *
  ******************************************************************************
  */
#include <conf_gpio.h>

void GPIO_Config(void){

	MX_GPIO_output_Init();
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_output_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  
  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  
  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);
}


void MX_GPIO_EVENT_Init(void)
{
	  __HAL_RCC_AFIO_CLK_ENABLE();

	  /* GPIO Ports Clock Enable */
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  __HAL_RCC_GPIOC_CLK_ENABLE();

	  /*EVENT IN*/
	  /*Configure GPIO pin : PC2 */
	  GPIO_InitStruct.Pin = GPIO_PIN_2;
	  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*EVENT OUT*/
	  /*Configure GPIO pin : PC3 */
	  GPIO_InitStruct.Pin = GPIO_PIN_3;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configures the port and pin on which the EVENTOUT Cortex signal will be connected */
	  HAL_GPIOEx_ConfigEventout(AFIO_EVENTOUT_PORT_C, AFIO_EVENTOUT_PIN_3);

	  /*Enables the Event Output */
	  HAL_GPIOEx_EnableEventout();
}

void MX_GPIO_EXIT_Init(void)
{

	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* 配置 AFIO_EXTICRx 寄存器需要开启 AFIO 时钟 */
	  /* 其实，库函数内部会自动开启 */
	  __HAL_RCC_AFIO_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();

	  /*Configure GPIO pin : PC2 */
	  GPIO_InitStruct.Pin = GPIO_PIN_2;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}
