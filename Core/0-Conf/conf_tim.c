
/* Includes ------------------------------------------------------------------*/
#include "conf_tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;


void TIMx_CountSet(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period);
void TIM_CaputureChannel(TIM_TypeDef * TIM, uint32_t triger, uint32_t Channel);
void TIM_PWMSet(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);


void setSource(TIM_HandleTypeDef * htim, uint32_t source){
	
	TIM_ClockConfigTypeDef sClockSourceConfig;
  sClockSourceConfig.ClockSource = source;
  if (HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void setOutputCompare(TIM_HandleTypeDef * htim)
	{
	TIM_OC_InitTypeDef sConfigOC;
	
	if(htim->Instance == TIM4)
		{
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK){
			Error_Handler();
		}

		if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK){
			Error_Handler();
		}

		if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
			Error_Handler();
		}

		if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){
			Error_Handler();
		}
	}
}

/*void MX_TIM1_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}



 TIM2 init function
void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;//����ʱ��ѡ���ڲ�ʱ��
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}
 TIM3 init function
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}*/

/*
 * author lhx
 * Apr 5, 2020
 *
 * @brief : 普通周期中断模式配置
 * Prescale - timer unit. if it sets 1000, then when timer count is 1000, time passed is 1 second.
 * period - timer time
 * Window > Preferences > C/C++ > Editor > Templates.
 */

void MX_TIM_CounterInterrupt(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period)	//TIM miu_s
{
	if(TIM == TIM2){
		TIMx_CountSet(TIM2, Hz, period);
//		HAL_TIM_MspPostInit(&htim2);
//		//��������ź�
//		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//		if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//		{
//			Error_Handler();
//		}
	}

}

/*
 * author lhx
 * May 19, 2020
 *
 * @brief : 边沿捕获模式配置
 * Window > Preferences > C/C++ > Editor > Templates.
 */

void MX_TIM_CaptureInterrupt(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};

	if(TIM == TIM1){
		htim1.Instance = TIM1;
		htim1.Init.Prescaler = HAL_RCC_GetPCLK2Freq()/Hz;//72MHz APB1 1MHz
		htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim1.Init.Period = period;
		if (HAL_TIM_Base_Init(&htim1) != HAL_OK){
			Error_Handler();
		}

		//external Clock configuration
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK){
			Error_Handler();
		}

//		sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
//		sSlaveConfig.InputTrigger = TIM_TS_ITR0;
//		if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK){
//			Error_Handler();
//		}
//		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//		if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK){
//			Error_Handler();
//		}

		TIM_CaputureChannel(TIM1, TIM_INPUTCHANNELPOLARITY_RISING, TIM_CHANNEL_3);
	}
}

/*
 * author lhx
 * May 19, 2020
 *
 * @brief : PWM输出模式配置
 * Window > Preferences > C/C++ > Editor > Templates.
 */

void MX_TIM_PWMOUT(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period)
{
	TIM_PWMSet(TIM, Hz, period);
}




/*
 * author lhx
 * Apr 8, 2020
 *
 * @brief : 计数模式配置
 * Window > Preferences > C/C++ > Editor > Templates.
 */

void TIMx_CountSet(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period)	//TIM miu_s
{
	if(TIM == TIM1){
		htim1.Instance = TIM1;
		htim1.Init.Prescaler = HAL_RCC_GetPCLK1Freq()/Hz;//APB2 1MHz
		htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim1.Init.Period = period;
		htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;/*!< TIMx_ARR register is not buffered */
		if (HAL_TIM_Base_Init(&htim1) != HAL_OK){
			Error_Handler();
		}
	}
	else if(TIM == TIM2){
		htim2.Instance = TIM2;
		htim2.Init.Prescaler = HAL_RCC_GetPCLK2Freq()/Hz;//APB1 1MHz
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = period;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;/*!< TIMx_ARR register is not buffered */
		if (HAL_TIM_Base_Init(&htim2) != HAL_OK){
			Error_Handler();
		}
	}
	else if(TIM == TIM4){
		htim4.Instance = TIM4;
		htim4.Init.Prescaler = HAL_RCC_GetPCLK2Freq()/Hz;//APB1 1MHz
		htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim4.Init.Period = period;
		htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;/*!< TIMx_ARR register is not buffered */
		if (HAL_TIM_Base_Init(&htim4) != HAL_OK){
			Error_Handler();
		}
	}
}


void MX_TIM_SlaveSet(TIM_TypeDef * TIM)
{
	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	if(TIM == TIM1){
		sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
		sSlaveConfig.InputTrigger = TIM_TS_ITR3;					//input signal: master - TIM4
		if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK){
			Error_Handler();
		}
//默认		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//		if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK){
//			Error_Handler();
//		}
	}
}

void MX_TIM_MasterSet(TIM_TypeDef * TIM)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	if(TIM == TIM4){
		  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;			//触发输出
		  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;	//Enable master/slave mode  sync with slave
		  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK){
		    Error_Handler();
		  }
		  HAL_TIM_MspPostInit(&htim4);
	}
}

void TIM_PWMSet(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period)
{
	TIM_OC_InitTypeDef sConfigOC;
	TIMx_CountSet(TIM, Hz, period);	//counter set
	
	if(TIM == TIM4){
		if (HAL_TIM_PWM_Init(&htim4) != HAL_OK){
			Error_Handler();
		}

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 5;						// OCPolarity duration  高电平时间 25/200*20ms=2.5ms
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;	//有效电平,也就是小于比较值的电平，当触发比较后电平反转
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

		
		if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK){
			Error_Handler();
		}
		if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){
			Error_Handler();
		}
		HAL_TIM_MspPostInit(&htim4);
	}
	
	else if(TIM == TIM1){
		if (HAL_TIM_PWM_Init(&htim1) != HAL_OK){
			Error_Handler();
		}

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 5;						// OCPolarity duration  高电平时间 25/200*20ms=2.5ms
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;	//有效电平,也就是小于比较值的电平，当触发比较后电平反转
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK){
			Error_Handler();
		}
		HAL_TIM_MspPostInit(&htim1);
	}
	
}



/*
 * author lhx
 *
 * @brief : triger - TIM_INPUTCHANNELPOLARITY_RISING
 * Window > Preferences > C/C++ > Editor > Templates.
 */

void TIM_CaputureChannel(TIM_TypeDef * TIM, uint32_t triger, uint32_t Channel)
{
	TIM_IC_InitTypeDef sConfigIC = {0};

	if(TIM == TIM1){

		if (HAL_TIM_IC_Init(&htim1) != HAL_OK){
			Error_Handler();
		}

		sConfigIC.ICPolarity = triger;
		sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		sConfigIC.ICFilter = 0;
		if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, Channel) != HAL_OK){
			Error_Handler();
		}
	}
}



void MX_TIMx_Interrupt(TIM_TypeDef * TIM, uint32_t period)	//TIM miu_s
{
	TIM_MasterConfigTypeDef sMasterConfig;

	if(TIM == TIM2){
		htim2.Instance = TIM2;
		htim2.Init.Prescaler = HAL_RCC_GetPCLK2Freq()/1000000;//APB1 1MHz
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = period;
		if (HAL_TIM_Base_Init(&htim2) != HAL_OK){
			Error_Handler();
		}

		//Êä³öŽ¥·¢ÐÅºÅ
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
		{
			Error_Handler();
		}

	}
}


void MX_TIM_CLK(TIM_HandleTypeDef* tim_baseHandle);
void MX_NVIC(TIM_HandleTypeDef* tim_baseHandle);
/*
 * author lhx
 * Apr 8, 2020
 *
 * @brief : TIM时钟代开，中断开关配置
 * Window > Preferences > C/C++ > Editor > Templates.
 */

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
	MX_TIM_CLK(tim_baseHandle);

	if(tim_baseHandle->Instance==TIM1){
 		MX_NVIC(tim_baseHandle);
	}
	else if(tim_baseHandle->Instance==TIM2)
	{
		MX_NVIC(tim_baseHandle);
	}
	else if(tim_baseHandle->Instance==TIM3)
	{
		MX_NVIC(tim_baseHandle);
	}
	else if(tim_baseHandle->Instance==TIM4)
	{
		MX_NVIC(tim_baseHandle);
	}

}

void MX_TIM_CLK(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
	else if(tim_baseHandle->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
}

void MX_NVIC(TIM_HandleTypeDef* tim_baseHandle)
{
	if(tim_baseHandle->Instance==TIM1){
//		HAL_NVIC_SetPriority(TIM1_UP_IRQn, 1, 2);
//		HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
//	    HAL_NVIC_SetPriority(TIM1_BRK_IRQn, 0, 0);
//	    HAL_NVIC_EnableIRQ(TIM1_BRK_IRQn);
//	    HAL_NVIC_SetPriority(TIM1_TRG_COM_IRQn, 0, 0);
//	    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_IRQn);
//	    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
//	    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	}
	else if(tim_baseHandle->Instance==TIM2)
	{
		HAL_NVIC_SetPriority(TIM2_IRQn, 1, 2);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
	}
	else if(tim_baseHandle->Instance==TIM3)
	{
		HAL_NVIC_SetPriority(TIM3_IRQn, 1, 2);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
	}
	else if(tim_baseHandle->Instance==TIM4)
	{
		HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM4_IRQn);
	}
}


/*
 * author lhx
 * Apr 8, 2020
 *
 * @brief : TIM底层引脚配置
 * Window > Preferences > C/C++ > Editor > Templates.
 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance == TIM1){

	/** TIM1 GPIO Configuration
	  PA10     ------> TIM1_CH3
	  PB0     ------> TIM1_CH2N
	*/
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  if(timHandle->Instance==TIM2)
  {

    /**TIM2 GPIO Configuration    
    PA1     ------> TIM2_CH2
    PA0/WKUP     ------> TIM2_CH1
    PA2     ------> TIM2_CH3
    PA3     ------> TIM2_CH4 
    */
  }
  else if(timHandle->Instance==TIM3)
  {
    /**TIM3 GPIO Configuration    
    PB4     ------> TIM3_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    //GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  }
  else if(timHandle->Instance==TIM4)
  {
    /**TIM4 GPIO Configuration    
    PB9     ------> TIM4_CH4
    PB8     ------> TIM4_CH3
    PB7     ------> TIM4_CH2
    PB6     ------> TIM4_CH1
    */
	__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    //GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
  else if(tim_baseHandle->Instance==TIM4)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();
  }
} 

void setTimeCount(TIM_TypeDef * TIMx, uint32_t count)
{
	TIMx->CNT = count;

}

uint32_t getTimeCount(TIM_TypeDef * TIMx)
{
	return TIMx->CNT;
}

void SetTIMCCR(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t compare)
{
	if(Channel == TIM_CHANNEL_1){
		TIMx->CCR1 = compare;
	}
	if(Channel == TIM_CHANNEL_2){
		TIMx->CCR2 = compare;
	}
	if(Channel == TIM_CHANNEL_3){
		TIMx->CCR3 = compare;
	}
	if(Channel == TIM_CHANNEL_4){
		TIMx->CCR4 = compare;
	}
}

uint32_t getCaptureCount(TIM_TypeDef * TIMx, uint32_t Channel)
{
	if(Channel == TIM_CHANNEL_1){
		return TIMx->CCR1;
	}
	if(Channel == TIM_CHANNEL_2){
		return TIMx->CCR2;
	}
	if(Channel == TIM_CHANNEL_3){
		return TIMx->CCR3;
	}
	if(Channel == TIM_CHANNEL_4){
		return TIMx->CCR4;
	}
	return 0;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/









//之前代码
///* Includes ------------------------------------------------------------------*/
//#include "conf_tim.h"

///* USER CODE BEGIN 0 */

///* USER CODE END 0 */
//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef htim4;
//TIM_HandleTypeDef htim5;
//TIM_HandleTypeDef htim6;
//TIM_HandleTypeDef htim8;
//TIM_HandleTypeDef htim12;


//void TIMx_CountSet(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period);
//void TIM_CaputureChannel(TIM_TypeDef * TIM, uint32_t triger, uint32_t Channel);
//void TIM_PWMSet(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period);
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);


//void setSource(TIM_HandleTypeDef * htim, uint32_t source){
//	
//	TIM_ClockConfigTypeDef sClockSourceConfig;
//  sClockSourceConfig.ClockSource = source;
//  if (HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

//void setOutputCompare(TIM_HandleTypeDef * htim)
//	{
//	TIM_OC_InitTypeDef sConfigOC;
//	
//	if(htim->Instance == TIM4)
//		{
//		sConfigOC.OCMode = TIM_OCMODE_PWM1;
//		sConfigOC.Pulse = 0;
//		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//		if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK){
//			Error_Handler();
//		}

//		if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK){
//			Error_Handler();
//		}

//		if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK){
//			Error_Handler();
//		}

//		if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){
//			Error_Handler();
//		}
//	}
//}

///*void MX_TIM1_Init(void)
//{
//	TIM_ClockConfigTypeDef sClockSourceConfig;
//  TIM_MasterConfigTypeDef sMasterConfig;
//  TIM_OC_InitTypeDef sConfigOC;

//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 84-1;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 1;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  HAL_TIM_MspPostInit(&htim1);
//}



// TIM2 init function
//void MX_TIM2_Init(void)
//{
//  TIM_ClockConfigTypeDef sClockSourceConfig;
//  TIM_MasterConfigTypeDef sMasterConfig;
//  TIM_OC_InitTypeDef sConfigOC;

//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = 72-1;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 1;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;//����ʱ��ѡ���ڲ�ʱ��
//  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  HAL_TIM_MspPostInit(&htim2);

//}
// TIM3 init function
//void MX_TIM3_Init(void)
//{
//  TIM_ClockConfigTypeDef sClockSourceConfig;
//  TIM_MasterConfigTypeDef sMasterConfig;
//  TIM_OC_InitTypeDef sConfigOC;

//  htim3.Instance = TIM3;
//  htim3.Init.Prescaler = 84-1;
//  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim3.Init.Period = 0;
//  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  HAL_TIM_MspPostInit(&htim3);
//}*/

///*
// * author lhx
// * Apr 5, 2020
// *
// * @brief : 普通周期中断模式配置
// * Prescale - timer unit. if it sets 1000, then when timer count is 1000, time passed is 1 second.
// * period - timer time
// * Window > Preferences > C/C++ > Editor > Templates.
// */

//void MX_TIM_CounterInterrupt(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period)	//TIM miu_s
//{
//	if(TIM == TIM2){
//		TIMx_CountSet(TIM2, Hz, period);
////		HAL_TIM_MspPostInit(&htim2);
////		//��������ź�
////		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
////		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
////		if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
////		{
////			Error_Handler();
////		}
//	}

//}

///*
// * author lhx
// * May 19, 2020
// *
// * @brief : 边沿捕获模式配置
// * Window > Preferences > C/C++ > Editor > Templates.
// */

//void MX_TIM_CaptureInterrupt(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period)
//{
//	TIM_ClockConfigTypeDef sClockSourceConfig = {0};

//	if(TIM == TIM1){
//		htim1.Instance = TIM1;
//		htim1.Init.Prescaler = HAL_RCC_GetPCLK2Freq()/Hz;//72MHz APB1 1MHz
//		htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//		htim1.Init.Period = period;
//		if (HAL_TIM_Base_Init(&htim1) != HAL_OK){
//			Error_Handler();
//		}

//		//external Clock configuration
//		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//		if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK){
//			Error_Handler();
//		}

////		sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
////		sSlaveConfig.InputTrigger = TIM_TS_ITR0;
////		if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK){
////			Error_Handler();
////		}
////		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
////		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
////		if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK){
////			Error_Handler();
////		}

//		TIM_CaputureChannel(TIM1, TIM_INPUTCHANNELPOLARITY_RISING, TIM_CHANNEL_3);
//	}
//}

///*
// * author lhx
// * May 19, 2020
// *
// * @brief : PWM输出模式配置
// * Window > Preferences > C/C++ > Editor > Templates.
// */

//void MX_TIM_PWMOUT(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period)
//{
//	TIM_PWMSet(TIM, Hz, period);
//}




///*
// * author lhx
// * Apr 8, 2020
// *
// * @brief : 计数模式配置
// * Window > Preferences > C/C++ > Editor > Templates.
// */

//void TIMx_CountSet(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period)	//TIM miu_s
//{
//	if(TIM == TIM2){
//		htim2.Instance = TIM2;
//		htim2.Init.Prescaler = HAL_RCC_GetPCLK2Freq()/Hz;//APB1 1MHz
//		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//		htim2.Init.Period = period;
//		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;/*!< TIMx_ARR register is not buffered */
//		if (HAL_TIM_Base_Init(&htim2) != HAL_OK){
//			Error_Handler();
//		}
//	}
//	else if(TIM == TIM4){
//		htim4.Instance = TIM4;
//		htim4.Init.Prescaler = HAL_RCC_GetPCLK2Freq()/Hz;//APB1 1MHz
//		htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//		htim4.Init.Period = period;
//		htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;/*!< TIMx_ARR register is not buffered */
//		if (HAL_TIM_Base_Init(&htim4) != HAL_OK){
//			Error_Handler();
//		}
//	}
//}


//void MX_TIM_SlaveSet(TIM_TypeDef * TIM)
//{
//	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
//	if(TIM == TIM1){
//		sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
//		sSlaveConfig.InputTrigger = TIM_TS_ITR3;					//input signal: master - TIM4
//		if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK){
//			Error_Handler();
//		}
////默认		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
////		if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK){
////			Error_Handler();
////		}
//	}
//}

//void MX_TIM_MasterSet(TIM_TypeDef * TIM)
//{
//	TIM_MasterConfigTypeDef sMasterConfig = {0};
//	if(TIM == TIM4){
//		  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;			//触发输出
//		  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;	//Enable master/slave mode  sync with slave
//		  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK){
//		    Error_Handler();
//		  }
//		  HAL_TIM_MspPostInit(&htim4);
//	}
//}

//void TIM_PWMSet(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period)
//{
//	TIM_OC_InitTypeDef sConfigOC;
//	TIMx_CountSet(TIM, Hz, period);	//counter set
//	if(TIM == TIM4){
//		if (HAL_TIM_PWM_Init(&htim4) != HAL_OK){
//			Error_Handler();
//		}

//		sConfigOC.OCMode = TIM_OCMODE_PWM1;
//		sConfigOC.Pulse = 5;						// OCPolarity duration  高电平时间 25/200*20ms=2.5ms
//		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;	//有效电平,也就是小于比较值的电平，当触发比较后电平反转
//		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//		if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK){
//			Error_Handler();
//		}
//		if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK){
//			Error_Handler();
//		}
//		HAL_TIM_MspPostInit(&htim4);
//	}
//}



///*
// * author lhx
// *
// * @brief : triger - TIM_INPUTCHANNELPOLARITY_RISING
// * Window > Preferences > C/C++ > Editor > Templates.
// */

//void TIM_CaputureChannel(TIM_TypeDef * TIM, uint32_t triger, uint32_t Channel)
//{
//	TIM_IC_InitTypeDef sConfigIC = {0};

//	if(TIM == TIM1){

//		if (HAL_TIM_IC_Init(&htim1) != HAL_OK){
//			Error_Handler();
//		}

//		sConfigIC.ICPolarity = triger;
//		sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
//		sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
//		sConfigIC.ICFilter = 0;
//		if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, Channel) != HAL_OK){
//			Error_Handler();
//		}
//	}
//}



//void MX_TIMx_Interrupt(TIM_TypeDef * TIM, uint32_t period)	//TIM miu_s
//{
//	TIM_MasterConfigTypeDef sMasterConfig;

//	if(TIM == TIM2){
//		htim2.Instance = TIM2;
//		htim2.Init.Prescaler = HAL_RCC_GetPCLK2Freq()/1000000;//APB1 1MHz
//		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//		htim2.Init.Period = period;
//		if (HAL_TIM_Base_Init(&htim2) != HAL_OK){
//			Error_Handler();
//		}

//		//Êä³öŽ¥·¢ÐÅºÅ
//		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//		if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//		{
//			Error_Handler();
//		}

//	}
//}


//void MX_TIM_CLK(TIM_HandleTypeDef* tim_baseHandle);
//void MX_NVIC(TIM_HandleTypeDef* tim_baseHandle);
///*
// * author lhx
// * Apr 8, 2020
// *
// * @brief : TIM时钟代开，中断开关配置
// * Window > Preferences > C/C++ > Editor > Templates.
// */

//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
//{
//	MX_TIM_CLK(tim_baseHandle);

//	if(tim_baseHandle->Instance==TIM1){
// 		MX_NVIC(tim_baseHandle);
//	}
//	else if(tim_baseHandle->Instance==TIM2)
//	{
//		MX_NVIC(tim_baseHandle);
//	}
//	else if(tim_baseHandle->Instance==TIM3)
//	{
//		MX_NVIC(tim_baseHandle);
//	}
//	else if(tim_baseHandle->Instance==TIM4)
//	{
//		MX_NVIC(tim_baseHandle);
//	}

//}

//void MX_TIM_CLK(TIM_HandleTypeDef* tim_baseHandle)
//{
//  if(tim_baseHandle->Instance==TIM1)
//  {
//    __HAL_RCC_TIM1_CLK_ENABLE();
//  }
//  else if(tim_baseHandle->Instance==TIM2)
//  {
//    __HAL_RCC_TIM2_CLK_ENABLE();
//  }
//  else if(tim_baseHandle->Instance==TIM3)
//  {
//    __HAL_RCC_TIM3_CLK_ENABLE();
//  }
//	else if(tim_baseHandle->Instance==TIM4)
//  {
//    __HAL_RCC_TIM4_CLK_ENABLE();
//  }
//}

//void MX_NVIC(TIM_HandleTypeDef* tim_baseHandle)
//{
//	if(tim_baseHandle->Instance==TIM1){
//		HAL_NVIC_SetPriority(TIM1_UP_IRQn, 1, 2);
//		HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
////	    HAL_NVIC_SetPriority(TIM1_BRK_IRQn, 0, 0);
////	    HAL_NVIC_EnableIRQ(TIM1_BRK_IRQn);
////	    HAL_NVIC_SetPriority(TIM1_TRG_COM_IRQn, 0, 0);
////	    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_IRQn);
//	    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
//	    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
//	}
//	else if(tim_baseHandle->Instance==TIM2)
//	{
//		HAL_NVIC_SetPriority(TIM2_IRQn, 1, 2);
//		HAL_NVIC_EnableIRQ(TIM2_IRQn);
//	}
//	else if(tim_baseHandle->Instance==TIM3)
//	{
//		HAL_NVIC_SetPriority(TIM3_IRQn, 1, 2);
//		HAL_NVIC_EnableIRQ(TIM3_IRQn);
//	}
//	else if(tim_baseHandle->Instance==TIM4)
//	{
//		HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
//		HAL_NVIC_EnableIRQ(TIM4_IRQn);
//	}
//}


///*
// * author lhx
// * Apr 8, 2020
// *
// * @brief : TIM底层引脚配置
// * Window > Preferences > C/C++ > Editor > Templates.
// */

//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
//{
//  GPIO_InitTypeDef GPIO_InitStruct;
//  if(timHandle->Instance==TIM1){
//	__HAL_RCC_GPIOA_CLK_ENABLE();
//	/**TIM1 GPIO Configuration
//	PA10     ------> TIM1_CH3
//	*/
//	GPIO_InitStruct.Pin = GPIO_PIN_10;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//  }
//  if(timHandle->Instance==TIM2)
//  {

//    /**TIM2 GPIO Configuration    
//    PA1     ------> TIM2_CH2
//    PA0/WKUP     ------> TIM2_CH1
//    PA2     ------> TIM2_CH3
//    PA3     ------> TIM2_CH4 
//    */
//  }
//  else if(timHandle->Instance==TIM3)
//  {
//    /**TIM3 GPIO Configuration    
//    PB4     ------> TIM3_CH1 
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_4;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    //GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//  }
//  else if(timHandle->Instance==TIM4)
//  {
//    /**TIM4 GPIO Configuration    
//    PB9     ------> TIM4_CH4
//    PB8     ------> TIM4_CH3
//    PB7     ------> TIM4_CH2
//    PB6     ------> TIM4_CH1
//    */
//	__HAL_RCC_GPIOB_CLK_ENABLE();
//    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_6;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    //GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//  }
//}

//void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
//{

//  if(tim_baseHandle->Instance==TIM2)
//  {
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM2_CLK_DISABLE();
//  }
//  else if(tim_baseHandle->Instance==TIM3)
//  {
//    __HAL_RCC_TIM3_CLK_DISABLE();
//  }
//  else if(tim_baseHandle->Instance==TIM4)
//  {
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM4_CLK_DISABLE();
//  }
//} 

//void setTimeCount(TIM_TypeDef * TIMx, uint32_t count)
//{
//	TIMx->CNT = count;

//}

//uint32_t getTimeCount(TIM_TypeDef * TIMx)
//{
//	return TIMx->CNT;
//}

//void SetTIMCCR(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t compare)
//{
//	if(Channel == TIM_CHANNEL_1){
//		TIMx->CCR1 = compare;
//	}
//	if(Channel == TIM_CHANNEL_2){
//		TIMx->CCR2 = compare;
//	}
//	if(Channel == TIM_CHANNEL_3){
//		TIMx->CCR3 = compare;
//	}
//	if(Channel == TIM_CHANNEL_4){
//		TIMx->CCR4 = compare;
//	}
//}

//uint32_t getCaptureCount(TIM_TypeDef * TIMx, uint32_t Channel)
//{
//	if(Channel == TIM_CHANNEL_1){
//		return TIMx->CCR1;
//	}
//	if(Channel == TIM_CHANNEL_2){
//		return TIMx->CCR2;
//	}
//	if(Channel == TIM_CHANNEL_3){
//		return TIMx->CCR3;
//	}
//	if(Channel == TIM_CHANNEL_4){
//		return TIMx->CCR4;
//	}
//	return 0;
//}


///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
