/*
 * conf_adc.c
 *
 *  Created on: Aug 14, 2020
 *      Author: test		
 * https://www.cnblogs.com/cat-li/p/4982510.html
 * https://cpp.hotexamples.com/examples/-/-/HAL_ADC_PollForConversion/cpp-hal_adc_pollforconversion-function-examples.html
 */
#include "conf_adc.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

//#define ADC3 ADC2
//#define __HAL_RCC_ADC3_CLK_DISABLE 	__HAL_RCC_ADC2_CLK_DISABLE
//#define __HAL_RCC_ADC3_CLK_ENABLE 	__HAL_RCC_ADC2_CLK_ENABLE

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
//void MX_ADC1_Init(void)
//{
//	RCC_PeriphCLKInitTypeDef ADC_CLKInit;

//	ADC_CLKInit.PeriphClockSelection=RCC_PERIPHCLK_ADC;			//ADC clk
//	ADC_CLKInit.AdcClockSelection=RCC_ADCPCLK2_DIV6;			//72M/6=12MHz
//	HAL_RCCEx_PeriphCLKConfig(&ADC_CLKInit);

//  ADC_ChannelConfTypeDef sConfig = {0};
//  ADC_InjectionConfTypeDef sConfigInjected = {0};

//  /** Common config
//  */
//  hadc1.Instance = ADC1;
//  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
//  hadc1.Init.ContinuousConvMode = DISABLE;
//  hadc1.Init.DiscontinuousConvMode = ENABLE;
//  hadc1.Init.NbrOfDiscConversion = 1;
//  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//  hadc1.Init.NbrOfConversion = 1;
//  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//  {
//    Error_Handler()
//  }
//  
//  
//  /** Configure Regular Channel 
//  */
//  sConfig.Channel = ADC_CHANNEL_0;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler()
//  }
//  /** Configure Injected Channel 
//  */
//  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
//  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
//  sConfigInjected.InjectedNbrOfConversion = 2;
//  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_EXT_IT15;
//  sConfigInjected.AutoInjectedConv = DISABLE;
//  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
//  sConfigInjected.InjectedOffset = 0;
//  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
//  {
//    Error_Handler()
//  }
//  /** Configure Injected Channel 
//  */
//  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
//  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
//  {
//    Error_Handler()
//  }
//  
//  HAL_ADCEx_Calibration_Start(&hadc1);
//}

void MX_ADC1_Init(void)
{
	RCC_PeriphCLKInitTypeDef ADC_CLKInit;

	ADC_CLKInit.PeriphClockSelection=RCC_PERIPHCLK_ADC;			//ADC clk
	ADC_CLKInit.AdcClockSelection=RCC_ADCPCLK2_DIV6;			//72M/6=12MHz
	HAL_RCCEx_PeriphCLKConfig(&ADC_CLKInit);


  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler()
  }
  
  HAL_ADCEx_Calibration_Start(&hadc1);
  
  /** Configure Regular Channel 
  */
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC2_Init(void)
{
	RCC_PeriphCLKInitTypeDef ADC_CLKInit;

	ADC_CLKInit.PeriphClockSelection=RCC_PERIPHCLK_ADC;			//ADC clk
	ADC_CLKInit.AdcClockSelection=RCC_ADCPCLK2_DIV6;			//72M/6=12MHz
	HAL_RCCEx_PeriphCLKConfig(&ADC_CLKInit);


  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler()
  }
  
  
  HAL_ADCEx_Calibration_Start(&hadc2);
  

  /** Configure Regular Channel 
  */
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_0;				//ADC2 channel 0
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  
  sConfig.Channel = ADC_CHANNEL_15;				//ADC2 channel 15
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
 void MX_ADC3_Init(void)
{
	RCC_PeriphCLKInitTypeDef ADC_CLKInit;

	ADC_CLKInit.PeriphClockSelection=RCC_PERIPHCLK_ADC;			//ADC clk
	ADC_CLKInit.AdcClockSelection=RCC_ADCPCLK2_DIV6;			//72M/6=12MHz
	HAL_RCCEx_PeriphCLKConfig(&ADC_CLKInit);

  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 13;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_12;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  /** Configure Regular Channel 
  */
  sConfig.Rank = ADC_REGULAR_RANK_13;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler()
  }
  
  HAL_ADCEx_Calibration_Start(&hadc3);
}


void ADC_Hardware_Init(ADC_HandleTypeDef* hadc)
{
   GPIO_InitTypeDef GPIO_InitStruct = {0};

   if(hadc->Instance==ADC1)
   {
	 /* Peripheral clock enable */
	 __HAL_RCC_ADC1_CLK_ENABLE();

	 __HAL_RCC_GPIOC_CLK_ENABLE();
	 __HAL_RCC_GPIOA_CLK_ENABLE();
	 /**ADC1 GPIO Configuration
	 PC2     ------> ADC1_IN12
	 PA0-WKUP     ------> ADC1_IN0
	 PA1     ------> ADC1_IN1
	 PC5     ------> ADC1_IN15
	 */
	 GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5;
	 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	 GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   }
   else if(hadc->Instance==ADC2)
   {

	 /* Peripheral clock enable */
	 __HAL_RCC_ADC2_CLK_ENABLE();

	 __HAL_RCC_GPIOA_CLK_ENABLE();
	 __HAL_RCC_GPIOC_CLK_ENABLE();
	 /**ADC2 GPIO Configuration
	 PA0-WKUP     ------> ADC2_IN0
	 PC5     ------> ADC2_IN15
	 */
	 GPIO_InitStruct.Pin = GPIO_PIN_0;
	 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	 GPIO_InitStruct.Pin = GPIO_PIN_5;
	 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   }
   else if(hadc->Instance==ADC3)
   {
	 /* Peripheral clock enable */
	 __HAL_RCC_ADC3_CLK_ENABLE();

	 __HAL_RCC_GPIOC_CLK_ENABLE();
	 __HAL_RCC_GPIOA_CLK_ENABLE();
	 /**ADC3 GPIO Configuration
	 PC3     ------> ADC3_IN13
	 PA0-WKUP     ------> ADC3_IN0
	 */
	 GPIO_InitStruct.Pin = GPIO_PIN_3;
	 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	 GPIO_InitStruct.Pin = GPIO_PIN_0;
	 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   }
}

void ADC_Hardware_DeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
	/* Peripheral clock disable */
	__HAL_RCC_ADC1_CLK_DISABLE();

	/**ADC1 GPIO Configuration
	PC2     ------> ADC1_IN12
	PA0-WKUP     ------> ADC1_IN0
	PA1     ------> ADC1_IN1
	PC5     ------> ADC1_IN15
	*/
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2|GPIO_PIN_5);

	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);
  }
  else if(hadc->Instance==ADC2)
  {
	/* Peripheral clock disable */
	__HAL_RCC_ADC2_CLK_DISABLE();

	/**ADC2 GPIO Configuration
	PA0-WKUP     ------> ADC2_IN0
	PC5     ------> ADC2_IN15
	*/
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5);
  }
  else if(hadc->Instance==ADC3)
  {
	/* Peripheral clock disable */
	__HAL_RCC_ADC3_CLK_DISABLE();

	/**ADC3 GPIO Configuration
	PC3     ------> ADC3_IN13
	PA0-WKUP     ------> ADC3_IN0
	*/
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);

	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

  }

}



/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	ADC_Hardware_Init(hadc);
}




/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
	ADC_Hardware_DeInit(hadc);
}


