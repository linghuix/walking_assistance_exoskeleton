/*
 * func_can.c
 *
 *  Created on: Mar 14, 2020
 *      Author: test
 */
#include "func_adc.h"


#define ADC_TEST


uint16_t GetSingleDiscontinueADCData(ADC_HandleTypeDef* phadc)
{
  HAL_ADC_Start(phadc);  // ADC

  HAL_ADC_PollForConversion(phadc, 20);

  return (uint16_t)HAL_ADC_GetValue(phadc);
}


#ifdef ADC_TEST

#include "BSP.h"

/**
 * @brief  Sngle DAC channel sample. discontinue and software trigger.
 */

TEST getSingleChannelData(void)
{
  //	MX_ADC2_Init();
  //	while(1){
  //	uint16_t data = GetSingleDiscontinueADCData(&hadc2);
  //	MSG("%.5f mV\r\n", (float)data/4096.0*3300.0);
  //		HAL_Delay(5);
  //	}

  MX_ADC1_Init();
  while (1) {
    uint16_t data = GetSingleDiscontinueADCData(&hadc1);
    MSG("%.5f mV\r\n", (float)data / 4096.0 * 3300.0);
    HAL_Delay(5);
  }
}


/**
 * ready for test.
 */
TEST getData(void)
{
  MX_ADC2_Init();

  while (1) {
    HAL_ADC_Start(&hadc2);  // ADC

    HAL_ADC_PollForConversion(&hadc2, 5);
    uint16_t data1 = (uint16_t)HAL_ADC_GetValue(&hadc2);

    HAL_ADC_PollForConversion(&hadc2, 5);
    uint16_t data2 = (uint16_t)HAL_ADC_GetValue(&hadc2);
    MSG("%.5f %.5f mV\r\n", (float)data1 / 4096.0 * 3300.0,
        (float)data2 / 4096.0 * 3300.0);

    HAL_Delay(5);
  }
}

#endif
