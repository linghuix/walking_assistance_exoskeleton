/*
 * conf_rtc.c
 *
 *  Created on: Apr 11, 2020
 *      Author: test
 */

#include "conf_rtc.h"

RTC_HandleTypeDef hrtc;

void RTC_conf(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV128;	//精度可达 15s/天 以内
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
		Error_Handler();
	}

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef DateToUpdate = {0};
	/** Initialize RTC Only
	*/
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
	if (HAL_RTC_Init(&hrtc) != HAL_OK){
		Error_Handler();
	}

	if(HAL_RTCEx_BKUPRead(&hrtc, 1) != 0x6F6A){	//判断是否初始化
		/** Initialize RTC and set the Time and Date
		*/
		sTime.Hours = 0x6;
		sTime.Minutes = 0x56;
		sTime.Seconds = 0x50;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK){
			Error_Handler();
		}

		DateToUpdate.WeekDay = RTC_WEEKDAY_SATURDAY;
		DateToUpdate.Month = RTC_MONTH_APRIL;
		DateToUpdate.Date = 11;
		DateToUpdate.Year = 20;
		if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK){
			Error_Handler();
		}

		HAL_RTCEx_BKUPWrite(&hrtc, 1, 0x6F6A);
		HAL_RTCEx_BKUPWrite(&hrtc, 2, DateToUpdate.Month);
		HAL_RTCEx_BKUPWrite(&hrtc, 3, DateToUpdate.Date);
		HAL_RTCEx_BKUPWrite(&hrtc, 4, DateToUpdate.Year);
	}
	else{	//恢复年月日
		DateToUpdate.Month = HAL_RTCEx_BKUPRead(&hrtc, 2);
		DateToUpdate.Date = HAL_RTCEx_BKUPRead(&hrtc, 3);
		DateToUpdate.Year = HAL_RTCEx_BKUPRead(&hrtc, 4);
		if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK){
			Error_Handler();
		}
	}
}



/**
* @brief RTC MSP Initialization
* This function configures the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC){
    HAL_PWR_EnableBkUpAccess();
    /* Enable BKP CLK enable for backup registers */
    __HAL_RCC_BKP_CLK_ENABLE();
    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);
    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  }

}

/**
* @brief RTC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
    HAL_NVIC_DisableIRQ(RTC_IRQn);
  }

}
