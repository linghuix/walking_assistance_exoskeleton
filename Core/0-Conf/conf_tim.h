
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __conf_tim_H
#define __conf_tim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "device_bsp.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;

/* 初始化操作 */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM5_Init(void);
void MX_TIM6_Init(void);
void MX_TIM8_Init(void);
void MX_TIM12_Init(void);

void MX_TIMx_Interrupt(TIM_TypeDef * TIM, uint32_t period);
void MX_TIM_CounterInterrupt(TIM_TypeDef * TIM, uint32_t Prescale, uint32_t period);
void MX_TIM_CaptureInterrupt(TIM_TypeDef * TIM, uint32_t unit, uint32_t period);
void TIM_CaputureChannel(TIM_TypeDef * TIM, uint32_t triger, uint32_t Channel);
//void MX_TIM4_PWMOUT_Init(void);
void MX_TIM_PWMOUT(TIM_TypeDef * TIM, uint32_t Hz, uint32_t period);


/* 硬件底层操作操作 */
void setTimeCount(TIM_TypeDef * TIMx, uint32_t count);
uint32_t getTimeCount(TIM_TypeDef * TIMx);
uint32_t getCaptureCount(TIM_TypeDef * TIMx, uint32_t Channel);

void SetTIMCCR(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t compare);

#define TimeCounter_Start(phtmx)  HAL_TIM_Base_Start(phtmx)
#define TimeCounter_Stop(phtmx) HAL_TIM_Base_Stop(phtmx)

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
