
#include "ECON_controller_I.h"


#define ECON_2_TIM TIM4
#define ECON_2_HTIM htim4
#define ECON_2_CH TIM_CHANNEL_4

#define ECON_1_TIM TIM4
#define ECON_1_HTIM htim4
#define ECON_1_CH TIM_CHANNEL_3

/**
 * @author lhx
 * @date May 24, 2020
 *
 * @brief : 启动电机控制器(ESCON)的PWM引脚
 */
void ECON_action(void)
{
  HAL_TIM_Base_Start_IT(&ECON_2_HTIM);
  HAL_TIM_PWM_Start(&ECON_2_HTIM, ECON_2_CH);

  HAL_TIM_Base_Start_IT(&ECON_1_HTIM);
  HAL_TIM_PWM_Start(&ECON_1_HTIM, ECON_1_CH);
}


/**
 * @author lhx
 * @date May 24, 2020
 *
 * @brief : 设置 ECON 的 PWM 频率，500HZ；和对应的引脚
 */
void ECON_I_init(void)
{
  MX_GPIO_output_Init();

  MX_TIM_PWMOUT(ECON_1_TIM, 50000, 100);  // 1/500 s = 2 ms  500Hz

  MX_TIM_PWMOUT(ECON_2_TIM, 50000, 100);  // 1/500 s = 2 ms  500Hz
}


/**
 * @author lhx
 * @date May 24, 2020
 *
 * @brief : 设置左侧电机控制器PWM的占空比
 */
void setPWM_1(float dutyfactor)
{
  uint32_t pwm = dutyfactor * 100;
  SetTIMCCR(ECON_1_TIM, ECON_1_CH, pwm);
}
/**
 * @author lhx
 * @date May 24, 2020
 *
 * @brief : 设置右侧电机控制器PWM的占空比
 */
void setPWM_2(float dutyfactor)
{
  uint32_t pwm = dutyfactor * 100;
  SetTIMCCR(ECON_2_TIM, ECON_2_CH, pwm);
}

/**
 * @author lhx
 * @date May 24, 2020
 *
 * @brief : 设置 ESCON 驱动器的电流值大小。占空比 0.9 对应 3A，0.1 对应 0 A
 */
extern uint8_t CANID_righthip_odriver, CANID_lefthip_odriver;
void           set_I_direction(uint8_t node, float I)
{
  float   max_I = 3;
  float   dutyfactor;
  uint8_t isclk;
  if (I > 0) {
    isclk = 1;
  }
  else {
    I     = -I;
    isclk = 0;
  }
  if (I > max_I) {
    I = max_I;
  }

  if (node == 1) {
    dutyfactor = I / max_I * (0.9 - 0.1) + 0.1;
    if (isclk) {
      RESET1;
      CLK1
    }
    else {
      RESET1;
      UNCLK1
    }
    setPWM_1(dutyfactor);
  }
  else if (node == 2) {
    dutyfactor = I / max_I * (0.9 - 0.1) + 0.1;
    if (isclk) {
      RESET2;
      CLK2
    }
    else {
      RESET2;
      UNCLK2
    }
    setPWM_2(dutyfactor);
  }
  // TESTOUT("f2 - %.2f, %.2f \r\n\r\n",dutyfactor,I);
}


// -------------------*---------------------------*---------- TEST ---------*----------------------*-----------------//
// -------------------*---------------------------*---------- TEST ---------*----------------------*-----------------//

#ifdef ECON_TEST

TEST test_ECON_I_controller(void)
{
  ECON_I_init();
  ECON_action();
  setPWM_1(0.8);
  setPWM_2(0.3);
}

/*
uint16_t pwm = 1;
uint16_t elapsed = 1;
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
        if(htim->Instance == TIM4){
                pwm = -1*pwm;
        }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
        if(htim->Instance == TIM4){
                elapsed = -1*elapsed;
        }
}
*/

#endif
