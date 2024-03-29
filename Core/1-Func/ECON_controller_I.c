#include "ECON_controller_I.h"
#define controller_TIM TIM4
#define controller_HTIM htim4



void ECON_action(void)
{
	HAL_TIM_Base_Start_IT(&controller_HTIM);
	HAL_TIM_PWM_Start(&controller_HTIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&controller_HTIM, TIM_CHANNEL_2);
}

void ECON_I_init(void)
{
	MX_GPIO_output_Init();

	MX_TIM_PWMOUT(controller_TIM, 50000, 100);		// 1/500 s = 2 ms
}


/*
 * author lhx
 * May 24, 2020
 *
 * @brief : 设置占空比
 * Window > Preferences > C/C++ > Editor > Templates.
 */

void setPWM_1(float dutyfactor)
{
	uint32_t pwm = dutyfactor*100;
	SetTIMCCR(controller_TIM, TIM_CHANNEL_1, pwm);
}

void setPWM_2(float dutyfactor)
{
	uint32_t pwm = dutyfactor*100;
	SetTIMCCR(controller_TIM, TIM_CHANNEL_2, pwm);
}

extern uint8_t CANID_righthip_odriver, CANID_lefthip_odriver;
void set_I_direction(uint8_t node, float I)
{
	if(node == 1){
		ODrive_Set_Input_Current(CANID_lefthip_odriver, I);
	}
	else if(node == 2){
		ODrive_Set_Input_Current(CANID_righthip_odriver, -I);
	}
	TESTOUT("f2 - %.2f\r\n", I);
	
}




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
