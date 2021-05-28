#include "ECON_controller_I.h"

#define controller_TIM TIM4
#define controller_HTIM htim4



void ECON_action(void)
{
////	HAL_TIM_Base_Start_IT(&controller_HTIM);
////	HAL_TIM_PWM_Start(&controller_HTIM, TIM_CHANNEL_1);
////	HAL_TIM_PWM_Start(&controller_HTIM, TIM_CHANNEL_2);
}


extern uint8_t ID_lefthip_odriver;
extern uint8_t ID_righthip_odriver;
void ECON_I_init(void)
{
	MSG_BSTART("odrive","init");

	
	MX_TIM_PWMOUT(controller_TIM, 50000, 100);		// 1/500 s = 2 ms
	Odrive_Init(ID_righthip_odriver);
	Odrive_Init(ID_lefthip_odriver);
	
//	HAL_Delay(10);
	Current_conf(ID_righthip_odriver);
	
//	HAL_Delay(10);
	Current_conf(ID_lefthip_odriver);
	MSG_ASTART("odrive","init");
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


void set_I_direction(uint8_t node, float I)
{
	float max_I = 2;
	float dutyfactor;
	uint8_t isclk;
	if(I > 0){
		isclk = 1;
	}
	else{
		I = -I;
		isclk = 0;
	}
	if(I>max_I){
		I = max_I;
	}
	
	if(node == 1){
//		ODrive_Set_Input_Current(ID_righthip_odriver, I);
		ODrive_Set_Input_Current(ID_lefthip_odriver, I);	
		AssistF("node=1 - %f A", I);
////		dutyfactor = I/max_I*(0.9-0.1)+0.1;
////		if(isclk){
////			RESET1; 
////			CLK1
////		} 
////		else {
////			RESET1; 
////			UNCLK1
////		}
////		setPWM_1(dutyfactor);
	}
	else if(node == 2){
//		ODrive_Set_Input_Current(ID_righthip_odriver, I);
//		ODrive_Set_Input_Current(ID_lefthip_odriver, I);		
		AssistF("node=2 - %f A", I);
////		dutyfactor = I/max_I*(0.9-0.1)+0.1;
////		if(isclk){
////			RESET2; 
////			CLK2
////		}
////		else {
////			RESET2; 
////			UNCLK2
////		}
////		setPWM_2(dutyfactor);
	}
	//TESTOUT("f2 - %.2f, %.2f \r\n\r\n",dutyfactor,I);
	
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
