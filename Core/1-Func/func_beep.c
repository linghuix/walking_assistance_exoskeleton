/*
	����TIMӲ����PWM���
*/
#include <func_beep.h>


const uint16_t tone_tab[] = 
{
  3822,  3405, 3033, 2863, 2551, 2272, 2024,	//bass 1~7
  1911,  1702, 1526, 1431, 1275, 1136, 1012,	//mid 1~7
  955,   851,  758,  715,   637, 568,   506,	//treble 1~7
};

typedef enum{	//[22]

  Do1L = 0, ///*261.63Hz*/    3822us
  Re2L,     ///*293.66Hz*/    3405us
  Mi3L,     ///*329.63Hz*/    3034us
  Fa4L,     ///*349.23Hz*/    2863us
  So5L,     ///*392.00Hz*/    2551us
  La6L,     ///*440.00Hz*/    2272us
  Si7L,     ///*493.88Hz*/    2052us

  Do1M,     ///*523.25Hz*/    1911us
  Re2M,     ///*587.33Hz*/    1703us
  Mi3M,     ///*659.26Hz*/    1517us
  Fa4M,     ///*698.46Hz*/    1432us
  So5M,     ///*784.00Hz*/    1276us
  La6M,     ///*880.00Hz*/    1136us
  Si7M,     ///*987.77Hz*/    1012us

  Do1H,     ///*1046.50Hz*/   956us
  Re2H,     ///*1174.66Hz*/   851us
  Mi3H,     ///*1318.51Hz*/   758us
  Fa4H,     ///*1396.91Hz*/   716us
  So5H,     ///*1567.98Hz*/   638us
  La6H,     ///*1760.00Hz*/   568us
  Si7H,     ///*1975.53Hz*/   506us

  Silent,
}Sound_tone_e;

#define BEEP_ARR    (TIM4->ARR)
#define BEEP_CH     (TIM4->CCR1)

void warning_Block(uint8_t SingTimes, uint16_t period, Sound_tone_e tone, int8_t time);
void beep_init(void)
	{
	MX_TIM_PWMOUT(TIM4, 10000, 20);
}

void beep_start(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void Sing(Sound_tone_e tone)
{
  if(tone == Silent)
    BEEP_CH = 0;
  else 
  {
    BEEP_ARR = tone_tab[tone];
    BEEP_CH = tone_tab[tone] / 2;
  }
}

#define warning_music_length  50
uint32_t warning_music_index = 0;
Sound_tone_e warning_music[warning_music_length] =
{
	Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,
	Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,
	Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,
	Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,
	Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent,
};

/*
 * author lhx
 * Apr 6, 2020
 *
 * @brief :
 *  每个周期鸣叫SingTimes次，周期长度为period个time_step，不断重复.
 *  每次鸣叫长度由time_step确定，可由定时器频率确定
 *  建议将该函数放入定时器中断服务函数中，或者一个循环体中
 * Window > Preferences > C/C++ > Editor > Templates.
 */
uint32_t time_step_warning_Loop;
void warning_Loop(uint8_t SingTimes, uint16_t period, Sound_tone_e tone)
{
	int tone_duration_step = 80;
	int index;

	time_step_warning_Loop++;
	
	/*　设置　warning_music[]音乐序列 */
	for(index = 0;index < SingTimes;index++){
		warning_music[index*2] = tone;
	}
	
	for(;index < warning_music_length;index++){
		if(warning_music[index*2] != Silent)
			warning_music[index*2] = Silent;
		else
			break;
	}
	
	if(time_step_warning_Loop % tone_duration_step == 0){
		time_step_warning_Loop=0;
		if(warning_music_index < period){
			Sing(warning_music[warning_music_index]);
		  warning_music_index++;
		}
		else{
			warning_music_index=0;
		}
	}
}

/*
 * author lhx
 * Apr 6, 2020
 *
 * @brief : 阻塞性鸣叫
 * time - 重复周期次数
 * period - 周期长度
 * SingTimes　-　每个周期内的鸣叫次数
 * Window > Preferences > C/C++ > Editor > Templates.
 */

void warning_Block(uint8_t SingTimes, uint16_t period, Sound_tone_e tone, int8_t time)
{
	int index;

	/* ��ʼ�� warning_music[] */
	for(index = 0;index < SingTimes;index++){
		warning_music[index*2] = tone;
	}
	
	for(;index < warning_music_length;index++){
		if(warning_music[index*2] != Silent)
			warning_music[index*2] = Silent;
		else
			break;
	}
	
	for(index = 0; index < time; index++){
		for(warning_music_index = 0;warning_music_index < period;warning_music_index++){
			Sing(warning_music[warning_music_index]);
			HAL_Delay(100);//80ms
		}
	}
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ test
#include "main.h"
void test_beep(void)
{
	Core_Config();
	tick_init(1000);
	beep_init();
	beep_start();
	for(int i=0;i<22;i++){
		Sing((Sound_tone_e)i);
		HAL_Delay(1000);
	}
}

void test_warning_Loop(void)
{
	Core_Config();
	tick_init(1000);
	beep_init();
	beep_start();
	uint32_t timestep = 0;
	int period = 8000;
	
	while(1){
		if(period == 0){
			period = 8000;
			if(timestep < 60*8000*80)
				warning_Loop(10,30,Si7L);
			else{
				warning_Loop(3,30,Do1H);
			}
		}
		else if(period<0){
			period = 0;
		}
		else{
			period--;
		}
		
		timestep++;
	}
}

void test_warning_Block(void)
{
	Core_Config();
	tick_init(1000);
	beep_init();
	beep_start();
	warning_Block(8, 30, Si7L, 2);
	warning_Block(2, 30, Do1H, 3);
}

/* ����ж��Ƿ����� */
void test_time_interrupt(void)
{
	Core_Config();
	tick_init(1);
	MX_USART1_UART_Init();
	MSG("USART1_UART_Init\r\n");
	
	MX_TIMx_Interrupt(TIM2, 1000);//1ms������
	HAL_TIM_Base_Start_IT(&htim2);
	
	beep_init();
	beep_start();
}

uint32_t time_1ms;
uint32_t tick_Start,last_Start, time, time_loop;
void test_time_interrupt_IRQ(void)
{
	//MSG("in TIM2 Interrupt\r\n");
	last_Start = tick_Start;
	tick_Start = HAL_GetTick();
	
	warning_Loop(3,30,Do1H); 
	time_1ms++;
	if(time_1ms>1000){
		time_1ms=0;
		MSG("i\r\n");
	}
	
	time = HAL_GetTick()-tick_Start;
	time_loop = tick_Start-last_Start;
}
