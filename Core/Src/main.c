
#include "main.h"


int main(void)
{
	Core_Config();
	Jlink_Init();
	
	//test_HC05_communication();

	/*初始化*/
	Acc1_Init();
	Acc2_Init();
	HC05_Init();
	AO_Init();
	ECON_I_init();


//	/*启动外设*/
	Acc1_Start();
	Acc2_Start();
	ECON_action();
//	
//	/*代码启动*/
	float hip1_w, hip1_d, I1;
	float hip2_w, hip2_d, I2;

	//test_AO();

	int8_t assive_mode;
	while (1){

		/*左髋关节*/
		if(flag_1 ==1&&flag_2 == 1&&flag_3 == 1){
			flag_1=0;flag_2=0;flag_3=0;
			hip1_d = angle1[1]/32768.0*180;	hip1_w = w1[2]/32768.0*2000;
			hip1_d = angle1[1]/32768.0*180;	
			hip1_d = angle1[1]/32768.0*180;	
			/*I1 = PO(hip1_d,hip1_w, 1);
			set_I_direction(1,I1);*/
		}

		/*右髋关节*/
		if(flag_11 ==1&&flag_22 == 1&&flag_33 == 1){
			flag_11=0;flag_22=0;flag_33=0;
			hip2_d = angle2[1]/32768.0*180;	hip2_w = w2[1]/32768.0*2000;
			/*I2= PO(hip2_d,hip2_w, 2);
			set_I_direction(2,I2);*/
		}
		
		/*无线传输显示实时数据*/
		if(inc % 30 == 0){
//			printf("acc1 d %.2f\tw %.2f\t",hip1_d,hip1_w);
//			printf("acc2 d %.2f\tw %.2f\t",hip2_d,hip2_w);
			//printf("%d\t",inc);
			printf("%.2f\t%.2f\t",hip1_d,hip1_w);
			printf("%.2f\t%.2f\t",hip2_d,hip2_w);
//			AO(hip1_d,1);
//			AO(hip2_d,2);
//			index ++;

//			assive_mode = switch_task( &hip1, hip1_d, hip1_w,1);
//			if(assive_mode == -20){
				I1 = PO(hip1_d,hip1_w, 1);
//			}
//			else{
//				I1 = assive_torque(&hip1, hip1_d);
//			}
			set_I_direction(1,I1);
//			printf("I1 %.2f\t",I1);
			printf("%.2f\t",I1);
			
			/**/
//			assive_mode = switch_task( &hip2, hip2_d, hip2_w,2);
//			if(assive_mode == -20){
				I2 = PO(hip2_d,hip2_w, 2);
//			}
//			else{
//				I2 = assive_torque(&hip2, hip2_d);
//			}
			set_I_direction(2,I2);
			
//			printf("I2 %.2f\r\n",I2);
			printf("%.2f\r\n",I2);
		}
  }

}
