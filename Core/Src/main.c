
#include "main.h"

int CONTROL_PERIOD = 30;			//ms


int main(void)
{
	Core_Config();
	Jlink_Init();
	

	/*初始化*/
	Acc1_Init();
	Acc2_Init();
	HC05_Init();
	AO_Init();
	ECON_I_init();


	/*启动外设*/
	Acc1_Start();
	Acc2_Start();
	ECON_action();
	
	
	/*代码启动*/
	float hip1_w, hip1_d, I1;
	float hip2_w, hip2_d, I2;

	printf("ABOUT ANGLE AND SPEED couterclock is postive from outside. 从外部看向电机侧");
	printf("the acc1 of left hip - d w | the acc2 of right hip - d w | I1 ,I2\r\n");
	int8_t assive_mode;
	
	
	//test_HC05_communication();
	test_AO();
	
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
		if(inc % CONTROL_PERIOD == 0){
			printf("acc1d\t%.2f\tw\t%.2f\t",hip1_d,hip1_w);
			printf("acc2d\t%.2f\tw\t%.2f\t",hip2_d,hip2_w);
			printf("%lld\t",Aoindex);
//			printf("%.2f\t%.2f\t",hip1_d,hip1_w);
//			printf("%.2f\t%.2f\t",hip2_d,hip2_w);
			AO(hip1_d,1);
			AO(hip2_d,2);
			Aoindex++;

			assive_mode = switch_task( &hip1, hip1_d, hip1_w,1);
			if(assive_mode == POMODE){
				I1 = PO(hip1_d,hip1_w, 1);
			}
			else if(assive_mode == AOMODE){
				I1 = assive_torque(&hip1, hip1_d);
			}
			else{
				while(1){ printf("assive_mode error\r\n"); }
			}
			
			set_I_direction(1,I1);
			printf("I1 %.2f\t",I1);

			I1 = PO(hip1_d,hip1_w, 1);
			set_I_direction(1,I1);

			I2 = PO(hip2_d,hip2_w, 2);
			set_I_direction(2,I2);
			
			printf("I1\t%.2f\tI2\t%.2f\r\n",I1, I2);
//			printf("%.2f\t%.2f\r\n",I1, I2);
		}
  }

}
