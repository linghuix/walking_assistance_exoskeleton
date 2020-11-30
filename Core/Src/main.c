
#include "main.h"

int CONTROL_PERIOD = 50;			//ms

#define IMUMonitor(...) printf(__VA_ARGS__)
#define AOMonitor(...) printf(__VA_ARGS__)
#define POMonitor(...) printf(__VA_ARGS__)
#define AssisMonitor(...) printf(__VA_ARGS__)


WIN d1minwin_w, d2minwin_w;				//极小值检测窗口
ElementType d1Win[3] = {0};
ElementType d2Win[3] = {0};

uint32_t inc=0;							//控制周期计时器

#define Buffsize 2
WIN acc1win_w, acc2win_w, acc1win_d, acc2win_d;			//滤波窗口
ElementType acc1WinArray_d[Buffsize] = {0};
ElementType acc2WinArray_d[Buffsize] = {0};
ElementType acc1WinArray_w[Buffsize] = {0};
ElementType acc2WinArray_w[Buffsize] = {0};
float weights[Buffsize] = {0.04,0.96};


uint32_t peaktimestamp[2] = {0};
uint16_t period = 30;
float dt = 0.05;										//控制周期和采样周期
float phase[2];	//0左，1右

int main(void)
{
	Core_Config();
	Jlink_Init();
	

	/*初始化*/
	Acc1_Init();
	Acc2_Init();
	WinBuffer(&acc1win_d, acc1WinArray_d, Buffsize);
	WinBuffer(&acc2win_d, acc2WinArray_d, Buffsize);
	WinBuffer(&acc1win_w, acc1WinArray_w, Buffsize);
	WinBuffer(&acc2win_w, acc2WinArray_w, Buffsize);
	
	/*峰值检测*/
	WinBuffer(&d1minwin_w, d1Win, 3);
	WinBuffer(&d2minwin_w, d2Win, 3);
	
	HC05_Init();
	AO_Init(21*dt);
	ECON_I_init();


	/*启动外设*/
	Acc1_Start();
	Acc2_Start();
	ECON_action();
	
	
	/*代码启动*/
	float hip1_w, hip1_d, I1;
	float hip2_w, hip2_d, I2;
	float hip1_raww, hip1_rawd;
	float hip2_raww, hip2_rawd;
	
	float AOoffset[2] = {0};					//AO 相位补偿器

	printf("ABOUT ANGLE AND SPEED couterclock is postive from outside. 从外部看向电机侧");
	printf("the acc1 of left hip - d w | the acc2 of right hip - d w | I1 ,I2\r\n");
	int8_t assive_mode;
	
	
	//test_HC05_communication();
//	test_AO();
	
	while (1){

		/*左髋关节*/
		if(flag_1 ==1&&flag_2 == 1&&flag_3 == 1){
			flag_1=0;flag_2=0;flag_3=0;
			hip1_rawd = angle1[1]/32768.0*180;	hip1_raww = w1[2]/32768.0*2000;
			addToBuff(&acc1win_d ,hip1_rawd);
			addToBuff(&acc1win_w ,hip1_raww);
			ChangeLastestValue(&acc1win_d, AvergeWin(&acc1win_d, weights, Buffsize));
			ChangeLastestValue(&acc1win_w, AvergeWin(&acc1win_w, weights, Buffsize));
			hip1_d = getLastestValue(acc1win_d);
			hip1_w = getLastestValue(acc1win_w);
			/*I1 = PO(hip1_d,hip1_d, 1);
			set_I_direction(1,I1);*/
		}

		/*右髋关节*/
		if(flag_11 ==1&&flag_22 == 1&&flag_33 == 1){
			flag_11=0;flag_22=0;flag_33=0;
			hip2_rawd = angle2[1]/32768.0*180;	hip2_raww = w2[1]/32768.0*2000;
			addToBuff(&acc2win_d ,hip2_rawd);
			addToBuff(&acc2win_w ,hip2_raww);
			ChangeLastestValue(&acc2win_d, AvergeWin(&acc2win_d, weights, Buffsize));
			ChangeLastestValue(&acc2win_w, AvergeWin(&acc2win_w, weights, Buffsize));
			hip2_d = getLastestValue(acc2win_d);
			hip2_w = getLastestValue(acc2win_w);
			/*I2= PO(hip2_d,hip2_w, 2);
			set_I_direction(2,I2);*/
		}
		
		/*无线传输显示实时数据*/
		if(inc % CONTROL_PERIOD == 0){
			IMUMonitor("acc1rawd\t%.2f\tw\t%.2f\t",hip1_rawd,hip1_raww);
			IMUMonitor("acc2rawd\t%.2f\tw\t%.2f\t",hip2_rawd,hip2_raww);
			IMUMonitor("acc1d\t%.2f\tw\t%.2f\t",hip1_d,hip1_w);
			IMUMonitor("acc2d\t%.2f\tw\t%.2f\t",hip2_d,hip2_w);
			printf("%lld\t",Aoindex);
//			printf("%.2f\t%.2f\t",hip1_d,hip1_w);
//			printf("%.2f\t%.2f\t",hip2_d,hip2_w);
			
			addToBuff(&d2minwin_w ,hip2_d);
			addToBuff(&d1minwin_w ,hip1_d);
			
			AO(hip1_d,1);
			AO(hip2_d,2);
			Aoindex++;

			
			/* 左 */
			assive_mode = switch_task( &hip1, hip1_d, hip1_w,1);
			if(assive_mode == POMODE){
				phase[0] = PO_phase(hip1_d, hip1_w);		//相位
			}
			else if(assive_mode == AOMODE){
				if(findpeak(d1minwin_w)){
					period = inc - peaktimestamp[0];		// gait period get
					peaktimestamp[0] = inc;
					AOoffset[0] = hip1.phase[1];
				}
				phase[0] = hip1.phase[1] - AOoffset[0];
			}
			else{
				while(1){ printf("assive_mode error\r\n"); }
			}
			I1 = 10*sin(phase[0]);
			set_I_direction(1,I1);
			AssisMonitor("I1 %.2f\t",I1);
			
			
			
			/* 右 */
			assive_mode = switch_task( &hip2, hip2_d, hip2_w,2);
			if(assive_mode == POMODE){
				phase[1] = PO_phase(hip2_d, hip2_w);
			}
			else if(assive_mode == AOMODE){
				if(findpeak(d2minwin_w)){
					period = inc - peaktimestamp[1];		// gait period get
					peaktimestamp[1] = inc;
					AOoffset[1] = hip2.phase[1];
				}
				phase[1] = hip2.phase[1] - AOoffset[1];
			}
			else{
				while(1){ printf("assive_mode error\r\n"); }
			}
			I2 = 10*sin(phase[1]);
			set_I_direction(1,I2);
			AssisMonitor("I2 %.2f\t",I2);
			
			printf("\r\n");
		}
  }

}
