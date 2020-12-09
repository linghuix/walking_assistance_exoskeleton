#include "main.h"


uint32_t inc=0;							//2ms 周期计时器
int CONTROL_PERIOD = 50/2;				//ms 实际控制周期 CONTROL_PERIOD*2 ms


//极小值检测窗口
WIN d1minwin_w, d2minwin_w;				
ElementType d1Win[3] = {0};
ElementType d2Win[3] = {0};


// 原始数据滤波窗口
#define Buffsize 2
WIN acc1win_w, acc2win_w, acc1win_d, acc2win_d;			
ElementType acc1WinArray_d[Buffsize] = {0};
ElementType acc2WinArray_d[Buffsize] = {0};
ElementType acc1WinArray_w[Buffsize] = {0};
ElementType acc2WinArray_w[Buffsize] = {0};
float weights[Buffsize] = {0.08,0.92};


uint32_t peaktimestamp[2] = {0};				// 峰值对应时间记录
uint16_t period = 20;							// 人体步态周期估计
float dt = 0.050;								// 控制周期
float phase[2];									// 0左，1右
int8_t assive_mode[2] = {0};					// 当前助力模式

/*代码启动*/
float hip1_w, hip1_d, I1;
float hip2_w, hip2_d, I2;
float hip1_raww, hip1_rawd;
float hip2_raww, hip2_rawd;

// Jscope 调试
int debug_hip1_d, debug_hip1_rawd, debug_hip2_d, debug_hip2_rawd;
int debug_hip1_w, debug_hip1_raww, debug_hip2_w, debug_hip2_raww;

uint8_t debug_peak[2] = {0}, found_peak[2] = {0};
int debug_AOphase1=0, debug_AOoutput1=0, debug_AOpre1=0, debug_AOphase_offset1=0, debug_AOw1, debug_AOphasePre1;
int debug_AOphase2=0, debug_AOoutput2=0, debug_AOpre2=0, debug_AOphase_offset2=0, debug_AOw2, debug_AOphasePre2;
int debug_AOIndex = 0;
int temp = 0;//临时查看变量
float AOoffset[2] = {0};					//AO 相位补偿器

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
	AO_Init(20.0*dt);
	ECON_I_init();


	/*启动外设*/
	Acc1_Start();
	Acc2_Start();
	ECON_action();
	
	

	printf("ABOUT ANGLE AND SPEED couterclock is postive from outside. 从外部看向电机侧");
	printf("the acc1 of left hip - d w | the acc2 of right hip - d w | I1 ,I2\r\n");
	
	//	test_win_buff();
	//	test_HC05_communication();
//	test_AOs();
	
	while (1){

		/* 左髋关节 加速度信号采集  采样周期约100Hz以上 */
		if(flag_1 ==1&&flag_2 == 1&&flag_3 == 1){
			flag_1=0;flag_2=0;flag_3=0;
			hip1_rawd = angle1[1]/32768.0*180;	hip1_raww = w1[2]/32768.0*2000;
			addToBuff(&acc1win_d ,hip1_rawd);
			addToBuff(&acc1win_w ,hip1_raww);
			ChangeLastestValue(&acc1win_d, AvergeWin(&acc1win_d, weights, Buffsize));
			ChangeLastestValue(&acc1win_w, AvergeWin(&acc1win_w, weights, Buffsize));
			hip1_d = getLastestValue(acc1win_d);
			hip1_w = getLastestValue(acc1win_w);
			
			addToBuff(&d1minwin_w ,hip1_d);
			debug_peak[0] = findpeak(&d1minwin_w);		// 髋关节伸展角度极值检测
			if(debug_peak[0] == 1){
				found_peak[0] = 1;
			}
			/*I1 = PO(hip1_d,hip1_d, 1);
			set_I_direction(1,I1);*/
		}

		/* 右髋关节 加速度信号采集  采样周期约100Hz以上 */
		if(flag_11 ==1&&flag_22 == 1&&flag_33 == 1){
			flag_11=0;flag_22=0;flag_33=0;
			hip2_rawd = angle2[1]/32768.0*180;	hip2_raww = w2[1]/32768.0*2000;
			addToBuff(&acc2win_d ,hip2_rawd);
			addToBuff(&acc2win_w ,hip2_raww);
			ChangeLastestValue(&acc2win_d, AvergeWin(&acc2win_d, weights, Buffsize));
			ChangeLastestValue(&acc2win_w, AvergeWin(&acc2win_w, weights, Buffsize));
			hip2_d = getLastestValue(acc2win_d);
			hip2_w = getLastestValue(acc2win_w);

			addToBuff(&d2minwin_w ,hip2_d);
			debug_peak[1] = findpeak(&d2minwin_w);		// 髋关节伸展角度极值检测
			if(debug_peak[1] == 1){
				found_peak[1] = 1;
			}
			/*I2= PO(hip2_d,hip2_w, 2);
			set_I_direction(2,I2);*/
		}
		
		/* 控制周期 2ms x CONTROL_PERIOD  */
		if(inc % CONTROL_PERIOD == 0){
			
			IMUMonitor("acc1rawd\t%.2f\tw\t%.2f\t",hip1_rawd,hip1_raww);
			IMUMonitor("acc2rawd\t%.2f\tw\t%.2f\t",hip2_rawd,hip2_raww);
			IMUMonitor("%.2f\t%.2f\t",hip1_d,hip1_w);
			IMUMonitor("%.2f\t%.2f\t",hip2_d,hip2_w);
			
			Aoindex++;
			debug_AOIndex++; if(debug_AOIndex > 100){ debug_AOIndex = 0;}
			
			// 左 
			AO(hip1_d,1);
			if(found_peak[0] == 1){
				period = Aoindex - peaktimestamp[0];		// gait period get
				peaktimestamp[0] = Aoindex;
				AOoffset[0] = hip1.phase[1];
				found_peak[0] = 0;
			}
					
			assive_mode[0] = switch_task( &hip1, hip1_d, hip1_w, 1);
			if(assive_mode[0] == POMODE){
				phase[0] = PO_phase(hip1_d, hip1_w);		//相位
				phase[0] = -phase[0] + 3.1415;
			}
			else if(assive_mode[0] == AOMODE){
//				if(found_peak[0]==1){
//					period = Aoindex - peaktimestamp[0];		// gait period get
//					peaktimestamp[0] = Aoindex;
//					AOoffset[0] = hip1.phase[1];
//					found_peak[0] = 0;
//				}
				phase[0] = hip1.phase[1] - AOoffset[0];
				
				while(phase[0] > 0){
					phase[0] = phase[0]-6.2830;
				}
				while(phase[0] < 0){
					phase[0] = phase[0]+6.2830;
				}
			}
			else{
				while(1){ MSG_ERR(123, "assive_mode error\r\n", 123); }
			}
			I1 = 0.1*sin(phase[0]);
			set_I_direction(1,I1);
			AssisMonitor("I1 %.2f\t",I1);
			
			
			
			// 右 
			AO(hip2_d,2);
			if(found_peak[1] == 1){
				period = Aoindex - peaktimestamp[1];		// gait period get
				peaktimestamp[1] = Aoindex;
				AOoffset[1] = hip2.phase[1];
				found_peak[1] = 0;
			}
			
			assive_mode[1] = switch_task( &hip2, hip2_d, hip2_w,2);
			if(assive_mode[1] == POMODE){
				phase[1] = PO_phase(hip2_d, hip2_w);		//相位
				phase[1] = -phase[1] + 3.1415;
			}
			else if(assive_mode[1] == AOMODE){
				phase[1] = hip2.phase[1] - AOoffset[1];
				
				while(phase[1] > 0){
					phase[1] = phase[1]-6.2830;
				}
				while(phase[0] < 0){
					phase[1] = phase[1]+6.2830;
				}
			}
			else{
				while(1){ MSG_ERR(123, "assive_mode error\r\n", 123); }
			}
			I2 = 0.1*sin(phase[1]);
			set_I_direction(2,I2);
			AssisMonitor("I2 %.2f\t",I2);
			
			
			
			printf("\r\n");
			
			
			
			debug_AOphase_offset1 = 1000*phase[0]; 
		}
		
			debug_hip1_d = (int)hip1_d;
			debug_hip1_rawd = (int)hip1_rawd;
			debug_hip2_d = (int)hip2_d;
			debug_hip2_rawd = (int)hip2_rawd;
			debug_hip1_w = (int)hip1_w;
			debug_hip1_raww = (int)hip1_raww;
			debug_hip2_w = (int)hip2_w;
			debug_hip2_raww = (int)hip2_raww;
		
			debug_AOphase1 = 1000.0*hip1.phase[1]; 
			debug_AOpre1 = (int)hip1.predict;
			debug_AOoutput1 = (int)hip1.output;
		
			debug_AOphase2 = 1000.0*hip2.phase[1]; 
			debug_AOpre2 = (int)hip2.predict;
			debug_AOoutput2 = (int)hip2.output;
		
			debug_AOw1 = 1000.0*hip1.ww;
			debug_AOphasePre1 = 1000.0*hip1.predictedBasicPhase;
		
			debug_AOw2 = 1000.0*hip2.ww;
			debug_AOphasePre2 = 1000.0*hip2.predictedBasicPhase;
		
			temp = 1000.0*AOoffset[0];
	}
}
