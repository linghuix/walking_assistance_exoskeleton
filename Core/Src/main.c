
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
float weights[Buffsize] = {0.05,0.95};

/**
 * @para 用于检测人体是否静止
 */
int8_t stopCounter[2] = {0}, stopFlag[2] = {0};


uint16_t Interaction_force=0;

uint32_t peaktimestamp[2] = {0};				// 峰值对应时间记录
uint16_t period[2] = {20, 20};					// 人体步态周期估计
float dt = 0.050;								// 控制周期
float phase[2], predictPhase[2];				// 0左，1右
int8_t assive_mode[2] = {0};					// 当前助力模式

int32_t peak_delay_time[2] = {0};

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
int debug_assisTorque1 = 0, debug_assisTorque2 = 0;					// 临时查看变量
float AOoffset[2] = {0};					// AO 相位补偿器


extern int16_t delaySwitch[2];


extern float floatabs(float x);
#define AssisTor 0.1
#define RightTorRatio 3	// assist gain 
#define D_area 2.0		// 2.0			// for eliminate chattering
#define W_area 2.0		// 1.0
#define MAX_D_area 50.0	// for safety
float k = 0.0;
float K = 0.0;

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
	AO_Init(period[0]*dt, 1);
	AO_Init(period[1]*dt, 2);
	ECON_I_init();


	/*启动外设*/
	Acc1_Start();
	Acc2_Start();
	ECON_action();
	
	FSR_Init();
	

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
			/* detect the stop state */
//			if(floatabs(hip1_w) < 0.2){
//				stopCounter[0]++;
//			}
//			else{
//				stopCounter[0] = 0;
//			}
//			if(stopCounter[0] > 15){
//				stopFlag[0] = 1;
//			}
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
			
			/* detect the stop state */
//			if(floatabs(hip2_w) < 0.2){
//				stopCounter[1]++;
//			}
//			else{
//				stopCounter[1] = 0;
//			}
//			if(stopCounter[1] > 15){
//				stopFlag[1] = 1;
//			}
		}
		
		/* 控制周期 2ms x CONTROL_PERIOD */
		if(inc % CONTROL_PERIOD == 0){
			Interaction_force = GetFSRForce();
			INTERFORCE_Monitor("F %d\t", Interaction_force);
			
			IMUMonitor("acc1rawd\t%.2f\tw\t%.2f\t",hip1_rawd,hip1_raww);
			IMUMonitor("acc2rawd\t%.2f\tw\t%.2f\t",hip2_rawd,hip2_raww);
			IMUMonitor("%.2f\t%.2f\t",hip1_d,hip1_w);
			IMUMonitor("%.2f\t%.2f\t",hip2_d,hip2_w);
			
			Aoindex++;
			debug_AOIndex++; if( debug_AOIndex > 100 ){ debug_AOIndex = 0;}
			
			// 左 
			AO(hip1_d,1);
			
			/* summit detect and period predict*/
			peak_delay_time[0]--;
			if(found_peak[0] == 1){										// 检测峰值，估计人体步态周期并记录需要补偿的相位值
				if( peak_delay_time[0] <= 0 ){
					period[0] = Aoindex - peaktimestamp[0];				// gait period get
					peaktimestamp[0] = Aoindex;
					AOoffset[0] = hip1.phase[1];
					found_peak[0] = 0;
					peak_delay_time[0] = 10;							// 峰值检测延迟时间
				}
			}
			
			assive_mode[0] = switch_task( &hip1, hip1_d, hip1_w, 1);	// 模式切换
			if(stopFlag[0] == 1){
				assive_mode[0] = POMODE;
				stopFlag[0]=0;
			}
			
			/* get phase */
			k = AssisTor;
			if(assive_mode[0] == POMODE){
				phase[0] = PO_phase(hip1_d, hip1_w);
				phase[0] = phase[0] + PI;
				if(floatabs(hip1_d) < D_area || floatabs(hip1_w) < W_area || floatabs(hip1_d) > MAX_D_area){
					k = 0.0;
				}
			}
			else if(assive_mode[0] == AOMODE){
				phase[0] = hip1.predictedBasicPhase - AOoffset[0];
				phase[0] = fitIn(phase[0], 2*PI, 0);	
			}
			else{
				while(1){ MSG_ERR(123, "assive_mode error\r\n", 123); }
			}
			

			I1 = k*sin(phase[0]);
			set_I_direction(1,I1);
			AssisMonitor("I1 %.2f\t",I1);
			
			
			
			// 右 
			AO(hip2_d,2);
			
			/* summit detect */
			peak_delay_time[1]--;
			if(found_peak[1] == 1){							// 检测峰值，估计人体步态周期并记录需要补偿的相位值
				if( peak_delay_time[1] <= 0 ){
					period[1] = Aoindex - peaktimestamp[1];	// gait period get
					peaktimestamp[1] = Aoindex;
					AOoffset[1] = hip2.phase[1];
					found_peak[1] = 0;
				peak_delay_time[1] = 10;
				}
			}
			
			assive_mode[1] = switch_task( &hip2, hip2_d, hip2_w, 2);
			if(stopFlag[1] == 1){
				assive_mode[1] = POMODE;
				stopFlag[1]=0;
			}
			
			/* get phase */
			K = 3*AssisTor;
			if(assive_mode[1] == POMODE){
				phase[1] = PO_phase(hip2_d, hip2_w);
				phase[1] = -phase[1] + PI;
				if(floatabs(hip2_d) < D_area || floatabs(hip2_w) < W_area || floatabs(hip2_d) > MAX_D_area){
					K = 0.0;
				}
			}
			else if(assive_mode[1] == AOMODE){
				phase[1] = hip2.predictedBasicPhase - AOoffset[1];
				phase[1] = fitIn(phase[1], 2*PI, 0);				
			}
			else{
				while(1){ MSG_ERR(123, "assive_mode error\r\n", 123); }
			}
			
			I2 = K*sin(phase[1]);
			set_I_direction(2,I2);
			AssisMonitor("I2 %.2f\t",I2);
			printf("\r\n");
		}
		
			debug_assisTorque1 = 1000.0*I1;
			debug_assisTorque2 = 1000.0*I2;
		
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
		
			debug_AOphase_offset1 = 1000*phase[0]; 
			debug_AOphase_offset2 = 1000*phase[1]; 
	}
}
