#include "AO.h"

int PREDICT_TIME = 4;	// 4*50ms ahead
#define GAIN 5

extern uint16_t period[2];


/**
 * @parameter
  T - 步态周期
  node - 关节电机编号 1-left 2-right
 * @constant
  va - 幅值学习参数
  vw - 频率学习参数
  vph - 相位学习参数
  gain - 最优学习参数增益，0.2~5
*/

uint64_t Aoindex = 0;
struct Adaptive_Oscillators hip1,hip2;
extern int CONTROL_PERIOD;
void set(struct Adaptive_Oscillators* AO, float vw,float va,float vph, int order,float dt,float ww,float aa[],float pphh[],int step);
void AO_Init(float T, uint8_t node)
{
    float va,vw,vph,dt;
    float aa[3] = {5.0, 10.0, 20.0}, pphh[3] = {0, PI/3, PI/6};
    int step;

	dt = (float)CONTROL_PERIOD*2.0/1000.0;
    // 1/100 = 0   1.0/100 = 0.01 先按 int 类型计算，然后强制转化为 float
//	float gain = 5.0;
//    va = 2.0/(gain* T);
	va = 2.0/(GAIN * T);
    vw = va;
	vph = sqrt(24.2*vw);
    step = 5;
    if(node == 1){
		set(&hip1,vw,va,vph,2,dt,3*PI/T,aa,pphh,step);
	}
	if(node == 2){
		set(&hip2,vw,va,vph,2,dt,3*PI/T,aa,pphh,step);
	}
}


/**
 *@brief AO 参数设置/配置
 *@parameter
  va - 幅值学习参数
  vw - 频率学习参数
  vph - 相位学习参数
  order - 振荡器池中的振荡器个数，除了没有频率的offset振荡器，因此总共 order+1 个振荡器
  dt - 时间间隔/采样间隔
  ww - 基频频率初值
  aa[] - 幅值初值
  pphh[] - 相位初值
  step - 预测步长
*/
void set(struct Adaptive_Oscillators* AO, float vw,float va,float vph, int order,float dt,float ww,float aa[],float pphh[],int step)
{
        AO->va = va;			    // learning parameter
        AO->vw = vw;
        AO->vph = vph;
	
        AO->order = order;		    // order+1 AOs
	
        AO->ww = ww;			    // basic frequency
        AO->init_ww = ww;
        AO->dt = dt;
        AO->step = step;		    // predict step

        for(uint8_t i=0;i<order+1;i++){
            AO->aa[i] = aa[i];		// amplitude
            AO->init_aa[i] = aa[i];
            AO->pphh[i] = pphh[i];	// phase
            AO->init_pphh[i] = pphh[i];
            //TESTOUT("amplify[%d] = %.3f\r\n", i,aa[i]);
            //TESTOUT("phase[%d] = %.3f\r\n", i,pphh[i]);
        }
        //TESTOUT("inital w = %.3f\r\n",ww);
        
        for(uint8_t i=0;i < AO->order+1;i++){
            /* w_real[0]=0; w_real[1]=w_now; w_real[2]=2*w_now ... */
			AO->w_real[i] = i*AO->ww;    
        }

        /*存储下标*/
        AO->index = 0;
}

/**
 *@brief 实时输入关节角度，进行AO迭代
 *@parameter
  node - 关节电机编号 1-left 2-right
  d - 关节角度
*/
void input(struct Adaptive_Oscillators* AO, float y_now, float t_now, int sync, float vw_sync);
void AO(float d,uint8_t node)
{
	if(node == 1){
		input(&hip1,d,Aoindex,0,0);
		//show(&hip);
		//TESTOUT("%.2f\t%.2f\t%.2f\t",d, hip1.output,hip1.predict);
	}
	else if(node == 2){
		input(&hip2,d,Aoindex,0,0);
		//show(&hip);
		//TESTOUT("%.1f\t%.1f\t%.1f\t",d, hip2.output,hip2.predict);
	}
}

/**
 * @parameter:
    order -     非零 w 的联级振荡器数量,不包含 w=0
    a_now -     振荡器当前幅值 order+1个阶数
    w_now -     振荡器当前频率
    ph_now -    振荡器当前相位 order+1 ,第一个 ph_now[0] 强制为 0
    e -         外部输入-估计值
*/
/**
 * @brief: 迭代算法
 * @parameter:
    y_now -     关节角度
    t_now -     关节角度对应采集时间
    sync -      是否同步
    vw_sync -   同步的学习参数
*/

void Oscillators(struct Adaptive_Oscillators* AO, float e, int sync, float vw_sync);
float curvePredict(struct Adaptive_Oscillators* AO, float delta_t);
void phasePredict(struct Adaptive_Oscillators* AO, float delta_t);
float fitIn(float data, float up, float down);
void input(struct Adaptive_Oscillators* AO, float y_now, float t_now, int sync, float vw_sync)
{
	AO->output = curvePredict(AO,0);
	float e = y_now - AO->output;
	Oscillators(AO,e,sync,vw_sync);
    
	for(int i=0;i < AO->order+1;i++){
		AO->phase[i] = AO->pphh[i];
		AO->phase[i] = fitIn(AO->phase[i], 2*PI, 0);
	}
	AO->predict = curvePredict(AO, AO->step*AO->dt);    // step*2*CONTROL_PERIOD ms
	phasePredict(AO, PREDICT_TIME*AO->dt);							// predict 200 ms 
	AO->outputSave[AO->index] = AO->output;
	AO->predictedSaveData[AO->index] = AO->predict;
	if(++(AO->index) == MaxSize){
		AO->index = 0;
	}
}

/**
 * @brief fit data into some zone
 * @parameter:
 *      data            input data need to
 *      up            	up bound
 *      down         	down bound
 * @note:
 *      fitIn(x,0,2pi)
 *      
 */
float fitIn(float data, float up, float down)
{
	 float period = up-down;
	 if(data <= up ){
		 if(data >= down){ return data; }
		 else{
			while(data < down){ data += period;}
		 }
	 }
	 else{
		while(data > up){ data -= period;}
	 }
	 return data;
}

/**
 * @brief: Oscillators differential function
 * @parameter:
 *      e               外部输入-估计值
 *      sync            Adaptive Oscillators
 *      vw_sync         
 * @note:
 *      it is called in input() function
 *      
 */
void Oscillators(struct Adaptive_Oscillators* AO, float e, int sync, float vw_sync)
{
    float va = AO->va,vw=AO->vw,vph=AO->vph; 
    float Dph,Da,Dw;
    float dt = AO->dt;
    float sum_a;
    
    sum_a = 0;
    for(int j=0;j< AO->order+1;j++){
        sum_a += AO->aa[j];
    }
    
    for(int i=0;i< AO->order+1;i++){
        if(i == 0){     /* w=0 */
            Da = AO->va * e;
            Dph = 0;    
        }
        else{           /* w!=0 */
            Dph = AO->w_real[i] + vph * e * cos(AO->pphh[i])/sum_a;
            AO->pphh[i] = dt * Dph + AO->pphh[i];
            Da = va * e * sin(AO->pphh[i]);
        }
        AO->aa[i] = dt*Da + AO->aa[i];

        if(i == 1 && sync == 0){
            Dw = vw*e*cos(AO->pphh[i])/sum_a;
            AO->ww = dt*Dw + AO->ww;
            //TESTOUT("dw = %f",Dw);
        }
        else if(sync == 1)            // multi oscillators freq sync
            AO->ww = dt*vw_sync + AO->ww;
    }
    
    for(int i=0;i< AO->order+1;i++){
        AO->w_real[i] = i*AO->ww;
	}
}


/**
 * @brief: AO curve value Predict using state.
 * @parameter:
 *      delta_t	-	degree Predict Time interval. It can maxmize independence of this function.
 *      AO 		-	Adaptive Oscillators
 * @note:
 *      it is called in input() function
 *      
 */
float curvePredict(struct Adaptive_Oscillators* AO, float delta_t)
{
	int order = AO->order;
	float y_pre = 0;

	for(int i=0;i < order+1;i++){
		if(i == 0)
			y_pre = AO->aa[i];
		else
			y_pre += AO->aa[i]*sin(AO->w_real[i] * delta_t + AO->pphh[i]);
	}
	return y_pre;
}


/** 
 * @brief 单独调用获取相位预测 AO basic phase value (phase[1] which is corresponding to ww's phase) Predict using state
 * @paras
 *      delta_t	-	Phase Predict Time interval
 *      AO 		-	Adaptive Oscillators
 */
void phasePredict(struct Adaptive_Oscillators* AO, float delta_t)
{
//	float delta_t = AO->step*AO->dt;
	AO->predictedBasicPhase = AO->phase[1]+AO->ww*delta_t;
	AO->predictedBasicPhase = fitIn(AO->predictedBasicPhase, 2*3.1416, 0);
}


/**
 * brief test
 * @para  AO 		-	Adaptive Oscillators
 */
void show(struct Adaptive_Oscillators* AO)
{
    TESTOUT("\r\nva - %.2f\r\n",AO->va);
    TESTOUT("vw - %.2f\r\n",AO->vw);
    TESTOUT("vph - %.2f\r\n",AO->vph);
    TESTOUT("order - %d\r\n",AO->order);
    TESTOUT("dt - %.4f\r\n",AO->dt);
    TESTOUT("phase - %.2f\r\n",AO->phase[1]);
    TESTOUT("step - %d\r\n",AO->step);
    TESTOUT("ww = %.5f\r\n",AO->ww);
    TESTOUT("output - %.5f\r\n",AO->output);
    TESTOUT("predict = %.5f\r\n",AO->predict);
    
    for(int i=0;i<AO->order+1;i++){
        TESTOUT("phi[%d] = %.5f\r\n", i,AO->pphh[i]);
    }

    for(int i=0;i<AO->order+1;i++){
        TESTOUT("amplify[%d] = %.5f\r\n", i,AO->aa[i]);
    }

    for(int i=0;i<AO->order+1;i++){
        TESTOUT("real w[%d] = %.5f\r\n", i,AO->w_real[i]);
    }
}


// -------------------*---------------------------*-------------------*----------------------*-----------------
// -------------------*---------------------------*-------------------*----------------------*-----------------
#define TEST_ON

#ifdef TEST_ON
void test_Osc(void)
{
    struct Adaptive_Oscillators hip;
    float va,vw,vph,dt,aa[2] = {0.0, 5.0},pphh[2] = {0,0};
    int step;
    float pi = 3.1415926;
    
    va = 2*pi/5 * 5;	// 5T
    vw = va;
    vph = sqrt(24.2*vw);
    dt = 0.01;
    step = 10;
    
    set(&hip,vw,va,vph,1,dt,5,aa,pphh,step);
    show(&hip);
    Oscillators(&hip,0.125,0,0);
    show(&hip);
}

void test_Pre(void)
{
    volatile float pre; 
    struct Adaptive_Oscillators hip;
    float va,vw,vph,dt,aa[2] = {1.25, 3.523},pphh[2] = {0,0};
    int step;
    float pi = 3.1415926;

    va = 2*pi/5 * 5;
    vw = va;
    vph = sqrt(24.2*vw);
    dt = 0.01;
    step = 10;

    set(&hip,vw,va,vph,1,dt,4,aa,pphh,step);
    show(&hip);
    pre = curvePredict(&hip, 10);
    TESTOUT("pre = %.5f, truth value = 4.466306118313432",pre);
}

int test_phase, test_w, test_output, test_input, test_predict, test_phasePredict, test_j;	//jscope
void test_AO(void)
{
    struct Adaptive_Oscillators hip;
    float va, vw, dt, T, aa[2] = {0.0, 15.0}, pphh[2] = {PI/5.0, PI/3.0};
    int step;
	
	TESTOUT("AO test for sine wave\r\n");
	dt = 0.005;
	T = 2*PI/(23.0);
    va = 2.0/(T * 1.0);
    vw = va;
    step = 2;
    
    set(&hip,vw,va,sqrt(24.2*vw),1,dt,15,aa,pphh,step);
    //show(&hip);

    float y;
    uint64_t j=0;
    while(j++ < 5000000000){
        y = 30*sin(23 * (j*dt) + 56) + 20;
        input(&hip,y,j,0,0);
        TESTOUT("y\t%.2f\tpredict\t%.2f\t%.2f\r\n",y, hip.predict, hip.phase[1]);
        //show(&hip);
		HAL_Delay(5);
		test_phase = 1000.0*hip.phase[1];
		test_w = 1000.0*hip.ww;
		test_output = 1000.0*hip.output;
		test_input = 1000.0*y;
		test_predict = 1000.0*hip.predict;
		test_phasePredict = 1000.0*hip.predictedBasicPhase;
		test_j = j % 100;
	}
}


void test_AOs(void)
{
    struct Adaptive_Oscillators hip;
    float va, vw, dt, T, aa[3] = {0.0, 15.0,1.0}, pphh[3] = {PI*0.2, PI/3.0, PI/3.0};
    int step;
	
	TESTOUT("AO test for sine wave\r\n");
	dt = 0.005;
	T = 2.0*PI/23.0;
    va = 2.0/(T * 1.0);
    vw = va;
    step = 2;
    
    set(&hip,vw,va,sqrt(24.2*vw),2,dt,15,aa,pphh,step);
    //show(&hip);

    float y;
    uint64_t j=0;
    while(j++ < 5000000000){
        y = 30*sin(23*j*dt+56)+20+10*sin(35*j*dt+56);
        input(&hip,y,j,0,0);
        TESTOUT("y\t%.2f\tpredict\t%.2f\t%.2f\r\n",y, hip.predict, hip.phase[1]);
        //show(&hip);
		HAL_Delay(5);
		test_phase = 1000.0*hip.phase[1];
		test_w = 1000.0*hip.ww;
		test_output = 1000.0*hip.output;
		test_input = 1000.0*y;
		test_predict = 1000.0*hip.predict;
		test_phasePredict = 1000.0*hip.predictedBasicPhase;
	}
}

#endif

/* switching task */
// -------------------*---------------------------*-------------------*----------------------*-----------------
// -------------------*---------------------------*-------------------*----------------------*-----------------
// -------------------*---------------------------*-------------------*----------------------*-----------------
// -------------------*---------------------------*-------------------*----------------------*-----------------
/* switching task */

#define SwitchMonitor(...) 		//printf

/**
 * @brief 关节角度最低值查找程序
 * @para  win 作用宽度
 * @return 1-表示找到了最低值 
 */
uint8_t findpeak(WINp win)
{
	ElementType end1 = GetValue(win,1);
	ElementType end2 = GetValue(win,3);
	ElementType mid = GetValue(win,2);
	
	if(mid < end1 && mid < end2 && mid<5){
		return 1;
	}
	return 0;
}


/**
 * @brief 判断是否两者是否相等
 * @para  i/j 用于判断是否相等的数值，此处为AO预测的髋关节角度与实际的角度 
 */
float floatabs(float x);
uint8_t Isequal(float i, float j)
{
	if(i==0) i = i+0.001;
	if(floatabs(i-j) < 8){	//尝试绝对值，相对误差，最小二乘
		return 1;
	}
	return 0;
}

/**
* @brief 判断数组中的数据是否波动不大
 * @para  float angle[] 装载AO输出的数组
 */
uint8_t Isstable(float angle[]){
	for(uint8_t i = 0; i < MaxSize-1; i++){
		if(angle[i]-angle[i+1] > 5){
			return 0;
		}
	}
	return 1;
}

/**
 * @brief 取浮点数的绝对值，由于库中的abs只针对int类型，因此特别创建一个关于浮点数的
 */
float floatabs(float x)
{
	return x>0 ? x:-x;
}


#include "PO.h"
/**
 * @brief 不同助力模式的切换策略
 * @para 	AO
 * @para 	d			关节角度，用于输入到AO中进行迭代，PO助力模式时被PO用于计算相位
 * @para 	w			关节角速度，PO助力模式时被PO用于计算相位
 * @para 	node		电机的编号，左为0，右为1
 * @factor  PO_time 	PO 运行时间长度度量，决定AO初始化
 * @factor  DELAY_TIME 	斯密特触发器的宽度
 * @factor  delaySwitch 斯密特触发器计数器，当值大于DELAY_TIME时为AO助力模式，小于0时为PO助力模式
 */
#define DELAY_TIME 15
#define RESET_TIME 400
int32_t PO_time[2]={0}, debug_preOffset[2]={0};	//PO 运行时间长度度量，决定AO初始化
int16_t delaySwitch[2] = {0, 0};				//切换延迟
int8_t assive[2] = {POMODE, POMODE};
extern float dt;
extern int16_t stopFlag[2];
extern int state[2];
int8_t switch_task(struct Adaptive_Oscillators * AO, float d, float w, uint8_t node)
{
	int8_t i = AO->index;
	int8_t j;

	if(stopFlag[node-1] == 1){
		assive[node-1] = POMODE;
		stopFlag[node-1] = 0;
		delaySwitch[node-1] = 0;
		state[node-1] = 0;
		PO_time[node-1] = RESET_TIME-140;			// convert stop state to walking state.
		return assive[node-1];
	}
	state[node-1] = 1;
	
	j = i-1-AO->step < 0 ? i-1-AO->step+MaxSize : i-1-AO->step;
	
	/* for debug */
	debug_preOffset[node-1] = AO->predictedSaveData[j];

	/* Smith trigger  --  delaySwitch between 0 to DELAY_TIME */
//	if(Isequal(d ,AO->predictedSaveData[j])&& (Isstable(AO->outputSave)==0)){//&& floatabs(w)>1){
	if(Isequal(d ,AO->predictedSaveData[j])){
		delaySwitch[node-1] = delaySwitch[node-1]+1; 
	}
	else{
		delaySwitch[node-1] = delaySwitch[node-1]-1; 
	}
	

	if(assive[node-1] == POMODE && delaySwitch[node-1]>=DELAY_TIME){
		assive[node-1] = AOMODE;
	}
	else if(assive[node-1] == AOMODE && delaySwitch[node-1]<=0){
		assive[node-1] = POMODE;
	}
	
	
	if(assive[node-1] == POMODE){
		PO_time[node-1] = PO_time[node-1]+1;
	}
	else if(assive[node-1] == AOMODE){
		PO_time[node-1] = PO_time[node-1]-0;
	}
	
	if(PO_time[node-1] > RESET_TIME){
		AO_Init( period[node-1] * dt, node);			//AO reset 	float w0 = 21*dt;							 // 21*dt
		PO_time[node-1] = 0;							//AO reset time
	}
	else if(PO_time[node-1] < 0){
		PO_time[node-1] = 0;
	}
	
	
	if(delaySwitch[node-1] > DELAY_TIME){
		delaySwitch[node-1] = DELAY_TIME;
	}
	if(delaySwitch[node-1] < 0){
		delaySwitch[node-1] = 0;
	}

	
	SwitchMonitor("PO_time\t%d\t",PO_time);
	SwitchMonitor("delaySwitch\t%d\t",delaySwitch);
	
	if(node == 1){
		SwitchMonitor("preErr1\t%.1f\tassiveMode1\t%d\t",AO->predict_10steps_save[j]-d,assive);
	}
	if(node == 2){
		SwitchMonitor("preErr2\t%.1f\tassiveMode2\t%d\t",AO->predict_10steps_save[j]-d,assive);
	}
	
	return assive[node-1];
}


/**
 * @brief   利用PID的P控制提供助力将下肢关节角度拉向AO预测的未来角度
 * @para 	AO
 * @para 	d			IMU采集的关节真实角度 degree
 */
float assive_torque(struct Adaptive_Oscillators * AO, float d)
{
	float assistive_torque;
	if(floatabs(AO->predict - d) < 3)				//当两者差距很小时，不提供助力
		assistive_torque = 0;
	else{
		assistive_torque = (AO->predict - d)*0.05;
	}
	return assistive_torque;
}

