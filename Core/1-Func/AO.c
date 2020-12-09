#include "AO.h"

extern uint16_t period;

/*
    va - 幅值学习参数
    vw - 频率学习参数
    vph - 相位学习参数
    
*/
uint64_t Aoindex = 0;
struct Adaptive_Oscillators hip1,hip2;
extern int CONTROL_PERIOD;
void set(struct Adaptive_Oscillators* AO, float vw,float va,float vph, int order,float dt,float ww,float aa[],float pphh[],int step);
void AO_Init(float T)
{
    float va,vw,vph,dt;
    float aa[3] = {10.0, 50.0, 20.0}, pphh[3] = {0, PI/3, PI/6};
    int step;

	dt = (float)CONTROL_PERIOD*2.0/1000.0;
    // 1/100 = 0   1.0/100 = 0.01 先按 int 类型计算，然后强制转化为 float
	
    va = 2.0/(5.0* T);
    vw = va;
	vph = sqrt(24.2*vw);
    step = 5;
    
	set(&hip1,vw,va,vph,2,dt,2*PI/T,aa,pphh,step);
	set(&hip2,vw,va,vph,2,dt,2*PI/T,aa,pphh,step);
	//show(&hip);
}


/*AO 参数配置*/
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
            //printf("amplify[%d] = %.3f\r\n", i,aa[i]);
            //printf("phase[%d] = %.3f\r\n", i,pphh[i]);
        }
        //printf("inital w = %.3f\r\n",ww);
        
        for(uint8_t i=0;i < AO->order+1;i++){
            /* w_real[0]=0; w_real[1]=w_now; w_real[2]=2*w_now ... */
			AO->w_real[i] = i*AO->ww;    
        }

        /*存储下标*/
        AO->index = 0;
}

void input(struct Adaptive_Oscillators* AO, float y_now, float t_now, int sync, float vw_sync);
void AO(float d,uint8_t node)
{
	if(node == 1){
		input(&hip1,d,Aoindex,0,0);
		//show(&hip);
//		printf("%.2f\t%.2f\t%.2f\t",d, hip1.output,hip1.predict);
	}
	else if(node == 2){
		input(&hip2,d,Aoindex,0,0);
		//show(&hip);
		//printf("%.1f\t%.1f\t%.1f\t",d, hip2.output,hip2.predict);
	}
}

/*
    order -     非零 w 的联级振荡器数量,不包含 w=0
    a_now -     振荡器当前幅值 order+1个阶数
    w_now -     振荡器当前频率
    ph_now -    振荡器当前相位 order+1 ,第一个 ph_now[0] 强制为 0
    e -         外部输入-估计值
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
		AO->phase[i] = fitIn(AO->phase[i], 2*3.1416, 0);
	}
	AO->predict = curvePredict(AO, AO->step*AO->dt);    // 10ms
	phasePredict(AO, AO->step*AO->dt);
	AO->outputSave[AO->index] = AO->output;
	AO->predictedSaveData[AO->index] = AO->predict;
	if(++(AO->index) == MaxSize){
		AO->index = 0;
	}
}

/*
 * @brief fit data into some zone
 * @parameter:
 *      data            input data need to
 *      up            	up bound
 *      dowm         	down bound
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

/*
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
            //printf("dw = %f",Dw);
        }
        else if(sync == 1)            // multi oscillators freq sync
            AO->ww = dt*vw_sync + AO->ww;
    }
    
    for(int i=0;i< AO->order+1;i++){
        AO->w_real[i] = i*AO->ww;
	}
}


/*
 * @brief: AO curve value Predict using state.
 * @parameter:
 *      delta_t	-	Predict Time interval. It can maxmize independence of this function.
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

/*
 * @brief 单独调用获取相位预测 AO basic phase value (phase[1] which is corresponding to ww's phase) Predict using state
 * @parameter
 *      delta_t	-	Predict Time interval
 *      AO 		-	Adaptive Oscillators
 */
void phasePredict(struct Adaptive_Oscillators* AO, float delta_t)
{
//	float delta_t = AO->step*AO->dt;
	AO->predictedBasicPhase = AO->phase[1]+AO->ww*delta_t;
	AO->predictedBasicPhase = fitIn(AO->predictedBasicPhase, 2*3.1416, 0);
}

/*
 * brief test
 * @parameter
 *      AO 		-	Adaptive Oscillators
 */
void show(struct Adaptive_Oscillators* AO)
{
    printf("\r\nva - %.2f\r\n",AO->va);
    printf("vw - %.2f\r\n",AO->vw);
    printf("vph - %.2f\r\n",AO->vph);
    printf("order - %d\r\n",AO->order);
    printf("dt - %.4f\r\n",AO->dt);
    printf("phase - %.2f\r\n",AO->phase[1]);
    printf("step - %d\r\n",AO->step);
    printf("ww = %.5f\r\n",AO->ww);
    printf("output - %.5f\r\n",AO->output);
    printf("predict = %.5f\r\n",AO->predict);
    
    for(int i=0;i<AO->order+1;i++){
        printf("phi[%d] = %.5f\r\n", i,AO->pphh[i]);
    }

    for(int i=0;i<AO->order+1;i++){
        printf("amplify[%d] = %.5f\r\n", i,AO->aa[i]);
    }

    for(int i=0;i<AO->order+1;i++){
        printf("real w[%d] = %.5f\r\n", i,AO->w_real[i]);
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
    float pre; /*Ԥ��ֵ*/
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
    printf("pre = %.5f, truth value = 4.466306118313432",pre);
}

int test_phase, test_w, test_output, test_input, test_predict, test_phasePredict, test_j;	//jscope
void test_AO(void)
{
    struct Adaptive_Oscillators hip;
    float va, vw, dt, T, aa[2] = {0.0, 15.0}, pphh[2] = {PI/5.0, PI/3.0};
    int step;
	
	printf("AO test for sine wave\r\n");
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
        printf("y\t%.2f\tpredict\t%.2f\t%.2f\r\n",y, hip.predict, hip.phase[1]);
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
	
	printf("AO test for sine wave\r\n");
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
        printf("y\t%.2f\tpredict\t%.2f\t%.2f\r\n",y, hip.predict, hip.phase[1]);
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

/* 	最低值查找程序
	1-表示找到了最低值 
 */
uint8_t findpeak(WINp win)
{
	ElementType end1 = GetValue(win,1);
	ElementType end2 = GetValue(win,3);
	ElementType mid = GetValue(win,2);
	
	if(mid < end1 && mid < end2){
		return 1;
	}
	return 0;
}
//

float floatabs(float x);
uint8_t Isequal(float i, float j)
{
	if(i==0) i = i+0.001;
	if(floatabs(i-j) < 0.1){
		return 1;
	}
	return 0;
}


float floatabs(float x)
{
	return x>0 ? x:-x;
}

#include "PO.h"

#define DEALY_TIME 40
int32_t PO_time = 0, debug_preOffset, debug_preOffset2;		//PO 运行时间长度度量，决定AO初始化
int16_t delaySwitch[2] = {DEALY_TIME, DEALY_TIME};			//切换延迟
int8_t assive = -20;
extern float dt;
float switch_task(struct Adaptive_Oscillators * AO, float d, float w, uint8_t node)
{
	int8_t i = AO->index;
	int8_t j;
	
	j = i-1-AO->step < 0 ? i-1-AO->step+MaxSize : i-1-AO->step;
	if(node==1){
		debug_preOffset = AO->predictedSaveData[j];
	}
	else if(node==2){
		debug_preOffset2 = AO->predictedSaveData[j];
	}
	
	if(Isequal(d ,AO->predictedSaveData[j])){
		delaySwitch[node-1]++;
		if(delaySwitch[node-1]>DEALY_TIME)  delaySwitch[node-1] = DEALY_TIME;
		
		if(assive == AOMODE){
			assive = AOMODE; 						// 20 * (AO->predict - d);
		}
		else if(assive == POMODE){
			if(delaySwitch[node-1] == DEALY_TIME){
				assive = AOMODE;
			}
			else{
				assive = POMODE;
			}
		}
	}
	else{
		delaySwitch[node-1]--;
		if(delaySwitch[node-1]<0)  delaySwitch[node-1] = 0;
		
		if(assive == POMODE){
			assive = POMODE; 			//O(d, w, node);
		}
		else if(assive == AOMODE){
			if(delaySwitch[node-1] == 0){
				assive = POMODE;
			}
			else{
				assive = AOMODE;
			}
		}
//		delaySwitch = 0;
	}
	
	if(assive == POMODE){
		PO_time = PO_time+1;
	}
	else if(assive == AOMODE){
		PO_time = PO_time-10;
	}
	
	if(PO_time > 1000){
		AO_Init(period*dt);					//AO reset 	float w0 = 21*dt;							 // 21*dt
		PO_time = 0;						//AO reset time
	}
	else if(PO_time < 0){
		PO_time = 0;
	}
	
	SwitchMonitor("PO_time\t%d\t",PO_time);
	SwitchMonitor("delaySwitch\t%d\t",delaySwitch);
	
	if(node == 1)
		SwitchMonitor("preErr1\t%.1f\tassiveMode1\t%d\t",AO->predict_10steps_save[j]-d,assive);
	if(node == 2)
		SwitchMonitor("preErr2\t%.1f\tassiveMode2\t%d\t",AO->predict_10steps_save[j]-d,assive);
	
	return assive;
}


float assive_torque(struct Adaptive_Oscillators * AO, float d)
{
	float assistive_torque;
	if(floatabs(AO->predict - d) < 3)
		assistive_torque = 0;
	else{
		assistive_torque = (AO->predict - d)*0.05;
	}
	return assistive_torque;

}


