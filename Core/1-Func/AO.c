#include "AO.h"


float abs(float x)
{
	return x>0 ? x:-x;
}


#define MaxSize 20
struct Adaptive_Oscillators{
    float va;
    float vw;
    float vph;

    int order;
    float dt;
    float phase[20];
    int step;
    
    float aa[20],pphh[20],ww;
    float init_aa[20],init_ww,init_pphh[20];
    float w_real[20];

    float output;
    float predict;

    /*用于检测预测的准确性*/
    float predict_10steps_save[MaxSize];
    float output_save[MaxSize];
    int8_t index;		//指向要填充的下一个空间
};

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

/*AO 参数配置*/
void set(struct Adaptive_Oscillators* AO, float vw,float va,float vph, int order,float dt,float ww,float aa[],float pphh[],int step)
{
        AO->va = va;
        AO->vw = vw;
        AO->vph = vph;
        AO->order = order;
        AO->ww = ww;
        AO->init_ww = ww;
        AO->dt = dt;
        AO->step = step;
        for(uint8_t i=0;i<order+1;i++){
            AO->aa[i] = aa[i];
            AO->init_aa[i] = aa[i];
            AO->pphh[i] = pphh[i];
            AO->init_pphh[i] = pphh[i];
            //printf("amplify[%d] = %.3f\r\n", i,aa[i]);
            //printf("phase[%d] = %.3f\r\n", i,pphh[i]);
        }
        //printf("inital w = %.3f\r\n",ww);
        
        for(int i=0;i< AO->order+1;i++){
            AO->w_real[i] = i*AO->ww;    /* w_real[0]=0;w_real[1]=w_now;w_real[2]=2*w_now ... */
        }

        /*存储下标*/
        AO->index = 0;
}


/*
    va - 幅值学习参数
    vw - 频率学习参数
    vph - 相位学习参数
    
    order - 非零w的联级振荡器数量,不包含w=0
    a_now - 振荡器当前幅值 order+1个阶数
    w_now - 振荡器当前频率
    ph_now - 振荡器当前相位 order+1 ,第一个ph_now[0]强制为0
    e - 外部输入-估计值
*/

void Oscillators(struct Adaptive_Oscillators* AO, float e, int sync, float vw_sync)
{       
    float va = AO->va,vw=AO->vw,vph=AO->vph;    /*ѧϰ����*/
    float Dph,Da,Dw;                            /*����*/
    float dt = AO->dt;                          /*�������*/
    float sum_a;                                /*������ֵ��*/
    
    {
    sum_a = 0;
    for(int j=0;j< AO->order+1;j++)
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

    for(int i=0;i< AO->order+1;i++)
        AO->w_real[i] = i*AO->ww;
	//printf("1\r\n");
}

/*
 * ����Ԥ�����
 * delta_t		-	Ԥ���δ��ʱ�䳤�ȣ���λs
 * AO 			-	�����ṹ
 */
float predict(struct Adaptive_Oscillators* AO, float delta_t)
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


void input(struct Adaptive_Oscillators* AO, float y_now, float t_now, int predict_steps, int sync, float vw_sync)
{
        AO->output = predict(AO,0);
        float e = y_now - AO->output;
        Oscillators(AO,e,sync,vw_sync);
	//printf("e = %.2f, ww=%.2f\r\n",e,AO->ww);

        for(int i=0;i < AO->order+1;i++){
            AO->phase[i] = AO->pphh[i];
	        while(AO->phase[i]>0)
	            AO->phase[i] = AO->phase[i]-2*3.1415926;
	        while(AO->phase[i]<0)
	            AO->phase[i] = AO->phase[i]+2*3.1415926;
        }
        AO->predict = predict(AO,predict_steps*AO->dt);    // 10ms

        AO->output_save[AO->index] = AO->output;
        AO->predict_10steps_save[AO->index] = AO->predict;
		if(++(AO->index) == MaxSize){
			AO->index = 0;
		}
}

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
    pre = predict(&hip, 10);
    printf("pre = %.5f, truth value = 4.466306118313432",pre);
}


void test_AO(void)
{
    struct Adaptive_Oscillators hip;
    float va,vw,dt,aa[2] = {0.0, 15.0},pphh[2] = {0,0};
    int step;
    float pi = 3.1415926;

	dt = 0.02;
    va = 2*pi/(23*dt) * 2;
    vw = va;
    step = 10;
    
    set(&hip,vw,va,sqrt(24.2*vw),1,dt,15,aa,pphh,step);
    //show(&hip);

    float y;
    uint8_t predict_step = 1;
    uint64_t j=0;
    while(j++ < 5000000000){
        y = 30*sin(23*j*dt+56)+20;
        input(&hip,y,j,predict_step,0,0);
        printf("%.2f\t%.2f\r\n",y, hip.predict);
        //show(&hip);
		HAL_Delay(20);
    }

}



uint8_t Isequal(float i, float j)
{
	if(abs(i-j)<5){
		return 1;
	}
	return 0;
}



uint8_t predict_step = 5;
uint64_t index = 0;
struct Adaptive_Oscillators hip1,hip2;
void AO_Init(void)
{
    float va,vw,vph,dt,aa[2] = {0.0, 15.0},pphh[2] = {0,0};
    int step;
    float pi = 3.1415926;

	dt = 20.0/1000;//   1/100 = 0   1.0/100 = 0.01 先按int类型计算，然后强制转化为float
	
	float w0 = 30*dt;
    va = 2*pi/w0 * 1;
    vw = va;
	vph = sqrt(24.2*vw);
    step = 2;
    
    //set(&hip1,vw,va,vph,1,dt,2,aa,pphh,step);
	//set(&hip2,vw,va,vph,1,dt,2,aa,pphh,step);
	set(&hip1,vw,va,vph,1,dt,1,aa,pphh,step);
	set(&hip2,vw,va,vph,1,dt,1,aa,pphh,step);
	//show(&hip);
	
    //predict_step = 5;
	//index = 0;
}

void AO(float d,uint8_t node)
{
	if(node == 1){
		input(&hip1,d,index,predict_step,0,0);
		//show(&hip);
		printf("%.2f\t%.2f\t%.2f\t",d, hip1.output,hip1.predict);
	}
	else if(node == 2){
		input(&hip2,d,index,predict_step,0,0);
		//show(&hip);
		//printf("%.1f\t%.1f\t%.1f\t",d, hip2.output,hip2.predict);
	}
}


#include "PO.h"
int16_t PO_time = 0;
int16_t AO_flag=0;
int8_t assive = -20;
float switch_task(struct Adaptive_Oscillators * AO, float d, float w,uint8_t node)
{
	int8_t i = AO->index;int8_t j;
	//int8_t m;
	j = i-1-predict_step < 0 ? i-1-predict_step+MaxSize : i-1-predict_step;
	//m = i-1 < 0 ? i-1+MaxSize : i-1;
	
	if(Isequal(d ,AO->predict_10steps_save[j])){
		if(assive == 20){
			assive = 20; // 20 * (AO->predict - d);
			PO_time = PO_time-10;
			AO_flag = 0;
			//printf("a");
		}
		else if(assive == -20){
			AO_flag++;
			if(AO_flag > 10){
				AO_flag = 0;
				assive = 20;
			}
			else{
				assive = -20;
			}
			//printf("b");
			PO_time++;
		}
		
	}
	else{
		assive = -20; //O(d, w, node);
		PO_time++;
		AO_flag = 0;
		//printf("c");
	}
	
	if(PO_time > 100){
		AO_Init();
		PO_time = 0;
	}
	else if(PO_time < 0){
		PO_time = 0;
	}
	
	
	if(node == 1)
	//printf("%.2f\r\n",AO->predict_10steps_save[m]);
		printf("%.1f\t%d\t",AO->predict_10steps_save[j]-d,assive);
	
	return assive;
}


float assive_torque(struct Adaptive_Oscillators * AO, float d)
{
	float assistive_torque;
	if(abs(AO->predict - d)<3)
		assistive_torque = 0;
	else{
		assistive_torque = (AO->predict - d)*0.05;
	}
	return assistive_torque;

}
