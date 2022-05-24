
/*
 * PO.c
 *
 *  Created on: Jun 3, 2020
 *      Author: test
 */

#include "PO.h"
#define PI 3.1415
#define poTest(...)
#include "PO.h"
#define JC 10 /* 阈值修改 */


//    float findmax(a[],j,segment_ith[])
//    {
//          return max,max_i;
//    }


/**
  * @brief  模仿math.atan2(y,x)的-pi-+pi的坐标角度算法
  *
  * @param  y xy坐标轴上的y坐标
  *
  * @param  x xy坐标轴上的x坐标
  *
  * @retval v - xy坐标轴上,坐标（x,y)到原点的直线与x轴正向的夹角弧度值，范围为-pi-+pi
*/
float myatan2(float y, float x)
{
    float v=0;
    if( y==0 && x==0){
        ERROR(2,"error y = 0, x = 0");
        return 0.0;
	}
    if( x > 0){
        v = atan(y/x);
	}
    if (y>=0 && x<0){
        v = PI + atan(y/x);
	}
    if (y<0 && x<0){
        v = -PI + atan(y/x);
	}
    if (y>0 && x==0){
        v = PI/2;
	}
    if (y<0 && x==0){
        v = -PI/2;
	}
    return v;
}


/**
 * author lhx
 * Jan 5, 2021
 *
 * @brief : PO 相位计算
 * @param d - 人体关节角度
 * @param w - 人体关节角速度
 * @return   相位
 */


float LastestAngleStore[500],LastestVelocityStore[500],LastestPoPhase[500]; //LastestAngleStore 存储关节角度  LastestVelocityStore 存储关节角速度  LastestPoPhase  存储普通PO算法得出的相位(用于划分步态周期)
float APO_d, APO_w, APO_phase;
float kk=1.0,alpha=0.0,beta=0.0;
float xlabel1=-1000.0,xlabel2=1000.0;
int debugPOphase;
int stopflag2 = 0, flaga=0;
int AdvancedPOindex=0,begin=1,lastSegment_ith, segment_ith;					// AdvancedPOindex 存储数据游标  segment_ith 存储分割序列索引
float debugjc;
int16_t stopCounter2[2] = {0};

float APOPhase1(float d, float w)// APOPhase1为hip1,APOPhase2为hip2
{
	
	LastestAngleStore[AdvancedPOindex] = d;
	LastestVelocityStore[AdvancedPOindex] = w;
	LastestPoPhase[AdvancedPOindex] = myatan2(w,d);
	
	debugPOphase = 1000*myatan2(w,d);/* PO放大1000倍*/
//	printf("\r\n%d\t%.2f\t%.2f",debugPOphase,w,d);
	
	/* 判断相位分割点 第一次运行*/
	if(AdvancedPOindex > 0 
		&& (LastestPoPhase[AdvancedPOindex]-LastestPoPhase[AdvancedPOindex-1]) > 1 
		&& begin == 1){
		segment_ith = AdvancedPOindex;
		lastSegment_ith = AdvancedPOindex;
		begin = 0;
		xlabel1=-1000.0;
		xlabel2=1000.0;
	}
	
	if(begin==0		/* 存在相位分割点 */
		&& (AdvancedPOindex > 0)
		&& ((LastestPoPhase[AdvancedPOindex]-LastestPoPhase[AdvancedPOindex-1])>1) 	/* 分割点 */
		&& (AdvancedPOindex-lastSegment_ith)>8)									/* 分割点间隔大于100 */
	{																				//寻找步态分割点
		segment_ith=AdvancedPOindex;
		
		/* 寻找极值 */
		float angmax=-100,angmin=100,wmax=-100,wmin=100; 
		for(int n = lastSegment_ith;n < segment_ith;n++){
			if(LastestAngleStore[n]>angmax){
				angmax=LastestAngleStore[n];
			}
			if(LastestAngleStore[n]<angmin){
				angmin=LastestAngleStore[n];
			}
			if(LastestVelocityStore[n]>wmax){
				wmax=LastestVelocityStore[n];
			}
			if(LastestVelocityStore[n]<wmin){
				wmin=LastestVelocityStore[n];
			}
		}
		
		//计算上个周期的k、alpha、beta
		kk = fabs(angmax-angmin)/fabs(wmax-wmin);
		alpha = (wmax+wmin)/2;
		beta = (angmax+angmin)/2;

		// 获取一次步态周期后重置极值与存储点
		xlabel1=-1000.0;
		xlabel2=1000.0;
		AdvancedPOindex = 0;
		lastSegment_ith = 0;
		
		
	}
	
	/* 计算改变后的相位 */
	APO_w = kk*(w-alpha);
	APO_d = d-beta;
	APO_phase = myatan2(APO_w,APO_d);
	
	/* 阈值判断（横轴差取10,xlabel1为当前周期内角度最大值，xlabel2为角度最小值，stopflag2为左边hip1停止判断标志，1走0停） */
	if( xlabel1 < APO_d )
	{
		xlabel1 = APO_d;
	}
	if( xlabel2 > APO_d )
	{
		xlabel2 = APO_d;
	}
	
	if((xlabel1 - xlabel2) <= JC ) /* 阈值修改 */
	{
		stopCounter2[0]++;
	}
	else{
		stopCounter2[0] = 0;
	}
			
	if(stopCounter2[0] > 10){					// 连续检测到10次，为 stop state 
		stopflag2 = 0;
		debugjc = xlabel1 - xlabel2;
		JCMonitor("JC1 %.2f\t",debugjc);
	}			
	else{
		stopflag2 = 1;
		debugjc = xlabel1 - xlabel2;
		JCMonitor("JC1 %.2f\t",debugjc);
	}
//	if ((APO_w*APO_w/25 + APO_d*APO_d/49) < 1){
//		stopflag2 = 0;
//	}
//	else{
//		stopflag2 = 1;
//	}
	
	// 移动光标到下一个存储点
	AdvancedPOindex = AdvancedPOindex + 1;
	// 防止空间溢出
	if(AdvancedPOindex >= 500) {
		AdvancedPOindex=0;
		begin = 1;
	}
//	if(stopflag2 == 1){
//		APOMonitor("\t\n     sflag_LEFT:  RUN!!!  RUN!!!      \n\t");
//	}
//	else{
//		APOMonitor("\t\n     sflag_LEFT:  STOP!!!  STOP!!!      \n\t");
//	}
		return APO_phase;
//	return APO_phase*stopflag2; /* 启用 */
}




float LastestAngleStore2[500],LastestVelocityStore2[500],LastestPoPhase2[500]; //LastestAngleStore 存储关节角度  LastestVelocityStore 存储关节角速度  LastestPoPhase  存储普通PO算法得出的相位(用于划分步态周期)
float APO2_d, APO2_w, APO2_phase;
float kk2=1.0,alpha2=0.0,beta2=0.0;
float xlabel3=-1000.0,xlabel4=1000.0;
int debugPOphase2;
int stopflag3 = 1;
int AdvancedPOindex2=0,begin2=1,lastSegment_ith2, segment_ith2;					// AdvancedPOindex 存储数据游标  segment_ith 存储分割序列索引
float debugjc2;

float APOPhase2(float d, float w)// APOPhase1为hip1,APOPhase2为hip2
{
	
	LastestAngleStore2[AdvancedPOindex2] = d;
	LastestVelocityStore2[AdvancedPOindex2] = w;
	LastestPoPhase2[AdvancedPOindex2] = myatan2(w,d);
	
	debugPOphase2 = 1000*myatan2(w,d);/* PO放大1000倍*/
//	printf("\r\n%d\t%.2f\t%.2f",debugPOphase,w,d);
	
	/* 判断相位分割点 第一次运行*/
	if(AdvancedPOindex2 > 0 
		&& ((LastestPoPhase2[AdvancedPOindex2]-LastestPoPhase2[AdvancedPOindex2-1]) > 1 )
		&& begin2 == 1){
		segment_ith2 = AdvancedPOindex2;
		lastSegment_ith2 = AdvancedPOindex2;
		begin2 = 0;
		xlabel3=-1000.0;
		xlabel4=1000.0;
	}
	
	if(begin2==0		/* 存在相位分割点 */
		&& (AdvancedPOindex2 > 0)
		&& (LastestPoPhase2[AdvancedPOindex2]-LastestPoPhase2[AdvancedPOindex2-1])> 1 	/* 分割点 */
		&& (AdvancedPOindex2-lastSegment_ith2)> 8 )									/* 分割点间隔大于100 */
	{																				//寻找步态分割点
		segment_ith2=AdvancedPOindex2;
		
		/* 寻找极值 */
		float angmax2=-100,angmin2=100,wmax2=-100,wmin2=100; 
		for(int n = lastSegment_ith2;n < segment_ith2;n++){
			if(LastestAngleStore2[n]>angmax2){
				angmax2=LastestAngleStore2[n];
			}
			if(LastestAngleStore2[n]<angmin2){
				angmin2=LastestAngleStore2[n];
			}
			if(LastestVelocityStore2[n]>wmax2){
				wmax2=LastestVelocityStore2[n];
			}
			if(LastestVelocityStore2[n]<wmin2){
				wmin2=LastestVelocityStore2[n];
			}
		}
		
		//计算上个周期的k、alpha、beta
		kk2 = fabs(angmax2-angmin2)/fabs(wmax2-wmin2);
		alpha2 = (wmax2+wmin2)/2;
		beta2 = (angmax2+angmin2)/2;

		// 获取一次步态周期后重置极值与存储点
		xlabel3=-1000.0;
		xlabel4=1000.0;
		AdvancedPOindex2 = 0;
		lastSegment_ith2 = 0;
		flaga=flaga +1;
		
	}
	
	/* 计算改变后的相位 */
	APO2_w = kk2*(w-alpha2);
	APO2_d = d-beta2;
	APO2_phase = myatan2(APO2_w,APO2_d);
	
	/* 阈值判断（横轴差取10,xlabe3为当前周期内角度最大值，xlabel2为角度最小值，stopflag3为右边hip2停止判断标志，1走0停） */
	if( xlabel3 < APO2_d )
	{
		xlabel3 = APO2_d;
	}
	if( xlabel4 > APO2_d )
	{
		xlabel4 = APO2_d;
	}
	
	if((xlabel3 - xlabel4) <= JC ) /* 阈值修改 */
	{
		stopCounter2[1]++;
	}
	else{
		stopCounter2[1] = 0;
	}
			
	if(stopCounter2[1] > 10){					// 连续检测到10次，为 stop state 
		stopflag3 = 0;
		debugjc2 = xlabel3 - xlabel4;
		JCMonitor("JC2 %.2f\t",debugjc2);
	}			
	else{
		stopflag3 = 1;
		debugjc2 = xlabel3 - xlabel4;
		JCMonitor("JC2 %.2f\t",debugjc2);
	}
	
	
	
//	if ((APO_w*APO_w/25 + APO_d*APO_d/49) < 1){
//		stopflag3 = 0;
//	}
//	else{
//		stopflag3 = 1;
//	}
	
	// 移动光标到下一个存储点
	AdvancedPOindex2 = AdvancedPOindex2 + 1;
	// 防止空间溢出
	if(AdvancedPOindex2 >= 500) {
		AdvancedPOindex2=0;
		begin2 = 1;
	}
//	if(stopflag3 == 1){
//		APOMonitor("\t\n     sflag_right2:  RUN!!!  RUN!!!      \n\t");
//	}
//	else{
//		APOMonitor("\t\n     sflag_right2:  STOP!!!  STOP!!!      \n\t");
//	}
		return APO2_phase;
//	return APO2_phase*stopflag3; /* 启用 */
}




/**
 * author lhx
 * Jun 3, 2020
 *
 * @brief : 助力系数非线性调节，以降低抖动和提高安全
 * @param ang - 人体关节角度
 * @param w - 人体关节角速度
 * @param @k - 助力系数 pointer
 * @return   助力力矩大小和方向
 * Window > Preferences > C/C++ > Editor > Templates.
 */
extern float floatabs(float x);
#define AssisTor 0.1
#define RightTorRatio 3	// assist gain 
#define D_area 0.0		// 2.0			// for eliminate chattering
#define W_area 0.0		// 1.0
#define MAX_D_area 50.0	// for safety

#include "math.h"
//阈值防抖动算法
void th_algori(float ang, float w, float * k)
{

}

/**
 * author lhx
 * Jun 3, 2020
 *
 * @brief : 相位振荡器算法
 * @param d - 人体关节角度
 * @param w - 人体关节角速度
 * @param sin_fai -角度与角速度的相位差。 如果角度与角速度同相位，则该值不变
 * @param @k - 助力系数
 * @return   助力力矩大小和方向
 * Window > Preferences > C/C++ > Editor > Templates.
 */
float PO(float d, float w,uint8_t node)
{
	float sin_fai = w/sqrt(d*d+w*w);
	float k;

	th_algori(d,w,&k);
	if(node == 2){						// Right Torque assive
		k = k*RightTorRatio;
	}
	float assistive_torque = sin_fai*k;
	return assistive_torque;
}


