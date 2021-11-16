
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

float PO_phase(float d, float w)
{
	return myatan2(w,d);
}


#include "win.h"
float LastestAngleStore[500], LastestVelocityStore[500]; 
//LastestAngleStore 存储关节角度  LastestVelocityStore 存储关节角速度  LastestPoPhase  存储普通PO算法得出的相位(用于划分步态周期)，存储空间至少存储一个步态周期

WIN gaitSegment; float gaitSegment_data[3] = {0}; int gaitSegment_size = 3;

float APO_d, APO_w, APO_phase;
float kk=1.0,alpha=0.0,beta=0.0;
int debugPOphase;
int stopflag = 1;
int AdvancedPOindex=0,begin=1,lastSegment_ith, segment_ith;
// AdvancedPOindex 存储数据游标  segment_ith 存储分割序列索引， begin=1 开始寻找第一个分割点

int once = 1;
float APOPhase(float d, float w)
{
	// 初始化，仅运行一次
	if(once){
		winBuffer(&gaitSegment, gaitSegment_data, gaitSegment_size);
		once = 0;
	}
	
	/* save para */
	LastestAngleStore[AdvancedPOindex] = d;
	LastestVelocityStore[AdvancedPOindex] = w;
	addToBuff(&gaitSegment, myatan2(w,d));

	AdvancedPOindex = AdvancedPOindex + 1;			// 移动光标到下一个存储点
	if(AdvancedPOindex >= 500) {					// 防止空间溢出
		AdvancedPOindex=0;
		begin = 1;
	}

	debugPOphase = 1000*myatan2(w,d);											/* debug  PO放大1000倍*/
	printf("\r\n%d\t%.2f\t%.2f",debugPOphase,w,d);
	

	/* 判断相位分割点 第一次运行 */
	if(AdvancedPOindex > 0 
		&& ( getValue(&gaitSegment,3) - getValue(&gaitSegment,1)) > 4 
		&& begin == 1)
	{
		//segment_ith = AdvancedPOindex;
		lastSegment_ith = AdvancedPOindex;
		begin = 0;
	}
	

	// 获取一个步态周期
	if  (begin == 0		/* 存在相位分割点 */
		&& (AdvancedPOindex > 0)
		&& (getValue(&gaitSegment,3) - getValue(&gaitSegment,1))>4 	/* 分割点 */
		&& (AdvancedPOindex-lastSegment_ith)>100)									/* 分割点间隔大于100 */
	{																				//寻找步态分割点
		segment_ith=AdvancedPOindex;
		
		/* 寻找极值 */
		float angmax=-100,angmin=100,wmax=-100,wmin=100; 
		for(int n = lastSegment_ith;n < segment_ith;n++) {

			if(LastestAngleStore[n] > angmax){ 
				angmax = LastestAngleStore[n];
			}

			if(LastestAngleStore[n] < angmin){
				angmin = LastestAngleStore[n];
			}

			if(LastestVelocityStore[n] > wmax){
				wmax = LastestVelocityStore[n];
			}

			if(LastestVelocityStore[n] < wmin){
				wmin = LastestVelocityStore[n];
			}
		}
		
		//计算上个周期的正值 k、alpha、beta
		kk = (angmax-angmin)/(wmax-wmin);
		alpha = (wmax+wmin)/2;
		beta = (angmax+angmin)/2;

		// 获取一次步态周期后重置存储点
		AdvancedPOindex = 0;
		lastSegment_ith = 0;
	}
	
	/* 计算改变后的相位 */
	APO_w = kk*(w-alpha);
	APO_d = d-beta;
	APO_phase = myatan2(APO_w,APO_d);
	
	/* 抖动判断，站立判断 */
//	if ((APO_w*APO_w/25 + APO_d*APO_d/49) < 1){
//		stopflag = 1;
//	}
//	else{
//		stopflag = 0;
//	}
	
	
	return APO_phase*stopflag;
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

