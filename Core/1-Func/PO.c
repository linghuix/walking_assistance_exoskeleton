
/*
 * PO.c
 *
 *  Created on: Jun 3, 2020
 *      Author: test
 */

#include "PO.h"
#define PI 3.1415
#define poTest(...)
/*
 * author lhx
 * Jun 3, 2020
 *
 * @brief : 相位振荡器算法
 * d - 人体关节角度
 * w - 人体关节角速度
 * sin_fai -角度与角速度的相位差。 如果角度与角速度同相位，则该值不变
 * k - 助力系数
 * return - 助力力矩大小和方向
 * Window > Preferences > C/C++ > Editor > Templates.
 */

#include "PO.h"

extern float floatabs(float x);

#define AssisTor 0.1
#define RightToleftTorRatio 3

#define D_area 0.0		//2.0
#define W_area 0.0		//1.0
#define MAX_D_area 50.0
//阈值防抖动算法
void th_algori(float ang, float w, float * k)
{
	/* 设定阈值，防止在关节角度较小的时候，助力系数k随着关节角速度的正负变化而产生抖动。 */
	if(floatabs(ang) < D_area || floatabs(w) < W_area || floatabs(ang) > MAX_D_area){
		*k = 0.0;
	}
	else{
		*k = AssisTor;
	}
	//poTest("\r\nw=%.2f.&&&*k=%.2f\r\n",w,*k);
}

/*
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


/* 获取相位 */
float PO_phase(float d, float w)
{
	return myatan2(w, d);
}



float PO(float d, float w,uint8_t node)
{
	float sin_fai = w/sqrt(d*d+w*w);
	float k;

	th_algori(d,w,&k);
	if(node == 2){						// Right Torque assive
		k = k*RightToleftTorRatio;
	}
	float assistive_torque = sin_fai*k;
	return assistive_torque;
}


