
/*
 * PO.c
 *
 *  Created on: Jun 3, 2020
 *      Author: test
 */

#include "PO.h"
#define PI 3.1415



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
//        ERROR(2,"error y = 0, x = 0");
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
	return myatan2(w, d);
}

