
/*
 * PO.c
 *
 *  Created on: Jun 3, 2020
 *      Author: test
 */

#include "PO.h"

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

extern float abs(float x);

#define D_area 10.0
#define W_area 1.0
//阈值防抖动算法
void th_algori(float ang, float w, float * k)
{
	/* 设定阈值，防止在关节角度较小的时候，助力系数k随着关节角速度的正负变化而产生抖动。 */
	if(abs(ang) < D_area || abs(w) < W_area){
		*k = 0.0;
	}
	else{
		*k = 1;
	}
	//printf("\r\nw=%.2f.&&&*k=%.2f\r\n",w,*k);
}


float PO(float d, float w,uint8_t node)
{
	float sin_fai = w/sqrt(d*d+w*w);
	float k;

	th_algori(d,w,&k);

	float assistive_torque = sin_fai*k;
	//printf("\r\n%.2f.&&&%.2f\r\n",sin_fai,k);
	return assistive_torque;
}
