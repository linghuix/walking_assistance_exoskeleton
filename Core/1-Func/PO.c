
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
#define RightTorRatio 3  // assist gain
#define D_area 0.0  // 2.0			// for eliminate chattering
#define W_area 0.0  // 1.0
#define MAX_D_area 50.0  // for safety


/**
 * @brief  模仿math.atan2(y,x)的-pi-+pi的坐标角度算法
 *
 * @param  y xy坐标轴上的y坐标
 *
 * @param  x xy坐标轴上的x坐标
 *
 * @retval v -
 * xy坐标轴上,坐标（x,y)到原点的直线与x轴正向的夹角弧度值，范围为-pi-+pi
 */
uint8_t error_atana = 0;
float   myatan2(float y, float x)
{
  float v = 0;
  if (y == 0 && x == 0 && error_atana == 0) {
    // ERROR(2,"error y = 0, x = 0");
    return 0.0;
  }
  if (x > 0) {
    v = atan(y / x);
  }
  if (y >= 0 && x < 0) {
    v = PI + atan(y / x);
  }
  if (y < 0 && x < 0) {
    v = -PI + atan(y / x);
  }
  if (y > 0 && x == 0) {
    v = PI / 2;
  }
  if (y < 0 && x == 0) {
    v = -PI / 2;
  }
  error_atana = 1;
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

float PO_phase(float d, float w) { return myatan2(w, d); }

int   gaitSegment_size = 5;
float APOPhase(struct APO* apo, float d, float w)
{
  // 初始化，仅运行一次
  if (apo->once == 1) {
    winBuffer(&(apo->gaitSegment), apo->gaitSegment_data, gaitSegment_size);
    apo->once = 0;
  }

  /* save para */
  apo->LastestAngleStore[apo->AdvancedPOindex]    = d;
  apo->LastestVelocityStore[apo->AdvancedPOindex] = w;
  addToBuff(&(apo->gaitSegment), d);

  apo->AdvancedPOindex = apo->AdvancedPOindex + 1;  // 移动光标到下一个存储点
  if (apo->AdvancedPOindex >= 500) {                // 防止空间溢出
    apo->AdvancedPOindex = 0;
    apo->begin           = 1;
  }

  apo->debugPOphase = 1000 * myatan2(w, d); /* debug  PO放大1000倍*/
  // printf("\r\n%d\t%.2f\t%.2f",apo->debugPOphase,w,d);


  /* 判断相位分割点 第一次运行 */
  if (apo->AdvancedPOindex > 0 && findPeak(&(apo->gaitSegment), 5) && apo->begin == 1) {
    // segment_ith = apo->AdvancedPOindex;
    apo->lastSegment_ith = apo->AdvancedPOindex;
    apo->begin           = 0;
  }


  // 获取一个步态周期
  if (apo->begin == 0                                                   /* 存在相位分割点 */
      && (apo->AdvancedPOindex > 0) && findPeak(&(apo->gaitSegment), 5) /* 分割点 */
      && (apo->AdvancedPOindex - apo->lastSegment_ith) >
             apo->APO_updateinterval) /* 分割点间隔大于100 */
  {                                   //寻找步态分割点
    apo->segment_ith = apo->AdvancedPOindex;

    /* 寻找极值 */
    float angmax = -100, angmin = 100, wmax = -100, wmin = 100;
    for (int n = apo->lastSegment_ith; n < apo->segment_ith; n++) {
      if (apo->LastestAngleStore[n] > angmax) {
        angmax = apo->LastestAngleStore[n];
      }

      if (apo->LastestAngleStore[n] < angmin) {
        angmin = apo->LastestAngleStore[n];
      }

      if (apo->LastestVelocityStore[n] > wmax) {
        wmax = apo->LastestVelocityStore[n];
      }

      if (apo->LastestVelocityStore[n] < wmin) {
        wmin = apo->LastestVelocityStore[n];
      }
    }

    //计算上个周期的正值 k、alpha、beta
    apo->kk    = (angmax - angmin) / (wmax - wmin);
    apo->alpha = (wmax + wmin) / 2;
    apo->beta  = (angmax + angmin) / 2;

    // 获取一次步态周期后重置存储点
    apo->AdvancedPOindex = 0;
    apo->lastSegment_ith = 0;
  }

  /* 计算改变后的相位 */
  apo->APO_w     = apo->kk * (w - apo->alpha);
  apo->APO_d     = d - apo->beta;
  apo->APO_phase = myatan2(apo->APO_w, apo->APO_d);

  /* 抖动判断，站立判断 */
  //	if ((APO_w*APO_w/25 + APO_d*APO_d/49) < 1){
  //		stopflag = 1;
  //	}
  //	else{
  //		stopflag = 0;
  //	}


  return apo->APO_phase;
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
#define D_area 0.0  // 2.0			// for eliminate chattering
#define W_area 0.0  // 1.0
#define MAX_D_area 50.0  // for safety

#include "math.h"
//阈值防抖动算法
void th_algori(float ang, float w, float* k) {}

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
//#define RightTorRatio 3	// assist gain
// float PO(float d, float w,uint8_t node)
//{
//	float sin_fai = w/sqrt(d*d+w*w);
//	float k;

//	th_algori(d,w,&k);
//	if(node == 2){						// Right Torque
// assive 		k = k*RightTorRatio;
//	}
//	float assistive_torque = sin_fai*k;
//	return assistive_torque;
//}
