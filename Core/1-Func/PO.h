/** 
 * PO.h
 *
 *  Created on: Jun 3, 2020
 *      Author: test
 */

#ifndef _FUNC_PO_H_
#define _FUNC_PO_H_


#define POMODE -20


#include "FUNC.h"
#include "math.h"


#include "win.h"




struct APO{
	float LastestAngleStore[500], LastestVelocityStore[500]; 
	//LastestAngleStore �洢�ؽڽǶ�  LastestVelocityStore �洢�ؽڽ��ٶ�  LastestPoPhase  �洢��ͨPO�㷨�ó�����λ(���ڻ��ֲ�̬����)���洢�ռ����ٴ洢һ����̬����

	WIN gaitSegment; 

	// adjust d,w and po_phase
	float APO_d, APO_w, APO_phase;
	
	float gaitSegment_data[5];
	
	// **APO_updateinterval** change according to the gait period
	// the small value can make APO reacts quickly to gait change 
	// and the lager value make APO less chattering when standing
	int APO_updateinterval;
	float kk,alpha,beta;
	int debugPOphase;
	int stopflag;
	int AdvancedPOindex,begin,lastSegment_ith, segment_ith;
	uint8_t once;	//ֻ����һ�εĴ���
};



float PO(float d, float w,uint8_t node);

float PO_phase(float d, float w);

float APOPhase(struct APO * apo, float d, float w);


#endif /* 1_FUNC_PO_H_ */
