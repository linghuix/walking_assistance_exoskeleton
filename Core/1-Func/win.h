
#ifndef __WIN_H_
#define __WIN_H_


#include <stdio.h> 
#include <stdlib.h>//malloc
#include "BSP.h"

#include "debug.h"

#define ListSize 100 


//Ԫ�����ͣ�Ĭ��Ϊ int
typedef float ElementType;

//���Ա����Ͷ��� 
typedef struct WinBuffer *WINp;
typedef struct WinBuffer WIN;

struct WinBuffer{
    
    ElementType *data;     	// ����data���ڴ�ű��㣬����е�Ԫ�ص������趨Ϊint 
    uint16_t length;            	// ��ǰ��˳���ĳ��� 
	uint16_t in;					// in - ָ����һ��Ҫ�����λ��
};


void test_win_buff(void);

void winBuffer(WINp winbuffer, ElementType *data,int size);

uint16_t getPreIndex(uint16_t index, uint16_t length);
#define getLastestValue(buff) buff.data[getPreIndex(buff.in,buff.length)]

void addToBuff(WINp winbuffer, ElementType data);
ElementType getValue(WINp winbuffer, uint8_t index);

void changeLastestValue(WINp winbuffer, ElementType data);

float avergeWin(WINp winbuffer, float *weight, int size);
uint8_t findPeak(WINp win, int width);
ElementType findMin(WINp win, int width);
ElementType findMax(WINp win, int width);
uint8_t findPeak(WINp win, int width);
void print(WINp winbuffer);

#endif

