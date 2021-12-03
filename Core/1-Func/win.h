
#ifndef __WIN_H_
#define __WIN_H_


#include <stdio.h> 
#include <stdlib.h>//malloc
#include "BSP.h"

#include "debug.h"

#define ListSize 100 


//元素类型，默认为 int
typedef float ElementType;

//线性表类型定义 
typedef struct WinBuffer *WINp;
typedef struct WinBuffer WIN;

struct WinBuffer{
    
    ElementType *data;     	// 数组data用于存放表结点，结点中的元素的类型设定为int 
    uint16_t length;            	// 当前的顺序表的长度 
	uint16_t in;					// in - 指向下一个要放入的位置
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

