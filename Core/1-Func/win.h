#include <stdio.h> 
#include <stdlib.h>//malloc
#include "BSP.h"


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
void WinBuffer(WINp winbuffer, ElementType *data,int size);
void addToBuff(WINp winbuffer, ElementType data);
uint16_t getPreIndex(uint16_t index, uint16_t length);
void ChangeLastestValue(WINp winbuffer, ElementType data);
float AvergeWin(WINp winbuffer, float *weight, int size);

#define getLastestValue(buff) buff.data[getPreIndex(buff.in,buff.length)]

