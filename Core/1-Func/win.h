#include <stdio.h> 
#include <stdlib.h>//malloc
#include "BSP.h"


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
void WinBuffer(WINp winbuffer, ElementType *data,int size);
void addToBuff(WINp winbuffer, ElementType data);
uint16_t getPreIndex(uint16_t index, uint16_t length);
void ChangeLastestValue(WINp winbuffer, ElementType data);
float AvergeWin(WINp winbuffer, float *weight, int size);

#define getLastestValue(buff) buff.data[getPreIndex(buff.in,buff.length)]

