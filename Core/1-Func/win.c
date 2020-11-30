#include <win.h> 

//创建
void WinBuffer(WINp winbuffer, ElementType *data,int size)
	{
    
    winbuffer->data = data;
    winbuffer->length = size;
	winbuffer->in = 0;
} 

 

uint16_t getPreIndex(uint16_t index, uint16_t length)
{

	uint16_t previous;
	if((uint16_t)(index-1) > index) {
		previous = (length-1);
	}
	else{
		previous = (index-1);
	}
	//MMSG("index=%d previous=%d\r\n", index, previous);
	return previous;
}

uint16_t getNexIndex(uint16_t index, uint16_t length)
{

	uint16_t next;
	if((uint16_t)(index+1) == length) {
		next = 0;
	}
	else{
		next = (index+1);
	}
	//MMSG("index=%d previous=%d\r\n", index, previous);
	return next;
}


void addToBuff(WINp winbuffer, ElementType data)
{
    
	winbuffer->data[winbuffer->in] = data;
	winbuffer->in = getNexIndex(winbuffer->in, winbuffer->length);
}


//输出线性表内容  o(n)
void print(WINp winbuffer)
{
	
    uint16_t index = winbuffer->in;
    for(uint16_t i=0; i<winbuffer->length; i++){
		index = getPreIndex(index,winbuffer->length);
        printf("%d/%.2f\t",index , winbuffer->data[index]);
    }
    printf("\r\n");
}

float AvergeWin(WINp winbuffer, float *weight, int size)
	{
	
	float sum = 0;
	int pre = winbuffer->in;
	if(size == winbuffer->length){
		for(int i=0; i<winbuffer->length; i++){
			pre = getPreIndex(pre, winbuffer->length);
			sum += weight[i] * winbuffer->data[pre];
		}
	}
//    printf("SUM %.2f\r\n", sum);
	return sum;
}


void ChangeValue(WINp winbuffer, int index, ElementType data)
	{
	int changeIndex = winbuffer->in;
	for(int i=0; i<index; i++){
		changeIndex = getPreIndex(changeIndex, winbuffer->length);
	}
	winbuffer->data[changeIndex] = data;
}

void ChangeLastestValue(WINp winbuffer, ElementType data)
	{
		
	ChangeValue(winbuffer, 1, data);
}

/*
 * index: 1 代表最新放入的数据  2 代表次新放入的数据 
 */
ElementType GetValue(WINp winbuffer, uint8_t index)
	{
		
	int changeIndex = winbuffer->in;
	for(int i=0; i<index; i++){
		changeIndex = getPreIndex(changeIndex, winbuffer->length);
	}
	return winbuffer->data[changeIndex];
}

///*  测试代码 */
void test_win_buff(void)
	{
		
    WIN winbuffer;
	ElementType data[2] = {0};
	int size = 2;

    
    WinBuffer(&winbuffer, data, size);
    
	printf("--- input data test ---\r\n");
    for(int i=1;i<=10;i++){
        addToBuff(&winbuffer ,i*2.0);
		printf("%d - in %d - ", i, winbuffer.in);
		print(&winbuffer);
	}
	print(&winbuffer);
	
	
	printf("--- average data test ---\r\n");
	float weights[2] = {0.1, 0.1};
	float avr = AvergeWin(&winbuffer, weights, 2);
	printf("avrage : %.2f\r\n", avr);
	print(&winbuffer);
		
	
	printf("--- change data test ---\r\n");
	ChangeLastestValue(&winbuffer, avr);
	print(&winbuffer);

	
	while(1){}
}
