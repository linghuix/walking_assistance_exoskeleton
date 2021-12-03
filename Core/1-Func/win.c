#include <win.h> 
#define winTest(...) //MSG(__VA_ARGS__)




/**
 * author lhx
 *
 * @brief : initalize an WIN object
 * @param	winbuffer - WIN structure pointer
 * @param	data - point to a array for saving data
 * @param	size - the bytes capacity of array
 * @return  None 
 */
void winBuffer(WINp winbuffer, ElementType *data,int size)
{
    
    winbuffer->data = data;
    winbuffer->length = size;
	winbuffer->in = 0;				//光标，光标所在处还未放入数据
} 

 


/**
 * author lhx
 * Nov 16, 2021
 *
 * @brief :	获取光标前一个数据的序列号
 * @param	index - 人体关节角度
 * @param	length- 人体关节角速度
 * @return		
 */
uint16_t getPreIndex(uint16_t index, uint16_t length)
{

	uint16_t previous;

	// index = 0 ?
	if((uint16_t)(index-1) > index) {
		previous = (length-1);
	}
	else{
		previous = (index-1);
	}
	//MMSG("index=%d previous=%d\r\n", index, previous);
	return previous;
}




/**
 * author lhx
 * Nov 16, 2021
 *
 * @brief :	获取光标所在的序列号
 * @param	index - 人体关节角度
 * @param	length- 人体关节角速度
 * @return		
 */
uint16_t getCurIndex(WINp winbuffer)
{
	return winbuffer->in;
}


/**
 * author lhx
 * Nov 16, 2021
 *
 * @brief :	获取光标后一个数据的序列号
 * @param	index - 人体关节角度
 * @param	length- 人体关节角速度
 * @return		
 */

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


/**
 * author lhx
 * Nov 16, 2021
 *
 * @brief :	add data to the WIN structure
 * @param	winbuffer - WIN structure pointer
 * @param	data - one elementtype data
 * @return		
 */
void addToBuff(WINp winbuffer, ElementType data)
{
    
	winbuffer->data[winbuffer->in] = data;		//光标所在处填入数据
	winbuffer->in = getNexIndex(winbuffer->in, winbuffer->length);
}

/** 
 * author lhx
 * Nov 16, 2021
 *
 * @brief :	输出线性表内容  o(n)
 * @param	winbuffer - WIN structure pointer
 * @return		
 */

void print(WINp winbuffer)
{
	
    uint16_t index = winbuffer->in;
    for(uint16_t i=0; i<winbuffer->length; i++){
		index = getPreIndex(index,winbuffer->length);
        TESTOUT("\t%d/%.2f",index , winbuffer->data[index]);
    }
    TESTOUT("\r\n");
}



/**
 * author lhx
 * Nov 16, 2021
 *
 * @brief :	加权平均 WIN 中所有的数据  o(n)
 * @param	winbuffer - WIN structure pointer
 * @param	weight - 加权数组
 * @param	size - WIN length
 * @return		
 */
float avergeWin(WINp winbuffer, float *weight, int size)
{
	
	float sum = 0;
	int pre = winbuffer->in;
	if(size == winbuffer->length){
		for(int i=0; i<winbuffer->length; i++){
			pre = getPreIndex(pre, winbuffer->length);
			sum += weight[i] * winbuffer->data[pre];
		}
	}
	else{
		ERROR(2,"size is not match");
	}
//    winTest("SUM %.2f\r\n", sum);
	return sum;
}



/**
 * author lhx
 * Nov 16, 2021
 *
 * @brief : change a value at index of WIN
 * @param	winbuffer - WIN structure pointer
 * @param	index - 序号，1 表示修改光标前一个数据，2 表示前前一个数据
 * @param	data - data value
 * @return		
 */
void changeValue(WINp winbuffer, int index, ElementType data)
{
	int changeIndex = winbuffer->in;
	for(int i=0; i<index; i++){
		changeIndex = getPreIndex(changeIndex, winbuffer->length);
	}
	winbuffer->data[changeIndex] = data;
}



/**
 * author lhx
 * Nov 16, 2021
 *
 * @brief : change latest value at index of WIN
 * @param	winbuffer - WIN structure pointer
 * @param	data - data value
 * @return		
 */
void changeLastestValue(WINp winbuffer, ElementType data)
{
		
	changeValue(winbuffer, 1, data);
}


/**
 * author lhx
 * Nov 16, 2021
 *
 * @brief : get value at index of WIN
 * @param	winbuffer - WIN structure pointer
 * @param	index - sequential index  1 代表最新放入的数据  2 代表次新放入的数据  注意 index 不能为0 
 * @return		
 */
ElementType getValue(WINp winbuffer, uint8_t index)
	{
		
	int changeIndex = winbuffer->in;

	for(int i=0; i<index; i++){
		changeIndex = getPreIndex(changeIndex, winbuffer->length);
	}
	return winbuffer->data[changeIndex];
}



/**
 * @brief find a peak in window
 * @para  win - windows structure pointer
 * @para  width - the range of value for finding a peak
 * @return 1 represents found
 */
uint8_t findPeak(WINp win, int width)
{
	ElementType end1 = getValue(win,1);
	ElementType mid  = getValue(win,(int) width/2);
	ElementType end2 = getValue(win,width);
	
	if(mid > end1 && mid > end2){
		return 1;
	}
	return 0;
}

/**
 * @brief find a maximum in window
 * @para  win - windows structure pointer
 * @para  width - the range of value for finding a peak
 * @return 1 represents found
 */
ElementType findMax(WINp win, int width)
{
    uint16_t index = getCurIndex(win);
	ElementType max = -5000;
    for(uint16_t i=0; i<width; i++){
		index = getPreIndex(index, win->length);
        if(max < win->data[index]){
			max = win->data[index];
		}
    }
	return max;
}

ElementType findMin(WINp win, int width)
{
    uint16_t index = getCurIndex(win);
	ElementType min = 5000;
    for(uint16_t i=0; i<width; i++){
		index = getPreIndex(index, win->length);
        if(min > win->data[index]){
			min = win->data[index];
		}
    }
	return min;
}


///*  测试代码 
void test_win_buff(void)
	{
		
    WIN winbuffer;
	ElementType data[3] = {0};
	int size = 3;

    
    winBuffer(&winbuffer, data, size);

    
	winTest("\r\n--- input data test ---\r\n");
    for(int i=1;i<=10;i++){
        addToBuff(&winbuffer ,i*3.0);
		winTest("%d - in %d - ", i, winbuffer.in);
		print(&winbuffer);
	}
	print(&winbuffer);
	
	
	winTest("\r\n--- average data test ---\r\n");
	float weights[3] = {0.1, 0.1,0.1};

	float avr = avergeWin(&winbuffer, weights, 3);

	winTest("avrage: %.2f\r\n", avr);
	print(&winbuffer);
		
	
	winTest("\r\n--- change data test ---\r\n");

	changeLastestValue(&winbuffer, avr);
	print(&winbuffer);
	winTest("\r\n--- change Lastest one\r\n");
	changeValue(&winbuffer, 1, 2.0);
	print(&winbuffer);
	winTest("\r\n--- change next Last one\r\n");
	changeValue(&winbuffer, 2, 5.0);
	print(&winbuffer);
	winTest("\r\n--- change next next one\r\n");
	changeValue(&winbuffer, 3, 10.0);
	print(&winbuffer);

	winTest("\r\n--- get data test ---\r\n");
	winTest("2-%.2f\t", getValue(&winbuffer, 2));
	winTest("3-%.2f\t", getValue(&winbuffer, 3));
	winTest("1-%.2f", getValue(&winbuffer, 1));
	
	
	while(1){}
}
