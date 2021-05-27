#ifndef __debug_H
#define __debug_H


#include "conf_usart.h"

//#define DEBUG_WAR_CONSOLE_ON

#define BufferSize 5000
#define PORT huart1


#define BUFF_Printf				//使用缓冲区进行串口 printf 发送
//#define NO_BUFF_Printf
#define KEIL					//使用的环境是keil5



/* Definition of error and warning macros */
/* -------------------------------------- */
#	define MSG(...) printf (__VA_ARGS__)


/* Definition of MSG_ERR */
/* --------------------- */

#    define MSG_ERR(num, string, value) MSG("\r\n%s,%d : 0X%x %s 0X%x\r\n",__FILE__, __LINE__,num, string, value);

/* Definition of MSG_WAR */
/* --------------------- */

#    define MSG_WAR(num, string, value) MSG("\r\n%s,%d : 0X%x %s 0X%x \r\n",__FILE__, __LINE__,num, string, value);

#define INF(...) 					MSG(__VA_ARGS__)
#define IMUMonitor(...) 			MSG(__VA_ARGS__)
#define AOMonitor(...) 				MSG(__VA_ARGS__)
#define POMonitor(...) 				MSG(__VA_ARGS__)
#define INTERFORCE_Monitor(...) 	MSG(__VA_ARGS__)
#define AssisMonitor(...) 			MSG(__VA_ARGS__)
#define ERROR(s,...)				MSG("#ERROR %d# ",s);MSG(__VA_ARGS__);MSG("\t--%s,%d\r\n",__FILE__, __LINE__)
#define TESTOUT(...)				MSG(__VA_ARGS__)

struct Buffer{
	char data[BufferSize];
	uint16_t in;
	uint16_t out;
};



char getDebugBuffer(void);
uint8_t addDebugBuffer(char c);
void debug_init(void);
void debug_IRQ(void);

	#ifdef TEST_ON
	void test_printf(void);
	TEST test_SpeedOfBuffer_printf(void);
	#endif
#endif
