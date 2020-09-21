/*
 * prinf_debug_ITM.h
 *
 *  Created on: Mar 2, 2020
 *      Author: test
 */

#ifndef _PRINF_DEBUG_ITM_H_
#define _PRINF_DEBUG_ITM_H_


void test_ITM_debug(void);

#define MSG_ITM(...) printf (__VA_ARGS__)


/* Definition of error and warning macros */
/* -------------------------------------- */
#	define MSG(...) printf (__VA_ARGS__)


/* Definition of MSG_ERR */
/* --------------------- */
#ifdef DEBUG_ERR_CONSOLE_ON
#    define MSG_ERR(num, string, value) MSG("\r\n%s,%d : 0X%x %s 0X%x\r\n",__FILE__, __LINE__,num, string, value);
#else
#    define MSG_ERR(num, string, value)
#endif


/* Definition of MSG_WAR */
/* --------------------- */
#ifdef DEBUG_WAR_CONSOLE_ON
#    define MSG_WAR(num, string, value) MSG("\r\n%s,%d : 0X%x %s 0X%x \r\n",__FILE__, __LINE__,num, string, value);
#else
#    define MSG_WAR(num, string, value)
#endif

#endif /* 1_FUNC_PRINF_DEBUG_ITM_H_ */
