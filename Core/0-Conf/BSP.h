#ifndef __BSP_H
#define	__BSP_H

#include "conf_usart.h"
#include "conf_tim.h"
#include "conf_usart.h"
#include "conf_rtc.h"
#include "conf_Jlink.h"
#include "conf_can.h"
#include "conf_global.h"


void SysTick_init(void);
void BSP_Init(void);

#endif // __BSP_H
