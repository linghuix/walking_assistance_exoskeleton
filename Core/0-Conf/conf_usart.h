
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __conf_usart_H
#define __conf_usart_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "device_bsp.h"


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

void USART_init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_UART4_UART_Init(void);
void MX_UART5_UART_Init(void);


#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
