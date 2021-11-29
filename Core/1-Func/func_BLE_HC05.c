/**
 * func_BLE.c
 *
 *  Created on: May 25, 2020
 *      Author: test
 */
 #include "func_BLE_HC05.h"

void HC05_Init(void)
{
	MX_USART1_UART_Init();
	MSG("beginning...\r\n");
}


void HC05_config(void)
{
	uint8_t test_data[] = "AT+UART\r\n";
	HAL_UART_Transmit(&HC05_huart, test_data, sizeof(test_data),500);
}

void HC05_send(uint8_t data[], uint8_t size)
{
	HAL_UART_Transmit_IT(&HC05_huart, data, size);
}


uint8_t hardtest_CommandReceive[100], hardtest_receivebyte, hardtest_length;
uint8_t receivebyte, CommandReceive[20], length =0;
void HC05_RcvCmd(void)
{
	MX_USART1_UART_Init();
	
	HAL_UART_Receive_IT(&huart1, &hardtest_receivebyte, 1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);	// �����ж�
//	HAL_UART_Receive_IT(&huart1, &receivebyte, 1);
}

/***********************************************************************************************************************/


/**
  * @brief  ���ڿ����жϻص�����
  * @retval None
  */
struct parasturct{
	
	int para[50];		// �������������������
	uint8_t paranum;	// �������������������
};

extern int PREDICT_TIME;
extern float tao_Ep,fai_Ep,fai_Er,fai_Ef,a[3],b[3],tao_Fp,fai_Fp,fai_Fr,fai_Ff;
struct parasturct para = {0};
void inputPara(struct parasturct * parasturct, uint8_t * paradata, uint8_t length);
TEST USAR_UART1_IDLECallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
//		HAL_UART_Transmit(&huart1, hardtest_CommandReceive, hardtest_length, 500);
		CMD("command %s - command length %d\r\n",hardtest_CommandReceive, hardtest_length );
		inputPara(&para, hardtest_CommandReceive, hardtest_length);
		
		int flag=0;							// Ϊ������������û�����
		for(int i=0;i<para.paranum;i++){
			printf("NO.%d - %d\r\n", i, para.para[i]);
			if(para.para[i] == 0){
				flag++;
			}
		}
		if(flag == 0){
			printf("get number int %d\r\n", para.paranum);
			if ( para.paranum >= 11){
				/* assistive torque curve*/
				tao_Ep = (float)para.para[0]/10.0;		// 5-10 Nm
				fai_Ep = (float)para.para[1]/1000.0; 	// 0.2-0.3
				fai_Er = (float)para.para[2]/1000.0;	// 0.1-0.2
				fai_Ef = (float)para.para[3]/1000.0;	// 0.1-0.2
				
				a[0] = (float)para.para[5]/1000.0;
				a[1] = (float)para.para[6]/1000.0;
				a[2] = (float)para.para[7]/1000.0;
				b[0] = (float)para.para[8]/1000.0;
				b[1] = (float)para.para[9]/1000.0;
				b[2] = (float)para.para[10]/1000.0;
				
				
				
				
				
				tao_Fp = tao_Ep;
				fai_Fp = 0.5 + fai_Ep;
				fai_Fr = fai_Er;	// 0.1-0.2
				fai_Ff = fai_Ef;	// 0.1-0.2
				
				// �Ͼ������ж��ڲ������ܻᷢ���������			
	//			float tmp1 = (fai_Er*fai_Er);
	//			float tmp2 = (fai_Ep*fai_Ep);
	//			float tmp3 = (fai_Ef*fai_Ef);
	//			a[0] = -tao_Ep/tmp1;
	//			a[1] = 2*tao_Ep*fai_Ep/tmp1;
	//			a[2] = tao_Ep - tao_Ep*tmp2/tmp1;
	//			b[0] = -tao_Ep/tmp3;
	//			b[1] = 2*tao_Ep*fai_Ep/tmp3;
	//			b[2] = tao_Ep - tao_Ep*tmp2/tmp3;
				
				/* AO */
				PREDICT_TIME = para.para[4];
				
				CMD("para change : PREDICT_TIME=%d\r\n", PREDICT_TIME);
				CMD("para change : [tao_Ep,fai_Ep,fai_Er,fai_Ef]=[%.3f,%.3f,%.3f,%.3f]\r\n", tao_Ep,fai_Ep,fai_Er,fai_Ef);
				CMD("para change : [a,b]=[[%.3f,%.3f,%.3f],[%.3f,%.3f,%.3f]]\r\n", a[0],a[1],a[2],b[0],b[1],b[2]);
		
			}

		}
		hardtest_length = 0;
	}
}



/**
	* @brief: ���������ַ���"5/6/7/8"������������������֡�֧���������֣�����������Ҫ����
	* @note ���������벻ȫ�ģ�������¶�Ӧ�����������������ģ�����¶�Ӧ������������ٵ���ȫ�ģ�����¡�
	* ��/70�� - ������  ��90/80�� ���� ��89/��������
	* @parameter parasturct:	�洢�������ݽṹ��
	* @parameter paradata:		��������������ַ���
	* @parameter length:		��������������ַ����ֽ���
*/

#include<stdlib.h>

TEST inputPara(struct parasturct * parasturct, uint8_t * paradata, uint8_t length)
{
	uint8_t para[50];						// paradata ��ȡ���������ַ���para ��ȡ����
	
	int index = 0;							// index ���
	parasturct->paranum = 0;				// paranum ��������
	
	for(int i=0; i<length; i++){
				
		if(paradata[i] == '/'){
			para[index] = '\0';								// para[]�ַ���Ϊһ������
			parasturct->para[parasturct->paranum++] = atoi( (char *)para );			// �ַ���ת����
			index = 0;
		}
		else if(paradata[i] == '\n'){
			parasturct->para[parasturct->paranum++] = atoi( (char *)para );
			break;
		}
		else{
			para[index++] = paradata[i];
		}
	}
	
//	uint8_t output[2];						//�������
//	for(int i=0;i<parasturct->paranum;i++){

//		HAL_UART_Transmit(&huart1, (uint8_t *)&parasturct->para[i], sizeof(output), 500);
//		output[0] = 0xFF;
//		HAL_UART_Transmit(&huart1, output, 1, 500);
//	}
}

			/**
			  * @brief 
			  * @note 	1. need to put into stm32f10x_it.c
			  * 
			  * @retval None
			  */
TEST IDLE_UART_IRQHandler (UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   // IDLE 
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);                     
            USAR_UART1_IDLECallback(huart);                        
        }
	}
}



uint8_t HardwareTestData[7] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77};
TEST test_USART1_communication(void)
{
	MX_USART1_UART_Init();

	HAL_UART_Transmit_IT(&huart1, HardwareTestData, 6);
//	
//	HAL_UART_Receive_IT(&huart1, HardwareTestData, 1);
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);	// �����ж�
}


