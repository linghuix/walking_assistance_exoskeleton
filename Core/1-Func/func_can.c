/*
 * func_can.c
 *
 *  Created on: Mar 14, 2020
 *      Author: test
 */
#include "func_can.h"



/** 
	error: HAL_ERROR / HAL_OK 
	
**/
Error MX_CANx_send(CAN_HandleTypeDef *phcan, CanTxMsg *msg, MAIL pmailbox)
{
	Error error;
	error = HAL_CAN_AddTxMessage(phcan, (CAN_TxHeaderTypeDef *)& (msg->head), msg->Data, & pmailbox);
	return error;
}

Error MX_CANx_get(CAN_HandleTypeDef *phcan, CanRxMsg *msg, uint32_t FIFO)
{
	Error error;
	error = HAL_CAN_GetRxMessage(phcan, FIFO, (CAN_RxHeaderTypeDef *)& (msg->head), (msg->Data));
	return error;
}


void CAN_Start(CAN_HandleTypeDef *phcan)
{
//	MSG_BSTART("CAN", "start");
	if(HAL_CAN_Start(phcan) != HAL_OK){
		Error_Handler()
	}
//	MSG_ASTART("CAN", "start");
}



#ifdef CAN_TEST

/*
 * author lhx
 *
 * @brief : tttttttttttttttttttttttttesttttttttttttttttttttttttttt
 * Window > Preferences > C/C++ > Editor > Templates.
 */

CanTxMsg txmsg; // CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2;
CanRxMsg rxmsg;
int Flag_receive;
void CAN_Send_test(void)
{
	printf("CAN test beginning ...\r\n");

	uint32_t mailbox = 0;
	uint32_t msg[8]={0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};

	//MX_CAN1_Test_Init(CAN_MODE_LOOPBACK);
	MX_CAN1_Test_Init(CAN_MODE_NORMAL);
	CAN_Start(&hcan1);

	txmsg.head.StdId = 0x002;
	txmsg.head.DLC = 8;
	txmsg.head.RTR = CAN_RTR_DATA;
	txmsg.head.IDE = CAN_ID_STD;

	for(int i = 0;i < 8;i++){
		txmsg.Data[i] = msg[i];
	}
	
	//int canerror;
	int i;
	while(1)
	{
		if(i++ > 500000){//周期 0.133s 的样子
			i = 0;
			mailbox = (mailbox+1)%3;
			MX_CANx_send(&hcan1, &txmsg, mailbox);
			
			printf("send ");
			for(int i = 0;i < 8;i++){
				txmsg.Data[i] = txmsg.Data[i]+1;
				printf("0x%X ",txmsg.Data[i]);
			}
			printf("\r\n");
		}
	}
}


void CAN_Rcv_test(void)
{
	printf("CAN receive test beginning ...\r\n");

	//MX_CAN1_Test_Init(CAN_MODE_LOOPBACK);
	MX_CAN1_Test_Init(CAN_MODE_NORMAL);
	CAN_Start(&hcan1);
	while(1){
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)>0) {
		int canerror = MX_CANx_get(&hcan1, &rxmsg, CAN_RX_FIFO0);
		uint32_t id = rxmsg.head.StdId;

		if(id == 0x80)
			printf("CAN receive -SYNC  ");
		else
			printf("CAN receive -0x%x  ",rxmsg.head.StdId);

		for(int i=0;i<rxmsg.head.DLC;i++){
			printf("-%x",rxmsg.Data[i]);
		}
		printf("  time-%d\r\n",HAL_GetTick());
	}
}
#endif
