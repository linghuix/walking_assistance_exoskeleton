
/*----------------------------------------------------------------------------
  更新日志:
  2018-11-14 V1.0.0:初始版本
  ----------------------------------------------------------------------------*/
/* 包含的头文件 --------------------------------------------------------------*/
#include "conf_can.h"

CAN_HandleTypeDef hcan1;


/**
  * @brief CAN Function Initialization. bps set and mode set
  * @param mode: CAN_MODE_NORMAL/CAN_MODE_LOOPBACK
  * @retval None
  * @note  bps: 36M/(SyncJumpWidth+TimeSeg1+TimeSeg2)/Prescaler		36M/(3+8+1)/3 = 1 MHz
  */
void CANHandle_Init(uint32_t mode)
{
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 3;
	hcan1.Init.Mode = mode;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan1.Init.TTCM = DISABLE;      //时间触发模式 - 时间戳
	hcan1.Init.ABOM = DISABLE;      //自动总线关闭管理模式
	hcan1.Init.AWUM = DISABLE;      //自动唤醒模式
	hcan1.Init.NART = DISABLE;      //禁止自动重发模式
	hcan1.Init.RFLM = DISABLE;      //接收FIFO锁定模式
	hcan1.Init.TXFP = ENABLE;       //发送FIFO顺序优先级
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler()
		;
	}
}
/**
  * @brief CAN Function Initialization. bps set and mode set
  * @param mode: CAN_MODE_NORMAL/CAN_MODE_LOOPBACK
  * @retval None
  * @note  bps: 36M/(SyncJumpWidth+TimeSeg1+TimeSeg2)/Prescaler		36M/(3+8+1)/6 = 500 KHz
  */
void CANHandle_Init_500K(uint32_t mode)
{
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 6;
	hcan1.Init.Mode = mode;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan1.Init.TTCM = DISABLE;      //时间触发模式 - 时间戳
	hcan1.Init.ABOM = DISABLE;      //自动总线关闭管理模式
	hcan1.Init.AWUM = DISABLE;      //自动唤醒模式
	hcan1.Init.NART = DISABLE;      //禁止自动重发模式
	hcan1.Init.RFLM = DISABLE;      //接收FIFO锁定模式
	hcan1.Init.TXFP = ENABLE;       //发送FIFO顺序优先级
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler()
		;
	}
}
/**
  * @brief CAN Function Initialization. bps set and mode set
  * @param mode: CAN_MODE_NORMAL/CAN_MODE_LOOPBACK
  * @retval None
  * @note  bps: 36M/(SyncJumpWidth+TimeSeg1+TimeSeg2)/Prescaler		36M/(3+8+1)/12 = 250 KHz
  */
void CANHandle_Init_250K(uint32_t mode)
{
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 12;
	hcan1.Init.Mode = mode;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan1.Init.TTCM = DISABLE;      //时间触发模式 - 时间戳
	hcan1.Init.ABOM = DISABLE;      //自动总线关闭管理模式
	hcan1.Init.AWUM = DISABLE;      //自动唤醒模式
	hcan1.Init.NART = DISABLE;      //禁止自动重发模式
	hcan1.Init.RFLM = DISABLE;      //接收FIFO锁定模式
	hcan1.Init.TXFP = ENABLE;       //发送FIFO顺序优先级
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler()
		;
	}
}

/**
  * @brief activate and start the function of CAN interrupt
  */
void CAN_ITEnable(void)
{
	if (HAL_CAN_ActivateNotification(&hcan1,
			CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING)
			!= HAL_OK)
	{
		Error_Handler()
		;
	}
}


/**
  * @brief unactivate CAN interrupt
  */
void CAN_ITDISEnable(void)
{
	if (HAL_CAN_DeactivateNotification(&hcan1,
			CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING)
			!= HAL_OK)
	{
		Error_Handler()
		;
	}
}

/************************************************
函数名称 ： CAN_GPIO_Configuration
功    能 ： CAN引脚配置
参    数 ： 无
返 回 值 ： 无
作    者 ： strongerHuang
*************************************************/

/**
 * @author lhx
 *
 * @brief : CAN的硬件功能初始化 including function, filter, and activate interrupt(此时接受到信息能够立即进入中断)
 * @param mode: CAN_MODE_NORMAL/CAN_MODE_LOOPBACK
 * @call CANHandle_Init()
 *       CanFilter_Init()
 *       CAN_ITEnable()
 * Window > Preferences > C/C++ > Editor > Templates.
 */
void MX_CAN1_Init(uint32_t mode)
{
//	MSG_BSTART("can1","conf");
	CANHandle_Init(mode);
	CanFilter_Init(&hcan1, CAN_FILTER_FIFO0);
//	CAN_ITEnable();
//	MSG_ASTART("can1","conf");
}

void MX_CAN1_Test_Init(uint32_t mode)
{
	CANHandle_Init(mode);
	CanFilter_Init(&hcan1, CAN_FILTER_FIFO0);
	CAN_ITEnable();
}



/**
  * @author lhx  May 13, 2020
  * @brief  USART MCU Specific Package Initialization. 库函数提供的底层初始化 hook function:
  * 		GPIO配置，
  * 		开启时钟，
  * 		中断启用设置
  * @calledIn HAL_CAN_Init()，用于硬件初始化
  * @param  canHandle: can structure Handler
  * @retval None
  */ 
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {

        /* configure PB8/9 to CAN1_RX/TX*/
        
	   /*
	    __HAL_RCC_CAN1_CLK_ENABLE();
	    __HAL_RCC_AFIO_CLK_ENABLE();
	    __HAL_RCC_GPIOB_CLK_ENABLE();


	    //CAN GPIO Configuration
	    //PB8     ------> CAN_RX
	    //PB9     ------> CAN_TX
	    
	    GPIO_InitStruct.Pin = GPIO_PIN_8;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    GPIO_InitStruct.Pin = GPIO_PIN_9;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    __HAL_AFIO_REMAP_CAN1_2();
	    
	    //CAN1 interrupt Init 
	    
	    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0,1);
	    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
	    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
	    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		*/
	
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    //CAN GPIO Configuration
    //PA11     ------> CAN_RX
    //PA12     ------> CAN_TX

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*CAN1 interrupt Init*/
    
    //HAL_NVIC_SetPriority(CAN1_TX_IRQn, 1, 1);
    //HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	
  }
}



/**
  * @author lhx  May 13, 2020
  * @brief  USART MCU Specific Package De-Initialization.
  * @calledIn HAL_CAN_deInit()，用于硬件初始化
  * @param  canHandle: can structure Handler
  * @retval None
  */ 
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, CAN_RX_PIN|CAN_TX_PIN);
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
	}
}



/**
  * @author lhx  May 13, 2020
  * @brief  过滤器配置
  * @param  canHandle: can structure Handler
  *         FILTER_FIFO: CAN_FILTER_FIFO0 or CAN_FILTER_FIFO1
  * @retval None
  */ 
void CanFilter_Init(CAN_HandleTypeDef* hcan, uint32_t FILTER_FIFO)
{
  CAN_FilterTypeDef canfilter;

  //use different filter for can1&can2
  if(hcan->Instance == CAN1)
  {
	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

	//Filtered any ID you want here
	canfilter.FilterIdHigh = 0x0000;
	canfilter.FilterIdLow = 0x0000;
	canfilter.FilterMaskIdHigh = 0x0000;
	canfilter.FilterMaskIdLow = 0x0000;

	canfilter.FilterFIFOAssignment = FILTER_FIFO;
	canfilter.FilterActivation = ENABLE;
	canfilter.FilterBank = 0;
  }


  HAL_CAN_ConfigFilter(hcan, &canfilter);
}


