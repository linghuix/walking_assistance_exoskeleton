
/**
  * @brief  	ChengTian Odrive motor driver.
  * @hardware 	CAN口
  *	@			D:\10-Chip_learning\0-硬件\3-motor\电机控制项目\0-程天电机\新版驱动器
  * @condition 	24V-36V
  * @year 		2021/05/11
  */


#include "Odrive.h"


//#define ODRIVE_TEST


CAN_TxHeaderTypeDef CAN1_TxHeader;

extern CAN_HandleTypeDef hcan1;
int ODrive_Estop_Message(uint8_t ID)
{
	uint32_t TxMailbox;
	CAN1_TxHeader.StdId = ID<<5|0x02;                           //32位ID
	uint8_t* msg;
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=0; 
	CAN1_TxHeader.RTR = 1;
	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Get_Motor_Error(uint8_t ID)
{
	uint32_t TxMailbox;
	CAN1_TxHeader.StdId = ID<<5|0x03;                           //32位ID
	uint8_t* msg;
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=0; 
	CAN1_TxHeader.RTR = 1;
	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Get_Encoder_Error(uint8_t ID)
{
	uint32_t TxMailbox;
	CAN1_TxHeader.StdId = ID<<5|0x04;                           //32位ID
	uint8_t* msg;
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=0; 
	CAN1_TxHeader.RTR = 1;
	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Get_Sensorless_Error(uint8_t ID)
{
	uint32_t TxMailbox;
	CAN1_TxHeader.StdId = ID<<5|0x05;                           //32位ID
	uint8_t* msg;
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=0; 
	CAN1_TxHeader.RTR = 1;
	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Set_Axis_Node_ID(uint8_t ID,uint8_t setID)
{
	uint32_t TxMailbox;
	CAN1_TxHeader.StdId = ID<<5|0x06;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=8; 
//	CAN1_TxHeader.RTR = 1;


	msg[0] = setID;
	msg[1] = 0x00;
	msg[2] = 0x00;
	msg[3] = 0x00;


	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Set_Axis_Requested_State(uint8_t ID,uint8_t State)
{
	uint32_t TxMailbox;
	CAN1_TxHeader.StdId = ID<<5|0x07;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=8; 
//	CAN1_TxHeader.RTR = 1;


	msg[0] = State;
	msg[1] = 0x00;
	msg[2] = 0x00;
	msg[3] = 0x00;


	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}


int ODrive_Set_Axis_Startup_Config(uint8_t ID,uint8_t State)
{
	uint32_t TxMailbox;
	CAN1_TxHeader.StdId = ID<<5|0x08;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=8; 
//	CAN1_TxHeader.RTR = 1;


	msg[0] = State;
	msg[1] = 0x00;
	msg[2] = 0x00;
	msg[3] = 0x00;


	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}


int ODrive_Get_Encoder_Estimates(uint8_t ID)
{
	uint32_t TxMailbox;
	CAN1_TxHeader.StdId = ID<<5|0x09;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=0; 
	CAN1_TxHeader.RTR = 1;


//	msg[0] = State;
//	msg[1] = 0x00;
//	msg[2] = 0x00;
//	msg[3] = 0x00;


	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}


int ODrive_Get_Encoder_Count(uint8_t ID)
{
	uint32_t TxMailbox;
	CAN1_TxHeader.StdId = ID<<5|0x0A;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=0; 
	CAN1_TxHeader.RTR = 1;


//	msg[0] = State;
//	msg[1] = 0x00;
//	msg[2] = 0x00;
//	msg[3] = 0x00;


	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Set_Controller_Modes(uint8_t ID,uint8_t Controlmod,uint8_t Inputmod)
{
	uint32_t TxMailbox;
	CAN1_TxHeader.StdId = ID<<5|0x0B;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=8; 
//	CAN1_TxHeader.RTR = 1;


	msg[0] = Controlmod;
	msg[1] = 0x00;
	msg[2] = 0x00;
	msg[3] = 0x00;
	msg[4] = Inputmod;
	msg[5] = 0x00;
	msg[6] = 0x00;
	msg[7] = 0x00;

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Set_Input_Pos(uint8_t ID,float Pos,int16_t Vel,int16_t Current)
{
	uint32_t TxMailbox;
	
	uint32_t floatBytes;
//	static_assert(sizeof pos == sizeof floatBytes);
	memcpy(&floatBytes, &Pos, sizeof floatBytes);
	
	CAN1_TxHeader.StdId = ID<<5|0x0C;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=8; 
//	CAN1_TxHeader.RTR = 1;


	msg[0] = floatBytes;;
	msg[1] = floatBytes >> 8;
	msg[2] = floatBytes >> 16;
	msg[3] = floatBytes >> 24;
	msg[4] = Vel%0x0100;
	msg[5] = Vel/0x0100;
	msg[6] = Current%0x0100;
	msg[7] = Current/0x0100;

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Set_Input_Vel(uint8_t ID,float Vel,float Current)
{
	uint32_t TxMailbox;
	
	uint32_t floatBytes;
//	static_assert(sizeof pos == sizeof floatBytes);
	memcpy(&floatBytes, &Vel, sizeof floatBytes);
	
	CAN1_TxHeader.StdId = ID<<5|0x0D;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=8; 
//	CAN1_TxHeader.RTR = 1;


	msg[0] = floatBytes;;
	msg[1] = floatBytes >> 8;
	msg[2] = floatBytes >> 16;
	msg[3] = floatBytes >> 24;
	
	memcpy(&floatBytes, &Current, sizeof floatBytes);
	
	msg[4] = floatBytes;;
	msg[5] = floatBytes >> 8;
	msg[6] = floatBytes >> 16;
	msg[7] = floatBytes >> 24;

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Set_Input_Current(uint8_t ID,float Current)
{
	uint32_t TxMailbox;
	
	uint32_t floatBytes;
//	static_assert(sizeof pos == sizeof floatBytes);
	memcpy(&floatBytes, &Current, sizeof floatBytes);
	
	CAN1_TxHeader.StdId = ID<<5|0x0E;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=8; 
//	CAN1_TxHeader.RTR = 1;


	msg[0] = floatBytes;
	msg[1] = floatBytes >> 8;
	msg[2] = floatBytes >> 16;
	msg[3] = floatBytes >> 24;
	msg[4] = 0x00;
	msg[5] = 0x00;
	msg[6] = 0x00;
	msg[7] = 0x00;	
	

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Set_Velocity_Limit(uint8_t ID,float Vel)
{
	uint32_t TxMailbox;
	
	uint32_t floatBytes;
//	static_assert(sizeof pos == sizeof floatBytes);
	memcpy(&floatBytes, &Vel, sizeof floatBytes);
	
	CAN1_TxHeader.StdId = ID<<5|0x0F;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=8; 
//	CAN1_TxHeader.RTR = 1;


	msg[0] = floatBytes;;
	msg[1] = floatBytes >> 8;
	msg[2] = floatBytes >> 16;
	msg[3] = floatBytes >> 24;
	

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}


int ODrive_Set_Traj_Vel_Limit(uint8_t ID,float Vel)
{
	uint32_t TxMailbox;
	
	uint32_t floatBytes;
//	static_assert(sizeof pos == sizeof floatBytes);
	memcpy(&floatBytes, &Vel, sizeof floatBytes);
	
	CAN1_TxHeader.StdId = ID<<5|0x11;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=8; 
//	CAN1_TxHeader.RTR = 1;


	msg[0] = floatBytes;;
	msg[1] = floatBytes >> 8;
	msg[2] = floatBytes >> 16;
	msg[3] = floatBytes >> 24;
	

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}


int ODrive_Set_Traj_Accel_Limit(uint8_t ID,float Accel)
{
	uint32_t TxMailbox;
	
	uint32_t floatBytes;
//	static_assert(sizeof pos == sizeof floatBytes);
	memcpy(&floatBytes, &Accel, sizeof floatBytes);
	
	CAN1_TxHeader.StdId = ID<<5|0x12;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=8; 
//	CAN1_TxHeader.RTR = 1;


	msg[0] = floatBytes;;
	msg[1] = floatBytes >> 8;
	msg[2] = floatBytes >> 16;
	msg[3] = floatBytes >> 24;
	

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}


int ODrive_Set_Traj_Inertia_Limit(uint8_t ID,float Inertia)
{
	uint32_t TxMailbox;
	
	uint32_t floatBytes;
//	static_assert(sizeof pos == sizeof floatBytes);
	memcpy(&floatBytes, &Inertia, sizeof floatBytes);
	
	CAN1_TxHeader.StdId = ID<<5|0x13;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=8; 
//	CAN1_TxHeader.RTR = 1;


	msg[0] = floatBytes;;
	msg[1] = floatBytes >> 8;
	msg[2] = floatBytes >> 16;
	msg[3] = floatBytes >> 24;
	

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Get_IQ(uint8_t ID)
{
	uint32_t TxMailbox;
	

	CAN1_TxHeader.StdId = ID<<5|0x14;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=0; 
	CAN1_TxHeader.RTR = 1;
	

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Get_Sensorless_Estimates(uint8_t ID)
{
	uint32_t TxMailbox;
	

	CAN1_TxHeader.StdId = ID<<5|0x15;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=0; 
	CAN1_TxHeader.RTR = 1;
	

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Reboot(uint8_t ID)
{
	uint32_t TxMailbox;
	

	CAN1_TxHeader.StdId = ID<<5|0x16;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=0; 
	CAN1_TxHeader.RTR = 1;
	

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Get_Vbus_Voltage(uint8_t ID)
{
	uint32_t TxMailbox;
	

	CAN1_TxHeader.StdId = ID<<5|0x17;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=0; 
	CAN1_TxHeader.RTR = 1;
	

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_Clear_Errors(uint8_t ID)
{
	uint32_t TxMailbox;
	

	CAN1_TxHeader.StdId = ID<<5|0x18;                           //32位ID
	uint8_t msg[8];
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=0; 
	CAN1_TxHeader.RTR = 1;
	

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}

int ODrive_SetID(uint8_t oldID, uint8_t newID)
{
	uint32_t TxMailbox;
	

	CAN1_TxHeader.StdId = oldID<<5|0x06;                           //32位ID
	uint8_t msg[8];

	msg[0] = newID;;
//	msg[1] = newID >> 8;
//	msg[2] = newID >> 16;
//	msg[3] = newID >> 24;
	
	CAN1_TxHeader.IDE=CAN_ID_STD;                  //标准ID
	CAN1_TxHeader.RTR=CAN_RTR_DATA;
	CAN1_TxHeader.DLC=8; 
	CAN1_TxHeader.RTR = 1;
	

	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, msg, &TxMailbox) != HAL_OK)
	{
		/* Transmission request Error */
		Error_Handler()
		//发送失败
	}
	return 1;
}





#define            CONTROL_MODE_VOLTAGE_CONTROL      0
#define            CONTROL_MODE_TORQUE_CONTROL       1 //力矩模式
#define            CONTROL_MODE_VELOCITY_CONTROL     2
#define            CONTROL_MODE_POSITION_CONTROL     3

#define            INPUT_MODE_INACTIVE               0
#define            INPUT_MODE_PASSTHROUGH            1 //直接执行
#define            INPUT_MODE_VEL_RAMP               2
#define            INPUT_MODE_POS_FILTER             3
#define            INPUT_MODE_MIX_CHANNELS           4
#define            INPUT_MODE_TRAP_TRAJ              5
#define            INPUT_MODE_TORQUE_RAMP            6
#define            INPUT_MODE_MIRROR                 7



// 电流模式配置
void Current_conf(uint8_t ID){
	
	ODrive_Clear_Errors(ID);
	ODrive_Set_Controller_Modes(ID, CONTROL_MODE_TORQUE_CONTROL, INPUT_MODE_PASSTHROUGH);

}

void Vel_conf(uint8_t ID){
	
	ODrive_Clear_Errors(ID);
	ODrive_Set_Controller_Modes(ID, CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
}

void Pos_conf(uint8_t ID){
	
	ODrive_Clear_Errors(ID);
	ODrive_Set_Controller_Modes(ID, CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_PASSTHROUGH);
}

int j;
void Odrive_Init(uint8_t ID){
	TESTOUT("can init\r\n");
	MX_CAN1_Init(CAN_MODE_NORMAL);
	CAN_Start(&hcan1);
	
	TESTOUT("can finish\r\n");
	TESTOUT("odrive reboot\r\n");
	ODrive_Reboot(ID);
	HAL_Delay(10000);
	TESTOUT("odrive reboot finish\r\n");
	Current_conf(ID);
}

#ifdef ODRIVE_TEST

#include "math.h"

uint8_t ID_lefthip_odriver = 0x1;
uint8_t ID_righthip_odriver = 0x2;


TEST right_current_control(void)
{
	Odrive_Init(ID_righthip_odriver);
	Current_conf(ID_righthip_odriver);

	
	for (int i=0;i<20000;i++){
		
		ODrive_Set_Input_Current(ID_righthip_odriver, 0.3*cos(i*0.5));		//left postive current value makes the leg move forward. 
		HAL_Delay(100);
	}										//right postive current value makes the leg move backward
	while(1);
}


TEST left_current_control(void)
{
	Odrive_Init(ID_lefthip_odriver);
	Current_conf(ID_lefthip_odriver);
	
	for (int i=0;i<20000;i++){
		
		ODrive_Set_Input_Current(ID_lefthip_odriver, 0.3*cos(i*0.5));		//left postive current value makes the leg move forward. 
		HAL_Delay(100);
	}										//right postive current value makes the leg move backward
	while(1);
}


TEST pos_control(void)
{
	Odrive_Init(ID_righthip_odriver);
	
	HAL_Delay(10);
	Pos_conf(ID_righthip_odriver);
	
	for (int i=0;i<20000;i++){
		
		ODrive_Set_Input_Current(ID_righthip_odriver, 0.4*sin(i*0.5));		//left postive current value makes the leg move forward. 

		HAL_Delay(100);
	}										//right postive current value makes the leg move backward
	while(1);
}

#endif
