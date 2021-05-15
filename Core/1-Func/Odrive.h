#include "BSP.h"
#include "func_can.h"
#include <string.h>

int ODrive_Estop_Message(uint8_t ID);
int ODrive_Get_Motor_Error(uint8_t ID);
int ODrive_Get_Encoder_Error(uint8_t ID);
int ODrive_Get_Sensorless_Error(uint8_t ID);
int ODrive_Set_Axis_Node_ID(uint8_t ID,uint8_t setID);
int ODrive_Set_Axis_Requested_State(uint8_t ID,uint8_t State);
int ODrive_Set_Axis_Startup_Config(uint8_t ID,uint8_t State);
int ODrive_Get_Encoder_Estimates(uint8_t ID);
int ODrive_Get_Encoder_Count(uint8_t ID);
int ODrive_Set_Controller_Modes(uint8_t ID,uint8_t Controlmod,uint8_t Inputmod);
int ODrive_Set_Input_Pos(uint8_t ID,float Pos,int16_t Vel,int16_t Current);
int ODrive_Set_Input_Vel(uint8_t ID,float Vel,float Current);
int ODrive_Set_Input_Current(uint8_t ID,float Current);
int ODrive_Set_Velocity_Limit(uint8_t ID,float Vel);
int ODrive_Set_Traj_Vel_Limit(uint8_t ID,float Vel);
int ODrive_Set_Traj_Accel_Limit(uint8_t ID,float Accel);
int ODrive_Set_Traj_Inertia_Limit(uint8_t ID,float Inertia);
int ODrive_Get_IQ(uint8_t ID);
int ODrive_Get_Sensorless_Estimates(uint8_t ID);
int ODrive_Reboot(uint8_t ID);
int ODrive_Get_Vbus_Voltage(uint8_t ID);
int ODrive_Clear_Errors(uint8_t ID);



void Odrive_Init(uint8_t ID);
void Current_conf(uint8_t ID);
TEST current_control(void);
	


