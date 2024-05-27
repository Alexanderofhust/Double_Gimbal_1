#ifndef __CHASSISTASK_H__
#define __CHASSISTASK_H__
#include "main.h"
#include "DataReceiveTask.h"

typedef struct 
{
	short carSpeedx;
	short carSpeedy;
	short carSpeedw;
} ChassisSpeed_t;

typedef enum{
	CHASSIS_NONE_FLAG = 0,
	CHASSIS_FOLLOW_FLAG = 1,
	CHASSIS_PROTECT_FLAG = 2
}CHASSIS_W_MODE_ENUM;

#define RC_SPEED_RATIO 4


void Chassis_Axis_Trans(short* x_,short* y_,float SinTheTa_,float CosTheTa_);

void Chassis_Powerdown_Cal(void);
//void Chassis_Act_Cal(Remote rc,Key key,CHASSIS_W_MODE_ENUM Chassis_w_Flag);


float ChassisPostionAngle_TranSform(short InitPos);

void Pid_ChassisPosition_Init(void);

void Chassis_Speed_Calc(void);

void Current_Filter_Excu(void);
void Current_Set_Jump(void);

void Chassis_Flag_Set(void);
void Chassis_task(void const *pvParameters);


#endif
