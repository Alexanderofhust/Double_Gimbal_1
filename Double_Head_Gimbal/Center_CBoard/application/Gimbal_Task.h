#ifndef __GIMBALTASK_H
#define __GIMBALTASK_H

#include "main.h"
#include "DataReceivetask.h"
#include "pid.h"
#define RC_GIMBAL_RATIO 0.00005f
#define RC_GIMBAL_YAW_RATIO 0.0005f

#define gimbal_pitch_mid -30

typedef struct{
	Pid_Typedef PidPitchPos;
	Pid_Typedef PidPitchSpeed;
	Pid_Typedef PidPitchCurrent;
	
	FeedForward_Typedef Pitch_FF;
	
	short PitchCurrent;
	float Pitch_Target_Pos;
	float Pitch_Speed_pn;
}Pitch_6020_t;

typedef struct{
	Pid_Typedef PidYawPos;
	Pid_Typedef PidYawSpeed;
	Pid_Typedef PidYawCurrent;
	
	short YawCurrent;
	float Yaw_Target_Pos;
	float Yaw_Target_Speed;
	float Yaw_Speed_pn;
}Yaw_6020_t;

typedef struct{
	char fri_mode;
	char bodan_mode;
	char AutoFire_Flag;
	char Laser_Flag;
	char Gimbal_Flag;
	char Freq_state;
}Gimbal_mode_t;

void get_F(void);
void T_change(void);

void Gimbal_Pose_Loop_Calc(void);
void Gimbal_Current_Calc(void);

void PidGimbalMotor_Init(void);
//static void Pitch_Current_Calc(char Pos_Pid_Type_,char Speed_Pid_Type_);
//static void Yaw_Current_Calc(char Pos_Pid_Type_,char Speed_Pid_Type_);


void Gimbal_task(void *pvParameters);
#endif
