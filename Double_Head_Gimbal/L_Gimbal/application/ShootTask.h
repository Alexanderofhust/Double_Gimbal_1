#ifndef __SHOOTTASK_H
#define __SHOOTTASK_H

#include "main.h"
#include "DataReceivetask.h"
#include "pid.h"
#define   	PluckThreholdPos        	5000    
#define		ShootInterval_Min_time 		40 //最快连射的速度是50ms一发(连射情况)
#define 	OneGrid 					36864.0f

typedef struct{
	short BodanMotorCurrent;
	Pid_Typedef PidBodanMotorPos;
	Pid_Typedef PidBodanMotorSpeed;
	float Onegrid;
}BODAN_MOTOR_t;

typedef struct{
	uint8_t Friction_cmd;
	uint8_t Shoot_State; 
	uint8_t Shoot_State_send;
	uint8_t Shoot_Freq_cmd;
}Shoot_Cmd_t;

typedef struct{
	struct{
		float ShootCount_IntervalTime;//子弹射击估计时上一颗子弹打出至今的时间
		float ShootContinue_IntervalTime;//连射时上一颗子弹打出至今的时间
		char HeatUpdateFlag;//热量信息更新
		char CurShootNumber;//还未处理热量的子弹数
		char IsShootAble;//可以打弹
		float HeatMax17, HeatCool17;
		float CurHeat17, LastHeat17;
		uint16_t Judge_Heat_recieve;
		
	}HeatControl;
	
	uint16_t remain_bullet_num;
	char ReverseRotation;
	char ShootContinue;
	short PullerSpeed ;
}ShootTask_typedef;

typedef struct{
	Pid_Typedef PidFrictionSpeed;
	short FrictionCurrent;
	short FrictionWheel_speed;
}Frition_Wheel_t;

extern Frition_Wheel_t Frition_Wheel_Motor[2];

void FrictionWheel_Configration(void);
void FrictionWheel_Set(short speed);

void FrictionSpeedChoose(void);
void ShootCount_Cal(void);
void HeatControl(float dt);
void Shoot_Speed_Loop_Calc(short Bodan_speed_set);
void Shoot_Pos_Loop_Calc(float Bodan_pos_set);

void Shoot_Check_Cal(void);
void Shoot_Fire_Cal(void);
void Shoot_Powerdown_Cal(void);
void Shoot_PC_Act(void);
void Shoot_Test_Cal(void);
void Shoot_init();

void BodanMotor_Loop_Calc(void);
void Pid_BodanMotor_Init(void);
void Pid_Friction_Init(void);
void Shoot_task_(void);

void Bodan_Motor_Back(void);


#endif
