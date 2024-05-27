#ifndef __DATARECEIVETASK_H__
#define __DATARECEIVETASK_H__
#include "main.h"
#include "stm32f4xx.h"
#include "struct_typedef.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/*遥控器结构体*/
typedef __packed struct
{
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned short s1;
		unsigned short s2;
}Remote;
/*鼠标结构体*/
typedef __packed struct
{
		short x;
		short y;
		short z;
		unsigned char press_l;
		unsigned char press_r;
}Mouse;
/*键盘结构体*/
typedef __packed struct
{
		unsigned short w,s,a,d,q,e,r,f,g,z,x,c,v,b,shift,ctrl;
}Key;
/*遥键鼠结构体综合*/
typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
	char RCrecvd,RCDisconnectCnt;//RCrecvd为数据接收标志位
}RC_Ctl_t;

typedef enum{
	REMOTE_UP = 1,
	REMOTE_DOWN = 2,
	REMOTE_MID = 3
}remote_value;

/*陀螺仪接收结构体*/
typedef struct GYRO{
		float GX;
		float GY;				//陀螺仪速度
		float GZ;
		float PITCH;		//陀螺仪角度
		float YAW;
		float ROLL;
}
Gyro_Typedef;
/*底盘接收结构体*/
typedef struct F105{
	short ChassisSpeedw;
	short Remain_power;
    char IsShootAble;
	char RobotRed;
    char BulletSpeedLevel;
	
    float Limit_Power_k;
	
	short HP;
	short Last_HP;
	float bulletSpeed;
	uint8_t bulletFreq;
	uint16_t HeatCool17;		//17mm枪口每秒冷却值
	uint16_t HeatMax17;			//17mm枪口热量上限
	short shooterHeat17;
	uint8_t RobotLevel;
    uint8_t whichbalance;
}F105_Typedef;

#pragma pack(push, 1)
typedef struct{
	uint8_t is_game_start : 1;
	uint8_t Heat_update : 1;
	uint8_t Robot_Red_Blue : 1; //1 -> red ; 0 -> blue
	uint8_t Enemy_Sentry_shootable : 1; //敌方哨兵是否无敌
	uint16_t self_outpost : 11;
	uint8_t Sentry_HomeReturned_flag : 1;
	uint16_t shooter1_heat;
	uint16_t bullet_remaining_num_17mm; //0x208
	uint16_t stage_remain_time; //0x0001
}JudgeData_1_t;

typedef struct{
	uint16_t x; //裁判系统给的机器人坐标(从float 映射到 uint16_t : float*100 -> uint16_t)
	uint16_t y;
	uint8_t commd_keyboard; //云台手指令
	uint8_t Base_Shield;
	uint8_t KeyBoard_Update : 1;
	uint8_t _ : 7;
	uint8_t __;
}JudgeData_2_t;
#pragma pack(pop)

typedef struct
{
	float RCPitch;
	float RCYaw;
	float RCdistance;
	short ReceiveFromTx2BullectCnt;
	short FrictionWheel_speed;
	short DisConnect;
}PC_Receive_t;

typedef struct{
	short Angle;
	short RealSpeed;  
  short Current;	
}BodanMotorReceive_Typedef;

typedef struct{
unsigned int RC_DisConnect;//遥控器掉线检测
unsigned int F105_DisConect;//功率板掉线检测
unsigned int PitchMotor_DisConnect;
unsigned int YawMotor_DisConnect;
unsigned int Gyro_DisConnect;
unsigned int Friction_DisConnect[2];
unsigned int Pluck_DisConnect;
unsigned int PC_DisConnect;
}Disconnect;

void RemoteReceive(volatile unsigned char rx_buffer[]);

void F105_Rst(void);

void RCReceive_task(void const * argument);
void RC_Ctl_t_Init(RC_Ctl_t *RC_Ctl);


extern RC_Ctl_t RC_Ctl;

enum ARMOR_ID
{
    ARMOR_AIM_LOST = 0,
    ARMOR_ID_1,
    ARMOR_ID_2,
    ARMOR_ID_3,
    ARMOR_ID_4,
    ARMOR_ID_5,
    ARMOR_ID_Sentry,
};

#endif
