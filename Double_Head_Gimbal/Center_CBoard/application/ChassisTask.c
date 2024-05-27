//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!发给底盘的线速度和角速度都应该有严格的物理单位，线速度是mm/s，角速度为mrad/s!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#include "main.h"
/*----------------------------------内部变量---------------------------*/
short ChassisAct_Init_Flag=0;
static CHASSIS_MODE_ENUM Last_Chassis_Mode;
float Theta,SinTheTa,CosTheTa,TanTheTa,Theta0,Speed_Theta;
float Theta_chassis;
char  SelfProtect_Cross_Flag;
static float ResetPos;
short Be_shooted_flag;

const short FollowMaxSpeedw = 2000;			//跟随最高转速
const short RotateMaxSpeedw = 6000;			//小陀螺最高转速
/*----------------------------------结构体-----------------------------*/
ChassisSpeed_t chassis;
Pid_Typedef pidChassisPosition,pidChassisPosition_Speed;
Pid_Typedef SOLO_pidChassisPosition;
FeedForward_Typedef FF_w;
/*----------------------------------外部变量---------------------------*/
extern Gimbal_Typedef Gimbal;
extern F105_Typedef F105;
extern F405_typedef F405;
extern short Turning_flag;
extern float YawMotorReceive;
static CHASSIS_MODE_ENUM Last_Chassis_state = CHASSIS_POWER_DOWN_MODE;
/**********************************************************************************************************
*函 数 名: Chassis_Axis_Trans
*功能说明: 旋转矩阵乘法的简单实现
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
inline void Chassis_Axis_Trans(short* x_,short* y_,float SinTheTa_,float CosTheTa_)
{
	short x_temp = *x_;
	short y_temp = *y_;
	*x_ = -y_temp * SinTheTa_ + x_temp * CosTheTa_;
	*y_ = x_temp * SinTheTa_ + y_temp * CosTheTa_;
}

/**********************************************************************************************************
*函 数 名: Chassis_Powerdown_Cal
*功能说明: 锁车模式
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_Powerdown_Cal()
{
	chassis.carSpeedx=0;
	chassis.carSpeedy=0;
	chassis.carSpeedw=0;
	Theta_chassis = ChassisPostionAngle_TranSform(Sentry.Yaw_init)/360.0f*6.28318f;
	Last_Chassis_Mode = Inf_All_State.Chassis_Mode;
}

/**********************************************************************************************************
*函 数 名: Chassis_Act_Cal
*功能说明: 正常模式
*形    参: rc  key
*返 回 值: 无
**********************************************************************************************************/
float Chassis_w_estimate;
float Speed_Follow_P;
float Chassis_w_set;
void Chassis_Act_Cal(Remote rc,Key key,CHASSIS_W_MODE_ENUM Chassis_w_Flag) 
{
	
	static float SinTheTa,CosTheTa;  //这种东西作用域最好放在函数里面
	extern Yaw_6020_t Yaw_Moto;
	extern short YawMotorSpeed;
	extern float Yaw_Nav_Aim_Center_Pose;
	if(Last_Chassis_Mode!=CHASSIS_RC_FOLLOW_MODE)
	{
		Speed_Follow_P = 0.7f;
		chassis.carSpeedw = 0;
		chassis.carSpeedx = 0;
		chassis.carSpeedy = 0;
		if(0 == RC_Ctl.rc.ch1 || 0 == RC_Ctl.rc.ch0)
		{
			return;
		}
		Last_Chassis_Mode = Inf_All_State.Chassis_Mode;
	}
	Theta= -ChassisPostionAngle_TranSform(Sentry.Yaw_init)/360.0f*6.28318f;  //弧度
    Theta_chassis = -ChassisPostionAngle_TranSform(Sentry.Yaw_init)/360.0f*6.28318f;
	
	if(CHASSIS_PROTECT_FLAG == Chassis_w_Flag)
		Theta += 3.14f/6; //补偿小陀螺
	
	Theta = 0;
	CosTheTa=cos(Theta);
	SinTheTa=sin(Theta);
	
	if(Inf_All_State.Global_Mode != INF_NAV_PLUS_AIM_MODE)
	{
			if((-1024+RC_Ctl.rc.ch1)>300)
				chassis.carSpeedx= (-1024+RC_Ctl.rc.ch1)*RC_SPEED_RATIO;
			else if((-1024+RC_Ctl.rc.ch1)<-300)
				chassis.carSpeedx= (-1024+RC_Ctl.rc.ch1)*RC_SPEED_RATIO;
			else
				chassis.carSpeedx= 0; 

			if((-1024+RC_Ctl.rc.ch0)>300)
				chassis.carSpeedy= (-1024+RC_Ctl.rc.ch0)*RC_SPEED_RATIO; 
			else if((-1024+RC_Ctl.rc.ch0)<-300)
				chassis.carSpeedy= (-1024+RC_Ctl.rc.ch0)*RC_SPEED_RATIO; 
			else
				chassis.carSpeedy= 0; 
	}
	
	else 
	{
		chassis.carSpeedx= -(NAV_cmd.Nav_Speed_x);
		chassis.carSpeedy= -(NAV_cmd.Nav_Speed_y); 
		//chassis.carSpeedw = 3000.0f;
//		if((1024 - rc.ch2)>200)
//			chassis.carSpeedw = 3600 * (1024.0f - rc.ch2) / 1024.0f;
//		else if((1024 - rc.ch2)<200)
//		{
//			chassis.carSpeedw = 3600 * (1024.0f - rc.ch2) / 1024.0f;
//		}
//		else
//		{
//			chassis.carSpeedw = 0;
//		}
	}
	
	Chassis_Axis_Trans(&chassis.carSpeedx, &chassis.carSpeedy, SinTheTa, CosTheTa); //传实参有点危险，到时候看看要不要改
	//上面的代码负责从遥控器解码控制信息，以及进行坐标系变换
	//下面的部分通过小陀螺标志位选择底盘角速度
	
	if(CHASSIS_PROTECT_FLAG == Chassis_w_Flag)
	{
		chassis.carSpeedw = 2000; //for test setting w as 0.5rad/s
	}
	
	else if(CHASSIS_FOLLOW_FLAG == Chassis_w_Flag)
	{
		
	}
	
	else
	{
		chassis.carSpeedw = 0;
	}
	
}
/**********************************************************************************************************
*函 数 名: ChassisPostionAngle_TranSform
*功能说明: 角度转换函数
           由初始角度和Zero_CheckYawPosition()来计算-180~180的角度
           用过零检测后的值和初始化的Yaw来做位置差获取角度差
           考虑过零检测的Yaw是否大于初始化的值，大于初始化的角度值时TheTa为正，否则为负；
           并且规定TheTa值的范围为与初始值的较小角度
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/

void Chassis_MouseKey_Cal(Key key)
{
	static float SinTheTa,CosTheTa;  //这种东西作用域最好放在函数里面
	//Theta_chassis = ChassisPostionAngle_TranSform(Sentry.Yaw_init)/360.0f*6.28318f;
	Theta=ChassisPostionAngle_TranSform(Sentry.Yaw_init)/360.0f*6.28318f;
	
	CosTheTa = cos(Theta);
	SinTheTa = sin(Theta);
	
	if(0x01 == key.w )   //写等于判断的时候记得不要把变量弄成左值
		chassis.carSpeedx= 1000;  //测试的时候使用1m/s
	else if(0x01 == key.s)
		chassis.carSpeedx= -1000;
	else
		chassis.carSpeedx= 0;
	
	if(0x01 == key.a)
		chassis.carSpeedy = 1000;
	else if(0x01 == key.d)
		chassis.carSpeedy = -1000;
	else
		chassis.carSpeedy = 0;
	
	if(0x01 == key.shift)
	{
		chassis.carSpeedx *=2;
		chassis.carSpeedy *=2;
		F405.SuperPowerLimit = 0x1;
	}
	else
	{
		F405.SuperPowerLimit = 0x0;
	}
	
	if(0x01 == key.ctrl)
	{
		chassis.carSpeedw = -500;  //0.5rad/s 
	}
	else
	{
		ResetPos = (Theta)/6.28318f*8192;
		if((-3.1416f/2)>Theta)
		{
			pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor-ResetPos-4096);
		}
		else if((3.1416f/2)<Theta)
		{
			pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor-ResetPos+4096);
		}
		else 
		{
			pidChassisPosition.SetPoint = +(Gimbal.Yaw.Motor-ResetPos);
		}
		// 关于正方向的问题
		chassis.carSpeedw = (PID_Calc(&pidChassisPosition,Gimbal.Yaw.Motor)) + Gimbal.Yaw.AngularSpeed /180.0f * 3.1415926f ; //  mrad/s
	}
}


float ChassisPostionAngle_TranSform(short InitPos)  //得到一个-180到180的\delta \theta
{
	static float Bias_Angle;
	int32_t bias;
	bias=YawMotorReceive-InitPos;
	
	bias = (bias + 32768)% 65536 - 32768;
	
	
	Bias_Angle=bias/65536.0*360.0;
	return Bias_Angle;
}

/**********************************************************************************************************
*函 数 名: Chassis_Speed_Calc
*功能说明: 向底盘发送xyw向速度
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_Speed_Calc(void)
{
	switch (Inf_All_State.Chassis_Mode)
	{
		case CHASSIS_RC_FOLLOW_MODE:
			Chassis_Act_Cal(RC_Ctl.rc,RC_Ctl.key,CHASSIS_FOLLOW_FLAG);
			break;

		case CHASSIS_RC_PROTECT_MODE:
			 Chassis_Act_Cal(RC_Ctl.rc,RC_Ctl.key,CHASSIS_PROTECT_FLAG);
			break;
		
		case CHASSIS_POWER_DOWN_MODE:
			Chassis_Powerdown_Cal();
			break;
		
		case CHASSIS_MOUSEKEY_MODE:
			Chassis_MouseKey_Cal(RC_Ctl.key);
			break;
	
		default:
			Chassis_Powerdown_Cal();
			break;
	}
	ChassisCan2Send(&chassis.carSpeedx,&chassis.carSpeedy,&chassis.carSpeedw);
}


/**********************************************************************************************************
*函 数 名: Pid_ChassisPosition
*功能说明: 底盘随云台旋转pid参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Pid_ChassisPosition_Init(void)                
{

	/********************************************* 44 号车 ***********************************************************/	
	pidChassisPosition.P = 3.50f;				//  位置环	数量级大概是1				44号车
	pidChassisPosition.I = 0.00f;					
	pidChassisPosition.D = 7.00f;	
	pidChassisPosition.IMax = 300.0f;
	pidChassisPosition.OutMax = 16000.0f;
	
	FF_w.K1 = -40000.0f;
	FF_w.K2 = 0.0f;
	FF_w.OutMax = 16000.0f;
 
}
/**********************************************************************************************************
*函 数 名: Chassis_task
*功能说明: 底盘任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t Chassis_high_water;
extern uint8_t Remote_Disconnect_Flag;
void Chassis_task(void const *pvParameters)
{
	while (1) 
	{
		if(Remote_Disconnect_Flag < 200)
			Remote_Disconnect_Flag ++;
		else
			Inf_All_State.Chassis_Mode = CHASSIS_POWER_DOWN_MODE;

		Chassis_Speed_Calc(); 
		
		Last_Chassis_Mode = Inf_All_State.Chassis_Mode;
		vTaskDelay(1); 
	#if INCLUDE_uxTaskGetStackHighWaterMark
		Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
	#endif
    }
}
