# include "main.h"

BODAN_MOTOR_t Bodan_Motor;
ShootTask_typedef Shoot;
Shoot_Cmd_t Shoot_Cmd;
static SHOOT_MODE_ENUM Last_Shoot_State = SHOOT_POWER_DOWN_MODE;

static int Shoot_Init_flag = 0; //
/*----------------------------------结构体------------------------------*/

/*----------------------------------外部变量---------------------------*/
extern float Bodan_Pos;
extern F105_Typedef F105;//F105
extern F405_typedef F405;
extern Gimbal_Typedef Gimbal;
extern PC_Receive_t PC_Receive;
extern BodanMotorReceive_Typedef BodanReceive;
extern short FrictionReceive[2];
extern uint8_t CoolBuffState;

extern Pitch_6020_t Pitch_Moto;
extern Yaw_6020_t Yaw_Moto;
Frition_Wheel_t Frition_Wheel_Motor[2];
int8_t Bodan_Block_flag = 0; 
int8_t Bodan_Powerdown_flag = 0;
int Target_Bodan_Pos = 0 ;

/**
  * @brief  堵转检查的处理
  * @param  None
  * @retval None
  */
int16_t block_bullet_tick = 0;
int16_t block_test = 0;
void Block_Check()
{	
	//先来堵转检测
	if(ABS(Bodan_Motor.PidBodanMotorPos.PreError)>0.5*OneGrid)
	{
		block_bullet_tick++;
	}else
	{
		block_bullet_tick = 0;
	}
	Bodan_Block_flag = 0;
	Bodan_Powerdown_flag = 0;
	if(block_bullet_tick >= 500)//堵转1s
	{
		if( block_bullet_tick == 500 )
		{
			Target_Bodan_Pos += OneGrid*5/4; //反转
	        block_test++;
		}
		if(block_bullet_tick >= 1500)
		{
			Bodan_Motor.BodanMotorCurrent = 0;
			Bodan_Powerdown_flag = 1;
		}
		Bodan_Block_flag = 1;
	}	
}
/**
  * @brief  打蛋数量估计
  * @param  None
  * @retval None
  */
uint32_t ShootCountTime_last=0;
float ShootCount_dt=0;
int ShootCount_Number = 0;//射击的总子弹数
void ShootCount_Cal(void)
{
	static short ShootMode_last = 0;
	ShootCount_dt = DWT_GetDeltaT(&ShootCountTime_last);
	Shoot.HeatControl.ShootCount_IntervalTime+=ShootCount_dt*1000;
	
	if(ShootMode_last!=Inf_All_State.Shoot_Mode){
		Shoot.HeatControl.ShootCount_IntervalTime = -200;//切换摩擦轮模式后设置一段时间保护，防止被认为射击
	}
	if(Frition_Wheel_Motor[0].PidFrictionSpeed.PreError>80 && Frition_Wheel_Motor[0].PidFrictionSpeed.PreError < 1000.0)//满足掉速要求
	{
		if(Shoot.HeatControl.ShootCount_IntervalTime>ShootInterval_Min_time){//连射的最小间隔
			ShootCount_Number++;//总射击子弹数
			Shoot.HeatControl.CurShootNumber++;
			Shoot.HeatControl.ShootCount_IntervalTime = 0;
			//23.11.19 增加打蛋处理完成标志
			Shoot_Cmd.Shoot_State_send = Shoot_Cmd.Shoot_State;
		}
	}
	ShootMode_last = Inf_All_State.Shoot_Mode;
}
/**
  * @brief  热量控制
  * @param  dt 调用时间差，单位ms 
  * @retval None
  * author：郭嘉豪
  */
const float BulletHeat17 = 10;
float HeatControlThreshold = 1.0f;   	//开启热量控制的阈值
float curHeat=0;
int BulletAllowCnt = 5; //热量余裕
extern JudgeData_1_t JudgeData_1;
void HeatControl(float dt)
{
	Shoot.HeatControl.HeatMax17 = F105.HeatMax17 - BulletHeat17*BulletAllowCnt;  		//判断能否打蛋的最大热量，保留一个子弹
	Shoot.HeatControl.HeatCool17 = F105.HeatCool17 * dt;          		// 当前热量检测的冷却值
	
	Shoot.HeatControl.HeatUpdateFlag = 0; //裁判系统拆了，先赋值为0
	
	/****************************更新当前热量**************************************/
	if(Shoot.HeatControl.HeatUpdateFlag == 1){												//热量更新了则使用裁判系统
		Shoot.HeatControl.HeatUpdateFlag = 0;
		Shoot.HeatControl.CurHeat17 = JudgeData_1.shooter1_heat;
	}
	else
	{																														//自行估计热量
		Shoot.HeatControl.CurHeat17 += Shoot.HeatControl.CurShootNumber*BulletHeat17;
		Shoot.HeatControl.CurShootNumber = 0;														//当前子弹处理完毕
		Shoot.HeatControl.CurHeat17 = Shoot.HeatControl.CurHeat17 - Shoot.HeatControl.HeatCool17;															//处理当前周期的冷却
		if(Shoot.HeatControl.CurHeat17<0) Shoot.HeatControl.CurHeat17 = 0;
	}
	
	if(Shoot.HeatControl.CurHeat17 < HeatControlThreshold*Shoot.HeatControl.HeatMax17)
	{
		Shoot.HeatControl.IsShootAble = 1;
	}
	else
	{
		Shoot.HeatControl.IsShootAble = 0;
	}
	Shoot.HeatControl.LastHeat17 = Shoot.HeatControl.CurHeat17;
	curHeat = Shoot.HeatControl.CurHeat17;
}

/**
  * @brief  任务初始化
  * @param  None
  * @retval None
  */

void Shoot_init(){
	F105.HeatMax17 = 240; //Shoot.HeatControl.HeatMax17 = 90;
	F105.HeatCool17 = 80;
	
	F105.BulletSpeedLevel = 2;//30
	F105.RobotLevel = 0;//默认
	
	Shoot.HeatControl.ShootCount_IntervalTime=0;
	Shoot.HeatControl.ShootContinue_IntervalTime=0;
}

/**
  * @brief  速度环拨盘电机转速选择
  * @param  None
  * @retval None
  */

void Pluck_Speed_Choose()
{
  	switch(F105.RobotLevel)
	{	
       /***** 1级 ******/
		case 1:
			if(!CoolBuffState)
			{
				Shoot.PullerSpeed = 3000;    //无增益
			}
			else
			{
				Shoot.PullerSpeed = 4000;    //有增益 
			}
		break;
		
		 /****** 2级 *****/
		case 2:
			if(!CoolBuffState)
			{
				Shoot.PullerSpeed = 3500;    //无增益
			}
			else
			{
				Shoot.PullerSpeed = 4000;    //有增益 
			}
		break;
		
    /******  3级 ******/		
		case 3:
			if(!CoolBuffState)
			{
				Shoot.PullerSpeed = 4000;    //无增益
			}
			else
			{
				Shoot.PullerSpeed = 4500;    //有增益 
			}
			break;
		
		
			default:
			Shoot.PullerSpeed = 8000;
	 }	
}
/**
  * @brief  速度环摩擦轮速度选择
  * @param  None
  * @retval None
  */
void FrictionSpeedChoose(void)
{
	switch(F105.BulletSpeedLevel)
	{
		case 0:
		{
			Frition_Wheel_Motor[0].FrictionWheel_speed = Sentry.Low_FrictionSpeed;
			Frition_Wheel_Motor[1].FrictionWheel_speed = Sentry.Low_FrictionSpeed;
			break;
		}
		case 1:
		{
			Frition_Wheel_Motor[0].FrictionWheel_speed = Sentry.Medium_FrictionSpeed;
			Frition_Wheel_Motor[1].FrictionWheel_speed = Sentry.Medium_FrictionSpeed;
			break;
		}
		case 2:
		{
			Frition_Wheel_Motor[0].FrictionWheel_speed = Sentry.High_FrictionSpeed;
			Frition_Wheel_Motor[1].FrictionWheel_speed = Sentry.High_FrictionSpeed;
			break;
		}
		default:
		{
			Frition_Wheel_Motor[0].FrictionWheel_speed = Sentry.High_FrictionSpeed;
			Frition_Wheel_Motor[1].FrictionWheel_speed = Sentry.High_FrictionSpeed;
			break;
		}
	}
}

/**
  * @brief  速度环锁定（效果不如位置环，后面可以更换）
  * @param  None
  * @retval None
  */
void Shoot_Powerdown_Cal(void)
{
	Shoot_Speed_Loop_Calc(0);
}

/**
  * @brief  遥控器模式手动打蛋
  * @param  None （md本来遥控器数据应该作为形参，不过既然都是全局变量了就无所谓了）
  * @retval None
  */

void Shoot_RC_Act()
{
	if(Last_Shoot_State != Inf_All_State.Shoot_Mode)
	{
		;
	}
	
	if((RC_Ctl.rc.ch1 - 1024) > 600)
	{
		if(1 == Shoot.HeatControl.IsShootAble )
		{
			Shoot_Speed_Loop_Calc(Shoot.PullerSpeed);
			return;
		}
	}
	
	Shoot_Speed_Loop_Calc(0);
}

float delta_Bodan_Pos = 0;//仅调试用
float Last_Bodan = 0; //仅调试用

void Shoot_RC_Postion_Act()
{
	static int bodanLastPos; 
	static uint32_t last_time = 0;
	static int Shoot_IntervalTime = 80;
	static int delay_num = 0;
	
	delay_num++;
	
	if(Last_Shoot_State != Inf_All_State.Shoot_Mode)
	{
		bodanLastPos = Bodan_Pos + OneGrid;//因为只要一开始一进入改函数，就一定会波一格
		last_time = xTaskGetTickCount();
		Target_Bodan_Pos = Bodan_Pos;
	}
	
	if((RC_Ctl.rc.ch1 - 1024) > 600 && !Bodan_Block_flag)
	{
		if(delay_num > Shoot_IntervalTime)
		{
			bodanLastPos = Bodan_Pos;
			delay_num = 0;
			Target_Bodan_Pos = bodanLastPos - OneGrid;
		}
	}
	
	Shoot_Pos_Loop_Calc(Target_Bodan_Pos);
	Block_Check();
}

/**
  * @brief  辅喵模式位置环打蛋
  * @param  None 
  * @retval None
  * 辅喵模式打蛋有很多问题需要注意，一个是云台到位了才能打蛋，这个到位判断可以通过辅喵识别的距离来算
	另一个是辅喵的标志位会寸止，需要位置环打蛋才能爽打；位置环的间隔时间得有一个最小值，在枪管热量够
	的时候先爆发一波，然后再等枪管冷却。
  */
uint8_t ShootAble = 0;
int Shoot_IntervalTime = 100;
void Shoot_PC_Act()
{
	static float bodanLastPos; 
	static float now_time = 0;
	
	static uint8_t Last_Shoot_state = 0;
	static uint8_t delay_num = 0;
//	switch (Shoot_Cmd.Shoot_Freq_cmd)
//	{
//		case 0x0:
//			Shoot_IntervalTime = 100;
//			break;
//		case 0x1:
//			Shoot_IntervalTime = 80;
//			break;
//		case 0x2:
//			Shoot_IntervalTime = 50;
//			break;
//		case 0x3:
//			Shoot_IntervalTime = 40;
//			break;
//		default:
//			Shoot_IntervalTime = 80;
//	}
	if(Last_Shoot_State != Inf_All_State.Shoot_Mode)
	{
		bodanLastPos = Bodan_Pos + OneGrid;//因为只要一开始一进入改函数，就一定会波一格
		now_time = xTaskGetTickCount();
		delay_num = 0;
		Target_Bodan_Pos = Bodan_Pos;
	}
	delay_num++;
	//0x3 即开摩擦轮 又 开火
	if((Last_Shoot_state == 0x3 && Shoot_Cmd.Shoot_State == 0x0) || (Last_Shoot_state == 0x0 && Shoot_Cmd.Shoot_State == 0x3))
	{
		ShootAble = 1;
	}
	
	if(ShootAble && (1 == Shoot.HeatControl.IsShootAble)) //最后一个标志位留给辅喵允许打蛋标志位
	{
		if(ABS(pc_pitch - Gimbal.Pitch.Gyro)< (0.12f/3.0f*180.0f/PI) && ABS(pc_yaw - Gimbal.Yaw.Gyro)< (0.15f/3.0f*180.0f/PI))	//已经辅瞄到位，自动开火
		{
			if(delay_num > Shoot_IntervalTime  && !Bodan_Block_flag)
			{
				bodanLastPos = Bodan_Pos;
				delay_num = 0;
				Target_Bodan_Pos = bodanLastPos - OneGrid;
			}
		}
	}
	
	ShootAble = 0;
	
	Shoot_Pos_Loop_Calc(Target_Bodan_Pos);
	Block_Check();
	Last_Shoot_state = Shoot_Cmd.Shoot_State;
}

/**
  * @brief  拨盘速度环模式封装
  * @param  Bodan_speed_set 拨蛋电机速度环目标值
  * @retval None
  */

void Shoot_Speed_Loop_Calc(short Bodan_speed_set)
{
	if(Inf_All_State.Shoot_Mode != SHOOT_POWER_DOWN_MODE)
	{
		Bodan_Motor.PidBodanMotorSpeed.SetPoint=-Bodan_speed_set;
		Bodan_Motor.BodanMotorCurrent = (short)PID_Calc(&Bodan_Motor.PidBodanMotorSpeed,BodanReceive.RealSpeed);
	}
	else
	{
		Bodan_Motor.PidBodanMotorSpeed.SetPoint=0;
		Bodan_Motor.BodanMotorCurrent = (short)PID_Calc(&Bodan_Motor.PidBodanMotorSpeed,BodanReceive.RealSpeed);
	}
	if(Bodan_Powerdown_flag)
		Bodan_Motor.BodanMotorCurrent = 0;
}

/**
  * @brief  拨盘位置环模式封装
  * @param  Bodan_speed_set 拨蛋电机位置环目标值
  * @retval None
  */

void Shoot_Pos_Loop_Calc(float Bodan_pos_set)
{
	if(Inf_All_State.Shoot_Mode != SHOOT_POWER_DOWN_MODE)
	{
		Bodan_Motor.PidBodanMotorPos.SetPoint = Bodan_pos_set;
		Bodan_Motor.PidBodanMotorSpeed.SetPoint=PID_Calc(&Bodan_Motor.PidBodanMotorPos,Bodan_Pos);
		Bodan_Motor.BodanMotorCurrent = (short)PID_Calc(&Bodan_Motor.PidBodanMotorSpeed,BodanReceive.RealSpeed);
	}
	else
	{
		Bodan_Motor.PidBodanMotorSpeed.SetPoint=0;
		Bodan_Motor.BodanMotorCurrent = (short)PID_Calc(&Bodan_Motor.PidBodanMotorSpeed,BodanReceive.RealSpeed);
	}
	if(Bodan_Powerdown_flag)
		Bodan_Motor.BodanMotorCurrent = 0;
}




extern uint8_t PC_Shoot_flag;


/**
  * @brief  拨蛋模式选择
  * @param  None
  * @retval None
  */
short Last_GimbalState;
void BodanMotor_Loop_Calc(void)
{
	
	switch(Inf_All_State.Shoot_Mode)//射击模式选择
	{
		case SHOOT_RC_TEST_MODE:
			Shoot_RC_Postion_Act();
			break;
		case SHOOT_RC_ACT_MODE:
			Shoot_RC_Postion_Act();
			break;

		case SHOOT_PC_ACT_MODE:
		case SHOOT_PC_TEST_MODE:
				Shoot_PC_Act();
			break;

		case SHOOT_POWER_DOWN_MODE:
			Shoot_Powerdown_Cal();
			break;

		default:
			break;
	}
	
	Last_Shoot_State = Inf_All_State.Shoot_Mode;
	
	if(Shoot_Init_flag == 0)
	{
		Shoot_Powerdown_Cal();
	}
	
}

/**********************************************************************************************************
*函 数 名: Pid_BodanMotor
*功能说明: 拨弹电机位置速度环pid参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Pid_BodanMotor_Init(void)
{	
	Bodan_Motor.PidBodanMotorPos.P=0.4f;  
	Bodan_Motor.PidBodanMotorPos.I=0.05f;
	Bodan_Motor.PidBodanMotorPos.D=0.0f;
	Bodan_Motor.PidBodanMotorPos.IMax=1500.0f;
	Bodan_Motor.PidBodanMotorPos.SetPoint=0.0f;
	Bodan_Motor.PidBodanMotorPos.OutMax=20000.0f;

	Bodan_Motor.PidBodanMotorSpeed.P=5.0f;  
	Bodan_Motor.PidBodanMotorSpeed.I=0.1f;
	Bodan_Motor.PidBodanMotorSpeed.D=0.0f;
	Bodan_Motor.PidBodanMotorSpeed.IMax=7500.0f;
	Bodan_Motor.PidBodanMotorSpeed.SetPoint=0.0f;
	Bodan_Motor.PidBodanMotorSpeed.OutMax = 18000.0f;
	
	Bodan_Motor.Onegrid=36864.0f;
}

/**********************************************************************************************************
*函 数 名: Pid_Friction_Init
*功能说明: 拨弹电机位置速度环pid参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Pid_Friction_Init(void)
{

/********************************************* 44号车 *******************************************************/	

	Sentry.Low_FrictionSpeed = 4370;
	Sentry.Medium_FrictionSpeed = 4800;
	Sentry.High_FrictionSpeed = 6750;
		
/********************************************* 缺省值 ******************************************************/		

	Frition_Wheel_Motor[0].PidFrictionSpeed.P=10.0f;//80.0f;
	Frition_Wheel_Motor[0].PidFrictionSpeed.I=0.0f;
	Frition_Wheel_Motor[0].PidFrictionSpeed.D=0.0f;
	Frition_Wheel_Motor[0].PidFrictionSpeed.IMax=1500.0f;
	Frition_Wheel_Motor[0].PidFrictionSpeed.SetPoint=0.0f;
	Frition_Wheel_Motor[0].PidFrictionSpeed.OutMax = 16384.0f;
	
	
	Frition_Wheel_Motor[1].PidFrictionSpeed.P=10.0f;//80.0f;
	Frition_Wheel_Motor[1].PidFrictionSpeed.I=0.0f;
	Frition_Wheel_Motor[1].PidFrictionSpeed.D=0.0f;
	Frition_Wheel_Motor[1].PidFrictionSpeed.IMax=1500.0f;
	Frition_Wheel_Motor[1].PidFrictionSpeed.SetPoint=0.0f;
	Frition_Wheel_Motor[1].PidFrictionSpeed.OutMax = 16384.0f;
	
}

void FrictionWheel_Set(short speed)
{	
	if(Inf_All_State.Shoot_Mode == SHOOT_POWER_DOWN_MODE)
	{
		Frition_Wheel_Motor[0].PidFrictionSpeed.SetPoint = 0;
		Frition_Wheel_Motor[1].PidFrictionSpeed.SetPoint = 0;
	}
	
	else
	{
		Frition_Wheel_Motor[0].PidFrictionSpeed.SetPoint=Sentry.FricMotor_pn[0]*speed;     //  accelerator+bias_speed;
		Frition_Wheel_Motor[1].PidFrictionSpeed.SetPoint=Sentry.FricMotor_pn[1]*speed;
	}

	Frition_Wheel_Motor[0].FrictionCurrent=PID_Calc(&(Frition_Wheel_Motor[0].PidFrictionSpeed),FrictionReceive[0]);
	Frition_Wheel_Motor[1].FrictionCurrent=PID_Calc(&(Frition_Wheel_Motor[1].PidFrictionSpeed),FrictionReceive[1]);
	
}

/**********************************************************************************************************
*函 数 名: Shoot_task
*功能说明: 拨弹任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static uint32_t ShootTask_TimeLast = 0;
void Shoot_task_()
{
	HeatControl(DWT_GetDeltaT(&ShootTask_TimeLast));
	
	if(Inf_All_State.Shoot_Mode == SHOOT_POWER_DOWN_MODE)
	{
		F405.Fric_Flag=0;
		Frition_Wheel_Motor[0].FrictionWheel_speed =0;
		Frition_Wheel_Motor[1].FrictionWheel_speed =0;
		Shoot_Init_flag = 0;				//禁止拨盘转
	}
	else		//已经初始化成功一次了，并且切回了非掉电模式
	{
		F405.Fric_Flag=1;
		FrictionSpeedChoose();
		Shoot_Init_flag = 1;
	}
	
	FrictionWheel_Set(Frition_Wheel_Motor[0].FrictionWheel_speed);
	Pluck_Speed_Choose();   //射频选择

	BodanMotor_Loop_Calc();
	BodanCan1Send(Bodan_Motor.BodanMotorCurrent);
	FrictionCan1Send(Frition_Wheel_Motor[0].FrictionCurrent,Frition_Wheel_Motor[1].FrictionCurrent);	
}