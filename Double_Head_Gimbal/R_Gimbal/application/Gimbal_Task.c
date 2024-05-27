
#include "main.h"
#include "TD.h"
/*----------------------------------内部变量---------------------------*/
Pitch_6020_t Pitch_Moto;
Yaw_6020_t Yaw_Moto;
Gimbal_mode_t R_Gimbal;
Pid_Typedef PidPitchAidPos, PidPitchAidSpeed, PidYawAidPos, PidYawAidSpeed;
/*----------------------------------外部变量---------------------------*/
static GIMBAL_MODE_ENUM Gimbal_Last_Mode = GIMBAL_POWER_DOWN_MODE;
extern char PitchMotor_ReceiveFlag;
extern Gimbal_Typedef Gimbal;
extern F405_typedef F405;
extern PC_Receive_t PC_Receive;
extern char Budan;
extern float Buff_Yaw_Motor;
extern char pitch_lose_flag;
extern PCRecvData pc_recv_data;

TD_t PitchTD,YawTD;

/**********************************************************************************************************
 *函 数 名: Pitch_Current_Calc
 *功能说明: Pitch轴电流计算，在封装电流计算的同时保证不同PId模式的可拓展性
 *形    参: Pos_Pid_Type_ 位置环所使用的PID策略，Speed_Pid_Type_所使用的PID策略
 *返 回 值: 无
 **********************************************************************************************************/
short B_C = 0;
static void Pitch_Current_Calc(char Pos_Pid_Type_,char Speed_Pid_Type_)
{
	
	switch(Pos_Pid_Type_)
	{
		case SIMPLE_PID:
			Pitch_Moto.PidPitchSpeed.SetPoint = PID_Calc(&Pitch_Moto.PidPitchPos,Gimbal.Pitch.Gyro);
			break;
		default:
			Pitch_Moto.PidPitchSpeed.SetPoint = PID_Calc(&Pitch_Moto.PidPitchPos,Gimbal.Pitch.Gyro);
	}
	switch(Speed_Pid_Type_)
	{
		case SIMPLE_PID:
     		Pitch_Moto.PitchCurrent = LIMIT_MAX_MIN(Pitch_Moto.Pitch_Speed_pn * (PID_Calc(&Pitch_Moto.PidPitchSpeed,Gimbal.Pitch.AngularSpeed)
		     + FeedForward_Calc(&Pitch_Moto.Pitch_FF,Pitch_Moto.PidPitchSpeed.SetPoint)-B_C),30000,-30000);
			break;
		default:
			Pitch_Moto.PitchCurrent = Pitch_Moto.Pitch_Speed_pn * PID_Calc(&Pitch_Moto.PidPitchSpeed,Gimbal.Pitch.AngularSpeed);
	}
}

static void Yaw_Current_Calc(char Pos_Pid_Type_,char Speed_Pid_Type_)
{
	switch(Pos_Pid_Type_)
	{
		case SIMPLE_PID:
			Yaw_Moto.PidYawSpeed.SetPoint = PID_Calc(&Yaw_Moto.PidYawPos,Gimbal.Yaw.Gyro);
			break;
		default:
			Yaw_Moto.PidYawSpeed.SetPoint = PID_Calc(&Yaw_Moto.PidYawPos,Gimbal.Yaw.Gyro);
	}
	switch(Speed_Pid_Type_)
	{
		case SIMPLE_PID:
			Yaw_Moto.YawCurrent = Yaw_Moto.Yaw_Speed_pn *  PID_Calc(&Yaw_Moto.PidYawSpeed,Gimbal.Yaw.AngularSpeed);
			break;
		default:
			Yaw_Moto.YawCurrent = Yaw_Moto.Yaw_Speed_pn * PID_Calc(&Yaw_Moto.PidYawSpeed,Gimbal.Yaw.AngularSpeed);
	}
}
static float Pitch_Moto_Limit(float Pitch_Moto_Set_)
{
	float Pitch_Gyro_Max = Gimbal.Pitch.Gyro + (Sentry.pitch_max_motor - Gimbal.Pitch.MotorTransAngle);
	float Pitch_Gyro_Min = Gimbal.Pitch.Gyro + (Sentry.pitch_min_motor - Gimbal.Pitch.MotorTransAngle);

	return LIMIT_MAX_MIN(Pitch_Moto_Set_,Sentry.pitch_max_gyro,Sentry.pitch_min_gyro);
}
extern char PC_Receive_Flag_2_Armor;
// Todo 激情代码
double LP_k = 0.1f;
float SimpleLP(float input)
{
	static char first_flg = 1;
	static float last_value;
	if(first_flg == 1)
	{
		last_value = input;
		first_flg = -1;
		return input;
	}
	else
	{
		last_value = (1.0f-LP_k)* input + LP_k * last_value;
		return last_value;
	}
}


extern float pitch_setpos,yaw_setpos;

float Yaw_Nav_Aim_Center_Pose;
void Gimbal_Act(void)
{
//	if (Gimbal_Last_Mode != GIMBAL_NAV_PLUS_AIM_MODE)
//	{
//		Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(Gimbal.Pitch.Gyro);
//		Yaw_Moto.Yaw_Target_Pos = Gimbal.Yaw.Gyro;
//		Yaw_Nav_Aim_Center_Pose = Gimbal.Yaw.Gyro;
//		Gimbal_Last_Mode = GIMBAL_NAV_PLUS_AIM_MODE;
//	}
	
	//else
	//{
//		Pitch_Moto.Pitch_Target_Pos = pitch_setpos;
//		Yaw_Moto.Yaw_Target_Pos = yaw_setpos;
		Yaw_Nav_Aim_Center_Pose = Gimbal.Yaw.Gyro;

	//}
	Pitch_Moto.PidPitchPos.SetPoint = Pitch_Moto.Pitch_Target_Pos;
	Yaw_Moto.PidYawPos.SetPoint = Yaw_Moto.Yaw_Target_Pos;
	
	Pitch_Current_Calc(SIMPLE_PID,SIMPLE_PID);
	Yaw_Current_Calc(SIMPLE_PID,SIMPLE_PID);
}

void Gimbal_Powerdown_Act()
{
	if (Gimbal_Last_Mode != GIMBAL_POWER_DOWN_MODE)
	{
		Gimbal_Last_Mode = GIMBAL_POWER_DOWN_MODE;
	}
	Pitch_Moto.PitchCurrent = 0;
	Yaw_Moto.YawCurrent = 0;
	
	TD_Clear(&YawTD,Gimbal.Yaw.Gyro);
	
}

/**********************************************************************************************************
 *函 数 名: Gimbal_CurrentPid_Cal
 *功能说明: 发送电流值
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
int32_t Yaw_Current_Last;
int32_t Pitch_Current_Last;
void Gimbal_Pose_Loop_Act(void)
{
	
	switch (Inf_All_State.Gimbal_Mode)
	{
	case GIMBAL_POWER_DOWN_MODE:
		Gimbal_Powerdown_Act();
		break;
	case GIMBAL_RC_TEST_MODE:
	case GIMBAL_RC_ACT_MODE:
		//Gimbal_RC_Act(RC_Ctl.rc);
		//Pitch_Motor_Test_G(0.5,750);
		//break;
	case GIMBAL_PC_ACT_MODE:
		//Gimbal_Armor_Act(RC_Ctl.rc);
		//break;
	case GIMBAL_PC_TEST_MODE:
		//Gimbal_Powerdown_Act();
		//break;
	case GIMBAL_MOUSEKEY_MODE:
		//Gimbal_MouseKey_Act(RC_Ctl.mouse);
		//break;
	case GIMBAL_NAV_PLUS_AIM_MODE:
		//Gimbal_Nav_Aim_Act(RC_Ctl.rc);
		//break;
	Gimbal_Act();
	default:
		Gimbal_Powerdown_Act();
		break;
	}
    

    F405.Gimbal_Flag = Inf_All_State.Gimbal_Mode;

	if (!pitch_lose_flag && Inf_All_State.Gimbal_Mode != GIMBAL_POWER_DOWN_MODE) //防堵转
	{
		//YawCan1Send(Yaw_Moto.YawCurrent);
		//Yaw_Moto.YawCurrent = 0.9 * Yaw_Current_Last + 0.1 * Yaw_Moto.YawCurrent;
		YawCan2Send(Yaw_Moto.YawCurrent);
		//Pitch_Moto.PitchCurrent = (int32_t)pitch_butter_filter(Pitch_Moto.PitchCurrent);
		//Pitch_Moto.PitchCurrent = 0.5 * Pitch_Current_Last + 0.5 * Pitch_Moto.PitchCurrent;
		PitchCan1Send(Pitch_Moto.PitchCurrent);
		//PitchCan2Send(Pitch_Moto.PitchCurrent);
		Yaw_Current_Last = Yaw_Moto.YawCurrent;
		Pitch_Current_Last = Pitch_Moto.PitchCurrent;
	}
	else
	{
		YawCan2Send(0);
		PitchCan1Send(0);
	}
	
	
}

/**********************************************************************************************************
 *函 数 名: Pid_Yaw_MotorPos_GyroSpeed
 *功能说明: Yaw轴辅瞄双环PID
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void PidGimbalMotor_Init(void)
{

	Pitch_Moto.PidPitchPos.P = 55.0f; //手动pitch角度环
	Pitch_Moto.PidPitchPos.I = 0.1f;
	Pitch_Moto.PidPitchPos.D = 200.0f;
	Pitch_Moto.PidPitchPos.IMax = 300.0f;
	Pitch_Moto.PidPitchPos.SetPoint = 0.0f;
	Pitch_Moto.PidPitchPos.OutMax = 350.0f;
	Pitch_Moto.PidPitchPos.ErrorMax = 0.5f;
	
	Pitch_Moto.PidPitchSpeed.P = 120.0f; //手动pitch速度环
	Pitch_Moto.PidPitchSpeed.I = 0.4f;
	Pitch_Moto.PidPitchSpeed.D = 0.0f;
	Pitch_Moto.PidPitchSpeed.IMax = 20000.0f;
	Pitch_Moto.PidPitchSpeed.SetPoint = 0.0f;
	Pitch_Moto.PidPitchSpeed.OutMax = 30000.0f;
	Pitch_Moto.PidPitchSpeed.ErrorMax = 10.0f;
	Pitch_Moto.Pitch_Speed_pn = -1.0f;
//	Pitch_Moto.PidPitchPos.P = 25.0f; //手动pitch角度环
//	Pitch_Moto.PidPitchPos.I = 0.0f;
//	Pitch_Moto.PidPitchPos.D = 10.0f;
//	Pitch_Moto.PidPitchPos.IMax = 0.0f;
//	Pitch_Moto.PidPitchPos.SetPoint = 0.0f;
//	Pitch_Moto.PidPitchPos.OutMax = 350.0f;
//	Pitch_Moto.PidPitchPos.ErrorMax = 2.0f;
//	
//	Pitch_Moto.PidPitchSpeed.P = 125.0f; //手动pitch速度环
//	Pitch_Moto.PidPitchSpeed.I = 0.45f;
//	Pitch_Moto.PidPitchSpeed.D = 0.0f;
//	Pitch_Moto.PidPitchSpeed.IMax = 2000.0f;
//	Pitch_Moto.PidPitchSpeed.SetPoint = 0.0f;
//	Pitch_Moto.PidPitchSpeed.OutMax = 30000.0f;
//	Pitch_Moto.PidPitchSpeed.ErrorMax = 50.0f;
//	Pitch_Moto.Pitch_Speed_pn = -1.0f;

	//手动yaw双环                                 // 5号车
	Yaw_Moto.PidYawPos.P = 18.0f; //手动yaw角度环
	Yaw_Moto.PidYawPos.I = 0.00f;
	Yaw_Moto.PidYawPos.D = 0.0f;
	Yaw_Moto.PidYawPos.IMax = 10.0f;
	Yaw_Moto.PidYawPos.SetPoint = 0.0f;
	Yaw_Moto.PidYawPos.OutMax = 550.0f;

	Yaw_Moto.PidYawSpeed.P = 30.0f; //手动yaw速度环  本来是不想加符号的）
	Yaw_Moto.PidYawSpeed.I = 0.0f;
	Yaw_Moto.PidYawSpeed.D = 0.0f;
	Yaw_Moto.PidYawSpeed.IMax = 1200.0f;
	Yaw_Moto.PidYawSpeed.ErrorMax = 30.0f;
	Yaw_Moto.PidYawSpeed.SetPoint = 0.0f;
	Yaw_Moto.PidYawSpeed.OutMax = 2000.0f;
	Yaw_Moto.Yaw_Speed_pn = -1;
	
	Pitch_Moto.Pitch_FF.K1 = 0;
	Pitch_Moto.Pitch_FF.K2 = 0;
	Pitch_Moto.Pitch_FF.Last_DeltIn = 0;
	Pitch_Moto.Pitch_FF.Now_DeltIn = 0;
	Pitch_Moto.Pitch_FF.Out = 0;
	Pitch_Moto.Pitch_FF.OutMax = 2000.0f;
}

/**********************************************************************************************************
 *函 数 名: Gimbal_task
 *功能说明: 云台任务函数
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
uint32_t Gimbal_high_water;
extern TaskHandle_t RCReceiveTask_Handler; //任务句柄
extern TaskHandle_t PCReceiveTask_Handler; //任务句柄
extern uint8_t Remote_Receive_Flag;
double cos_now = 0;

void Gimbal_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 1;
	vTaskDelay(1000);
	
	
	while (1)
	{
		xLastWakeTime = xTaskGetTickCount();
		ShootCount_Cal();
		Shoot_task_();

		if(Remote_Receive_Flag) //数据解码
		{
			xTaskNotifyGive(RCReceive_task_handle);
		}
		
		ZeroCheck_cal();
		Gimbal_Pose_Loop_Act();
		PitchMotor_ReceiveFlag = 0;
		
		SendtoPC();

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
