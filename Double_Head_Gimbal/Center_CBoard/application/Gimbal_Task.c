
#include "main.h"
#include "TD.h"
/*----------------------------------内部变量---------------------------*/
Pitch_6020_t Pitch_Moto;
Yaw_6020_t Yaw_Moto;
Pitch_6020_t L_Pitch_Moto,R_Pitch_Moto;
Yaw_6020_t L_Yaw_Moto,R_Yaw_Moto;
Gimbal_mode_t L_Gimbal,R_Gimbal;
Pid_Typedef PidPitchAidPos, PidPitchAidSpeed, PidYawAidPos, PidYawAidSpeed;
Double_Gimbal_Typedef Double_Gimbal;
int gimbal_choose;
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

float mg_M = 7500;
float mg_K = 1.05;

float Balance_Pitch = 0;
short Balance_Pitch_Current = 0; 
float Balance_Pitch_Current_Float = 0;
//p1 = 0.8806
//p2 = 10.85
// p3 = -747.2
// p4 = -6044
// Current = p1*x^3 + p2*x^2


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


void Gimbal_Armor_Act(Remote rc)
{
	if (Gimbal_Last_Mode != GIMBAL_PC_ACT_MODE)
	{
		if(1==gimbal_choose)
		{
		L_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(Double_Gimbal.L_Pitch.Gyro);
		L_Yaw_Moto.Yaw_Target_Pos = Double_Gimbal.L_Yaw.Gyro;
		Gimbal_Last_Mode = GIMBAL_PC_ACT_MODE;
			//need to change the gimbal struct
		}
		if(2==gimbal_choose)
		{
		R_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(Double_Gimbal.R_Pitch.Gyro);
		R_Yaw_Moto.Yaw_Target_Pos = Double_Gimbal.R_Yaw.Gyro;
		Gimbal_Last_Mode = GIMBAL_PC_ACT_MODE;
			//need to change the gimbal struct
		}
	}
	else
	{
		if(PC_Receive_Flag_2_Armor)
		{
			if(ABS(pc_yaw - Gimbal.Yaw.Gyro) < 70 && ABS(pc_pitch - (Gimbal.Pitch.Gyro)) < 60)
			{//need intruction of this if
				if(1==gimbal_choose)
				{
				L_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(pc_pitch);
				L_Yaw_Moto.Yaw_Target_Pos = pc_yaw;//need to change the receive data of pc
				}
				if(2==gimbal_choose)
				{
				R_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(pc_pitch);
				R_Yaw_Moto.Yaw_Target_Pos = pc_yaw;
				}
				//need to change
			}	
			else
			{
				if(1==gimbal_choose)
				{
				L_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(Double_Gimbal.L_Pitch.Gyro);
				L_Yaw_Moto.Yaw_Target_Pos = Double_Gimbal.L_Yaw.Gyro;
				}
				if(2==gimbal_choose)
				{
				R_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(Double_Gimbal.R_Pitch.Gyro);
				R_Yaw_Moto.Yaw_Target_Pos = Double_Gimbal.R_Yaw.Gyro;
				}	
			}
			PC_Receive_Flag_2_Armor = 0;
		}
		else
		{
			if(1==gimbal_choose)
			{
			L_Yaw_Moto.Yaw_Target_Pos = L_Yaw_Moto.Yaw_Target_Pos + (1024 - rc.ch2) * RC_GIMBAL_RATIO;
			L_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(L_Pitch_Moto.Pitch_Target_Pos - (1024 - rc.ch3) * RC_GIMBAL_RATIO );
			}
			if(2==gimbal_choose)
			{
			R_Yaw_Moto.Yaw_Target_Pos = R_Yaw_Moto.Yaw_Target_Pos + (1024 - rc.ch2) * RC_GIMBAL_RATIO;
			R_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(R_Pitch_Moto.Pitch_Target_Pos - (1024 - rc.ch3) * RC_GIMBAL_RATIO );
			}
		}
	}
	L_Pitch_Moto.PidPitchPos.SetPoint = TD_Calculate(&PitchTD,L_Pitch_Moto.Pitch_Target_Pos);
	L_Yaw_Moto.PidYawPos.SetPoint = TD_Calculate(&YawTD,L_Yaw_Moto.Yaw_Target_Pos);
	
	R_Pitch_Moto.PidPitchPos.SetPoint = TD_Calculate(&PitchTD,R_Pitch_Moto.Pitch_Target_Pos);
	R_Yaw_Moto.PidYawPos.SetPoint = TD_Calculate(&YawTD,R_Yaw_Moto.Yaw_Target_Pos);
	
//	Pitch_Current_Calc(SIMPLE_PID,SIMPLE_PID);
//	Yaw_Current_Calc(SIMPLE_PID,SIMPLE_PID);
}

void Gimbal_RC_Act(Remote rc)
{
	if (Gimbal_Last_Mode != GIMBAL_RC_TEST_MODE && Gimbal_Last_Mode != GIMBAL_RC_ACT_MODE)
	{
		if(1==gimbal_choose)
		{
		L_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(Double_Gimbal.L_Pitch.Gyro);
		L_Yaw_Moto.Yaw_Target_Pos = Double_Gimbal.L_Yaw.Gyro;
		Gimbal_Last_Mode = Inf_All_State.Gimbal_Mode;
		}
		if(2==gimbal_choose)
		{
		R_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(Double_Gimbal.R_Pitch.Gyro);
		R_Yaw_Moto.Yaw_Target_Pos = Double_Gimbal.R_Yaw.Gyro;
		Gimbal_Last_Mode = Inf_All_State.Gimbal_Mode;
		}
			
	}
	else
	{
		if(1==gimbal_choose)
		{
		L_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(L_Pitch_Moto.Pitch_Target_Pos - (1024 - rc.ch3) * RC_GIMBAL_RATIO*3 );
		L_Yaw_Moto.Yaw_Target_Pos = L_Yaw_Moto.Yaw_Target_Pos + (1024 - rc.ch2) * RC_GIMBAL_YAW_RATIO;  //1ms内增加x°
		}
		if(2==gimbal_choose)
		{
		R_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(R_Pitch_Moto.Pitch_Target_Pos - (1024 - rc.ch3) * RC_GIMBAL_RATIO*3 );
		R_Yaw_Moto.Yaw_Target_Pos = R_Yaw_Moto.Yaw_Target_Pos + (1024 - rc.ch2) * RC_GIMBAL_YAW_RATIO;  //1ms内增加x°
		}
	}
	
	L_Pitch_Moto.PidPitchPos.SetPoint = L_Pitch_Moto.Pitch_Target_Pos;
	L_Yaw_Moto.PidYawPos.SetPoint = L_Yaw_Moto.Yaw_Target_Pos;
	R_Pitch_Moto.PidPitchPos.SetPoint = R_Pitch_Moto.Pitch_Target_Pos;
	R_Yaw_Moto.PidYawPos.SetPoint = R_Yaw_Moto.Yaw_Target_Pos;
//	Pitch_Current_Calc(SIMPLE_PID,SIMPLE_PID);
//	Yaw_Current_Calc(SIMPLE_PID,SIMPLE_PID);
	
}

//void Gimbal_MouseKey_Act(Mouse mouse)
//{
//	if (Gimbal_Last_Mode != GIMBAL_MOUSEKEY_MODE)
//	{
//		Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(Gimbal.Pitch.Gyro);
//		Yaw_Moto.Yaw_Target_Pos = Gimbal.Yaw.Gyro;
//		Gimbal_Last_Mode = GIMBAL_MOUSEKEY_MODE;
//	}
//	else
//	{
//		Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(Pitch_Moto.Pitch_Target_Pos - mouse.y * 0.005f - mouse.z * 0.001f);
//		Yaw_Moto.Yaw_Target_Pos -= mouse.x * 0.005f;
//	}
//	
//	Pitch_Moto.PidPitchPos.SetPoint = Pitch_Moto.Pitch_Target_Pos;
//	Yaw_Moto.PidYawPos.SetPoint = Yaw_Moto.Yaw_Target_Pos;
//	
//	Pitch_Current_Calc(SIMPLE_PID,SIMPLE_PID);
//	Yaw_Current_Calc(SIMPLE_PID,SIMPLE_PID);
//	
//}

float Yaw_Nav_Aim_Center_Pose;
void Gimbal_Nav_Aim_Act(Remote rc)
{
	if (Gimbal_Last_Mode != GIMBAL_NAV_PLUS_AIM_MODE)
	{
		L_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(Double_Gimbal.L_Pitch.Gyro);
		L_Yaw_Moto.Yaw_Target_Pos = Double_Gimbal.L_Yaw.Gyro;
		R_Pitch_Moto.Pitch_Target_Pos = Pitch_Moto_Limit(Double_Gimbal.R_Pitch.Gyro);
		R_Yaw_Moto.Yaw_Target_Pos = Double_Gimbal.R_Yaw.Gyro;
		Yaw_Nav_Aim_Center_Pose = Double_Gimbal.R_Yaw.Gyro;//这里可能要考虑改为大yaw
		Gimbal_Last_Mode = GIMBAL_NAV_PLUS_AIM_MODE;
	}
	
	else
	{
		L_Pitch_Moto.Pitch_Target_Pos = 0;
		L_Yaw_Moto.Yaw_Target_Pos = L_Yaw_Moto.Yaw_Target_Pos + (1024 - rc.ch2) * RC_GIMBAL_YAW_RATIO;  //1ms内增加x°
		R_Pitch_Moto.Pitch_Target_Pos = 0;
		R_Yaw_Moto.Yaw_Target_Pos = Yaw_Moto.Yaw_Target_Pos + (1024 - rc.ch2) * RC_GIMBAL_YAW_RATIO;  //1ms内增加x°
		Yaw_Nav_Aim_Center_Pose = Gimbal.Yaw.Gyro;

	}
	L_Pitch_Moto.PidPitchPos.SetPoint = L_Pitch_Moto.Pitch_Target_Pos;
	L_Yaw_Moto.PidYawPos.SetPoint = L_Yaw_Moto.Yaw_Target_Pos;
	R_Pitch_Moto.PidPitchPos.SetPoint = R_Pitch_Moto.Pitch_Target_Pos;
	R_Yaw_Moto.PidYawPos.SetPoint = R_Yaw_Moto.Yaw_Target_Pos;
	
//	Pitch_Current_Calc(SIMPLE_PID,SIMPLE_PID);
//	Yaw_Current_Calc(SIMPLE_PID,SIMPLE_PID);
}

void Gimbal_Powerdown_Act()
{
	if (Gimbal_Last_Mode != GIMBAL_POWER_DOWN_MODE)
	{
		Gimbal_Last_Mode = GIMBAL_POWER_DOWN_MODE;
	}
	L_Pitch_Moto.PitchCurrent = 0;
	L_Yaw_Moto.YawCurrent = 0;
	R_Pitch_Moto.PitchCurrent = 0;
	R_Yaw_Moto.YawCurrent = 0;
	
	TD_Clear(&YawTD,Gimbal.Yaw.Gyro);//TD函数具体含义是啥
	
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
		Gimbal_RC_Act(RC_Ctl.rc);
		//Pitch_Motor_Test_G(0.5,750);
		break;
	case GIMBAL_PC_ACT_MODE:
		Gimbal_Armor_Act(RC_Ctl.rc);
		break;
	case GIMBAL_PC_TEST_MODE:
		Gimbal_Powerdown_Act();
		break;
//	case GIMBAL_MOUSEKEY_MODE:
//		Gimbal_MouseKey_Act(RC_Ctl.mouse);
		break;
	case GIMBAL_NAV_PLUS_AIM_MODE:
		Gimbal_Nav_Aim_Act(RC_Ctl.rc);
		break;
	default:
		Gimbal_Powerdown_Act();
		break;
	}
    

    F405.Gimbal_Flag = Inf_All_State.Gimbal_Mode;

	if (!pitch_lose_flag && Inf_All_State.Gimbal_Mode != GIMBAL_POWER_DOWN_MODE) //防堵转
	{
//		YawCan2Send(Yaw_Moto.YawCurrent);
//		PitchCan1Send(Pitch_Moto.PitchCurrent);
//		Yaw_Current_Last = Yaw_Moto.YawCurrent;
//		Pitch_Current_Last = Pitch_Moto.PitchCurrent;
		LC_BoardCan2Send(L_Pitch_Moto.Pitch_Target_Pos,L_Yaw_Moto.Yaw_Target_Pos);
		RC_BoardCan2Send(R_Pitch_Moto.Pitch_Target_Pos,R_Yaw_Moto.Yaw_Target_Pos);
	}
	else
	{
		LC_BoardCan2Send(0,0);
		RC_BoardCan2Send(0,0);//这两个用输送角度来失能肯定不对，需要修正
		//目前来看使用can2发送一个云台状态标志比较合理
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
