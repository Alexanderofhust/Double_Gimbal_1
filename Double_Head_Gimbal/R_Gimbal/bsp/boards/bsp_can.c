#include "bsp_can.h"
#include "main.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern Disconnect Robot_Disconnect;
extern RobotInit_Struct Sentry;
extern BodanMotorReceive_Typedef BodanReceive;
extern Pitch_6020_t Pitch_Moto;
extern Yaw_6020_t Yaw_Moto;
static float pitch_setpos,yaw_setpos;
static CAN_TxHeaderTypeDef  Can2YawSend,Can1BodanSend,Can2ChassisSend,Can2F405Send,Can1PitchSend,Can1FrictionSend;
void Can1Receive0(CAN_RxHeaderTypeDef rx_message1);
void Can2Receive0(CAN_RxHeaderTypeDef rx_message0);
float PitchMotorReceive, YawMotorReceive; // Pitch,Yaw编码器
short PitchMotorSpeed,YawMotorSpeed;

JudgeData_1_t JudgeRecieveData;

extern F105_Typedef F105;
extern ShootTask_typedef Shoot;
extern uint8_t CoolBuffState;
char PitchMotor_ReceiveFlag;
extern ZeroCheck_Typedef ZeroCheck_Pitch;
short FrictionReceive[2];
extern F405_typedef F405;
extern Gimbal_Typedef Gimbal;
extern char q_flag, f_flag, HighFreq_flag;
extern float Theta_chassis;
extern uint32_t cnt_last;
extern double yuzhi;
/**********************************************************************************************************
 *函 数 名: can_filter_init
 *功能说明: can配置
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
//CAN 2 FIFO0 接收中断	
	  can_filter_st.FilterBank = 15;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_st.FilterIdHigh =0x096<<5;//大符 0x096<<5;
    can_filter_st.FilterIdLow =0x094<<5; //裁判系统0x094<<5;
    can_filter_st.FilterMaskIdHigh =0x100<<5;//底盘控制反馈 0x100<<5;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
		can_filter_st.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
//CAN 2 FIFO1 接收中断
	  can_filter_st.FilterBank = 16;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_st.FilterIdHigh = Sentry.YawMotorID<<5;//YAW电机
    can_filter_st.FilterIdLow =  0x204<<5;//拨弹电机0x204
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
		can_filter_st.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);	
//CAN 1 FIFO0 接收中断
    can_filter_st.FilterBank = 0;
		can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_st.FilterIdHigh = Sentry.PitchMotorID<<5;//PITCH电机
    can_filter_st.FilterIdLow =  0x203<<5;//摩擦轮1
    can_filter_st.FilterMaskIdHigh =0x201<<5;//摩擦轮2
    can_filter_st.FilterMaskIdLow = 0x202<<5;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
		can_filter_st.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//CAN 1 FIFO1 接收中断		
		can_filter_st.FilterBank = 1;
		can_filter_st.FilterActivation = ENABLE;
		can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
		can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
		can_filter_st.FilterIdHigh = 0x0000;//老陀螺仪，现在这块闲置
		can_filter_st.FilterIdLow =  0x0000;
		can_filter_st.FilterMaskIdHigh =0x0000;
		can_filter_st.FilterMaskIdLow = 0x0000;
		can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
		can_filter_st.SlaveStartFilterBank = 15;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
//CAN 1 发送中断	
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
//CAN 2 发送中断
		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
} 
/**********************************************************************************************************
 *函 数 名: YawCan1Send
 *功能说明: 发送yaw电流值
 *形    参: yaw电流值
 *返 回 值: 无
 **********************************************************************************************************/
static uint8_t yaw_send_data[8];
uint16_t current_limit_9025 = 1600;
void YawCan2Send(short tempX)
{
		uint32_t send_mail_box;
		
		Can2YawSend.IDE=CAN_ID_STD;
		Can2YawSend.RTR=CAN_RTR_DATA;
		Can2YawSend.DLC=0x08;
		Can2YawSend.StdId=0x280;
		tempX = LIMIT_MAX_MIN(tempX, 1600, -1600);
		memcpy(yaw_send_data,&tempX,2);
		HAL_CAN_AddTxMessage(&hcan2, &Can2YawSend, yaw_send_data, &send_mail_box);
}

/**********************************************************************************************************
 *函 数 名: BodanCan1Send
 *功能说明: 发送拨弹电机电流值
 *形    参: 拨弹电机电流值
 *返 回 值: 无
 **********************************************************************************************************/
static uint8_t bodan_send_data[8];
void BodanCan1Send(short tempX)
{
		uint32_t send_mail_box;
		Can1BodanSend.StdId=0x200;
		Can1BodanSend.IDE=CAN_ID_STD;
		Can1BodanSend.RTR=CAN_RTR_DATA;
		Can1BodanSend.DLC=0x08;
		tempX = LIMIT_MAX_MIN(tempX, 8000, -8000);
	switch (Sentry.BodanMotorID)
	{
	case 0x201:
		bodan_send_data[0] = (unsigned char)((tempX >> 8) & 0xff);
		bodan_send_data[1] = (unsigned char)(tempX & 0xff);
		break;
	case 0x202:
		bodan_send_data[2] = (unsigned char)((tempX >> 8) & 0xff);
		bodan_send_data[3] = (unsigned char)(tempX & 0xff);
		break;
	case 0x203:
		bodan_send_data[4] = (unsigned char)((tempX >> 8) & 0xff);
		bodan_send_data[5] = (unsigned char)(tempX & 0xff);
		break;
	case 0x204:
		bodan_send_data[6] = (unsigned char)((tempX>>8)&0xff);
		bodan_send_data[7] = (unsigned char)(tempX&0xff);
		break;
	default:
		break;
	}
  HAL_CAN_AddTxMessage(&hcan1, &Can1BodanSend, bodan_send_data, &send_mail_box);
}

/**********************************************************************************************************
 *函 数 名: ChassisCan1Send
 *功能说明: 把xyw向速度发送至B板 Can1Send0
 *形    参: (short *carSpeedx, short *carSpeedy, short *carSpeedw)
 *返 回 值: 无
 **********************************************************************************************************/
static uint8_t chassis_send_data[8];
void ChassisCan2Send(short *carSpeedx, short *carSpeedy, short *carSpeedw)
{
		uint32_t send_mail_box;
		Can2ChassisSend.StdId=0x101;
		Can2ChassisSend.IDE=CAN_ID_STD;
		Can2ChassisSend.RTR=CAN_RTR_DATA;
		Can2ChassisSend.DLC=0x08;

		F405.Yaw_100 = (short)(Theta_chassis/3.1415926f*180 * 100);
	
	memcpy(&chassis_send_data[0], carSpeedx, 2);
	memcpy(&chassis_send_data[2], carSpeedy, 2);
	memcpy(&chassis_send_data[4], carSpeedw, 2);
	memcpy(&chassis_send_data[6], &F405.Yaw_100, 2);
	
  HAL_CAN_AddTxMessage(&hcan2, &Can2ChassisSend, chassis_send_data, &send_mail_box);
}

/**********************************************************************************************************
 *函 数 名: F405Can1Send
 *功能说明: 与B板通信	Can1Send1
 *形    参: 超级电容使用允许位  小陀螺模式标志位  大陀螺模式标志位  单挑模式标志位
 *返 回 值: 无
 **********************************************************************************************************/
static uint8_t F405_send_data[8];
char HighFreq_flag  = 0;

void F405Can2Send(F405_typedef *F405_Send)
{
		uint32_t send_mail_box;
		Can2F405Send.StdId=0x102;
		Can2F405Send.IDE=CAN_ID_STD;
		Can2F405Send.RTR=CAN_RTR_DATA;
		Can2F405Send.DLC=0x08;

    if(Inf_All_State.Shoot_Mode == SHOOT_PC_ACT_MODE)
        F405.AutoFire_Flag = 1;
    else
        F405.AutoFire_Flag = 0;

	F405_Send->Pitch_100 = (short)(Gimbal.Pitch.Gyro * 100);
	F405_Send->Send_Pack1 = ((F405_Send->AutoFire_Flag & 0x01) << 0) |
							((F405_Send->Laser_Flag & 0x01) << 1) |
							((F405_Send->Graphic_Init_Flag & 0x01) << 2) |
							((HighFreq_flag & 0x01) << 3) |
							((F405_Send->Fric_Flag & 0x01) << 4) |
										((1111 & 0x07) << 5);//随便写的enemy_id
	F405_Send->Chassis_Flag = Inf_All_State.Chassis_Mode ;
	memcpy(&F405_send_data[0], &F405_Send->SuperPowerLimit, 1);
	memcpy(&F405_send_data[1], &F405_Send->Chassis_Flag, 1);
	memcpy(&F405_send_data[2], &F405_Send->Pitch_100, 2);
	memcpy(&F405_send_data[4], &F405_Send->Gimbal_Flag, 1);
	memcpy(&F405_send_data[5], &F405_Send->Send_Pack1, 1);
		
  HAL_CAN_AddTxMessage(&hcan2, &Can2F405Send, F405_send_data, &send_mail_box);
}

/**********************************************************************************************************
 *函 数 名: PitchCan2Send
 *功能说明: 发送pitch电流值
 *形    参: pitch电流值
 *返 回 值: 无
 **********************************************************************************************************/
static uint8_t pitch_send_data[8];
void PitchCan1Send(short tempX)
{
		uint32_t send_mail_box;
		Can1PitchSend.StdId=0x1FF;
		Can1PitchSend.IDE=CAN_ID_STD;
		Can1PitchSend.RTR=CAN_RTR_DATA;
		Can1PitchSend.DLC=0x08;
		tempX = LIMIT_MAX_MIN(tempX, 30000, -30000);
	switch (Sentry.PitchMotorID)
	{
	case 0x205:
		pitch_send_data[0] = (unsigned char)((tempX >> 8) & 0xff);
		pitch_send_data[1] = (unsigned char)(tempX & 0xff);
		break;
	case 0x206:
		pitch_send_data[2] = (unsigned char)((tempX >> 8) & 0xff);
		pitch_send_data[3] = (unsigned char)(tempX & 0xff);
		break;
	case 0x207:
		pitch_send_data[4] = (unsigned char)((tempX >> 8) & 0xff);
		pitch_send_data[5] = (unsigned char)(tempX & 0xff);
		break;
	case 0x208:
		pitch_send_data[6] = (unsigned char)((tempX>>8)&0xff);
		pitch_send_data[7] = (unsigned char)(tempX&0xff);
		break;
	default:
		break;
	}
  HAL_CAN_AddTxMessage(&hcan1, &Can1PitchSend, pitch_send_data, &send_mail_box);
}

/**********************************************************************************************************
 *函 数 名: FrictionCan2Send
 *功能说明: 发送摩擦轮电机电流值
 *形    参: 摩擦轮电机电流值
 *返 回 值: 无
 **********************************************************************************************************/
static uint8_t friction_send_data[8];
void FrictionCan1Send(short tempX,short tempY)
{
		uint32_t send_mail_box;
		Can1FrictionSend.StdId=0x200;
		Can1FrictionSend.IDE=CAN_ID_STD;
		Can1FrictionSend.RTR=CAN_RTR_DATA;
		Can1FrictionSend.DLC=0x08;
	
		tempX = LIMIT_MAX_MIN(tempX, 9000, -9000);
		tempY = LIMIT_MAX_MIN(tempY, 9000, -9000);
	switch (Sentry.FricMotorID[0])
	{
	case 0x201:
		friction_send_data[0] = (unsigned char)((tempX >> 8) & 0xff);
		friction_send_data[1] = (unsigned char)(tempX & 0xff);
		break;
	case 0x202:
		friction_send_data[2] = (unsigned char)((tempX >> 8) & 0xff);
		friction_send_data[3] = (unsigned char)(tempX & 0xff);
		break;
	case 0x203:
		friction_send_data[4] = (unsigned char)((tempX >> 8) & 0xff);
		friction_send_data[5] = (unsigned char)(tempX & 0xff);
		break;
	case 0x204:
		friction_send_data[6] = (unsigned char)((tempX>>8)&0xff);
		friction_send_data[7] = (unsigned char)(tempX&0xff);
		break;
	default:
		break;
	}
	switch (Sentry.FricMotorID[1])
	{
	case 0x201:
		friction_send_data[0] = (unsigned char)((tempY >> 8) & 0xff);
		friction_send_data[1] = (unsigned char)(tempY & 0xff);
		break;
	case 0x202:
		friction_send_data[2] = (unsigned char)((tempY >> 8) & 0xff);
		friction_send_data[3] = (unsigned char)(tempY & 0xff);
		break;
	case 0x203:
		friction_send_data[4] = (unsigned char)((tempY >> 8) & 0xff);
		friction_send_data[5] = (unsigned char)(tempY & 0xff);
		break;
	case 0x204:
		friction_send_data[6] = (unsigned char)((tempY>>8)&0xff);
		friction_send_data[7] = (unsigned char)(tempY&0xff);
		break;
	default:
		break;
	}
  HAL_CAN_AddTxMessage(&hcan1, &Can1FrictionSend, friction_send_data, &send_mail_box);
}

/**********************************************************************************************************
 *函 数 名: HAL_CAN_RxFifo0MsgPendingCallback
 *功能说明:FIFO 0邮箱中断回调函数
 *形    参: 
 *返 回 值: 无
 **********************************************************************************************************/
JudgeData_1_t JudgeData_1;
JudgeData_2_t JudgeData_2;
extern Gimbal_mode_t R_Gimbal;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	  CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	  __HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
		
	if (hcan->Instance == CAN1)
	{      // 处理CAN1的数据	
		if (rx_header.StdId == Sentry.PitchMotorID)
		{	
			PitchMotorReceive   = rx_data[0] << 8 | rx_data[1];
			PitchMotorSpeed 	= rx_data[2] << 8 | rx_data[3];
		
			if (!PitchMotor_ReceiveFlag)
			{
				ZeroCheck_Pitch.LastValue = PitchMotorReceive;
			}
			PitchMotor_ReceiveFlag = 1;
		}
		if (rx_header.StdId == Sentry.FricMotorID[0])
			{
					FrictionReceive[0] = rx_data[2] << 8 | rx_data[3];
					Robot_Disconnect.Friction_DisConnect[0] = 0;
			}
			else if (rx_header.StdId == Sentry.FricMotorID[1])
			{
					FrictionReceive[1] = rx_data[2] << 8 | rx_data[3];
					Robot_Disconnect.Friction_DisConnect[1] = 0;
			}
			else if (rx_header.StdId == 0x202)
			{
					BodanReceive.Angle 			= rx_data[0] << 8 | rx_data[1];
					BodanReceive.RealSpeed  = rx_data[2] << 8 | rx_data[3];
					Robot_Disconnect.Pluck_DisConnect = 0;
			}
	}
	else if (hcan->Instance == CAN2) 
		{
				// 处理CAN2的数据
			if (rx_header.StdId == Sentry.PitchMotorID)
			{	
					PitchMotorReceive   = rx_data[0] << 8 | rx_data[1];
					PitchMotorSpeed 		= rx_data[2] << 8 | rx_data[3];
					Robot_Disconnect.PitchMotor_DisConnect = 0;
					
			}else if (rx_header.StdId == 0x094)
			{	
				memcpy(&JudgeRecieveData,rx_data,8);
			}
			else if (rx_header.StdId == 0x092)
			{	
				memcpy(&pitch_setpos,rx_data,4);
				memcpy(&yaw_setpos,rx_data+4,4);
				Pitch_Moto.Pitch_Target_Pos=pitch_setpos;
				Yaw_Moto.Yaw_Target_Pos = yaw_setpos;
			}
			else if (rx_header.StdId == 0x093)
			{					
				R_Gimbal.fri_mode=rx_data[0];
				R_Gimbal.bodan_mode=rx_data[1];
				R_Gimbal.AutoFire_Flag=rx_data[2];
				R_Gimbal.Laser_Flag=rx_data[3];
				R_Gimbal.Gimbal_Flag=rx_data[4];
				R_Gimbal.Freq_state=rx_data[5];
			}
			
		}
}
/**********************************************************************************************************
 *函 数 名: HAL_CAN_RxFifo0MsgPendingCallback
 *功能说明:FIFO 1邮箱中断回调函数
 *形    参: 
 *返 回 值: 无
 **********************************************************************************************************/
uint8_t rx_data_test[8] = {0xff};
uint16_t YawMotorReceive_short = 0;
uint16_t YawMotorSpeed_short = 0;
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)//FIFO 1邮箱中断回调函数
{
		CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
	  __HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
		
	if (hcan->Instance == CAN1)
	{
			
	}
	else if(hcan->Instance == CAN2)
	{
			if(rx_header.StdId == Sentry.YawMotorID)
			{
				memcpy(&YawMotorReceive_short,rx_data+6,2);
				memcpy(&YawMotorSpeed_short,rx_data+4,2);
				YawMotorSpeed = YawMotorSpeed_short;
				YawMotorReceive = YawMotorReceive_short;
				memcpy(rx_data_test,rx_data,8);
				Robot_Disconnect.YawMotor_DisConnect = 0;
				Robot_Disconnect.Pluck_DisConnect = 0;
			}
	}	
}
