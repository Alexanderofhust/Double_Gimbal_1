#include "main.h"
Inf_All_State_t Inf_All_State = {INF_POWER_DOWN_MODE,CHASSIS_POWER_DOWN_MODE,GIMBAL_POWER_DOWN_MODE,SHOOT_POWER_DOWN_MODE}; 
short Turning_flag = 0;
extern F405_typedef F405;
/**********************************************************************************************************
 *函 数 名: ModeChoose_task
 *功能说明: 模式选择任务
 *形    参: pvParameters
 *返 回 值: 无
 **********************************************************************************************************/
static void ActionInit()
{
	Inf_All_State.Global_Mode = INF_POWER_DOWN_MODE;
	Inf_All_State.Chassis_Mode = CHASSIS_POWER_DOWN_MODE;
	Inf_All_State.Gimbal_Mode = GIMBAL_POWER_DOWN_MODE;
	Inf_All_State.Shoot_Mode = SHOOT_POWER_DOWN_MODE;
}

static void ActionUpdate()
{
	switch(RC_Ctl.rc.s2)
	{
		case REMOTE_UP:
			switch(RC_Ctl.rc.s1)
			{
				case REMOTE_UP: 
					Inf_All_State.Global_Mode = INF_POWER_DOWN_MODE;
					Inf_All_State.Chassis_Mode = CHASSIS_POWER_DOWN_MODE;
					Inf_All_State.Gimbal_Mode = GIMBAL_RC_ACT_MODE;
					Inf_All_State.Shoot_Mode = SHOOT_RC_ACT_MODE;
					break;
				case REMOTE_MID: 
					Inf_All_State.Global_Mode = INF_MOUSEKEY_MODE;
					Inf_All_State.Chassis_Mode = CHASSIS_RC_FOLLOW_MODE;
					Inf_All_State.Gimbal_Mode = GIMBAL_RC_ACT_MODE;
					Inf_All_State.Shoot_Mode = SHOOT_POWER_DOWN_MODE;
					break;
				case REMOTE_DOWN:  
					Inf_All_State.Global_Mode = INF_POWER_DOWN_MODE;
					Inf_All_State.Chassis_Mode = CHASSIS_RC_PROTECT_MODE;
					Inf_All_State.Gimbal_Mode = GIMBAL_RC_ACT_MODE;
					Inf_All_State.Shoot_Mode = SHOOT_POWER_DOWN_MODE;
					break;
				default:
					Inf_All_State.Global_Mode = INF_POWER_DOWN_MODE;
					Inf_All_State.Chassis_Mode = CHASSIS_POWER_DOWN_MODE;
					Inf_All_State.Gimbal_Mode = GIMBAL_POWER_DOWN_MODE;
					Inf_All_State.Shoot_Mode = SHOOT_POWER_DOWN_MODE;
					break;
			}
			break;
		case REMOTE_MID:
			switch(RC_Ctl.rc.s1)
			{
				case REMOTE_UP: //辅瞄
					Inf_All_State.Global_Mode = INF_POWER_DOWN_MODE;
					Inf_All_State.Chassis_Mode = CHASSIS_POWER_DOWN_MODE;
					Inf_All_State.Gimbal_Mode = GIMBAL_PC_ACT_MODE;
					Inf_All_State.Shoot_Mode = SHOOT_PC_ACT_MODE;
					break;
				case REMOTE_MID: //遥控器跟随
					Inf_All_State.Global_Mode = INF_NAV_PLUS_AIM_MODE;
					Inf_All_State.Chassis_Mode = CHASSIS_RC_FOLLOW_MODE;
					Inf_All_State.Gimbal_Mode = GIMBAL_PC_ACT_MODE;
					Inf_All_State.Shoot_Mode = SHOOT_PC_ACT_MODE;
					break;			
				case REMOTE_DOWN: //导航测试模式
					Inf_All_State.Global_Mode = INF_NAV_PLUS_AIM_MODE;
					Inf_All_State.Chassis_Mode = CHASSIS_RC_PROTECT_MODE;
					Inf_All_State.Gimbal_Mode = GIMBAL_NAV_PLUS_AIM_MODE;
					Inf_All_State.Shoot_Mode = SHOOT_POWER_DOWN_MODE;
					break;
				default:
					Inf_All_State.Global_Mode = INF_POWER_DOWN_MODE;
					Inf_All_State.Chassis_Mode = CHASSIS_POWER_DOWN_MODE;
					Inf_All_State.Gimbal_Mode = GIMBAL_POWER_DOWN_MODE;
					Inf_All_State.Shoot_Mode = SHOOT_POWER_DOWN_MODE;
					break;
			}
			break;
		case REMOTE_DOWN:
			Inf_All_State.Global_Mode = INF_POWER_DOWN_MODE;
			Inf_All_State.Chassis_Mode = CHASSIS_POWER_DOWN_MODE;
			Inf_All_State.Gimbal_Mode = GIMBAL_POWER_DOWN_MODE;
			Inf_All_State.Shoot_Mode = SHOOT_POWER_DOWN_MODE;
		break;
		default:
			Inf_All_State.Global_Mode = INF_POWER_DOWN_MODE;
			Inf_All_State.Chassis_Mode = CHASSIS_POWER_DOWN_MODE;
			Inf_All_State.Gimbal_Mode = GIMBAL_POWER_DOWN_MODE;
			Inf_All_State.Shoot_Mode = SHOOT_POWER_DOWN_MODE;
		break;
			
	}
	
	F405Can2Send(&F405);
}
void Action_Update_task(void *pvParameters)
{
	ActionInit();
	while (1)
	{
		ActionUpdate();
		vTaskDelay(3);
	}
}
