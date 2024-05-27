/**
 ******************************************************************************
 * @file    pc_uart.c
 * @brief   serial数据接发
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "pc_serial.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

unsigned char PCbuffer[PC_RECVBUF_SIZE];
unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

PCRecvData pc_recv_data;
PCSendData pc_send_data;

float pc_pitch,pc_yaw;
uint8_t PC_Shoot_flag;
uint8_t last_shoot_flag;
Nav_Cmd_t NAV_cmd;


extern Shoot_Cmd_t Shoot_Cmd;
char PC_Receive_Flag_2_Armor = 0;

void PCReceive(unsigned char *PCbuffer)
{
    if(PCbuffer[0] == '!' )
	{
		memcpy(&pc_recv_data,PCbuffer,PC_RECVBUF_SIZE);
		pc_yaw = pc_recv_data.Aim_Yaw;		
		pc_pitch = (pc_recv_data.Aim_Pitch/100.0f);
		NAV_cmd.Nav_Speed_x =  pc_recv_data.Aim_v_x*20.0f;
		NAV_cmd.Nav_Speed_y =  - pc_recv_data.Aim_v_y*20.0f;
		Shoot_Cmd.Shoot_State = pc_recv_data.FireState;
		Shoot_Cmd.Friction_cmd = pc_recv_data.FrictionState;
		Shoot_Cmd.Shoot_Freq_cmd = pc_recv_data.ShootFreqMod;
		PC_Receive_Flag_2_Armor = 1;
	}
}

/**
 * @brief 在这里写发送数据的封装
 * @param[in] void
 */
extern F105_Typedef F105;
extern Gimbal_Typedef Gimbal;
extern int ShootCount_Number;//射击的总子弹数

void SendtoPCPack(unsigned char *buff)
{
    pc_send_data.start_flag = '!';
    pc_send_data.Shoot_State = Shoot_Cmd.Shoot_State_send;
	pc_send_data.remain_bullet = ShootCount_Number;
	pc_send_data.vx = 0;
	pc_send_data.vy = 0;
    pc_send_data.pitch = (short)(Gimbal.Pitch.Gyro * 100.0f);
    pc_send_data.yaw = Gimbal.Yaw.Gyro;
	pc_send_data.crc8 = 0;
    //Append_CRC8_Check_Sum((unsigned char *)&pc_send_data, PC_SENDBUF_SIZE);
    memcpy(buff, (void *)&pc_send_data, PC_SENDBUF_SIZE);
}

/**
 * @brief 发送数据调用
 * @param[in] void
 */
void SendtoPC(void)
{
    SendtoPCPack(SendToPC_Buff);
	CDC_Transmit_FS(SendToPC_Buff,PC_SENDBUF_SIZE);
}
