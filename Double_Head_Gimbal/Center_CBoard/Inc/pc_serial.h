#ifndef _PC_SERIAL_H
#define _PC_SERIAL_H

#include "main.h"
#include "algorithmOfCRC.h"

#include "FreeRTOS.h"
#include "task.h"
#pragma pack(push, 1)     //不进行字节对齐
typedef struct PCRecvData
{
    uint8_t Head;
	int8_t Aim_v_x;
	int8_t Aim_v_y;
	float Aim_Yaw;
	int16_t Aim_Pitch;
	uint8_t FireState : 2;
	uint8_t FrictionState : 2; //开摩擦轮标志位
	uint8_t ShootFreqMod : 2;
	uint8_t _ : 2;
	uint8_t crc8;
} PCRecvData;
typedef struct PCSendData //数据顺序不能变,注意32字节对齐 //11 bytes
{
    uint8_t start_flag;
	float yaw;
    short pitch;
    uint16_t remain_bullet;
	int8_t vx;
	int8_t vy;
	uint8_t Shoot_State;
    int8_t crc8;
} PCSendData;
#pragma pack(pop) //不进行字节对齐


#define PC_SENDBUF_SIZE sizeof(PCSendData)
#define PC_RECVBUF_SIZE sizeof(PCRecvData)

extern unsigned char PCbuffer[PC_RECVBUF_SIZE];
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

typedef enum{
	ARMOR_NO_AIM,
	ARMOR_AIMED
}ARMOR_STATE_ENUM;

typedef struct{
	float Nav_Speed_x; //mm/s
	float Nav_Speed_y; //mm/s
	float Nav_Speed_w; //°/s
	short Nav_State;
}Nav_Cmd_t;


extern ARMOR_STATE_ENUM armor_state;
extern float pc_pitch,pc_yaw;
extern Nav_Cmd_t NAV_cmd;

void PCReceive(unsigned char *PCbuffer);
void SendtoPC(void);
void NAVReceive(uint8_t Buf[]);
void SendtoNAV(void);

extern PCRecvData pc_recv_data;

#endif // !_PC_SERIAL_H
