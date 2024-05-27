#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"
#include "main.h"
void can_filter_init(void);
void Yaw_Can1_Send(short);
void Bodan_Can1_Send(short tempX);
void Chassis_Can2_Send(short *carSpeedx, short *carSpeedy, short *carSpeedw);
void F405_Can1_Send(F405_typedef *F405_Send);
void Pitch_Can2_Send(short);
void Friction_Can2_Send(short tempX,short tempY);
void F405Can2Send(F405_typedef *F405_Send);

#endif
