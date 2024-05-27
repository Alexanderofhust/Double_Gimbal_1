#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H
#include "struct_typedef.h"
#include "main.h"

#define CALI_FUNC_CMD_ON        1                   //开始校准
#define CALI_FUNC_CMD_INIT      0                   //初始化

#define CALIBRATE_CONTROL_TIME  1                   //osDelay time,  means 1ms.1ms 系统延时

typedef __packed struct
{
    uint8_t name[3];                                    //device name
    uint8_t cali_done;                                  //0x55 means has been calibrated
    uint8_t flash_len : 7;                              //buf lenght
    uint8_t cali_cmd : 1;                               //1 means to run cali hook function,
    uint32_t *flash_buf;                                //link to device calibration data
    bool_t (*cali_hook)(uint32_t *point, bool_t cmd);   //cali function
} cali_sensor_t;

void Calibrate_task(void const * argument);




#endif
