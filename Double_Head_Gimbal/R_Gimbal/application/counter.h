#ifndef _COUNTER_H
#define _COUNTER_H

//#include <stm32f4xx.h>
//#include <stm32f4xx_conf.h>
//#include <string.h>
//#include <stdint.h>
//#include <math.h>
//#include <stdio.h>
//#include <stdlib.h>

#include "main.h"

/* Params START */
/* TIM_TimeBaseInitStructure配置到源码处修改 */
#define COUNTER_SAMPLING 1000000
/* Params END */

typedef struct SysTime_t
{
    uint32_t s;
    uint32_t ms;
    uint32_t us;
} SysTime_t;

//float GetDeltaT(uint32_t *cnt_last);
double GetDeltaT64(uint32_t *cnt_last);
void SysTimeUpdate(void);
float GetTime_s(void);
float GetTime_ms(void);
uint64_t GetTime_us(void);
void Delay(uint32_t Delay);
float GetDeltaTtoNow(uint32_t *cnt_last);

#endif // !_COUNTER_H
