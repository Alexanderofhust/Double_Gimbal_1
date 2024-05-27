/**
 ******************************************************************************
 * @file    counter.c
 * @brief   计时器，用于计算程序用时或者延时
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "main.h"
#include "counter.h"
/*  计时参数定义  */
#include "main.h"
uint32_t LastCNT32; // 上一个64位值
//SysTime_t SysTime;  //系统时间，秒，毫秒，微秒

extern TIM_HandleTypeDef htim2;

/**
 * @brief 计算两次调用之间的时间差
 * @param[in] cnt_last 上一次计数值
 */
//float GetDeltaT(uint32_t *cnt_last)
//{
//    volatile uint32_t cnt_now=	__HAL_TIM_GET_COUNTER(&htim2);
//    float dt = ((uint32_t)(cnt_now - *cnt_last)) / 1000000;//dt单位是us，计数器cnt到四千多秒会溢出
//    *cnt_last = cnt_now;

//    return dt;
//}

///**
// * @brief 计算两次调用之间的时间差(双精度)
// * @param[in] cnt_last 上一次计数值
// */
//double GetDeltaT64(uint32_t *cnt_last)
//{
//    volatile uint32_t cnt_now = COUNTER_TIMx->CNT;
//    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(COUNTER_SAMPLING));
//    *cnt_last = cnt_now;

//    return dt;
//}

///**
// * @brief 更新总时间 s/ms/us
// * @param[in] void
// */
//void SysTimeUpdate(void)
//{
//    volatile uint32_t cnt_now = COUNTER_TIMx->CNT;
//    static uint64_t CNT_TEMP1, CNT_TEMP2;
//    uint32_t diff_cnt = cnt_now - LastCNT32;

//    CNT_TEMP1 = (diff_cnt + SysTime.us) / 1000;             // ms
//    CNT_TEMP2 = (diff_cnt + SysTime.us) - CNT_TEMP1 * 1000; // us
//    SysTime.ms += CNT_TEMP1;
//    SysTime.us = CNT_TEMP2;
//    if (SysTime.ms >= 1000)
//    {
//        SysTime.s += 1;
//        SysTime.ms -= 1000;
//    }

//    LastCNT32 = cnt_now;
//}

/**
 * @brief 获取从程序开始到现在的时间 /s
 * @param[in] void
 */
//float GetTime_s(void)
//{
//    SysTimeUpdate();

//    float Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

//    return Timelinef32;
//}

///**
// * @brief 获取从程序开始到现在的时间 /s
// * @param[in] void
// */
//float GetTime_ms(void)
//{
//    SysTimeUpdate();

//    float Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

//    return Timelinef32;
//}

///**
// * @brief 获取从程序开始到现在的时间 /us
// * @param[in] void
// */
//uint64_t GetTime_us(void)
//{
//    SysTimeUpdate();

//    uint64_t Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

//    return Timelinef32;
//}

///**
// * @brief 延时 /us
// * @param[in] void
// */
//void Delay(uint32_t Delay)
//{
//    volatile uint32_t tickstart = COUNTER_TIMx->CNT;
//    uint32_t wait = Delay;

//    while ((COUNTER_TIMx->CNT - tickstart) < wait)
//    {
//    }
//}

///**
// * @brief 计算到输入时间值到现在的时间
// * @param[in] cnt_last 上一次计数值
// */
//float GetDeltaTtoNow(uint32_t *cnt_last)
//{
//    volatile uint32_t cnt_now = COUNTER_TIMx->CNT;
//    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(COUNTER_SAMPLING));

//    return dt;
//}
