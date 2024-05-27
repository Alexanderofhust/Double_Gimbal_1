/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "arm_math.h"
#include "HUST_PID.h"
#include "DataReceiveTask.h"
#include "Gimbal_Task.h"
#include "DataSendTask.h"
#include "INS_task.h"	
#include "ActionTask.h"		
#include "ShootTask.h"	
#include "ZeroCheckTask.h"	
#include "ControlTask.h"	
#include "ChassisTask.h"		
	
#include "bsp_delay.h"
#include "bsp_can.h"
#include "bsp_rc.h"
	
#include "counter.h"
#include "queueData.h"	
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/*步兵初始参数结构体*/
typedef struct
{
	  int8_t gyro_pn;
	  int8_t motor_pn;
	  float FricMotor_pn[2];
	  float BodanMotor_pn;
		unsigned short MagOpen;
		unsigned short MagClose;
	  unsigned short Pitch_init;
	  unsigned short Yaw_init;
		unsigned short Solo_Yaw_init;	//左单挑模式的Yaw_init
		unsigned short Low_FrictionSpeed;
		unsigned short Medium_FrictionSpeed;
		unsigned short High_FrictionSpeed;
	  unsigned short PitchMotorID;
	  unsigned short YawMotorID;
	  unsigned short FricMotorID[2];
	  unsigned short BodanMotorID;
    short pitch_max_motor;
		short pitch_min_motor;
		short pitch_max_gyro;
		short pitch_min_gyro;
    short init_delta_pitch; //平地上陀螺仪和电机角差值
		
}RobotInit_Struct;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))
#define ABS(x) ((x)>0? (x):(-(x))) 
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
