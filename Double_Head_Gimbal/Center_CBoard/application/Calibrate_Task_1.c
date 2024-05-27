#include "Calibrate_task.h"
#include "cmsis_os.h"
#include "bmi088driver.h"

//cali_sensor_t       cali_sensor_gyro;
static imu_cali_t   gyro_cali;       //gyro cali data陀螺仪校准数据
static uint32_t *cali_sensor_buf = (uint32_t *)&gyro_cali;//gyro_cali校准数据地址
char calibrate_done_flag=0;

/**
  * @brief          陀螺仪设备校准函数
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表初始化
                    CALI_FUNC_CMD_ON: 代表开始校准
  * @retval         0:校准任务还没有完成
                    1:校准任务已经完成
  */
static uint16_t count_time = 0;
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        INS_set_cali_gyro(local_cali_t->scale, local_cali_t->offset);    
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
				INS_cali_gyro(local_cali_t->scale, local_cali_t->offset, &count_time);

        if (count_time > GYRO_CALIBRATE_TIME)
        {
            count_time = 0;
//            cali_buzzer_off();
						RC_restart(SBUS_RX_BUF_NUM);
            return 1;
        }
        else
        {
						RC_unable();
//            imu_start_buzzer();    
            return 0;
        }
    }

    return 0;//不会运行到这里
}


extern TaskHandle_t Calibrate_task_handle;//任务句柄
extern bmi088_real_data_t bmi088_real_data;//获取温度
void Calibrate_task(void const * argument)
{
	
	cali_gyro_hook(cali_sensor_buf,CALI_FUNC_CMD_INIT);
	
	//温度达到44度才开始标定
	while(bmi088_real_data.temp<44.0f)
	{
		osDelay(100);
	}
	
	while(1)
	{
		if(cali_gyro_hook(cali_sensor_buf,CALI_FUNC_CMD_ON))
		{
			calibrate_done_flag=1;
			vTaskDelete(Calibrate_task_handle);
		}
		
	osDelay(CALIBRATE_CONTROL_TIME);
		
	}
}

