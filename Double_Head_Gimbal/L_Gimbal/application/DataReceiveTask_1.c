#include "DataReceiveTask.h"
#include "cmsis_os.h"
#include "main.h"

RC_Ctl_t RC_Ctl;
extern imu_data IMU_DATA;
F105_Typedef F105;
PC_Receive_t PC_Receive;
BodanMotorReceive_Typedef BodanReceive;
extern float PitchMotorReceive, YawMotorReceive; // Pitch,Yaw����Ƕ�
float yaw_gyro_temp, pitch_gyro_temp;

Disconnect Robot_Disconnect;

extern Gimbal_Typedef Gimbal;
extern volatile long run_time_check;
extern Status_t Status;
extern char Judge_Lost;
char Chassis_ID;

//extern PCSendData pc_send_data;
extern uint8_t Remote_Receive_Flag;
extern uint8_t PC_ReceiveFlag;
extern volatile unsigned char sbus_rx_buffer[18];
//extern unsigned char PCRecbuffer[PC_RECVBUF_SIZE];

//unsigned char tempPC[PC_RECVBUF_SIZE]; //����Ҫ��Ϊȫ�ֱ�������Ȼcrc��ͨ��
short crcNopass;
short buffindex;
uint8_t CoolBuffState;



extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
volatile unsigned char sbus_rx_buffer[18];
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
/**
  * @brief          ң������ʼ��
  * @param[in]      none
  * @retval         none
  */	
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
char temp1, temp2, temp3;
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_Ctl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch0 = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch1 = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch2 = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) &0x07ff; //!< Channel 2
    rc_ctrl->rc.ch3 = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s1 = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s2 = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
	rc_ctrl->key.w = sbus_buf[14] & 0x01;				   
	rc_ctrl->key.s = (sbus_buf[14] >> 1) & 0x01;
	rc_ctrl->key.a = (sbus_buf[14] >> 2) & 0x01;
	rc_ctrl->key.d = (sbus_buf[14] >> 3) & 0x01;
	rc_ctrl->key.shift = (sbus_buf[14] >> 4) & 0x01;
	rc_ctrl->key.ctrl = (sbus_buf[14] >> 5) & 0x01;
	rc_ctrl->key.q = (sbus_buf[14] >> 6) & 0x01;
	rc_ctrl->key.e = (sbus_buf[14] >> 7) & 0x01;
	rc_ctrl->key.r = (sbus_buf[15]) & 0x01;
	rc_ctrl->key.f = (sbus_buf[15] >> 1) & 0x01;
	rc_ctrl->key.g = (sbus_buf[15] >> 2) & 0x01;
	rc_ctrl->key.z = (sbus_buf[15] >> 3) & 0x01;
	rc_ctrl->key.x = (sbus_buf[15] >> 4) & 0x01;
	rc_ctrl->key.c = (sbus_buf[15] >> 5) & 0x01;
	rc_ctrl->key.v = (sbus_buf[15] >> 6) & 0x01;
	rc_ctrl->key.b = (sbus_buf[15] >> 7) & 0x01;

			//NULL
	temp1 = sbus_buf[16];
	temp2 = sbus_buf[17];
		
	if ((RC_Ctl.rc.ch0 - 1024 < 20) && (RC_Ctl.rc.ch0 - 1024 > -20))
		RC_Ctl.rc.ch0 = 1024;
	if ((RC_Ctl.rc.ch1 - 1024 < 20) && (RC_Ctl.rc.ch1 - 1024 > -20))
		RC_Ctl.rc.ch1 = 1024;
	if ((RC_Ctl.rc.ch2 - 1024 < 20) && (RC_Ctl.rc.ch2 - 1024 > -20))
		RC_Ctl.rc.ch2 = 1024;
	if ((RC_Ctl.rc.ch3 - 1024 < 20) && (RC_Ctl.rc.ch3 - 1024 > -20))
		RC_Ctl.rc.ch3 = 1024;		
	
		Robot_Disconnect.RC_DisConnect = 0;
}

uint8_t Remote_Receive_Flag;
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
							 //����ң�������ݵı�־
							Remote_Receive_Flag=2;
//                sbus_to_rc(sbus_rx_buf[0], &RC_Ctl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң�������ݵı�־
							Remote_Receive_Flag=1;
//                sbus_to_rc(sbus_rx_buf[1], &RC_Ctl);
            }
        }
    }

}


/**********************************************************************************************************
 *�� �� ��: RCReceive_task
 *����˵��: ң�������ݴ�������
 *��    ��: ��
 *�� �� ֵ: ��
 **********************************************************************************************************/
uint32_t RC_high_water;
void RCReceive_task(void const * argument)
{
		static BaseType_t DataReceive_Exit = pdFALSE ;  //ʹ��֪ͨʵ�ּ����ź��������ڲ�ͬ�ж���Ϣ����

	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //����֪ͨ���£��򲻻���

		/********************************* ң�������ݴ��� *****************************************************/
		if			(Remote_Receive_Flag==1)			
		{  /* Current memory buffer used is Memory 1 */
			 sbus_to_rc(sbus_rx_buf[1], &RC_Ctl);		//�ٷ����뺯��
		}else if(Remote_Receive_Flag==2)
		{  /* Current memory buffer used is Memory 2 */
			sbus_to_rc(sbus_rx_buf[0], &RC_Ctl);			//�ٷ����뺯��
		}
		
		Remote_Receive_Flag = 0;
        #if INCLUDE_uxTaskGetStackHighWaterMark
		RC_high_water = uxTaskGetStackHighWaterMark(NULL);
        #endif
	}
}
