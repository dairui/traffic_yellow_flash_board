#include "stm32f10x.h"
#include <RTL.h>
#include <RTX_CAN.h>
#include "stm32f10x_conf.h"
#include "user_header.h"

U8 watchdog_count = 0x07;

/*交流电计数*/
unsigned int AC_count = 0;

void Device_Init(void);
void AC_Detect_Enable(u8 enable);

OS_TID tid_send_CAN;
OS_TID tid_recv_CAN;
OS_TID tid_lamp_ctl;
OS_TID tid_green_led_flash;
OS_TID tid_red_led_flash;
OS_TID tid_conflict_monitor;
OS_TID tid_conflict_analysis;
OS_TID tid_heart_beat;
OS_TID tid_power_detect;

os_mbx_declare (CAN_send_mailbox, 20);
_declare_box(mpool,sizeof(CAN_msg),32);

__task void init(void);
__task void task_send_CAN(void);
__task void task_recv_CAN(void);
__task void task_lamp_ctl(void);
__task void task_green_led_flash(void);
__task void task_hard_dog_flash(void);
__task void task_conflict_monitor(void);
__task void task_conflict_analysis(void);
__task void task_heart_beat(void);
__task void task_power_detect(void);

u8 Lights_status_1[3] = {0};
u8 Lights_status_2[3] = {0};
u8 Picked_lights_status[12] = {0};
u16 Picked_lights_status_map;
u16 conflict_map;
u16 ADC_status_map;
u16 Channels_enabled = 0xffff;
u16 Fault_recovery_test =0;
u8 Walker_channels = 0;
u8 ID_Num = 1;
u8 Work_normal = 1;
__IO uint16_t  ADC_buffer[ADC_CHANNEL];
u16 Last_error = 0;

u16 timer_count = 0;

extern void DMAReConfig(void);


int main (void)
{
	os_sys_init(init);
	Fault_recovery_test_High();			//High =允许12V供电输出
}

__task void init (void) 
{
	Device_Init();
	
	os_mbx_init (CAN_send_mailbox, sizeof(CAN_send_mailbox));
	_init_box (mpool, sizeof(mpool), sizeof(CAN_msg));
	
	tid_send_CAN = os_tsk_create(task_send_CAN, 3);
	tid_recv_CAN = os_tsk_create(task_recv_CAN, 2);
	tid_green_led_flash = os_tsk_create(task_green_led_flash, 2);
	tid_red_led_flash = os_tsk_create(task_hard_dog_flash, 2);
	tid_power_detect = os_tsk_create(task_power_detect, 2);

	os_tsk_delete_self();
}

__task void task_send_CAN(void) 
{
	void *msg;

	CAN_init(1, 500000);               /* CAN controller 1 init, 500 kbit/s   */
	CAN_rx_object (1, 2,  33, DATA_TYPE | STANDARD_TYPE); /* Enable reception */
                                       /* of message on controller 1, channel */
                                       /* is not used for STM32 (can be set to*/
                                       /* whatever value), data frame with    */
                                       /* standard id 33                      */
	CAN_start (1);                     /* Start controller 1                  */

	while (1)
	{
		os_mbx_wait (CAN_send_mailbox, &msg, 0xffff);
		
		CAN_send (1, msg, 0x0F00);  /* Send msg_send on controller 1       */
		_free_box (mpool, msg);
	}
}
/*
  喂狗的心跳值  ，若相位板主控板can通讯故障，以及cpld的故障，则不给心跳值
  heart_to_dog.bit0   主控板心跳值
  heart_to_dog.bit1   相位板1心跳值
  heart_to_dog.bit2   相位板2心跳值
  heart_to_dog.bit3   相位板3心跳值
  heart_to_dog.bit4   相位板4心跳值
  一次类推
*/
u16  heart_to_dog = 0;
__task void task_recv_CAN(void)
{	
	CAN_msg RxMessage;
	
	for (;;)
	{
		if (CAN_receive (1, &RxMessage, 0x00FF) == CAN_OK)
		{
			if(RxMessage.data[0] == IPI && RxMessage.data[1] == 0xD1 && RxMessage.data[2] == 0x00)
			{//主控板数据接收，表明主控板正常
				heart_to_dog = (heart_to_dog | (1<< 0));
				// The bitmap of available vehicle channels
				Channels_enabled = ((RxMessage.data[4]<<8)|RxMessage.data[3]);
			}
			else if(RxMessage.data[0] == IPI && RxMessage.data[1] == 0x83 && RxMessage.data[4] == 0x02)
			{//相位板心跳	RxMessage.data[3]为相位板ID
				if(RxMessage.data[5]==0)
					heart_to_dog = (heart_to_dog | (1<<RxMessage.data[3]));				
			}
		}
	}
}

u8 Flashing_pulse_1s = 0;

void GPIO_PinReverse(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN(GPIO_Pin));
    
    GPIOx->ODR ^= GPIO_Pin;
}

__task void task_green_led_flash(void)
{
	u16 count = 0;
	
	os_itv_set (10);  //100ms
	while (1)
	{
		os_itv_wait ();
		count++;
		if ((count%5)==0) Flashing_pulse_1s = ~Flashing_pulse_1s;
		if(count > 6000 ) count = 0;
		if(Work_normal==1)
		{
			Fault_recovery_test_High();
			normal_flicker(Flashing_pulse_1s);                    //led1正常闪烁
			Fault_recovery_test=0;
		}			
		else
		{
			if(count > 500)    //短暂断电后再供电，主控板相位板重新上电测试故障
				{                  //100*500=5分钟，上电测试
					Fault_recovery_test_High();
					Fault_recovery_test=0;
				}
				
			else if(count < 400)  
				GPIO_PinReverse(GPIOD, GPIO_Pin_7);   //切断12V输出电源
			yellow_flicker(Flashing_pulse_1s);                    //且led2黄闪
			//蜂鸣器闪烁报警
			if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8))
				Alarm_On();
			else
				Alarm_Off();

			GPIO_SetBits(GPIOD, GPIO_Pin_2);
		}
			
	}
}
/*******************************************************************************
* Function Name  : GPIO_PinReverse
* Description    : GPIO位取反
* Input          : GPIO口：GPIO_TypeDef* GPIOx
* Output         : GPIO位：uint16_t GPIO_Pin
* Return         : None
*******************************************************************************/


__task void task_hard_dog_flash(void)
{
	U8 id=0,i;U16 id_wtdg = 0;
	U16 count_cpu = 0;
	U16 OPEN_OK = 0;  //上电开机设定时间后检测

	CAN_msg *msg_error;
	os_itv_set (1);//10ms
	while (1)
	{
		os_itv_wait ();
		if(count_cpu > 400 ) count_cpu = 0;
		count_cpu++;
		if(OPEN_OK < 3000)  OPEN_OK++;  //开机时间10s

		Feed_RESET_Dog_Low();    //CPU喂狗 Low=600ms
		if ((count_cpu%50)==0)  Feed_RESET_Dog_High();   //CPU喂狗  High=20ms

  	Feed_Dog_Low();    //黄闪喂狗 Low=1s  High=20
	if((count_cpu%100)==0 )     //1s内心跳小于节点数目,，则报警		
	{
		if((heart_to_dog != watchdog_count))    //所有板子心跳包没有在规定时间内到达
			{
				Work_normal=0;				
				/*相位板故障，  上报主控板故障黄闪*/
				if(OPEN_OK > 1000) 
				{
					id_wtdg = ((U16)(~heart_to_dog))& watchdog_count;
					for(i=0;i<16;i++)
					{
						if(((id_wtdg>>i) &1 )== 1)
						{
							id=i;break;
						}
					}
					msg_error = _calloc_box (mpool);
					//						Line_num  ID_Num	  bad_light_num	  error_type
					//{ 1, {IPI, 0xD2, 0x00, 0x01,    0xFF,        0xFF,         0xFF,     0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};			
					msg_error->id = 1;
					msg_error->data[0] = IPI;
					msg_error->data[1] = 0xD2;
					msg_error->data[2] = 0x0;
					msg_error->data[3] = 0x01;
					msg_error->data[4] = id;
					msg_error->data[7] = MSG_END;
					msg_error->len = sizeof(msg_error->data);
					msg_error->ch = 2;
					msg_error->format = STANDARD_FORMAT;
					msg_error->type = DATA_FRAME;		
					msg_error->data[5] = 0xFF;//相位板故障
					msg_error->data[6] = 0xFF;
					os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);	
				}
			}
			else                                                    //所有心跳到，黄闪喂狗
			{
				Work_normal=1;
				Alarm_Off();
				GPIO_PinReverse(GPIOB, GPIO_Pin_12);                  //所有心跳到，黄闪喂狗
//				Feed_Dog_High();
			}
		  heart_to_dog = 0;                                       //同步标记初始化
		}
	}
}


float V220 = 0.0,DC_IN = 0.0,V_A = 0.0,V_B = 0.0;
unsigned char pow_IOGA = 0;
unsigned char pow_IOGB = 0;
unsigned char pow_S = 0;
__task void task_power_detect(void)
{
	OS_RESULT result;
	int i, j;
	uint16_t ADC_samples[ADC_CHANNEL*ADC_EXCHANGE_COUNT];
	uint32_t adc_temp[ADC_CHANNEL];
	
	while (1)
	{
		// wait for the current to be stable
		os_dly_wait(500); //5000ms
		
		// sample ADC reading 3 times
		for (i=0; i<ADC_EXCHANGE_COUNT; i++)
		{
			DMAReConfig();
			result = os_evt_wait_and (EVT_ADC_DONE, 0xffff);
			if (result == OS_R_TMO)
			{
				i--;
			}
			else
			{
				for (j=0; j<ADC_CHANNEL; j++)
				{
					ADC_samples[ADC_EXCHANGE_COUNT*j+i] = ADC_buffer[j];
				}
				os_dly_wait(2); //20ms
			}
		}
		// samples done
		
		for (j=0; j<ADC_CHANNEL; j++)
		{
			for (i=0; i<ADC_EXCHANGE_COUNT; i++)
			{
				adc_temp[j] += ADC_samples[ADC_EXCHANGE_COUNT*j+i];
			}
			adc_temp[j] /= ADC_EXCHANGE_COUNT;
		}
		V220 = 3.3*adc_temp[0]/4095;
		
		DC_IN = 3.3*adc_temp[1]/4095;
		
		V_A = 3.3*adc_temp[2]/4095;
		
		V_B = 3.3*adc_temp[3]/4095;
		
		for (j=0; j<ADC_CHANNEL; j++)
		{
			adc_temp[j] = 0;
		}
	
		pow_IOGA = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5);
		pow_IOGB = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
		
		// get SWITs status map
		AC_Detect_Enable(1);
		os_dly_wait(6);
		AC_Detect_Enable(0);
		
		if (AC_count > AC_DETECT_INT_THRESHOLD)
		{
			pow_S = 1;
		}
		else
		{
			pow_S=0;
		}	
		AC_count = 0;
	}
}

