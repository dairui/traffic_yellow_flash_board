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
OS_TID tid_green_led_flash;
OS_TID tid_red_led_flash;
OS_TID tid_yellow_flash;

os_mbx_declare (CAN_send_mailbox, 20);
_declare_box(mpool,sizeof(CAN_msg),32);

__task void init(void);
__task void task_send_CAN(void);
__task void task_recv_CAN(void);
__task void task_green_led_flash(void);
__task void task_hard_dog_flash(void);
__task void task_yellow_flash(void);

u16 Channels_enabled = 0xffff;
u16 Fault_recovery_test =0;
__IO uint16_t  ADC_buffer[ADC_CHANNEL];
u8 Yellow_Flash = 1;
u8 TT_cont1 = 0;
u8 TT_cont2 = 0;
u8 TT_cont3 = 0;
u8 TT_cont4 = 0;
	U16 Yellow_count	= 0;
	u8 TT1_A6_0C ;

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
	tid_yellow_flash = os_tsk_create(task_yellow_flash, 2);

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
	CAN_msg *msg_error;
	
	for (;;)
	{
		if (CAN_receive (1, &RxMessage, 0x00FF) == CAN_OK)
		{
			if(RxMessage.data[0] == IPI && RxMessage.data[1] == 0xD1 && RxMessage.data[2] == 0x00)
			{//主控板数据接收，表明主控板正常
				heart_to_dog = (heart_to_dog | (1<< 0));
				// The bitmap of available vehicle channels
				Channels_enabled = ((RxMessage.data[4]<<8)|RxMessage.data[3]);
				TT_cont1++;
			}
			else if(RxMessage.data[0] == IPI && RxMessage.data[1] == 0x83 && RxMessage.data[4] == 0x02)
			{//相位板心跳	RxMessage.data[3]为相位板ID
				if(RxMessage.data[5]==0)
					heart_to_dog = (heart_to_dog | (1<<RxMessage.data[3]));
					TT_cont2++;
			}
			else if (RxMessage.data[0] == IPI && RxMessage.data[1] == 0xE1 && (RxMessage.data[5] && 0x40==0x40))
			{
				tsk_lock ();
				msg_error = _calloc_box (mpool);
				msg_error->id = CAN_ID_NO;
				msg_error->data[0] = IPI;
				msg_error->data[1] = 0xA6;
				msg_error->data[3] = RxMessage.data[3];
				msg_error->data[6] = 0x08;
				msg_error->data[7] = MSG_END;
				msg_error->len = sizeof(msg_error->data);
				msg_error->ch = 2;
				msg_error->format = STANDARD_FORMAT;
				msg_error->type = DATA_FRAME;
				tsk_unlock ();
				os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);

				Yellow_Flash = 1;
				heart_to_dog = 0;
			}
			else if (RxMessage.data[0] == IPI && RxMessage.data[1] == 0xE3 && (RxMessage.data[5] && 0x40==0x40))
			{
				tsk_lock ();
				msg_error = _calloc_box (mpool);
				msg_error->id = CAN_ID_NO;
				msg_error->data[0] = IPI;
				msg_error->data[1] = 0xA6;
				msg_error->data[3] = RxMessage.data[3];
				msg_error->data[6] = 0x06;
				msg_error->data[7] = MSG_END;
				msg_error->len = sizeof(msg_error->data);
				msg_error->ch = 2;
				msg_error->format = STANDARD_FORMAT;
				msg_error->type = DATA_FRAME;
				tsk_unlock ();
				os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);
				Yellow_Flash = 1;
				heart_to_dog = 0;
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
	CAN_msg *msg_error;
//	U16 Yellow_count	= 0;
//	u8 TT1_A6_0C ;
	os_itv_set (50);  //500ms
	while (1)
	{
		os_itv_wait ();
		if(Yellow_Flash==0)
		{
			GPIO_PinReverse(GPIOD, GPIO_Pin_2);  //正常LED闪烁
			GPIO_PinReverse(GPIOB, GPIO_Pin_12);  //给黄闪看门狗喂狗
			Alarm_Off();                          //关闭蜂鸣器
			Fault_recovery_test_High();          //供电输出打开
			Yellow_count=0;          //故障复位计数器清0

			if (TT1_A6_0C ==0)
			{
				tsk_lock ();  			//  disables task switching
				msg_error = _calloc_box (mpool);
				msg_error->id = CAN_ID_NO;
				msg_error->data[0] = IPI;
				msg_error->data[1] = 0xA6;
				msg_error->data[3] = 0x00;
				msg_error->data[6] = 0x0C;
				msg_error->data[7] = MSG_END;
				msg_error->len = sizeof(msg_error->data);
				msg_error->ch = 2;
				msg_error->format = STANDARD_FORMAT;
				msg_error->type = DATA_FRAME;
				tsk_unlock (); 			//  enable  task switching
				os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);
				TT1_A6_0C=1 ;
			}
		}
		else
		{
			TT1_A6_0C=0;
			GPIO_PinReverse(GPIOD, GPIO_Pin_1);    //黄闪故障灯
			GPIO_PinReverse(GPIOE, GPIO_Pin_3);   //黄闪开蜂鸣器
			normal_flicker(0) ;    //关闭正常LED闪烁
			Yellow_count++;
			if(Yellow_count>100)  //2*60=1分钟   重启设备
			{
				TT_cont3++;
				tsk_lock ();
				msg_error = _calloc_box (mpool);
				msg_error->id = CAN_ID_NO;
				msg_error->data[0] = IPI;
				msg_error->data[1] = 0xA7;
				msg_error->data[3] = 0xFF;
				msg_error->data[5] = 0x01;
				msg_error->data[7] = MSG_END;
				msg_error->len = sizeof(msg_error->data);
				msg_error->ch = 2;
				msg_error->format = STANDARD_FORMAT;
				msg_error->type = DATA_FRAME;
				tsk_unlock ();
				os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);
				Yellow_count=0;
			}
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
	CAN_msg *msg_error;

	while (1)
	{
		Yellow_Flash = 1;
		// yellow flash for boot-up
		while (1)
		{
			if(heart_to_dog&&0x01==1)
			{
				os_dly_wait(600);
				tsk_lock ();
				msg_error = _calloc_box (mpool);
				msg_error->id = CAN_ID_NO;
				msg_error->data[0] = IPI;
				msg_error->data[1] = 0xA6;
				msg_error->data[6] = 0x0B;   //黄闪板启动引起的黄闪
				msg_error->data[7] = MSG_END;
				msg_error->len = sizeof(msg_error->data);
				msg_error->ch = 2;
				msg_error->format = STANDARD_FORMAT;
				msg_error->type = DATA_FRAME;
				tsk_unlock ();
				os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);
				break;
			}
		}
//		while(1)
//		{
//			if(heart_to_dog == watchdog_count)
//			{
//				tsk_lock ();
//				msg_error = _calloc_box (mpool);
//				msg_error->id = CAN_ID_NO;
//				msg_error->data[0] = IPI;
//				msg_error->data[1] = 0xA7;
//				msg_error->data[3] = 0xFF;
//				msg_error->data[5] = 0x01;
//				msg_error->data[7] = MSG_END;
//				msg_error->len = sizeof(msg_error->data);
//				msg_error->ch = 2;
//				msg_error->format = STANDARD_FORMAT;
//				msg_error->type = DATA_FRAME;
//				tsk_unlock ();
//				os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);  //重启设备
//				heart_to_dog = 0;
//				break;
//			}
//		}
		while(1)
		{
			if(heart_to_dog == watchdog_count)
			{
				heart_to_dog = 0;
				Yellow_Flash = 0;
				break;
			}
		}
		break;
	}

	// stop yellow flash and tell every board
	os_dly_wait(200);
	tsk_lock ();
	msg_error = _calloc_box (mpool);
	msg_error->id = CAN_ID_NO;
	msg_error->data[0] = IPI;
	msg_error->data[1] = 0xA6;
	msg_error->data[6] = 0x0A;
	msg_error->data[7] = MSG_END;
	msg_error->len = sizeof(msg_error->data);
	msg_error->ch = 2;
	msg_error->format = STANDARD_FORMAT;
	msg_error->type = DATA_FRAME;
	tsk_unlock ();
	os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);
	Yellow_Flash = 0;

	os_itv_set (100);//1000ms
	while (1)
	{
		os_itv_wait ();
		if((heart_to_dog != watchdog_count) &&(Yellow_Flash==0))   //所有板子心跳包没有在规定时间内到达
		{
			Yellow_Flash=1;

			id_wtdg = ((U16)(~heart_to_dog))& watchdog_count;
			for(i=0;i<16;i++)
			{
				if(((id_wtdg>>i) &1 )== 1)
				{
					id=i;break;
				}
			}
				tsk_lock ();
				msg_error = _calloc_box (mpool);
				msg_error->id = CAN_ID_NO;
				msg_error->data[0] = IPI;
				msg_error->data[1] = 0xA6;
				msg_error->data[3] = id;
				msg_error->data[6] = 0x09;
				msg_error->data[7] = MSG_END;
				msg_error->len = sizeof(msg_error->data);
				msg_error->ch = 2;
				msg_error->format = STANDARD_FORMAT;
				msg_error->type = DATA_FRAME;
				tsk_unlock ();
				os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);
				Yellow_Flash = 1;
				heart_to_dog = 0;
        TT_cont2++;
		}
		else if (heart_to_dog == watchdog_count)
		{
			Yellow_Flash = 0;
			heart_to_dog = 0;                                       //同步标记初始化
		}
	}
}

void feed_cpu_dog(void)
{
	Feed_RESET_Dog_High();
	os_dly_wait(1);
	Feed_RESET_Dog_Low();
}

void feed_yf_dog(void)
{
	Feed_Dog_Low();
	os_dly_wait(1);
	Feed_Dog_High();
}
__task void task_yellow_flash(void)
{
	os_itv_set (1);
	u8 cpu_dog_cnt = 0;
	u8 yf_dog_cnt = 0;

	while (1)
	{
		os_itv_wait ();

		cpu_dog_cnt++;
		yf_dog_cnt++;

		// feed cpu dog
		if (cpu_dog_cnt % 2 == 0)
		{
			Feed_RESET_Dog_High();
		}
		ele if (cpu_dog_cnt % 2 == 1)
		{
			Feed_RESET_Dog_Low();
		}

		if (!Yellow_Flash)
		{
			if (yf_dog_cnt % 2 == 0)
			{
				Feed_Dog_Low();
			}
			else if (yf_dog_cnt % 2 == 1)
			{
				Feed_Dog_High();
			}
		}
	}
}

