
//蜂鸣器报警与关闭
#define Alarm_On(void)   GPIO_SetBits(GPIOE, GPIO_Pin_3)
#define Alarm_Off(void)  GPIO_ResetBits(GPIOE, GPIO_Pin_3)

//给黄闪看门狗芯片高电平与低电平
#define Feed_Dog_High(void) GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define Feed_Dog_Low(void)  GPIO_ResetBits(GPIOB, GPIO_Pin_12)
//给CPU看门狗芯片高电平与低电平
#define Feed_RESET_Dog_High(void) GPIO_SetBits(GPIOD, GPIO_Pin_4)
#define Feed_RESET_Dog_Low(void)  GPIO_ResetBits(GPIOD, GPIO_Pin_4)

//故障自动回复，供电输出断电上电。
#define Fault_recovery_test_High(void) GPIO_SetBits(GPIOD, GPIO_Pin_7)
#define Fault_recovery_test_Low(void)  GPIO_ResetBits(GPIOD, GPIO_Pin_7)

//当can通讯故障，或者cpld故障，led2黄闪报警，且蜂鸣器报警
#define yellow_flicker(flash)     if(flash)\
                                  {GPIO_ResetBits(GPIOD, GPIO_Pin_1);}\
                                  else \
                                  {GPIO_SetBits(GPIOD, GPIO_Pin_1);}
//当can通讯正常，或者cpld正常，led6黄闪
#define normal_flicker(flash)     if(flash)\
                                  GPIO_ResetBits(GPIOD, GPIO_Pin_2);\
                                  else \
                                  GPIO_SetBits(GPIOD, GPIO_Pin_2)  

#define Running_Normal_on(void)   GPIO_ResetBits(GPIOD, GPIO_Pin_1)
#define Running_Normal_off(void)  GPIO_SetBits(GPIOD, GPIO_Pin_1)
// #define Running_Error_on(void)    GPIO_ResetBits(GPIOA, GPIO_Pin_10)
// #define Running_Error_off(void)   GPIO_SetBits(GPIOA, GPIO_Pin_10)

#define EVT_DATA_1_RCVD			0x0001
#define EVT_DATA_2_RCVD			0x0002
#define EVT_DATA_3_RCVD			0x0004
#define EVT_CONFLICT_MONITOR	0x0008
#define EVT_CONFLICT_ANALYSIS	0x0010
#define EVT_ADC_DONE			0x0020
#define EVT_SEND_HEART_BEAT		0x0040

#define ADC_THRESHOLD			1000
#define SWIT_INT_THRESHOLD		4
#define AC_DETECT_INT_THRESHOLD	2

#define IPI						0x20
#define MSG_CONTINUE			0xFD
#define MSG_END					0xFE

#define ERROR_LIGHT				0x01
#define ERROR_SWIT_CLOSE		0x02
#define ERROR_SWIT_OPEN			0x03

#define ADC_CHANNEL			4
#define ADC_EXCHANGE_COUNT			3
