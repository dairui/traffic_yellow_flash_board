#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "user_header.h"

DMA_InitTypeDef      DMA_InitStructure;

extern __IO uint16_t  ADC_buffer[ADC_CHANNEL];
/*******************************************************************************
* Function Name	: RCC_Configuration
* Description		: 配置系统时钟
* Input			: None
* Output         		: None
* Return         		: None
*******************************************************************************/
void RCC_Configuration(void)
{
   //将外设RRC寄存器重设为默认值
    RCC_DeInit();
    //设置为外部高速晶振HSE
    RCC_HSEConfig(RCC_HSE_ON);
    //等待晶振起振
    while(!RCC_WaitForHSEStartUp())
    {
    }
    //HSEStartUpStatus = RCC_WaitForHSEStartUp();
    //预取指缓存使能
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK2Config(RCC_HCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while(RCC_GetSYSCLKSource() != 0X08)
    {
    } 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
  /* 
  //AHB=1,P2=1,P1=1,,ADC=2,USB=1.5,PLLSRC=HSI,HSION
    RCC_DeInit();

    //P1分频系数2
    RCC_PCLK1Config(RCC_HCLK_Div2);
    //PLL 8 * 9 = 72M
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    //FLASH访问等待设置
    FLASH_SetLatency(FLASH_Latency_2);
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	
    //开外部时钟
    RCC_HSEConfig(RCC_HSE_ON);
    //等待时钟准备好，否则一直等待
    while(RCC_WaitForHSEStartUp() != SUCCESS);	
    //PLL使能
    RCC_PLLCmd(ENABLE);
    //等待PLL准备好
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != SET);	

    //系统时钟源PLL
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);*/
    
}
/*******************************************************************************
* Function Name	: NVIC_Configuration
* Description		: 配置中断优先级
* Input			: None
* Output         		: None
* Return         		: None
*******************************************************************************/
void NVIC_Configuration(void)
{
// 		NVIC_InitTypeDef NVIC_Initstructure;
	
     //选择向量表位置，宏在编译器内定义或自己定义在文件中，以便切换编程模式
    #ifdef  VECT_TAB_RAM        
         NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
    #else        
        NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
    #endif
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

}

/*******************************************************************************
* Function Name	: GPIO_CAN_Init(void)
* Description		: 初始化CAN用到的各个引脚
* Input			: None
* Output         		: None
* Return         		: None
*******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//CAN_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//CAN_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
	
	//yellow_flick dog
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
	//reset dog=PD4/故障是否恢复测试=PD4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4| GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure); 

	//LED1~3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_3);
	GPIO_SetBits(GPIOD, GPIO_Pin_2);
	GPIO_SetBits(GPIOD, GPIO_Pin_1);
}

/*******************************************************************************
* Function Name	: CAN_Configuration(void)
* Description		: 配置CAN的各个寄存器
* Input          		: None
* Output         		: None
* Return         		: None
*******************************************************************************/
void CAN_Configuration(void)
{
    
    CAN_InitTypeDef CAN_InitStructure;
    
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
    
    CAN_DeInit(CAN1);
    
    CAN_StructInit(&CAN_InitStructure);
    
    CAN_InitStructure.CAN_TTCM = DISABLE;   //时间触发通信模式
    CAN_InitStructure.CAN_ABOM = ENABLE;    //自动离线管理
    CAN_InitStructure.CAN_AWUM = ENABLE;    //自动唤醒模式
    CAN_InitStructure.CAN_NART = DISABLE;   //禁止自动重传模式
    CAN_InitStructure.CAN_RFLM = ENABLE;    //接收FIFO锁定模式
    CAN_InitStructure.CAN_TXFP = ENABLE;    //发送FIFO优先级  
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;     	//CAN工作模式
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;            //重新同步跳跃宽度1个时间单位
    CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;            //时间段1为8个时间单位
    CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;            //时间段2为7个时间单位
    CAN_InitStructure.CAN_Prescaler = 20;                //一个时间单位的长度         
    
    CAN_Init(CAN1,&CAN_InitStructure);
    
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode   = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale  = CAN_FilterScale_16bit;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000 << 5;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x00 << 5;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; 
    CAN_FilterInit(&CAN_FilterInitStructure);     //过滤器初始化 
 
    //CAN中断设置
    
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);  
    CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE); 
}


/*******************************************************************************
* Function Name  : IWDG_Configuration
* Description    : ?????
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void IWDG_Configuration(void)
{
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); 	/* ??0x5555,????????????? */
  IWDG_SetPrescaler(IWDG_Prescaler_256);         /* ??????256?? 40K/256=156HZ(6.4ms) */ 
  IWDG_SetReload(156);							    		/* ???? 1s/6.4MS=156 .??????0xfff*/
  IWDG_ReloadCounter();											/* ??*/
  IWDG_Enable(); 														/* ?????*/
}



/*******************************************************************************
* Function Name	: ADC_Configuration_lxb
* Description		: 配置AD
* Input          		: None
* Output         		: None
* Return         		: None
*******************************************************************************/
#define ADC1_SampleTime_cicr   ADC_SampleTime_55Cycles5   
void ADC_Configuration_lxb(void)
{
  ADC_InitTypeDef      ADC_InitStructure;
  NVIC_InitTypeDef     NVIC_InitStructure;
  GPIO_InitTypeDef 		 GPIO_InitStructure;	
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
												 
  RCC_ADCCLKConfig(RCC_PCLK2_Div4);	
	
  //ADC_port_configuration									
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  /* DMA channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr 	= (u32)(&(ADC1->DR));
  DMA_InitStructure.DMA_MemoryBaseAddr 		= (u32)(&ADC_buffer);
  DMA_InitStructure.DMA_DIR 										= DMA_DIR_PeripheralSRC;		
  DMA_InitStructure.DMA_BufferSize 		= ADC_CHANNEL;					
  DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc 		= DMA_MemoryInc_Enable; 
  DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode 			= DMA_Mode_Normal;	 
  //DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	  
  DMA_InitStructure.DMA_Priority 		= DMA_Priority_High;
  DMA_InitStructure.DMA_M2M 			= DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); //DMA 通道1 传输完成中断
// //   /* Enable DMA1 channel1 */
// //   DMA_Cmd(DMA1_Channel1, ENABLE);
   
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode 			= ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode 		= ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode 	= DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv 	= ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign 		= ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel 		= ADC_CHANNEL;   //  ADC_CHANNEL个通道，每个通道转换1次
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular ADC_Channel_0~3(ADC_INPUT1~4) configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC1_SampleTime_cicr);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2,  ADC1_SampleTime_cicr);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3,  ADC1_SampleTime_cicr);	
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4,  ADC1_SampleTime_cicr);
  
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
// //   /* Start ADC1 Software Conversion */ 
// //   ADC_SoftwareStartConvCmd(ADC1, ENABLE);		

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	 
  
}
/*******************************************************************************
* Function Name  : DMAReConfig
* Description    : 重新允许DMA,
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void DMAReConfig(void)
{
  DMA_DeInit(DMA1_Channel1);
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
  DMA_Cmd(DMA1_Channel1, ENABLE);

  ADC_Cmd(ADC1, ENABLE);
}


void AC_Detect_Enable(u8 enable)
{
	NVIC_InitTypeDef NVIC_Initstructure;
	
	//EXTI interrupts for SWITs
  NVIC_Initstructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	if(enable)
	{
		NVIC_Initstructure.NVIC_IRQChannelCmd = ENABLE;
	}
	else
	{
		NVIC_Initstructure.NVIC_IRQChannelCmd = DISABLE;
	}
	NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Initstructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_Initstructure); 
}

void EXTI_configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//外部中断变量定义   
	EXTI_InitTypeDef exti_init;  
	NVIC_InitTypeDef NVIC_Initstructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
    
  /*检测按交流电中断PB9*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);
	
	exti_init.EXTI_Line = EXTI_Line9;    
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
	exti_init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_init);
	
	
}


/*******************************************************************************
* Function Name	: Device_Init(void)
* Description		: CPU初始化
* Input          		: None
* Output         		: None
* Return         		: None
*******************************************************************************/
void Device_Init(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    CAN_Configuration();
    
    NVIC_Configuration();
		ADC_Configuration_lxb();
	EXTI_configuration();
    //IWDG_Configuration();    
}
