#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "user_header.h"

DMA_InitTypeDef      DMA_InitStructure;

extern __IO uint16_t  ADC_buffer[ADC_CHANNEL];
/*******************************************************************************
* Function Name	: RCC_Configuration
* Description		: ����ϵͳʱ��
* Input			: None
* Output         		: None
* Return         		: None
*******************************************************************************/
void RCC_Configuration(void)
{
   //������RRC�Ĵ�������ΪĬ��ֵ
    RCC_DeInit();
    //����Ϊ�ⲿ���پ���HSE
    RCC_HSEConfig(RCC_HSE_ON);
    //�ȴ���������
    while(!RCC_WaitForHSEStartUp())
    {
    }
    //HSEStartUpStatus = RCC_WaitForHSEStartUp();
    //Ԥȡָ����ʹ��
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

    //P1��Ƶϵ��2
    RCC_PCLK1Config(RCC_HCLK_Div2);
    //PLL 8 * 9 = 72M
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    //FLASH���ʵȴ�����
    FLASH_SetLatency(FLASH_Latency_2);
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	
    //���ⲿʱ��
    RCC_HSEConfig(RCC_HSE_ON);
    //�ȴ�ʱ��׼���ã�����һֱ�ȴ�
    while(RCC_WaitForHSEStartUp() != SUCCESS);	
    //PLLʹ��
    RCC_PLLCmd(ENABLE);
    //�ȴ�PLL׼����
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != SET);	

    //ϵͳʱ��ԴPLL
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);*/
    
}
/*******************************************************************************
* Function Name	: NVIC_Configuration
* Description		: �����ж����ȼ�
* Input			: None
* Output         		: None
* Return         		: None
*******************************************************************************/
void NVIC_Configuration(void)
{
// 		NVIC_InitTypeDef NVIC_Initstructure;
	
     //ѡ��������λ�ã����ڱ������ڶ�����Լ��������ļ��У��Ա��л����ģʽ
    #ifdef  VECT_TAB_RAM        
         NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
    #else        
        NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
    #endif
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

}

/*******************************************************************************
* Function Name	: GPIO_CAN_Init(void)
* Description		: ��ʼ��CAN�õ��ĸ�������
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
	
	//reset dog=PD4/�����Ƿ�ָ�����=PD4
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
* Description		: ����CAN�ĸ����Ĵ���
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
    
    CAN_InitStructure.CAN_TTCM = DISABLE;   //ʱ�䴥��ͨ��ģʽ
    CAN_InitStructure.CAN_ABOM = ENABLE;    //�Զ����߹���
    CAN_InitStructure.CAN_AWUM = ENABLE;    //�Զ�����ģʽ
    CAN_InitStructure.CAN_NART = DISABLE;   //��ֹ�Զ��ش�ģʽ
    CAN_InitStructure.CAN_RFLM = ENABLE;    //����FIFO����ģʽ
    CAN_InitStructure.CAN_TXFP = ENABLE;    //����FIFO���ȼ�  
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;     	//CAN����ģʽ
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;            //����ͬ����Ծ���1��ʱ�䵥λ
    CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;            //ʱ���1Ϊ8��ʱ�䵥λ
    CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;            //ʱ���2Ϊ7��ʱ�䵥λ
    CAN_InitStructure.CAN_Prescaler = 20;                //һ��ʱ�䵥λ�ĳ���         
    
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
    CAN_FilterInit(&CAN_FilterInitStructure);     //��������ʼ�� 
 
    //CAN�ж�����
    
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
* Description		: ����AD
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

  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); //DMA ͨ��1 ��������ж�
// //   /* Enable DMA1 channel1 */
// //   DMA_Cmd(DMA1_Channel1, ENABLE);
   
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode 			= ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode 		= ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode 	= DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv 	= ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign 		= ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel 		= ADC_CHANNEL;   //  ADC_CHANNEL��ͨ����ÿ��ͨ��ת��1��
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
* Description    : ��������DMA,
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
	//�ⲿ�жϱ�������   
	EXTI_InitTypeDef exti_init;  
	NVIC_InitTypeDef NVIC_Initstructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
    
  /*��ⰴ�������ж�PB9*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);
	
	exti_init.EXTI_Line = EXTI_Line9;    
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
	exti_init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_init);
	
	
}


/*******************************************************************************
* Function Name	: Device_Init(void)
* Description		: CPU��ʼ��
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
