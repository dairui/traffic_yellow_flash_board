/*----------------------------------------------------------------------------
 *      RL-ARM - CAN
 *----------------------------------------------------------------------------
 *      Name:    CAN_STM32F103.c
 *      Purpose: CAN Driver, Hardware specific module for ST STM32F103
 *      Rev.:    V4.70
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>                      /* RTX kernel functions & defines      */
#include <RTX_CAN.h>                  /* CAN Generic functions & defines     */
#include <STM32F10x.h>                /* STM32F10x Definitions               */


#define __CTRL1                 1
#define __CTRL2                 2

#define __FILTER_BANK_MAX      13     /* filter banks 0..13 */


/************************* CAN Hardware Configuration ************************/

// *** <<< Use Configuration Wizard in Context Menu >>> ***

// <o> CAN Peripheral Clock (in Hz) <1-1000000000>
//     <i> Same as PCLK1
#define CAN_CLK               36000000

// *** <<< End of Configuration section             >>> ***


/*----------------------------------------------------------------------------
 *      CAN RTX Hardware Specific Driver Functions
 *----------------------------------------------------------------------------
 *  Functions implemented in this module:
 *    static      void CAN_set_timing      (U32 ctrl, U32 tseg1, U32 tseg2, U32 sjw, U32 brp)
 *    static CAN_ERROR CAN_hw_set_baudrate (U32 ctrl, U32 baudrate)
 *           CAN_ERROR CAN_hw_setup        (U32 ctrl)
 *           CAN_ERROR CAN_hw_init         (U32 ctrl, U32 baudrate)
 *           CAN_ERROR CAN_hw_start        (U32 ctrl)
 *           CAN_ERROR CAN_hw_testmode     (U32 ctrl, U32 testmode)
 *           CAN_ERROR CAN_hw_tx_empty     (U32 ctrl)
 *           CAN_ERROR CAN_hw_wr           (U32 ctrl,         CAN_msg *msg)
 *    static      void CAN_hw_rd           (U32 ctrl, U32 ch, CAN_msg *msg)
 *           CAN_ERROR CAN_hw_set          (U32 ctrl,         CAN_msg *msg)
 *           CAN_ERROR CAN_hw_rx_object    (U32 ctrl, U32 ch, U32 id, U32 object_para)
 *           CAN_ERROR CAN_hw_tx_object    (U32 ctrl, U32 ch,         U32 object_para)
 *    Interrupt fuction
 *---------------------------------------------------------------------------*/

/* Static functions used only in this module                                 */
static void CAN_hw_rd            (U32 ctrl, U32 ch, CAN_msg *msg);
static BOOL CAN_hw_rx_object_chk (U32 ctrl, U16 bank,         U32 object_para);
static void CAN_hw_rx_object_add (U32 ctrl, U16 bank, U32 id, U32 object_para);

/* CAN Controller Register Addresses                                         */
CAN_TypeDef *CAN_CTRL[] = { CAN1 };

/* Static variables used only in this module                                 */
static U16 CAN1_filterIdx = 0;
static U16 CAN2_filterIdx = 0;

/************************* Auxiliary Functions *******************************/

/*--------------------------- CAN_set_timing --------------------------------
 *
 *  Setup the CAN timing with specific parameters
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              tseg1:      Specifies Time Segment before the sample point
 *              tseg2:      Time Segment after the sample point
 *              sjw:        Synchronisation Jump Width
 *              brp:        Baud Rate Prescaler
 *
 *---------------------------------------------------------------------------*/

static void CAN_set_timing (U32 ctrl, U32 tseg1, U32 tseg2, U32 sjw, U32 brp) {
  CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];

  CANx->BTR &= ~(((          0x03) << 24) | ((            0x07) << 20) | ((            0x0F) << 16) | (          0x3FF));
  CANx->BTR |=  ((((sjw-1) & 0x03) << 24) | (((tseg2-1) & 0x07) << 20) | (((tseg1-1) & 0x0F) << 16) | ((brp-1) & 0x3FF));
}


/*--------------------------- CAN_set_baudrate ------------------------------
 *
 *  Setup the requested baudrate
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

static CAN_ERROR CAN_hw_set_baudrate (U32 ctrl, U32 baudrate)  {
  U32 brp;

  /* Note: this calculations fit for PCLK1 = 36MHz */
  /* Determine which nominal time to use for requested baudrate and set
     appropriate bit timing                                                  */
  if (baudrate <= 500000)  {
    brp  = (CAN_CLK / 18) / baudrate;

    /* Load the baudrate registers BTR                                       */
    /* so that sample point is at about 72% bit time from bit start          */
    /* TSEG1 = 12, TSEG2 = 5, SJW = 4 => 1 CAN bit = 18 TQ, sample at 72%    */
    CAN_set_timing(ctrl, 12, 5, 4, brp);
  }  else if (baudrate <= 1000000)  {
    brp  = (CAN_CLK / 9) / baudrate;

    /* Load the baudrate registers BTR                                       */
    /* so that sample point is at about 72% bit time from bit start          */
    /* TSEG1 = 5, TSEG2 = 3, SJW = 3 => 1 CAN bit = 9 TQ, sample at 66%      */
    CAN_set_timing(ctrl,  5, 3, 3, brp);
  }  else  {
    return CAN_BAUDRATE_ERROR;
  }

  return CAN_OK;
}


/*************************** Module Functions ********************************/

/*--------------------------- CAN_hw_setup ----------------------------------
 *
 *  Setup CAN transmit and receive PINs and interrupt vectors
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_setup (U32 ctrl)  {

  /* 1. Enable CAN controller Clock                                          */
  RCC->APB1ENR |= (1 << 25);

// // //   /* Note: MCBSTM32 uses PB8 and PB9 for CAN */
// // //   /* 2. Setup CAN Tx and Rx pins                                             */
// // //   RCC->APB2ENR |= (1 << 0);                         // enable clock for Alternate Function
// // //   AFIO->MAPR   &= 0xFFFF9FFF;                       // reset CAN remap
// // //   AFIO->MAPR   |= 0x00004000;                       //   set CAN remap, use PB8, PB9
// // //   /* GPIOB clock enable */
// // //   RCC->APB2ENR |= (1 << 3);
// // //   /* Configure CAN pin: RX PB.8 input push pull */
// // //   GPIOB->CRH &= ~(0x0F<<0);
// // //   GPIOB->CRH |=  (0x08<<0);
// // //   
// // //   /* Configure CAN pin: TX PB.9 alternate output push pull */
// // //   GPIOB->CRH &= ~(0x0F<<4);
// // //   GPIOB->CRH |=  (0x0B<<4);
// // // 	
	
//=======================================	
//=======================================	
//=======================================	
  /* Note: MCBSTM32 uses PB8 and PB9 for CAN */
  /* 2. Setup CAN Tx and Rx pins                                             */
  RCC->APB2ENR |= (1 << 0);                         // enable clock for Alternate Function
  AFIO->MAPR   &= 0xFFFF9FFF;                       // reset CAN remap
	AFIO->MAPR   |= 0x00000000;                       //   set CAN remap, use PA11, PA12
  /* GPIOA clock enable */
  RCC->APB2ENR |= (1 << 2);  
  /* Configure CAN pin: RX PA.11 input push pull */ 
  GPIOA->CRH &= ~(0x0F<<12);
  GPIOA->CRH |=  (0x08<<12);
  
  /* Configure CAN pin: TX PA.12 alternate output push pull */
  GPIOA->CRH &= ~(0x0F<<16);
  GPIOA->CRH |=  (0x0B<<16);
	


  /* 3. Setup IRQ vector for CAN interrupt                                   */
  /* not necessary */

  /* 4. Enable CAN interrupt                                                 */
  NVIC->IP  [4] |= 0x10000000;                /* set priority lower than SVC */
  NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);        /* enable CAN TX interrupt     */

                                              /* only FIFO 0 is used         */
  NVIC->IP  [5] |= 0x00000010;                /* set priority lower than SVC */
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);       /* enable CAN RX interrupt     */

  return CAN_OK;
}

/*--------------------------- CAN_hw_init -----------------------------------
 *
 *  Initialize the CAN hardware
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_init (U32 ctrl, U32 baudrate)  {
  CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];

  CANx->MCR = (1 << 0);               /* Init mode, with enabled automatic   */
                                      /* retransmission only FIFO 0,         */
                                      /* transmit mailbox 0 used             */
  CANx->IER = ((1 << 1) | (1 << 0));  /* FIFO 0 msg pending,                 */
                                      /* Transmit mbx empty                  */

  if (CAN_hw_set_baudrate(ctrl, baudrate) != CAN_OK)         /* Set baudrate */
    return CAN_BAUDRATE_ERROR;

  return CAN_OK;
}

/*--------------------------- CAN_hw_start ----------------------------------
 *
 *  reset CAN initialisation mode
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_start (U32 ctrl)  {
  CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];

  CANx->MCR &= ~(1 << 0);             /* normal operating mode, reset INRQ   */
  while (CANx->MSR & (1 << 0));

  return CAN_OK;
}

/*--------------------------- CAN_set_testmode ------------------------------
 *
 *  Setup the CAN testmode
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              testmode:   Kind of testmode
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_testmode (U32 ctrl, U32 testmode) {
  CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];
	
	CANx->BTR &= ~((1U << 31) | (1 << 30)); 
  CANx->BTR |=   ((0 << 31) | (0 << 30));	
	
  return CAN_OK;
}

/*--------------------------- CAN_hw_tx_empty -------------------------------
 *
 *  Check if transmit mailbox 0 is available for usage (empty)
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_empty (U32 ctrl)  {
  CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];

  if ((os_sem_wait (wr_sem[ctrl-1], 0) != OS_R_TMO)){ /* If semaphore is free*/
    if ((CANx->TSR & (1 << 26)) != 0) /* Transmit mailbox 0 is empty         */
      return CAN_OK;
    else 
      os_sem_send(wr_sem[ctrl-1]);    /* Return a token back to semaphore    */
  }

  return CAN_TX_BUSY_ERROR;
}

/*--------------------------- CAN_hw_wr -------------------------------------
 *
 *  Write CAN_msg to the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              msg:        Pointer to CAN message to be written to hardware
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_wr (U32 ctrl, CAN_msg *msg)  {
  CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];

  /* Reset TIR register                                                      */
  CANx->sTxMailBox[0].TIR  = (U32)0;                       /* reset TXRQ bit */

  /* Setup the identifier information                                        */
  if (msg->format == STANDARD_FORMAT)  {                   /*    Standard ID */
    CANx->sTxMailBox[0].TIR |= (U32)(msg->id << 21);
  }  else  {                                               /* Extended ID    */
    CANx->sTxMailBox[0].TIR |= (U32)(msg->id <<  3) | (1 << 2);
  }

  /* Setup type information                                                  */
  if (msg->type == REMOTE_FRAME)  {                        /* REMOTE FRAME   */
    CANx->sTxMailBox[0].TIR |= (1 << 1);
  }

  /* Setup data bytes                                                        */
  CANx->sTxMailBox[0].TDLR = (((U32)msg->data[3] << 24) |
                              ((U32)msg->data[2] << 16) |
                              ((U32)msg->data[1] <<  8) |
                              ((U32)msg->data[0])       );
  CANx->sTxMailBox[0].TDHR = (((U32)msg->data[7] << 24) |
                              ((U32)msg->data[6] << 16) |
                              ((U32)msg->data[5] <<  8) |
                              ((U32)msg->data[4])       );

  /* Setup length                                                            */
  CANx->sTxMailBox[0].TDTR &= ~0x0000000F;
  CANx->sTxMailBox[0].TDTR |=  (msg->len & 0x0000000F);

  CANx->IER |= (1 << 0);                           /*  enable  TME interrupt */ 

  /*  transmit message                                                       */
  CANx->sTxMailBox[0].TIR  |=  (1 << 0);                   /*   set TXRQ bit */

  return CAN_OK;
}

/*--------------------------- CAN_hw_rd -------------------------------------
 *
 *  Read CAN_msg from the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              ch:         Ignored
 *              msg:        Pointer where CAN message will be read
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_hw_rd (U32 ctrl, U32 ch, CAN_msg *msg)  {
  CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];

  /* Read identifier information                                             */
  if ((CANx->sFIFOMailBox[0].RIR & (1 << 2)) == 0) {        /* Standard ID   */
    msg->format = STANDARD_FORMAT;
    msg->id     = 0x000007FFUL & (CANx->sFIFOMailBox[0].RIR >> 21);
  }  else  {                                                /* Extended ID   */
    msg->format = EXTENDED_FORMAT;
    msg->id     = 0x1FFFFFFFUL & (CANx->sFIFOMailBox[0].RIR >> 3);
  }

  /* Read type information                                                   */
  if ((CANx->sFIFOMailBox[0].RIR & (1 << 1)) == 0) {
    msg->type =   DATA_FRAME;                               /* DATA   FRAME  */
  }  else  {
    msg->type = REMOTE_FRAME;                               /* REMOTE FRAME  */
  }

  /* Read length (number of received bytes)                                  */
  msg->len = (U32)0x0000000F & CANx->sFIFOMailBox[0].RDTR;

  /* Read data bytes                                                         */
  msg->data[0] = (U32)0x000000FF & (CANx->sFIFOMailBox[0].RDLR);
  msg->data[1] = (U32)0x000000FF & (CANx->sFIFOMailBox[0].RDLR >> 8);
  msg->data[2] = (U32)0x000000FF & (CANx->sFIFOMailBox[0].RDLR >> 16);
  msg->data[3] = (U32)0x000000FF & (CANx->sFIFOMailBox[0].RDLR >> 24);

  msg->data[4] = (U32)0x000000FF & (CANx->sFIFOMailBox[0].RDHR);
  msg->data[5] = (U32)0x000000FF & (CANx->sFIFOMailBox[0].RDHR >> 8);
  msg->data[6] = (U32)0x000000FF & (CANx->sFIFOMailBox[0].RDHR >> 16);
  msg->data[7] = (U32)0x000000FF & (CANx->sFIFOMailBox[0].RDHR >> 24);
}

/*--------------------------- CAN_hw_set ------------------------------------
 *  Set a message that will automatically be sent as an answer to the REMOTE
 *  FRAME message
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              msg:        Pointer to CAN message to be set
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_set (U32 ctrl, CAN_msg *msg)  {

  return CAN_NOT_IMPLEMENTED_ERROR;
}

/*--------------------------- CAN_hw_rx_object_chk --------------------------
 *
 *  This function checks if an object that is going to be used for the message 
 *  reception can be added to the Filter Bank
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              bank:       Index of the Filter bank (0..__FILTER_BANK_MAX)
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     BOOL:  True   Object can     be added to Filter Bank
 *                     False  Object can not be added to Filter Bank
 *---------------------------------------------------------------------------*/

BOOL CAN_hw_rx_object_chk (U32 ctrl, U16 bank, U32 object_para)  {
  CAN_TypeDef *CANx = CAN_CTRL[0];

  if ((CANx->FA1R & (1UL << bank)) == 0) {                            /* filter bank unused ? */
    return __TRUE;
  }
  else {
    /* check if another identifier is possible */
    if      (((CANx->FS1R & (1UL << bank)) == 0) &&                   /* 16-bit identifier format is used */
            ((object_para & FORMAT_TYPE) == STANDARD_TYPE)) {         /* Standard identifier must be added */
      /* check filter bank for free 16 bit filter */
                                                                      /*          position n+0 is always used */
      if      ((CANx->sFilterRegister[bank].FR1 & 0xFFFF0000) == 0) { /* check if position n+1 is used */
        return __TRUE;
      }
      else if ((CANx->sFilterRegister[bank].FR2 & 0x0000FFFF) == 0) { /* check if position n+2 is used */
        return __TRUE;
      }
      else if ((CANx->sFilterRegister[bank].FR2 & 0xFFFF0000) == 0) { /* check if position n+3 is used */
        return __TRUE;
      }
 
    }
    else if (((CANx->FS1R & (1UL << bank)) != 0) &&                   /* 32-bit identifier format is used */
            ((object_para & FORMAT_TYPE) == EXTENDED_TYPE)) {         /* Extended identifier must be added */
      /* check filter bank for free 32 bit filter */
                                                                      /*          position n+0 is always used */
      if      ((CANx->sFilterRegister[bank].FR2             ) == 0) { /* check if position n+1 is used */
        return __TRUE;
      }
    }

  }

  return __FALSE;
}

/*--------------------------- CAN_hw_rx_object_add --------------------------
 *
 *  This function adds an object that is going to be used for the message 
 *  reception to the Filter Bank
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              bank:       Index of the Filter bank (0..__FILTER_BANK_MAX)
 *              id:         Identifier of receiving messages
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     none
 * *---------------------------------------------------------------------------*/

void CAN_hw_rx_object_add (U32 ctrl, U16 bank, U32 id, U32 object_para)  {
  CAN_TypeDef *CANx = CAN_CTRL[0];
  U32 can2sb;

  CANx->FMR  |=  (1 << 0);                                            /* set Initialisation mode for filter banks */

  if ((CANx->FA1R & (1UL << bank)) == 0) {                            /* filter bank unused ? */
                                                                      /* add identifier */
    if ((object_para & FORMAT_TYPE) == STANDARD_TYPE)  {              /* Standard format used */ 
      CANx->FM1R |=  (1UL << bank);                                   /* Two 32-bit registers of filter bank x are in Identifier List mode */
      CANx->FS1R &= ~(1UL << bank);                                   /* Dual 16-bit scale configuration */

      CANx->sFilterRegister[bank].FR1 = (U32)(id << 5) |              /* 16-bit identifier */
                           ((object_para & FRAME_TYPE) << 3);         /* RTR bit */
      CANx->sFilterRegister[bank].FR2 = 0;                            /* empty */
    }
    else {                                                            /* Extended format used */
      CANx->FM1R |=  (1UL << bank);                                   /* Two 32-bit registers of filter bank x are in Identifier List mode */
      CANx->FS1R |=  (1UL << bank);                                   /* Single 32-bit scale configuration */

      CANx->sFilterRegister[bank].FR1 = (U32)(id << 3) | (1 << 2) |   /* 32-bit identifier */
                           (object_para & FRAME_TYPE);                /* RTR bit */
      CANx->sFilterRegister[bank].FR2 = 0;                            /* empty */
    }

    if (ctrl == __CTRL1)                                              /* increase filter number depending on CAN controller */
      CAN1_filterIdx++;
    else
      CAN2_filterIdx++;

  }
  else {                                                              /* filter bank used */
    /* check for free position */
    if      (((CANx->FS1R & (1UL << bank)) == 0) &&                   /* 16-bit identifier format is used */
             ((object_para & FORMAT_TYPE) == STANDARD_TYPE)) {        /* Standard identifier must be added */
      /* check filter bank for free 16 bit filter */
                                                                      /*          position n+0 is always used */
      if      ((CANx->sFilterRegister[bank].FR1 & 0xFFFF0000) == 0) { /* check if position n+1 is used */
        CANx->sFilterRegister[bank].FR1 |= (((U32)(id << 5)) |        /* store 16-bit identifier on position n+1 */
                             ((object_para & FRAME_TYPE) << 3)) << 16;/* RTR bit */
      }
      else if ((CANx->sFilterRegister[bank].FR2 & 0x0000FFFF) == 0) { /* check if position n+2 is used */
        CANx->sFilterRegister[bank].FR2  =  ((U32)(id << 5)) |        /* store 16-bit identifier on position n+2 */
                             ((object_para & FRAME_TYPE) << 3);       /* RTR bit */
      }
      else if ((CANx->sFilterRegister[bank].FR2 & 0xFFFF0000) == 0) { /* check if position n+3 is used */
        CANx->sFilterRegister[bank].FR2 |= (((U32)(id << 5)) |        /* store 16-bit identifier on position n+3 */
                             ((object_para & FRAME_TYPE) << 3)) << 16;/* RTR bit */
      }
    }
    else if (((CANx->FS1R & (1UL << bank)) != 0) &&                   /* 32-bit identifier format is used */
             ((object_para & FORMAT_TYPE) == EXTENDED_TYPE)) {        /* Extended identifier must be added */
      /* check filter bank for free 32 bit filter */
                                                                      /*          position n+0 is always used */
      if      ((CANx->sFilterRegister[bank].FR2             ) == 0) { /* check if position n+1 is used */
        CANx->sFilterRegister[bank].FR2 = (U32)(id << 3) | (1 << 2) | /* store 32-bit identifier on position n+1 */
                             (object_para & FRAME_TYPE);              /* RTR bit */
      }
    }
  }

  CANx->FFA1R &= ~(1UL << bank);                                      /* assign filter to FIFO 0 */
  CANx->FA1R  |=  (1UL << bank);                                      /* activate filter */
  can2sb = (CAN2_filterIdx == 0) ? __FILTER_BANK_MAX : (__FILTER_BANK_MAX - (CAN2_filterIdx - 1));
  CANx->FMR    =  ((can2sb) << 8);                                    /* reset Initialisation mode for filter banks */
                                                                      /* and  set CAN2 start bank */
}

/*--------------------------- CAN_hw_rx_object ------------------------------
 *
 *  This function setups object that is going to be used for the message 
 *  reception
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              ch:         Index of object used for reception
 *              id:         Identifier of receiving messages
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_rx_object (U32 ctrl, U32 ch, U32 id, U32 object_para)  {
  U16 CAN_filterBnk = 0;
  U16 CAN_filterIdx = 0;
  
  /* find free filter */
  switch (ctrl) {
    case __CTRL1:        /* for CTRL1 we check from 0 .. __FILTER_BANK_MAX - CAN2_filterIdx */
      CAN_filterIdx = (CAN2_filterIdx == 0) ? __FILTER_BANK_MAX : __FILTER_BANK_MAX - CAN2_filterIdx - 1; 

      for (CAN_filterBnk = 0; CAN_filterBnk < CAN_filterIdx; CAN_filterBnk++) {
        if (CAN_hw_rx_object_chk (ctrl, CAN_filterBnk, object_para) == __TRUE) {
          CAN_hw_rx_object_add (ctrl, CAN_filterBnk, id, object_para);
          break;                                    /* stop the loop if we have added the object */
        }
      }
      break;

    case __CTRL2:        /* for CTRL2 we check from __FILTER_BANK_MAX .. [1 | CAN1_filterIdx] */
      CAN_filterIdx = (CAN1_filterIdx == 0) ? 0 : CAN1_filterIdx - 1; 

      for (CAN_filterBnk = __FILTER_BANK_MAX; CAN_filterBnk > CAN_filterIdx; CAN_filterBnk--) {
        if (CAN_hw_rx_object_chk (ctrl, CAN_filterBnk, object_para) == __TRUE) {
          CAN_hw_rx_object_add (ctrl, CAN_filterBnk, id, object_para);
          break;                                    /* stop the loop if we have added the object */
        }
      }
      break;
  }

  if (CAN_filterBnk != CAN_filterIdx)
    return CAN_OK;
  else
    return CAN_OBJECTS_FULL_ERROR;
}


/*--------------------------- CAN_hw_tx_object ------------------------------
 *
 *  This function setups object that is going to be used for the message 
 *  transmission, the setup of transmission object is not necessery so this 
 *  function is not implemented
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              ch:         Index of object used for transmission
 *              object_para:Object parameters (standard or extended format, 
 *                          data or remote frame)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_object (U32 ctrl, U32 ch, U32 object_para)  {

  return CAN_NOT_IMPLEMENTED_ERROR;
}


/************************* Interrupt Functions *******************************/

/*--------------------------- CAN_IRQ_Handler -------------------------------
 *
 *  CAN interrupt function 
 *  If transmit interrupt occured and there are messages in mailbox for 
 *  transmit it writes it to hardware and starts the transmission
 *  If receive interrupt occured it reads message from hardware registers 
 *  and puts it into receive mailbox
 *---------------------------------------------------------------------------*/

void USB_HP_CAN1_TX_IRQHandler (void) {
  CAN_msg *ptrmsg;

  if (CAN1->TSR & (1 << 0)) {                      /* request completed mbx 0 */
    CAN1->TSR |= (1 << 0);                    /* reset request complete mbx 0 */

    /* If there is a message in the mailbox ready for send, read the 
       message from the mailbox and send it                                  */
    if (isr_mbx_receive (MBX_tx_ctrl[__CTRL1-1], (void **)&ptrmsg) != OS_R_OK) {

      CAN_hw_wr (__CTRL1, ptrmsg);
      _free_box(CAN_mpool, ptrmsg);
    } else {
      isr_sem_send(wr_sem[__CTRL1-1]);   /* Return a token back to semaphore */

      CAN1->IER &= ~(1 << 0);                       /* disable  TME interrupt */ 
    }
  }
}

void USB_LP_CAN1_RX0_IRQHandler (void) {
  CAN_msg *ptrmsg;

  if (CAN1->RF0R & 3) {                                  /* message pending ? */
    /* If the mailbox isn't full read the message from the hardware and
       send it to the message queue                                          */
    if (isr_mbx_check (MBX_rx_ctrl[__CTRL1-1]) > 0)  {

      ptrmsg = _alloc_box (CAN_mpool);
      if (ptrmsg) {
        CAN_hw_rd (__CTRL1, 0, ptrmsg);             /* Read received message */
        isr_mbx_send (MBX_rx_ctrl[__CTRL1-1], ptrmsg);
      }
    }
    CAN1->RF0R |= (1 << 5);                  /* Release FIFO 0 output mailbox */
  }
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
