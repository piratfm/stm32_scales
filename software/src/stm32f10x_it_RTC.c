/**
  ******************************************************************************
  * @file GlassLCD/src/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and 
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "glasslcd_RTC.h"


/** @addtogroup GlassLCD
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern NVIC_InitTypeDef NVIC_InitStructure; 
extern GPIO_InitTypeDef GPIO_InitStructure;
extern EXTI_InitTypeDef EXTI_InitStructure;
bool EnableTask1 = TRUE;
bool EnableTask2_3 = FALSE;
extern uint16_t SegmentsValues_Lower_Quarter_Digits[4];
extern const uint16_t CommonLine[4];
extern uint32_t CommonLine_OUT_PP[4];
extern uint32_t CommonLine_VDD_2[4];
uint32_t lcdcr=0;
__IO uint32_t LCDPowerOn=1;
__IO uint32_t TarePressed=0;
__IO uint32_t VoltageFlag=0x000;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
}


void EXTI1_IRQHandler(void)
{
 if(EXTI_GetITStatus(EXTI_Line1) != RESET)
 {
	 TarePressed=1;
	 EXTI_ClearITPendingBit(EXTI_Line1);
 }
}


/**
  * @brief  This function handles EXTI0_IRQHandler .
  * @param  None
  * @retval : None
  */
void EXTI0_IRQHandler(void)
{
 if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    if(LCDPowerOn == 1)
    {
     /* Disable RTC Clock */
     RCC_RTCCLKCmd(DISABLE);

     /* Disable the RTC Alarm */
     RTC_ITConfig(RTC_IT_ALR, DISABLE);

     /* Wait until last write operation on RTC registers has finished */
     RTC_WaitForLastTask();

     /* Disable the RTC Alarm Interrupt */
     NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
     NVIC_Init(&NVIC_InitStructure); 

     /* All Common lines = 0 */
     /* Configure all Common lines on CommonLines port pins as Out_PP */    
     GPIOB->CRH = 0x33333333;
     GPIOB->CRL = 0x33333333;

     /* All Segment lines = 0 ( all SegmentsLines_LQD_Port pins = 0 ) */
     GPIOB->ODR = 0;

     /* LCD Bias Plus Pin = 0V  */
     GPIO_ResetBits(LCD_Bias_Port, LCD_BiasPlus_Pin);

     LCDPowerOn = 0;
   } else if(LCDPowerOn == 0) {
     /* Enable the RTC Alarm */
     RTC_ITConfig(RTC_IT_ALR, ENABLE); 

     /* Wait until last write operation on RTC registers has finished */
     RTC_WaitForLastTask();

     /* Enable RTC Clock */
     RCC_RTCCLKCmd(ENABLE);

     /* Wait for RTC registers synchronization */
     RTC_WaitForSynchro();

    /* Enable the RTC Alarm Interrupt */
     NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure); 
    
     /* Power on the resistor bridge  */
     GPIO_SetBits(LCD_Bias_Port, LCD_BiasPlus_Pin);

     LCDPowerOn = 1;
   }

    /* Clear EXTI pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}


void PVD_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line16) != RESET)
  {
	//VoltageFlag=0x000;
	VoltageFlag = (PWR_GetFlagStatus(PWR_FLAG_PVDO)==SET) ? 0x800 : 0x000;

    /* Clear the Key Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line16);
  }
}


/**
  * @brief  This function handles TIM3_IRQHandler .
  * @param  None
  * @retval : None
  */
void TIM3_IRQHandler(void)
{
}

/**
  * @brief  This function handles RTCAlarm_IRQHandler .
  * @param  None
  * @retval : None
  */
void RTCAlarm_IRQHandler(void)
{
   /* Sequence of Tasks:
   Task1 -> Task2 -> Task1 -> Task3 -> Task1 -> Task2 -> Task1 -> Task3 -> ...
  
   * Task 1: all Common Lines and all Segment Lines are in push-pull and are set to 0.
   * Task 2: - Segment_lines[lcdcr] to be turned on are loaded with the value 1 otherwise 0.
             - Common_line[lcdcr] is set to low. 
             - Other Common lines are set to Vdd/2.
   * Task 3: - Segment_lines[lcdcr] values are inverted.
             - Common_line[lcdcr] is set to high.
             - Other Common lines are set to Vdd/2.
  */ 
 
    /* Clear EXTI line17 pending bit (RTC Alarm) */
    EXTI->PR = EXTI_Line17;
     
    if(EnableTask1 == TRUE)  /*------------- Task 1 -------------*/
    {
     /* Reset RTC Counter */
     /* Set the CNF flag to enter in the Configuration Mode */
     RTC->CRL |= (uint16_t)0x0010;
     /* Set RTC COUNTER MSB word */
     RTC->CNTH = 0x0;
     /* Set RTC COUNTER LSB word */
     RTC->CNTL = 0x0;
     /* Reset the CNF flag to exit from the Configuration Mode */
     RTC->CRL &= (uint16_t)0xFFEF;
    
  
    /* Wait until last write operation on RTC registers has finished */
    /* Loop until RTOFF flag is set */
    while ((RTC->CRL & RTC_FLAG_RTOFF) == (uint16_t)RESET)
    {
    }

     /* Set the next occuring time of alarm interrupt  */
     /* Set the CNF flag to enter in the Configuration Mode */
     RTC->CRL |= (uint16_t)0x0010;
     /* Set the Dead time value */
     /* Set the ALARM MSB word */
     RTC->ALRH = DeadTimeValueHigh;
     /* Set the ALARM LSB word */
     RTC->ALRL = DeadTimeValueLow;
     /* Reset the CNF flag to exit from the Configuration Mode */
     RTC->CRL &= (uint16_t)0xFFEF;

  
    /* Wait until last write operation on RTC registers has finished */
    /* Loop until RTOFF flag is set */
    while ((RTC->CRL & RTC_FLAG_RTOFF) == (uint16_t)RESET)
    {
    }

     /* All Segment lines = 0 ( all SegmentsLines_LQD_Port pins = 0 ) */
     //SegmentsLines_LQD_Port->ODR = (~ALL_COMMON_LINES) & ((uint16_t)~ALL_SEGMENT_LINES); 
    GPIOB->ODR = 0;
    
     /* All Common lines = 0 */
     //CommonLines_Port->ODR &= 0xF000 & (~ALL_COMMON_LINES);
    
     /* Configure all Common lines on CommonLines port pins as Out_PP */    
    //GPIOB->CRH |= ALL_COMMON_LINES_PP;
    GPIOB->CRH = 0x33333333;
    GPIOB->CRL = 0x33333333;

     if(EnableTask2_3 == TRUE) /* If Task 2 has been executed previously (before Task 1) then prepare */
     {                         /* the RTC interrupt to execute Task 3 in the next interrupt */
       /* Next interrupt will execute Task 3 and avoid it to execute Task 2 */
       EnableTask2_3 = FALSE;
     }
     else  /* If Task 3 has been executed previously (before Task 1) then prepare */
     {     /* the RTC interrupt to execute Task 2 in the next interrupt */
       /* Next interrupt will execute Task 2 and avoid it to execute Task 3 */
       EnableTask2_3 = TRUE; 
     }
   
     /* Avoid next interrupt to execute Task 1, so the interrupt will execute Task 2 or Task 3 */ 
     EnableTask1 = FALSE;
    }
    else
    { 
      if(EnableTask2_3 == TRUE)  /*------------- Task 2 --------------*/
      {
       /* Reset RTC Counter */
       /* Set the CNF flag to enter in the Configuration Mode */
       RTC->CRL |= (uint16_t)0x0010;
       /* Set RTC COUNTER MSB word to 0 */
       RTC->CNTH = 0x0;
       /* Set RTC COUNTER LSB word to 0 */
       RTC->CNTL = 0x0;
       /* Reset the CNF flag to exit from the Configuration Mode */
       RTC->CRL &= (uint16_t)0xFFEF;
    
    
    /* Wait until last write operation on RTC registers has finished */
    /* Loop until RTOFF flag is set */
    while ((RTC->CRL & RTC_FLAG_RTOFF) == (uint16_t)RESET)
    {
    }
       
       /* Set the next occuring time of alarm interrupt */
       /* Set the CNF flag to enter in the Configuration Mode */
       RTC->CRL |= (uint16_t)0x0010;
       /* Set the ALARM MSB word */
       RTC->ALRH = PulseValueForContrastHigh;
       /* Set the ALARM LSB word */
       RTC->ALRL = PulseValueForContrastLow;
       /* Reset the CNF flag to exit from the Configuration Mode */
       RTC->CRL &= (uint16_t)0xFFEF;

    
    /* Wait until last write operation on RTC registers has finished */
    /* Loop until RTOFF flag is set */
    while ((RTC->CRL & RTC_FLAG_RTOFF) == (uint16_t)RESET)
    {
    }

       /* Segment_lines[lcdcr] to be turned on are loaded with the value 1 otherwise 0 */
    	GPIOB->ODR = (GPIOB->ODR & 0xF000) | (0x0FFF & SegmentsValues_Lower_Quarter_Digits[lcdcr]);

       /* Common_line[lcdcr] is set to low */
    	GPIOB->BRR = CommonLine[lcdcr];
       
       /* Other Common lines set to Vdd/2 */
    	GPIOB->CRH &= CommonLine_VDD_2[lcdcr];
       
       /* Set Common_line[lcdcr] out push pull */
    	GPIOB->CRH |= CommonLine_OUT_PP[lcdcr];
    	//GPIOB->CRH = CommonLine_OUT_PP[lcdcr];

       /* Next interrupt will execute Task 1 */
       EnableTask1 = TRUE;
      
    }
    else                   /*------------- Task 3 -------------*/
    {
     /* Reset RTC Counter */
     /* Set the CNF flag to enter in the Configuration Mode */
     RTC->CRL |= (uint16_t)0x0010;
     /* Set RTC COUNTER MSB word */
     RTC->CNTH = 0x0;
     /* Set RTC COUNTER LSB word */
     RTC->CNTL = 0x0;
     /* Reset the CNF flag to exit from the Configuration Mode */
     RTC->CRL &= (uint16_t)0xFFEF;
    
    
    /* Wait until last write operation on RTC registers has finished */
    /* Loop until RTOFF flag is set */
    while ((RTC->CRL & RTC_FLAG_RTOFF) == (uint16_t)RESET)
    {
    }

    
     /* Set the next occuring time of alarm interrupt */
     /* Set the CNF flag to enter in the Configuration Mode */
     RTC->CRL |= (uint16_t)0x0010;
     /* Set the ALARM MSB word */
     RTC->ALRH = PulseValueForContrastHigh;
     /* Set the ALARM LSB word */
     RTC->ALRL = PulseValueForContrastLow;
     /* Reset the CNF flag to exit from the Configuration Mode */
     RTC->CRL &= (uint16_t)0xFFEF;     

    
    /* Wait until last write operation on RTC registers has finished */
    /* Loop until RTOFF flag is set */
    while ((RTC->CRL & RTC_FLAG_RTOFF) == (uint16_t)RESET)
    {
    }

     /* Segment_lines[lcdcr] values are inverted */ 
     GPIOB->ODR = (GPIOB->ODR & 0xF000) | (0x0FFF & (~SegmentsValues_Lower_Quarter_Digits[lcdcr]));
     
     /* Common_line[lcdcr] is set to high */
     GPIOB->BSRR = CommonLine[lcdcr];
     
     /* Other Common lines set to Vdd/2 */
     GPIOB->CRH &= CommonLine_VDD_2[lcdcr];
     
     /* Other Common lines out push pull */
     GPIOB->CRH |= CommonLine_OUT_PP[lcdcr];
     //GPIOB->CRH = CommonLine_OUT_PP[lcdcr];

     /* Next interrupt will execute Task 1 */
     EnableTask1 = TRUE;  
       
     lcdcr++;
      
     if(lcdcr>3)
     {
       lcdcr = 0;
     }
   }
  }
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (GlassLCD), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles GlassLCD interrupt request.
  * @param  None
  * @retval : None
  */
/*void GlassLCD_IRQHandler(void)
{
}*/



/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
