/**
  ******************************************************************************
  * @file GlassLCD/inc/SystemConfig_RTC.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  System Configuration header file for RTC method.
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


#ifndef __SYSTEM_CONFIG_RTC_H
#define __SYSTEM_CONFIG_RTC_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "glasslcd_RTC.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define RCC_APB1Periph_Used_LCD_GPIO  RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/ 
/* Private function prototypes -----------------------------------------------*/
void SystemConfiguration(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void); 
void EXTI_Configuration(void);
void RTC_Configuration(void);
void PVD_Configuration(void);
void RTC_Init(void);
void USART2_Init(void);
	  
/* Private functions ---------------------------------------------------------*/
#endif /* __SYSTEM_CONFIG_RTC_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
