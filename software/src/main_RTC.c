/**
  ******************************************************************************
  * @file GlassLCD/src//main_RTC.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Main program body using RTC method and STOP mode
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
#include "stm32f10x.h"
#include "SystemConfig_RTC.h"
#include "hx711.h"


/** @addtogroup GlassLCD
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/ 

extern uint16_t  SegmentsValues_Lower_Quarter_Digits[4];
extern __IO uint32_t LCDPowerOn;
extern __IO uint32_t TarePressed;

void __cxa_pure_virtual(void) {Error_Handler();}
void __cxa_deleted_virtual(void) {Error_Handler();}

#ifdef DEBUG_TO_UART
int _write(int file, char *data, int len) {
	int i;
	for(i=0;i<len;i++) {
		USART_SendData(USART2, data[i]);
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {}
	}
	return len;
}
int _write_r (struct _reent *r, int file, char * ptr, int len) {
	int i;
	for(i=0;i<len;i++) {
		USART_SendData(USART2, ptr[i]);
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {}
	}
	return len;
}
#endif
#define printf(x,...)

void Delay(int val) {
	__IO uint32_t i = val;
	for(; i != 0; i--)
	{
	}
}


#define DEFAULT_SCALE 200
/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{
  int loopcnt = 0;
  int cnt_raw = 0;
  int tare = 0;

  LCDPowerOn=1;

  int scale = DEFAULT_SCALE;
  /* System configuration */
  SystemConfiguration();
  GPIO_SetBits(LCD_Bias_Port, LCD_BiasPlus_Pin);
  LCD_WriteLines();

do_calc_scale:
  if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET) {
	 printf("do configuration!\r\n");
	 while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET) {}

	 //TODO: do configuration!
	 LCD_WriteCfg();
     printf("Calculating scale...\r\n");
     printf("Getting zero weight....\r\n");
     cnt_raw = HX711_Average_Value(3, 32);
     printf("cnt[0.000]: %d => %d\r\n", cnt_raw);
     LCD_WriteInt(cnt_raw/100);
     printf("Now put 10.000Kg and measure weight again...\r\n");
     Delay(1000000);
     LCD_WriteInt(10000);
     Delay(1000000);
     tare = HX711_Average_Value(3, 32);
     printf("cnt[10.000]: %d\r\n", tare);
     LCD_WriteInt(tare/100);
     Delay(1000000);
     float scale_f = ((float)(tare - cnt_raw)) / 10000;
     scale = scale_f > 0 ? scale_f : -scale_f;
     printf("scale[1/10.000]: %d\r\n", scale);
     LCD_WriteInt(scale);
     Delay(1000000);

     if(scale < 35 || scale > 1000) {
    	 printf("scale is too bad: %d!\r\n", scale);
    	 scale=DEFAULT_SCALE;
     } else {
         FLASH_Unlock();
         FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    	 FLASH_ErasePage(0x800FC00);
         FLASH_ProgramWord(0x800FC04, 0xa5a5a5a5);
         uint32_t scale_u = scale;
         FLASH_ProgramWord(0x800FC08, scale_u);
         FLASH_Lock();
         printf("Done, scale saved!\r\n");
     }
  } else {
	  uint32_t *scale_sign_u32 =  (uint32_t*)0x800FC04;
	  uint32_t *scale_u32 =  (uint16_t*)0x800FC08;
	  if(*scale_sign_u32 != 0xa5a5a5a5) {
		  scale = DEFAULT_SCALE;
		  printf("scale is unknown! needs fixing (%d != %d)!\r\n", *scale_sign_u32, 0xa5a5a5a5);
		  LCD_WriteLines();
		  Delay(1000000);
	  } else {
		  //scale = *scale_u32;
		  uint32_t scale_u = *scale_u32;
		  scale = scale_u;
		  printf("scale is: %u\r\n", (uint32_t) (*scale_u32));
		  if(scale == 0xFFFFFFFF) {
			  scale = DEFAULT_SCALE;
			  printf("scale is bad value! needs fixing!\r\n");
			  LCD_WriteLines();
			  Delay(1000000);
		  }
	  }
  }


  /* Power on the resistor bridge */

  int is_intial = 1;
  int is_tare_setted = 0;

  printf("Hello main! Scale is: %d\r\n", scale);
  TarePressed=0;

  while(1) {
	  //read data....
#if 1
	if(!LCDPowerOn) {
re_sleep:
		printf("Powering off...\r\n");
		/* Request to enter STOP mode with regulator low power */
#if 1
		GPIO_InitTypeDef GPIO_InitStructure;
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		  GPIO_Init(GPIOA, &GPIO_InitStructure);
		  GPIO_Init(GPIOB, &GPIO_InitStructure);
		  GPIO_Init(GPIOC, &GPIO_InitStructure);

//		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|
//				                   GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
//		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//		  GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
	    TarePressed=0;
		//PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

		//GPIO_Configuration();
	    PWR_WakeUpPinCmd(ENABLE);
		PWR_EnterSTANDBYMode();
		//tune scale
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET && GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET)
			goto do_calc_scale;

		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) != Bit_RESET)
			goto re_sleep;

		continue;
        //RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        //SCB->SCR |= SCB_SCR_SLEEPDEEP;
        //PWR->CR |= PWR_CR_PDDS;
        //PWR->CR |= PWR_CR_CWUF;
        //PWR->CSR |= PWR_CSR_EWUP;
        //__WFE();
	}
#endif

	cnt_raw = HX711_Average_Value(3, 8);
	printf("cnt: %d => %d, LCD: %d\r\n", cnt_raw, cnt_raw - tare, (cnt_raw - tare)/scale);

	if(is_intial) {
		LCD_WriteInt(88888);
		is_intial=0;
		continue;
	} else {
		LCD_WriteInt((cnt_raw - tare)/scale);
	}

	if(!is_tare_setted) {
		tare=cnt_raw;
		printf("new tare: %d\r\n", tare);
		is_tare_setted=1;
	}

	if(TarePressed==1 && GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_SET) {
		printf("Set tare next time!\r\n");
		is_tare_setted=0;
		TarePressed=0;
	}

	if(loopcnt == 1) {
		loopcnt=0;
	}
	loopcnt++;
  }

}


#ifdef USE_FULL_ASSERT


/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 



/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
