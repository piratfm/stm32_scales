/**
  ******************************************************************************
  * @file GlassLCD/src/glasslcd_RTC.c 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  LCD glass driver file using RTC method and STOP mode
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
#include "glasslcd_RTC.h"
extern __IO uint32_t VoltageFlag;

/** @addtogroup GlassLCD
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
 #define NumberOfUsedDigits 4  /* For CT4_098 LCD reference */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables definition and initialization ----------------------------*/
uint16_t  SegmentsValues_Lower_Quarter_Digits[4]; /* LCD frame buffer for low 
                                               quarter digits (digits 1 -> 4) */
uint16_t  digit[4];     /* Digit frame buffer */

const  uint16_t  CommonLine[4]={
                             0x1000,
							 0x2000,
							 0x4000,
							 0x8000
};

/* GPIOs to be configured to VDD/2 */
uint32_t  CommonLine_VDD_2[4]={ 
                           0x000FFFFF,
                           0x00F0FFFF,
                           0x0F00FFFF,
                           0xF000FFFF
                          };

/* GPIOs to be configured to Output PP */
uint32_t  CommonLine_OUT_PP[4]={
                            0x00030000,
                            0x00300000,
                            0x03000000,
                            0x30000000
                           };


/*  =========================================================================
                                 LCD MAPPING
    =========================================================================

               A
          ----------
          |         |
         F|         |B
          |         |
          -----G-----
          |         |
        E |         |C
          |         |   _
          -----------  | |DP   
              D         -

A LCD character coding is based on the following matrix:


*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Initialize GPIOs to set segments lines and common line
  *   values to 0.
  * @param ne.
  * @retval : None.
  */
void LCD_GPIO_Init(void)
{
  /* Compute and load the GPIOs masks of the Common lines */
  //LCD_GPIO_Coms_Masks_Init();
    
  /* All common lines = 0 */
  GPIOB->ODR = 0;
   
  /* Configure all common lines on CommonLines port pins as Out_PP */    
  /* Configure all segment lines of Lower Quarter digits as Out_PP */ 
  GPIOB->CRH = 0x33333333;
  GPIOB->CRL = 0x33333333;
}


const uint16_t numMap[11][4] = {
	{0x003, 0x002, 0x003, 0x001}, //0
	{0x000, 0x002, 0x002, 0x000}, //1
	{0x002, 0x003, 0x001, 0x001}, //2
	{0x002, 0x003, 0x002, 0x001}, //3
	{0x001, 0x003, 0x002, 0x000}, //4
	{0x003, 0x001, 0x002, 0x001}, //5
	{0x003, 0x001, 0x003, 0x001}, //6
	{0x002, 0x002, 0x002, 0x000}, //7
	{0x003, 0x003, 0x003, 0x001}, //8
	{0x003, 0x003, 0x002, 0x001}, //9
	{0x000, 0x001, 0x000, 0x000} //-
};

void LCD_WriteInt(int val, int flag)
{


	SegmentsValues_Lower_Quarter_Digits[0] = 0x000 | 0x400 | VoltageFlag;//kg
	SegmentsValues_Lower_Quarter_Digits[1] = 0x000;
	SegmentsValues_Lower_Quarter_Digits[2] = 0x000;
	SegmentsValues_Lower_Quarter_Digits[3] = 0x000 | 0x008;//dot
	if(val < 0) {
		SegmentsValues_Lower_Quarter_Digits[3] |= 0x002;
		val = -val;
	}

	if(flag)
		SegmentsValues_Lower_Quarter_Digits[3] |= 0x800;


	int divider = 10000;

	int i;

	for(i=0;i<5;i++) {
		int cval = (val/divider);
		divider = divider / 10;

		if(i==0 && cval > 9) {
			// "error string"
			LCD_WriteError();
			return;
		}

		cval = cval % 10;
		if(i==0 && cval == 0)
			continue;

		SegmentsValues_Lower_Quarter_Digits[0] |= numMap[cval][0] << 2*i;
		SegmentsValues_Lower_Quarter_Digits[1] |= numMap[cval][1] << 2*i;
		SegmentsValues_Lower_Quarter_Digits[2] |= numMap[cval][2] << 2*i;
		SegmentsValues_Lower_Quarter_Digits[3] |= numMap[cval][3] << 2*i;
	}
}


void LCD_WriteCfg()
{
	SegmentsValues_Lower_Quarter_Digits[0] = 0x003 | 0x00C | 0x030 | 0x400 | VoltageFlag;
	SegmentsValues_Lower_Quarter_Digits[1] = 0x000 | 0x004 | 0x000;
	SegmentsValues_Lower_Quarter_Digits[2] = 0x001 | 0x004 | 0x030;
	SegmentsValues_Lower_Quarter_Digits[3] = 0x001 | 0x000 | 0x010;
}

void LCD_WriteError()
{
	SegmentsValues_Lower_Quarter_Digits[0] |= 0x003 | 0x000 | 0x000 | 0x000 | 0x000 | 0x400 | VoltageFlag;
	SegmentsValues_Lower_Quarter_Digits[1] |= 0x001 | 0x004 | 0x010 | 0x040 | 0x100;
	SegmentsValues_Lower_Quarter_Digits[2] |= 0x001 | 0x004 | 0x010 | 0x0C0 | 0x100;
	SegmentsValues_Lower_Quarter_Digits[3] |= 0x001 | 0x000 | 0x000 | 0x040 | 0x000;
}

void LCD_WriteLines()
{
	SegmentsValues_Lower_Quarter_Digits[0] = 0 | 0x400 | VoltageFlag;
	SegmentsValues_Lower_Quarter_Digits[1] = 0x155; // -----
	SegmentsValues_Lower_Quarter_Digits[2] = 0;
	SegmentsValues_Lower_Quarter_Digits[3] = 0;
}

void LCD_WriteNone()
{
	SegmentsValues_Lower_Quarter_Digits[0] = 0 | 0x400 | VoltageFlag;//kg;
	SegmentsValues_Lower_Quarter_Digits[1] = 0; // -----
	SegmentsValues_Lower_Quarter_Digits[2] = 0;
	SegmentsValues_Lower_Quarter_Digits[3] = 0;
}

void LCD_WriteAll()
{
	SegmentsValues_Lower_Quarter_Digits[0] = 0xFFF;//kg;
	SegmentsValues_Lower_Quarter_Digits[1] = 0xFFF;
	SegmentsValues_Lower_Quarter_Digits[2] = 0xFFF;
	SegmentsValues_Lower_Quarter_Digits[3] = 0xFFF;
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
