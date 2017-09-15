/*
 * hx711.c
 *
 *  Created on: 15 вер. 2017 р.
 *      Author: tipok
 */


#include "stm32f10x.h"
#include "hx711.h"

extern void Delay(int val);

#define SCK_PIN GPIO_Pin_5
#define DAT_PIN GPIO_Pin_6

void HX711_Init()
{
	GPIO_WriteBit(GPIOA, SCK_PIN, Bit_SET);
	Delay(100000);
	GPIO_WriteBit(GPIOA, SCK_PIN, Bit_RESET);
	Delay(1000);
}

int HX711_Average_Value(uint8_t gain, uint8_t times)
{
    int sum = 0;
    for (int i = 0; i < times; i++)
    {
        sum += HX711_Value(gain);
    }

    return sum / times;
}

int HX711_Value(uint8_t gain)
{
	long buffer = 0;
    uint8_t i;

    while (GPIO_ReadInputDataBit(GPIOA, DAT_PIN)==Bit_SET) {};

    __disable_irq();

    for (i = 0; i < 24; i++)
    {
    	GPIO_WriteBit(GPIOA, SCK_PIN, Bit_SET);
        buffer = buffer << 1;
        GPIO_WriteBit(GPIOA, SCK_PIN, Bit_RESET);
        if(GPIO_ReadInputDataBit(GPIOA, DAT_PIN) == Bit_SET) {
        	buffer++;
        }
    }

    for (i = 0; i < gain; i++)
    {
    	GPIO_WriteBit(GPIOA, SCK_PIN, Bit_SET);
    	GPIO_WriteBit(GPIOA, SCK_PIN, Bit_RESET);
    }

    __enable_irq();

    buffer = buffer ^ 0x800000;

    //if (buffer & 0x800000) {
    //	buffer |= (long) ~0xffffff;
    //}

	//if (buffer & 0x800000) {
	//	buffer |= 0xFF800000;
	//}
    return buffer;
}

int HX711_Tare(uint8_t gain, uint8_t times)
{
	return HX711_Average_Value(gain, times);
}
