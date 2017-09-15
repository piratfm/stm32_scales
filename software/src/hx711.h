/*
 * hx711.h
 *
 *  Created on: 15 вер. 2017 р.
 *      Author: tipok
 */

#ifndef HX711_H_
#define HX711_H_

// GAINs:
// 1: channel A, gain factor 128
// 2: channel B, gain factor 32
// 3: channel A, gain factor 64

void HX711_Init();
int HX711_Tare(uint8_t gain, uint8_t times);
int HX711_Value(uint8_t gain);
int HX711_Average_Value(uint8_t gain, uint8_t times);


#endif /* HX711_H_ */
