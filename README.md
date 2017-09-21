# STM32 scales

Software and hardware for child weight meter (scales) with glass LCD display
Based on branded Chicco weight meter model # 8312.60.001

![scales image](https://github.com/piratfm/stm32_scales/blob/master/hardware/chicco_comfort_1.jpg)

Due original "brains" PCB became broken for some reason (operational amplifier burned-out, and scales always shows zeroes), the decision to replace
it by STM32 is made. As additional "pro", scales precision has been 10x increased, comparing to original "brains" of scales.

## Original parts that been used:
 * load sensors bridge (3 pins each).
 * original 5 symbol glass LCD display.
 * original plastic enclosure with pushbuttons.

## Used hardware:
 * HX711 ADC
 * STM32F103C8T6 as MCU and software-base LCD controller (by using RTC interrupts)
 
 schematic of the board: [schematic](https://github.com/piratfm/stm32_scales/blob/master/hardware/weightmeter_schematic.pdf)

## To calibrate:
 1. Put weight meter into the flat table (without any load).
 2. Press "tare" button when powering-on device (press both buttons, tare and power).
 3. You will see some 5-digit number, this is initial ADC value without scale.
 4. When You see "10.000" on display, put 10.000 Kg reference load onto weight meter.
 5 . You will see some 5-digit number again, but a little bit higher, this is ADC value with load and without scale.
 6. Then You will see 4-digit number in range 35...300. This is calculated "scale" value for calibration, that number will be saved into flash.
 7. Done, Your weight meter is now calibrated and displays correct value in Kg.


## Power consumption:
Active mode: 6 mA
Power down mode: 20 uA

License: GPL
