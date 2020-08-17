/**
// File     : BH1750.h
 * Общие сведения
Датчик освещенности  GY-30 BH1750 — используется для определения освещенности и имеет большой интервал измерений от 1 до 65535 люксов. Модуль выполнен на базе BH1750.

Аналог данного модуля GY-302 Цифровой оптический датчик освещенности BH1750FVI

Характеристики
Напряжение питания: 3-5;
Напряжение на шине: 3.3/5;
Измеряемые значения: 1 - 65535 лк;
*/
#ifndef __BH1750_H
#define __BH1750_H

//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
//#include "I2C1.h"

#include <stddef.h>
#include <_ansi.h>

_BEGIN_STD_C

#if defined(STM32F0)
#include "stm32f0xx_hal.h"
#elif defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#elif defined(STM32L0)
#include "stm32l0xx_hal.h"
#elif defined(STM32L4)
#include "stm32l4xx_hal.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#else
 #error "BH1750 library was tested only on STM32F1, STM32F3, STM32F4, STM32F7, STM32L0, STM32L4, STM32H7 MCU families. Please modify ssd1306.h if you know what you are doing. Also please send a pull request if it turns out the library works on other MCU's as well!"
#endif


/* vvv I2C config vvv */

#ifndef BH1750_I2C_PORT
#define BH1750_I2C_PORT		hi2c1
#endif

#ifndef BH1750_I2C_ADDR
#define BH1750_I2C_ADDR        (0x23 << 1)
#endif

extern I2C_HandleTypeDef BH1750_I2C_PORT;

/* ^^^ I2C config ^^^ */
//--------------------------------------------------------------
// Global Function
//--------------------------------------------------------------
void BH1750_Init();
uint16_t BH1750_Read(void);
//--------------------------------------------------------------

_END_STD_C

#endif // __BH1750_H
