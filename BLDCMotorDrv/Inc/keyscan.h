#ifndef _KEYSCAN_H
#define _KEYSCAN_H

#include "bldc.h"
#include "myprint.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_gpio.h"

#define RUN_STATUS      HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5)
#define STOP_STATUS     HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)
#define UP_STATUS       HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)
#define DOWN_STATUS     HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)
#define DIR_STATUS      HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)

void run(void);
void up(void);
void down(void);
void dir(void);
void keyScan(void);

#endif