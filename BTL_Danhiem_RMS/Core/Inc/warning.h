/*
 * warning.h
 *
 *  Created on: Nov 13, 2024
 *      Author: DELL
 */

#ifndef INC_WARNING_H_
#define INC_WARNING_H_

#include "stm32f1xx_hal.h"  // Include STM32 HAL library for GPIO definitions
#include "main.h"

#define L1_LED_GPIO_Port GPIOB
#define L1_LED_Pin GPIO_PIN_12

#define L2_LED_GPIO_Port GPIOB
#define L2_LED_Pin GPIO_PIN_13

#define L3_LED_GPIO_Port GPIOB
#define L3_LED_Pin GPIO_PIN_14

extern uint16_t CO_ppm;
extern uint16_t CO2_ppm;
extern uint16_t Tvoc_ppb;

void L2_Warning_On();
void L3_Warning_On();
void L1_Warning_On();
void Warning ();
#endif /* INC_WARNING_H_ */
