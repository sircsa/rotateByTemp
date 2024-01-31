/*
 * step_motor_driver.h
 *
 *  Created on: Jan 26, 2024
 *      Author: CSA
 */

#ifndef INC_STEP_MOTOR_DRIVER_H_
#define INC_STEP_MOTOR_DRIVER_H_

#include "stm32f1xx_hal.h"

#define IN1_PIN GPIO_PIN_6
#define IN1_PORT GPIOB
#define IN2_PIN GPIO_PIN_5
#define IN2_PORT GPIOB
#define IN3_PIN GPIO_PIN_4
#define IN3_PORT GPIOB
#define IN4_PIN GPIO_PIN_3
#define IN4_PORT GPIOB

void stepCCV (int steps, uint16_t delay); // CCV - Counter Clockwise
void stepCV (int steps, uint16_t delay); // CV - Clockwise;

#endif /* INC_STEP_MOTOR_DRIVER_H_ */
