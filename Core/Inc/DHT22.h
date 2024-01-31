/*
 * DHT22.h
 *
 *  Created on: Jan 27, 2024
 *      Author: CSA
 */

#ifndef INC_DHT22_H_
#define INC_DHT22_H_

#include "stm32f1xx_hal.h"

#define DHT22_PORT GPIOB
#define DHT22_PIN GPIO_PIN_12

//*** Functions prototypes ***//

uint8_t DHT22_Start (void);
uint8_t DHT22_Read (void);
void StartAcqusition(float *temp, float *hum);

#endif /* INC_DHT22_H_ */
