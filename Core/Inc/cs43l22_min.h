/*
 * cs43l22_min.h
 *
 *  Created on: Sep 4, 2025
 *      Author: alifa
 */

#ifndef INC_CS43L22_MIN_H_
#define INC_CS43L22_MIN_H_
#pragma once
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// 7-bit I2C addr = 0x4A → HAL 8-bit olarak 0x94
#define CS43L22_I2C_ADDR (0x4A << 1)

void CS43L22_Reset(GPIO_TypeDef* port, uint16_t pin);
HAL_StatusTypeDef CS43L22_Init(I2C_HandleTypeDef* hi2c, uint8_t volume_0_255);
HAL_StatusTypeDef CS43L22_Play(I2C_HandleTypeDef* hi2c);

#ifdef __cplusplus
}
#endif



#endif /* INC_CS43L22_MIN_H_ */
