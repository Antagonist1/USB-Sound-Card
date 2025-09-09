#include "cs43l22_min.h"

static HAL_StatusTypeDef wr(I2C_HandleTypeDef* hi2c, uint8_t reg, uint8_t val){
  uint8_t p[2] = {reg, val};
  return HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, p, 2, HAL_MAX_DELAY);
}

void CS43L22_Reset(GPIO_TypeDef* port, uint16_t pin){
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  HAL_Delay(2);
}

// Minimal, 16-bit I2S, HP out
HAL_StatusTypeDef CS43L22_Init(I2C_HandleTypeDef* hi2c, uint8_t vol){
  HAL_StatusTypeDef s;
  // Power down
  if((s = wr(hi2c, 0x02, 0x01)) != HAL_OK) return s;
  // Auto clock, MCLK enable, HP power on
  if((s = wr(hi2c, 0x04, 0xAF)) != HAL_OK) return s;
  // Interface: I2S, 16-bit
  if((s = wr(hi2c, 0x06, 0x07)) != HAL_OK) return s;
  // Unmute
  if((s = wr(hi2c, 0x0F, 0x00)) != HAL_OK) return s;
  // Volume L/R
  if((s = wr(hi2c, 0x20, vol)) != HAL_OK) return s;
  if((s = wr(hi2c, 0x21, vol)) != HAL_OK) return s;
  return HAL_OK;
}

HAL_StatusTypeDef CS43L22_Play(I2C_HandleTypeDef* hi2c){
  // Power up playback path
  return wr(hi2c, 0x02, 0x9E);
}
