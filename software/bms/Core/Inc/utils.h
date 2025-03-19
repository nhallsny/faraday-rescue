#ifndef __UTILS_H
#define __UTILS_H



#include "stdint.h"
#include "stm32l0xx_hal.h"

void delayUS(TIM_HandleTypeDef* timer, uint32_t us);
void delayMS(TIM_HandleTypeDef* timer, uint32_t ms);
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);
float ReverseFloat(const float inFloat);
uint32_t FloatToUInt32(float n);
int8_t FloatToInt8t(float n);
float UInt32ToFloat(uint32_t n);
float unpackFloat(const void *buf);

#endif
