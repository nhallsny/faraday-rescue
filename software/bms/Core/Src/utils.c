
#include "utils.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_tim.h"

void delayUS(TIM_HandleTypeDef* timer,uint32_t us) {   // Sets the delay in microseconds.
	__HAL_TIM_SET_COUNTER(timer, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(timer) < us);
}

void delayMS(TIM_HandleTypeDef* timer,uint32_t ms) {
	for (int i = 0; i < ms; i++) {
		delayUS(timer,1000);
	}
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count) {
	uint8_t copyIndex = 0;
	for (copyIndex = 0; copyIndex < count; copyIndex++) {
		dest[copyIndex] = source[copyIndex];
	}
}

float ReverseFloat(const float inFloat) {
	float retVal;
	char *floatToConvert = (char*) &inFloat;
	char *returnFloat = (char*) &retVal;

	// swap the bytes into a temporary buffer
	returnFloat[0] = floatToConvert[3];
	returnFloat[1] = floatToConvert[2];
	returnFloat[2] = floatToConvert[1];
	returnFloat[3] = floatToConvert[0];

	return retVal;
}

uint32_t FloatToUInt(float n) {
	return (uint32_t) (*(uint32_t*) &n);
}

float UIntToFloat(uint32_t n) {
	return (float) (*(float*) &n);
}

float unpackFloat(const void *buf) {
	const unsigned char *b = (const unsigned char*) buf;
	uint32_t temp = 0;
	temp = ((b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]);
	return *((float*) &temp);
}
