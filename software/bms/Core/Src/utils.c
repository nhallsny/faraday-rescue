#include "stdint.h"

void delayUS(uint32_t us) {   // Sets the delay in microseconds.
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us)
		;
}

void delayMS(uint32_t ms) {
	for (int i = 0; i < ms; i++) {
		delayUS(1000);
	}
}
