#ifndef _SERVO_H_
#define _SERVO_H_
#include "main.h"
HAL_StatusTypeDef servo_init(TIM_HandleTypeDef *htim, uint32_t channel);
void servo_setpulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse);
void servo_180_setangle(TIM_HandleTypeDef *htim, uint32_t channel, double angle);

#endif // _SERVO_H_
