#include "servo.h"
#include "tim.h"
/*
本程序使用c板控制舵机使用的是7路pwm分别为tim1的通道1、2、3、4
tim8的通道1、2、3
*/
HAL_StatusTypeDef servo_init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    return HAL_TIM_PWM_Start(htim, channel);
}

void servo_setpulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us)
{
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    __HAL_TIM_SET_COMPARE(htim, channel,pulse_us);
}

void servo_180_setangle(TIM_HandleTypeDef *htim, uint32_t channel, double angle)
{
    if (angle < 0.0) angle = 0.0;
    if (angle > 180.0) angle = 180.0;
    uint16_t pulse_us = (uint16_t)(500+ (angle / 180.0) * 2000);
    servo_setpulse(htim, channel, pulse_us);
}
