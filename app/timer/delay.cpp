#include "app/timer/delay.hpp"
#include "app/buzzer/buzzer.hpp"
#include "app/led/led.hpp"

#include <stm32f4xx_hal.h>

extern "C" {

// The STM32 DWT (Data Watchpoint and Trace) unit is used to rewrite the Hal_Delay function to
// ensure that it works when interrupts are disabled, while significantly improving accuracy.
// 使用DWT单元重写HAL_Delay，确保中断禁用时仍能工作，精度更高
void HAL_Delay(uint32_t delay) { timer::delay(std::chrono::milliseconds(delay)); }

// Hack this useless function to perform regular low-priority tasks, eliminating the need for a
// dedicated timer peripheral.
// 利用SysTick中断执行低优先级定时任务，无需额外定时器
void HAL_IncTick() {
    uint32_t tick = uwTick + 1;
    uwTick        = tick;
    led::led->update(tick);      // 每1ms更新LED状态
    buzzer::buzzer->update(tick);  // 每1ms更新蜂鸣器播放
}

} // extern "C"