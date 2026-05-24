#pragma once

#include <cstdint>

#include <atomic>

#include <tim.h>

#include "utility/lazy.hpp"

namespace led {

// LED指示灯控制类
// 通过TIM5的3个PWM通道控制RGB LED
// 状态指示:
//   - 绿色呼吸灯: 正常工作
//   - 黄色闪烁: 上行缓冲区满(CAN/UART/SPI)
//   - 青色闪烁: 下行缓冲区满(USB)
//   - 黄青交替: 两者都满
class Led {
public:
    Led() {
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
        reset();
    };

    void reset() {
        uplink_full_reset_counter_.store(0, std::memory_order::relaxed);
        downlink_full_reset_counter_.store(0, std::memory_order::relaxed);
        std::atomic_signal_fence(std::memory_order_release);
        user_controlling_.store(false, std::memory_order::relaxed);
    }

    // Lighting effect lasts for 5 seconds
    // 上行缓冲区满告警: 持续5秒(5000ms)
    void uplink_buffer_full() {
        uplink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }
    // 下行缓冲区满告警: 持续5秒(5000ms)
    void downlink_buffer_full() {
        downlink_full_reset_counter_.store(5000, std::memory_order::relaxed);
    }

    // 定时更新LED状态: 由HAL_IncTick()每毫秒调用
    void update(uint32_t tick) {
        // Do not change the lighting when user controlling
        if (user_controlling_.load(std::memory_order::relaxed))
            return;

        // Atomic decrease counter
        uint16_t uplink_full;
        do {
            uplink_full = uplink_full_reset_counter_.load(std::memory_order::relaxed);
            if (uplink_full == 0)
                break;
        } while (!uplink_full_reset_counter_.compare_exchange_weak(
            uplink_full, uplink_full - 1, std::memory_order::relaxed));

        uint16_t downlink_full;
        do {
            downlink_full = downlink_full_reset_counter_.load(std::memory_order::relaxed);
            if (downlink_full == 0)
                break;
        } while (!downlink_full_reset_counter_.compare_exchange_weak(
            downlink_full, downlink_full - 1, std::memory_order::relaxed));

        if (uplink_full && downlink_full) {
            // Both full: yellow and aqua lights flashing alternately
            if (tick & 128)
                set_value(255, 255, 0);
            else
                set_value(0, 255, 255);
        } else if (uplink_full) {
            // Uplink full: yellow light flashing
            if (tick & 128)
                set_value(255, 255, 0);
            else
                set_value(0, 0, 0);
        } else if (downlink_full) {
            // Downlink full: aqua light flashing
            if (tick & 128)
                set_value(0, 0, 0);
            else
                set_value(0, 255, 255);
        } else {
            // Working well: green breathing light
            auto brightness = (tick >> 2) & 511;
            if (brightness > 255)
                brightness = 511 - brightness;
            set_value(0, brightness, 0);
        }
    }

    void set_value(uint8_t red, uint8_t green, uint8_t blue) {
        htim5.Instance->CCR1 = blue;
        htim5.Instance->CCR2 = green;
        htim5.Instance->CCR3 = red;
    }

private:
    std::atomic<bool> user_controlling_;  // 用户控制标志: 为true时不自动更新LED

    std::atomic<uint16_t> uplink_full_reset_counter_, downlink_full_reset_counter_;  // 告警倒计时(ms)
};

inline utility::Lazy<Led> led;

} // namespace led