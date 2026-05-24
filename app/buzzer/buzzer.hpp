#pragma once

#include <cstdint>

#include <atomic>

#include <tim.h>

#include "utility/lazy.hpp"

namespace buzzer {

// 蜂鸣器控制类
// 支持4音符旋律播放，通过定时器PWM输出音频
// 音符频率表: {静音, C5(523Hz), D5(587Hz), G5(783Hz)}
// 从上位机接收乐谱数据，自动循环播放
class Buzzer {
public:
    Buzzer() {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
        reset();
    };

    void reset() {
        last_playing     = 0b1111'1111;
        playing_progress = 255;
        buzzer_score.store(0, std::memory_order::relaxed);
        std::atomic_signal_fence(std::memory_order_release);
    }

    void read_buffer_write_device(std::byte*& buffer) {
        auto& data = *std::launder(reinterpret_cast<BuzzerField*>(buffer));
        buffer += sizeof(BuzzerField);
        if (data.buzzer_data)
            buzzer_score.store(data.buzzer_data, std::memory_order::relaxed);
        else
            reset();
    }

    // 播放指定音符: 设置TIM4的ARR和CCR3寄存器控制频率和音量
    void play(uint8_t note_id) {
        uint32_t note_index[4]   = {1, 523, 587, 783};    // 音符频率表
        uint32_t volume_index[4] = {0, 1, 1, 1};           // 音量系数

        htim4.Instance->ARR  = (5 * 100000 / note_index[note_id] - 1) * 1u;
        htim4.Instance->CCR3 = (8 * 10500 / note_index[note_id] - 1) * volume_index[note_id] * 1u;
    }

    // 定时更新播放状态: 由HAL_IncTick()每毫秒调用
    // 使用状态机控制旋律播放: 4个小节 × 4个音符的循环
    void update(uint32_t tick) {
        if (tick & 0b1)
            return;

        uint8_t score_composed = buzzer_score.load(std::memory_order::relaxed);
        uint8_t score[4];
        for (int i = 0; i < 4; ++i)
            score[i] = (score_composed >> (i * 2)) & 0b11;
        uint8_t score_id = score[3];
        score[3]         = 0;

        if (playing_progress < 210) {
            play(playing_progress > 140 ? 0 : score[last_playing & 0b1111]);
            playing_progress++;
        } else {
            if ((last_playing & 0b1111) < 3) {
                last_playing++;
                playing_progress = 0;
            } else {
                if (((last_playing >> 4) & 0b1111) == score_id) {
                    play(0);
                } else {
                    last_playing     = score_id << 4;
                    playing_progress = 0;
                }
            }
        }
    }

private:
    uint8_t last_playing;     // format: id'part (高4位:当前乐谱ID, 低4位:当前音符位置)
    uint8_t playing_progress; // range: 0 to 255 (音符播放进度)

    std::atomic<uint8_t> buzzer_score;  // 乐谱数据: 8位编码4个音符(每2位一个音符)

    struct __attribute__((packed)) BuzzerField {
        uint8_t field_id    : 8;
        uint8_t buzzer_data : 8;
    };
};

inline utility::Lazy<Buzzer> buzzer;

} // namespace buzzer