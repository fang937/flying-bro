#pragma once

#include <cstdint>

#include <atomic>

#include <tim.h>

#include "utility/lazy.hpp"

namespace buzzer {
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

    void play(uint8_t note_id) {
        uint32_t note_index[4]   = {1, 523, 587, 783};
        uint32_t volume_index[4] = {0, 1, 1, 1};

        htim4.Instance->ARR  = (5 * 100000 / note_index[note_id] - 1) * 1u;
        htim4.Instance->CCR3 = (8 * 10500 / note_index[note_id] - 1) * volume_index[note_id] * 1u;
    }

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
    uint8_t last_playing;     // format: id'part
    uint8_t playing_progress; // range: 0 to 255

    std::atomic<uint8_t> buzzer_score;

    struct __attribute__((packed)) BuzzerField {
        uint8_t field_id    : 8;
        uint8_t buzzer_data : 8;
    };
};

inline utility::Lazy<Buzzer> buzzer;

} // namespace buzzer