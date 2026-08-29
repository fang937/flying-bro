#include "app/servo/servo.hpp"

#include "app/usb/cdc.hpp"
#include "bsp/HAL/Core/Inc/tim.h"
#include <algorithm>

namespace servo {
Controller controller;

void Controller::init() {
    angles_.fill(90);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    for (uint8_t i = 1; i <= 7; ++i) set_angle(i, 90);
}

void Controller::set_angle(uint8_t id, uint8_t angle) {
    if (id < 1 || id > 7) return;
    angle = std::min<uint8_t>(180, angle); angles_[id - 1] = angle;
    uint32_t pulse = 500u + (static_cast<uint32_t>(angle) * 2000u) / 180u;
    TIM_HandleTypeDef* timer = id <= 4 ? &htim1 : &htim8;
    uint32_t channel = 1u << ((id <= 4 ? id : id - 4) - 1);
    __HAL_TIM_SET_COMPARE(timer, channel, pulse);
}

void Controller::reply(uint8_t command, uint8_t sequence, uint8_t status, uint8_t id) {
    auto& tx = usb::cdc->get_transmit_buffer();
    auto* out = tx.allocate(9); if (!out) return;
    auto* p = reinterpret_cast<uint8_t*>(out); p[0] = static_cast<uint8_t>(13u | (8u << 4));
    ++p; p[0] = 0x5A; p[1] = command;
    p[2] = sequence; p[3] = 3; p[4] = status; p[5] = id;
    p[6] = id >= 1 && id <= 7 ? angles_[id - 1] : 0;
    uint8_t crc = 0; for (uint8_t i = 1; i < 7; ++i) crc ^= p[i];
    p[3] = 3; p[7] = crc;
}

bool Controller::handle(const uint8_t* data, uint8_t length) {
    if (length < 5 || data[0] != 0xA5 || data[3] + 5u != length) return false;
    uint8_t crc = 0; for (uint8_t i = 1; i < length - 1; ++i) crc ^= data[i];
    if (crc != data[length - 1]) return false;
    auto command = static_cast<Command>(data[1]); uint8_t seq = data[2];
    if (command == Command::SetAngle && data[3] == 2) {
        if (data[4] < 1 || data[4] > 7) { reply(data[1] | 0x80, seq, 2, data[4]); return true; }
        set_angle(data[4], data[5]); reply(data[1] | 0x80, seq, 0, data[4]);
    } else if (command == Command::Action && data[3] == 1) {
        (void)data[4];
        reply(data[1] | 0x80, seq, 3);
    } else if (command == Command::Status && data[3] == 0) reply(data[1] | 0x80, seq, 0);
    else if (command == Command::Status && data[3] == 1) reply(data[1] | 0x80, seq, 0, data[4]);
    else reply(data[1] | 0x80, seq, 1);
    return true;
}

bool Controller::handle_fixed(const uint8_t* data, uint8_t length) {
    if (length != 6 || data[0] != 0xA5 || data[1] != 0x5A || data[5] != 0x0D)
        return false;
    uint8_t checksum = static_cast<uint8_t>(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4]) return false;
    uint16_t centidegrees = static_cast<uint16_t>(data[2] | (data[3] << 8));
    set_angle(1, static_cast<uint8_t>(std::min<uint16_t>(180, (centidegrees + 50) / 100)));
    return true;
}

void Controller::read_buffer_write_device(std::byte*& buffer) {
    const auto header = static_cast<uint8_t>(*buffer++);
    const auto size = static_cast<uint8_t>(header >> 4);
    auto* payload = reinterpret_cast<const uint8_t*>(buffer);
    handle(payload, size);
    buffer += size;
}

}
