#pragma once

#include <array>
#include <cstdint>
#include <cstddef>

namespace servo {

enum class Command : uint8_t { SetAngle = 1, Action = 2, Status = 3 };
enum class Action : uint8_t { Stop = 0, Fire = 1, Reload = 2 };

class Controller {
public:
    void init();
    bool handle(const uint8_t* data, uint8_t length);
    bool handle_fixed(const uint8_t* data, uint8_t length);
    void read_buffer_write_device(std::byte*& buffer);
    const std::array<uint8_t, 7>& angles() const { return angles_; }

private:
    void set_angle(uint8_t id, uint8_t angle);
    void reply(uint8_t command, uint8_t sequence, uint8_t status, uint8_t id = 0);
    std::array<uint8_t, 7> angles_{};
};

extern Controller controller;
}
