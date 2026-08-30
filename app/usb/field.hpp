#pragma once

#include <cstdint>

// USB通信协议字段ID定义
// 上行: 单片机 → 上位机 (外设数据)
// 下行: 上位机 → 单片机 (控制命令)
namespace usb::field {

// All id enumeration items have a underscore.
// Because the HAL library defines these SHORT words as GLOBAL macros.

// Note: id only occupies 4 bits, the remaining are defined independently by each field.

// 上行ID: 每个外设接收数据后写入上行缓冲区时使用
enum class UplinkId : uint8_t {
    CONTROL_ = 0,

    GPIO_ = 1,

    CAN1_ = 2,
    CAN2_ = 3,
    CAN3_ = 4,

    UART1_ = 5,
    UART2_ = 6,
    UART3_ = 7,
    UART4_ = 8,
    UART5_ = 9,
    UART6_ = 10,

    IMU_ = 11,
};

// 下行ID: 上位机下发数据时，根据ID分发到对应外设
enum class DownlinkId : uint8_t {
    CONTROL_ = 0,

    GPIO_ = 1,

    CAN1_ = 2,
    CAN2_ = 3,
    CAN3_ = 4,

    UART1_ = 5,
    UART2_ = 6,
    UART3_ = 7,
    UART4_ = 8,
    UART5_ = 9,
    UART6_ = 10,

    LED_    = 11,
    BUZZER_ = 12,
    SERVO_  = 13,
};

} // namespace usb::field
