#include "app/app.hpp"

#include <main.h>

#include "app/buzzer/buzzer.hpp"
#include "app/can/can.hpp"
#include "app/spi/bmi088/accel.hpp"
#include "app/spi/bmi088/gyro.hpp"
#include "app/uart/uart.hpp"
#include "app/usb/cdc.hpp"

// C链接入口点，供HAL启动代码调用
extern "C" {
void AppEntry() { app.init().main(); }
}

App::App() {
    // 按优先级顺序初始化各外设模块
    led::led.init();
    buzzer::buzzer.init();
    usb::cdc.init();  // USB CDC: 主通信接口
    can::can1.init();  // CAN总线1: 1Mbps
    can::can2.init();  // CAN总线2: 1Mbps
    uart::uart1.init();
    uart::uart2.init();
    uart::uart_dbus.init();  // DBUS: 遥控器接口，100000 baud
    spi::bmi088::accelerometer.init();  // 加速度计: 1600Hz
    spi::bmi088::gyroscope.init();  // 陀螺仪: 2000Hz
    __enable_irq();  // 启用全局中断，开始接收数据
};

[[noreturn]] void App::main() {
    // 主循环: 轮询转发各接口数据到USB
    // 在每个外设转发之间调用try_transmit，最大化USB吞吐
    while (true) {
        usb::cdc->try_transmit();
        can::can1->try_transmit();  // CAN1 → USB
        usb::cdc->try_transmit();
        can::can2->try_transmit();  // CAN2 → USB
        usb::cdc->try_transmit();
        uart::uart1->try_transmit();  // UART1 → USB
        usb::cdc->try_transmit();
        uart::uart2->try_transmit();  // UART2 → USB
        usb::cdc->try_transmit();
        uart::uart_dbus->try_transmit();  // DBUS → USB
    }
}