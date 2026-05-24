#pragma once

#include <main.h>

#include "utility/assert.hpp"
#include "utility/immovable.hpp"

namespace utility {

// 中断互斥锁: 支持嵌套的中断禁用/启用
// 使用引用计数确保多层lock/unlock配对正确
class InterruptMutex : Immovable {
public:
    static void lock() {
        __disable_irq();
        ++lock_count_;
    }

    static void unlock() {
        if (--lock_count_ == 0) {
            __enable_irq();
        }
    }

private:
    static inline int lock_count_;
};

// RAII中断锁守卫: 构造时禁用中断，析构时恢复
class InterruptLockGuard : Immovable {
public:
    InterruptLockGuard() { InterruptMutex::lock(); }
    ~InterruptLockGuard() { InterruptMutex::unlock(); }
};

} // namespace utility
