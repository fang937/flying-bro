#pragma once

#include "utility/immovable.hpp"
#include "utility/lazy.hpp"

// 应用程序主类
// 使用Lazy模式延迟初始化，确保全局对象在main()前不执行构造
class App : private utility::Immovable {
public:
    using Lazy = utility::Lazy<App>;

    App();

    [[noreturn]] void main();  // 主循环，永不返回
};

// 全局应用实例，constinit确保编译期零初始化
inline constinit App::Lazy app;
