#pragma once
// PiMutex 已移至 hal/ 层作为 OS 级同步原语，此文件保留为兼容转发头。
// 已有代码 #include "driver/pi_mutex.h" 且使用 robot::driver::PiMutex 的无需修改。
#include "pv_cleaning_robot/hal/pi_mutex.h"

namespace robot::driver {
    using PiMutex = robot::hal::PiMutex;
} // namespace robot::driver