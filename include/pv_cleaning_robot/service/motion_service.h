#pragma once
#include "pv_cleaning_robot/device/walk_motor.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/middleware/thread_executor.h"
#include <memory>

namespace robot::service {

/// @brief 运动控制服务——协调行走电机与辊刷电机
///
/// 提供高层运动语义（向前清扫、停止、返回等），
/// 内部管理电机状态与速度设定。
class MotionService : public middleware::IRunnable {
public:
    struct Config {
        float clean_speed_rpm{300.0f};   ///< 清扫行进速度
        float return_speed_rpm{500.0f};  ///< 返回速度（快速）
        int   brush_rpm{1200};           ///< 辊刷转速
    };

    MotionService(std::shared_ptr<device::WalkMotor>  walk,
                  std::shared_ptr<device::BrushMotor> brush,
                  middleware::EventBus&               bus,
                  Config                              cfg);

    /// 开始清扫前进（使能行走 + 辊刷）
    bool start_cleaning();

    /// 停止清扫（停辊刷，行走归零）
    void stop_cleaning();

    /// 以返回速度反向行进
    bool start_returning();

    /// 原地急停
    void emergency_stop();

    /// 设置行走速度（RPM，正向/反向）
    bool set_walk_speed(float rpm);

    /// 查询运动状态
    bool is_moving() const;
    bool is_brush_running() const;

    void update() override;  ///< 由 ThreadExecutor 10ms 调用

private:
    std::shared_ptr<device::WalkMotor>  walk_;
    std::shared_ptr<device::BrushMotor> brush_;
    middleware::EventBus&               bus_;
    Config                              cfg_;
};

} // namespace robot::service
