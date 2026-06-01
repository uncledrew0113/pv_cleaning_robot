#pragma once

#include <cstdint>

namespace robot::service {

/// @brief 姿态恢复动作编排器。
///
/// RecoveryMotion 只描述“停车、稳定、采样、微动、验证”的阶段推进，不直接
/// 持有传感器或电机。后续接入真实恢复动作时，由 MotionService/RobotController
/// 在每个 phase 上绑定具体采样和微动指令，避免 FSM 膨胀成运动算法容器。
class RecoveryMotion {
   public:
    enum class Phase {
        Idle,
        Stop,
        Stabilizing,
        Sampling,
        MicroMoving,
        Verifying,
        Done,
        Failed,
    };

    enum class Result {
        Running,
        Done,
        Failed,
    };

    void start();
    Result step();
    Phase phase() const noexcept;
    void set_pose_error_deg(float value) noexcept;
    void set_max_attempts(uint32_t value) noexcept;

   private:
    Phase phase_{Phase::Idle};
    float pose_error_deg_{0.0f};
    uint32_t attempts_{0};
    uint32_t max_attempts_{3};
};

}  // namespace robot::service
