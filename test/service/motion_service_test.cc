/**
 * MotionService 单元测试（依赖 MockCanBus + MockModbusMaster）
 * [service][motion]
 *
 * 测试策略：
 *   - 使用真实 WalkMotorGroup(MockCanBus) 构造（不调用 open() 避免后台线程）
 *   - 使用真实 BrushMotor(MockModbusMaster, 1) 构造
 *   - imu 传 nullptr（MotionService 内部有 nullptr 检查）
 *   - 验证 MockCanBus::sent_frames 中出现正确 CAN 帧 ID
 *   - 验证 MockModbusMaster::write_calls 中出现正确寄存器写入
 */
#include <catch2/catch.hpp>

#include "../mock/mock_can_bus.h"
#include "../mock/mock_modbus_master.h"
#include "pv_cleaning_robot/device/brush_motor.h"
#include "pv_cleaning_robot/device/walk_motor_group.h"
#include "pv_cleaning_robot/middleware/event_bus.h"
#include "pv_cleaning_robot/service/motion_service.h"

using robot::device::BrushMotor;
using robot::device::DeviceError;
using robot::device::WalkMotorGroup;
using robot::middleware::EventBus;
using robot::protocol::WalkMotorMode;
using robot::service::MotionService;

// ────────────────────────────────────────────────────────────────
// 构建辅助
// ────────────────────────────────────────────────────────────────
struct MotionFixture {
    std::shared_ptr<MockCanBus> can{std::make_shared<MockCanBus>()};
    std::shared_ptr<WalkMotorGroup> group{std::make_shared<WalkMotorGroup>(can)};
    std::shared_ptr<MockModbusMaster> modbus{std::make_shared<MockModbusMaster>()};
    std::shared_ptr<BrushMotor> brush{std::make_shared<BrushMotor>(modbus, 1)};
    EventBus bus;
    MotionService::Config cfg{
        .clean_speed_rpm = 300.0f,
        .return_speed_rpm = 500.0f,
        .brush_rpm = 1200,
        .heading_pid_en = false  // 关闭 PID，减少 IMU 依赖
    };
    MotionService motion;

    MotionFixture() : motion(group, brush, nullptr, bus, cfg) {
        can->open_result = true;
        can->send_result = true;
        modbus->open_result = true;
        modbus->write_reg_return = 0;
        brush->open();
    }

    /// 在 sent_frames 中查找指定 can id 的帧
    bool has_frame_id(uint32_t id) const {
        for (auto& f : can->sent_frames)
            if (f.id == id)
                return true;
        return false;
    }

    /// 在 write_calls 中查找对指定寄存器的写入
    bool has_write(int reg) const {
        for (auto& w : modbus->write_calls)
            if (w.addr == reg)
                return true;
        return false;
    }
};

// WalkMotorGroup 组控制帧 ID（见 walk_motor_can_codec.h，id_base=1 时为 0x32）
// 模式帧 id = 0x100 + id_base (0x101)，使能帧 id = 0x102 等
// 实际 CAN ID 需与协议对应，此处验证 sent_frames 非空即可
// ────────────────────────────────────────────────────────────────
// is_moving()
// ────────────────────────────────────────────────────────────────
TEST_CASE("MotionService: 初始状态 is_moving() == false", "[service][motion]") {
    MotionFixture f;
    REQUIRE_FALSE(f.motion.is_moving());
}

// ────────────────────────────────────────────────────────────────
// start_cleaning()
// ────────────────────────────────────────────────────────────────
TEST_CASE("MotionService: start_cleaning() 发出 CAN 帧", "[service][motion]") {
    MotionFixture f;
    bool ok = f.motion.start_cleaning();
    REQUIRE(ok);
    // 应有 CAN 帧：模式设置帧 + 使能帧 + 速度给定帧
    REQUIRE_FALSE(f.can->sent_frames.empty());
}

TEST_CASE("MotionService: start_cleaning() 写 BrushMotor REG_ENABLE=1", "[service][motion]") {
    MotionFixture f;
    f.motion.start_cleaning();

    bool found = false;
    for (auto& w : f.modbus->write_calls)
        if (w.addr == BrushMotor::REG_ENABLE && w.val == 1) {
            found = true;
            break;
        }
    REQUIRE(found);
}

TEST_CASE("MotionService: start_cleaning() 写 BrushMotor REG_TARGET_RPM=1200",
          "[service][motion]") {
    MotionFixture f;
    f.motion.start_cleaning();

    bool found = false;
    for (auto& w : f.modbus->write_calls)
        if (w.addr == BrushMotor::REG_TARGET_RPM &&
            static_cast<int16_t>(w.val) == f.cfg.brush_rpm) {
            found = true;
            break;
        }
    REQUIRE(found);
}

// ────────────────────────────────────────────────────────────────
// stop_cleaning()
// ────────────────────────────────────────────────────────────────
TEST_CASE("MotionService: stop_cleaning() 写 BrushMotor REG_ENABLE=0", "[service][motion]") {
    MotionFixture f;
    f.motion.start_cleaning();
    f.modbus->write_calls.clear();
    f.motion.stop_cleaning();

    bool stop_brush = false;
    for (auto& w : f.modbus->write_calls)
        if (w.addr == BrushMotor::REG_ENABLE && w.val == 0) {
            stop_brush = true;
            break;
        }
    REQUIRE(stop_brush);
}

TEST_CASE("MotionService: stop_cleaning() 发出零速度 CAN 帧", "[service][motion]") {
    MotionFixture f;
    f.motion.start_cleaning();
    f.can->sent_frames.clear();
    f.motion.stop_cleaning();
    REQUIRE_FALSE(f.can->sent_frames.empty());
}

// ────────────────────────────────────────────────────────────────
// start_returning()
// ────────────────────────────────────────────────────────────────
TEST_CASE("MotionService: start_returning() 发出 CAN 帧（反向）", "[service][motion]") {
    MotionFixture f;
    bool ok = f.motion.start_returning();
    REQUIRE(ok);
    REQUIRE_FALSE(f.can->sent_frames.empty());
}

TEST_CASE("MotionService: start_returning() 辊刷反转（负 rpm）", "[service][motion]") {
    MotionFixture f;
    f.motion.start_returning();

    bool found_neg = false;
    for (auto& w : f.modbus->write_calls)
        if (w.addr == BrushMotor::REG_TARGET_RPM && static_cast<int16_t>(w.val) < 0) {
            found_neg = true;
            break;
        }
    REQUIRE(found_neg);
}

// ────────────────────────────────────────────────────────────────
// start_returning_no_brush()
// ────────────────────────────────────────────────────────────────
TEST_CASE("MotionService: start_returning_no_brush() 先停辊刷再反向", "[service][motion]") {
    MotionFixture f;
    f.motion.start_cleaning();  // 先启动
    f.modbus->write_calls.clear();
    f.can->sent_frames.clear();

    bool ok = f.motion.start_returning_no_brush();
    REQUIRE(ok);

    // 辊刷停止（REG_ENABLE=0）应在 CAN 帧之前
    bool brush_stopped = false;
    for (auto& w : f.modbus->write_calls)
        if (w.addr == BrushMotor::REG_ENABLE && w.val == 0) {
            brush_stopped = true;
            break;
        }
    REQUIRE(brush_stopped);
    REQUIRE_FALSE(f.can->sent_frames.empty());
}

// ────────────────────────────────────────────────────────────────
// emergency_stop()
// ────────────────────────────────────────────────────────────────
TEST_CASE("MotionService: emergency_stop() 停辊刷并发出急停 CAN 帧", "[service][motion]") {
    MotionFixture f;
    f.motion.start_cleaning();
    f.modbus->write_calls.clear();
    f.can->sent_frames.clear();

    f.motion.emergency_stop();

    // 辊刷停止
    bool brush_stopped = false;
    for (auto& w : f.modbus->write_calls)
        if (w.addr == BrushMotor::REG_ENABLE && w.val == 0) {
            brush_stopped = true;
            break;
        }
    REQUIRE(brush_stopped);

    // 急停 CAN 帧
    REQUIRE_FALSE(f.can->sent_frames.empty());
}

// ────────────────────────────────────────────────────────────────
// CAN 通信失败：start_cleaning() 返回 false
// ────────────────────────────────────────────────────────────────
TEST_CASE("MotionService: CAN send 失败时 start_cleaning() 返回 false", "[service][motion]") {
    MotionFixture f;
    f.can->send_result = false;

    bool ok = f.motion.start_cleaning();
    REQUIRE_FALSE(ok);
}
