# 硬件测试设计规格

**日期**：2026-04-07  
**范围**：真实硬件单元测试 + 全层栈集成测试  
**已安装硬件**：前/后限位传感器、四轮行走电机组、IMU、BMS

---

## 1. 背景与目标

现有 `hw_tests` 可执行文件已包含 `bms_hw_test.cc`、`imu_hw_test.cc`、
`full_sweep_hw_test.cc`，但缺少：

1. **限位传感器专项测试**（GPIO 边沿触发、回调、消抖）
2. **行走电机组专项测试**（联机、急停、PID 开/关直线对比）
3. **全层栈清扫流程集成测试**（与 `main.cc` 结构对齐，FSM 驱动，PID 开/关双场景）

---

## 2. 文件结构

```
test/integration/hardware/
├── hw_config.h                  （新增）硬件常量 + 两级共享 Fixture
├── limit_switch_hw_test.cc      （新增）限位传感器单元测试
├── walk_motor_group_hw_test.cc  （新增）行走电机组单元测试
└── clean_cycle_hw_test.cc       （新增）全层栈清扫流程集成测试

test/CMakeLists.txt              （修改）hw_tests 目标加入 3 个新文件
```

---

## 3. `hw_config.h` — 硬件常量与公共 Fixture

### 3.1 硬件接线常量（与 `config/config.json` 对齐）

```cpp
namespace hw {

// CAN / 行走电机
constexpr char     kCanIface[]       = "can0";
constexpr uint8_t  kMotorIdBase      = 1u;
constexpr uint16_t kCommTimeoutMs    = 500u;   // 测试用加大余量（update 50ms × 10）
constexpr float    kTestSpeedRpm     = 30.0f;  // 安全低速
constexpr float    kTestReturnRpm    = 30.0f;

// IMU（WIT Motion，/dev/ttyS1, 9600 baud）
constexpr char kImuPort[]  = "/dev/ttyS1";
constexpr int  kImuBaud    = 9600;

// BMS（嘉佰达通用协议 V4，/dev/ttyS8, 9600 baud）
constexpr char kBmsPort[]  = "/dev/ttyS8";
constexpr int  kBmsBaud    = 9600;

// GPIO 限位开关（gpiochip5 line0=前, line1=后）
constexpr char     kGpioChip[]   = "gpiochip5";
constexpr unsigned kFrontLine    = 0u;
constexpr unsigned kRearLine     = 1u;

// 超时常量
constexpr int kLimitTimeoutSec   = 60;   // 等待限位触发最大时间
constexpr int kOnlineTimeoutMs   = 600;  // 等待电机上线最大时间

} // namespace hw
```

### 3.2 两级 Fixture

#### `hw::DeviceFixture`（设备层）
用于限位和电机单元测试，仅构造 Driver + Device 层：
- `LinuxCanSocket` → `WalkMotorGroup`
- `LibSerialPort(IMU)` → `ImuDevice`
- `LibSerialPort(BMS)` → `BMS`
- `LibGpiodPin(front/rear)` → `LimitSwitch(front/rear)`

析构时自动 `close()` 所有设备。

#### `hw::FullSystemFixture`（全层栈）
继承 `DeviceFixture`，额外构造：
- `MockModbusMaster` → `BrushMotor`（辊刷未安装，Mock 替代）
- `EventBus`
- `SafetyMonitor`（持有 walk_group 引用）
- `NavService`（10ms 周期）
- `MotionService`（Config 从常量设置）
- `FaultService`
- `WatchdogMgr`（hw_watchdog 路径留空，不开硬件看门狗）
- `RobotFsm`

关键方法：
- `start_loops()` — 启动 walk_ctrl 和 nav_exec 后台线程（`std::thread`，SCHED_OTHER，测试中不需要 FIFO）
- `stop_loops()` — request_stop + join
- `wait_state(name, timeout)` — 轮询 FSM 状态，超时返回 `false`
- `dispatch(Event)` — 转发至 `fsm->dispatch()`

---

## 4. `limit_switch_hw_test.cc` — 8 个测试段

**运行前提**：机器人停在停机位（尾端限位已触发）。

| Tag | 说明 | 验收条件 |
|-----|------|---------|
| `[hw_limit][open]` | front/rear GPIO open 成功 | `open()` 返回 `true` |
| `[hw_limit][read_level_home]` | 停机位电平自检 | rear 低（触发）/ front 高（未触发） |
| `[hw_limit][callback_front]` | 前传感器回调链路 | 注册回调，打印提示，3s 内手动触发，`callback` 被调用 1 次 |
| `[hw_limit][callback_rear]` | 后传感器回调链路 | 同上针对后传感器 |
| `[hw_limit][is_triggered]` | 触发后状态查询 | 触发后 `is_triggered()==true` |
| `[hw_limit][clear_trigger]` | 清除触发标志 | `clear_trigger()` 后 `is_triggered()==false` |
| `[hw_limit][debounce]` | 硬件消抖 | 30ms 内模拟多次边沿，`callback` 仅调用 1 次 |
| `[hw_limit][side_enum]` | 回调参数正确 | 前传感器 callback 收到 `LimitSide::FRONT`，后收到 `REAR` |

> **安全注意**：测试中电机未使能，限位触发仅验证 GPIO 链路，不涉及急停指令。

---

## 5. `walk_motor_group_hw_test.cc` — 13 个测试段

**运行前提**：机器人在轨道空旷区域，`can0` 已配置（`ip link set can0 up type can bitrate 500000`）。

| Tag | 说明 | 验收条件 |
|-----|------|---------|
| `[hw_walk][open_close]` | CAN 初始化 | `open()` 返回 `DeviceError::OK` |
| `[hw_walk][enable_disable]` | 模式控制 | `enable_all()` + 200ms + `disable_all()` 无异常 |
| `[hw_walk][all_online]` | 全轮联机 | `set_feedback_mode_all(10)` 后 600ms 内 4 轮 `online==true` |
| `[hw_walk][fwd_no_pid]` | PID 关闭前进 | 30 RPM 前进 3s，4 轮 `speed_rpm ∈ [15, 50]` |
| `[hw_walk][fwd_with_pid]` | PID 开启前进 | 30 RPM 前进 5s，IMU `yaw` 漂移 < 5° |
| `[hw_walk][pid_straightness]` | PID 直线对比 | 关闭/开启 PID 各行走 5s，打印两次 yaw 漂移量，log 可量化对比 |
| `[hw_walk][rev_speed]` | 反转 | -30 RPM 反转 2s，4 轮 `speed_rpm < 0` |
| `[hw_walk][emergency_override]` | 急停 | 运行中调 `emergency_override(0)`，50ms 内下一帧 `target_value == 0` |
| `[hw_walk][override_blocks_set_speeds]` | override 封锁 | override 期间调 `set_speeds(30...)`，`update()` 不重发运动帧（帧计数不增加） |
| `[hw_walk][clear_override]` | 解除急停 | `clear_override()` 后 30 RPM 可重新驱动 |
| `[hw_walk][comm_timeout_self_stop]` | 通信超时自保护 | 设 timeout=300ms，`close()` CAN，400ms 后电机应自保护停转（人工观察 + log 打印） |
| `[hw_walk][frame_stats_no_pid]` | 帧统计（PID 关） | 10s 连续运行，`ctrl_frame_count > 100`，`ctrl_err_count == 0` |
| `[hw_walk][frame_stats_with_pid]` | 帧统计（PID 开） | 同上，PID 开启，同等帧率无额外错误 |

---

## 6. `clean_cycle_hw_test.cc` — 9 个测试段

**运行前提**：机器人在停机位（后限位触发），轨道前方畅通，BMS 和 IMU 均已接线。  
**安全要求**：每次清扫测试后电机自动失能；异常退出时 `FullSystemFixture` 析构调用 `emergency_stop()`。

| Tag | 说明 | 验收条件 |
|-----|------|---------|
| `[hw_cycle][startup]` | 全层栈初始化 | 所有设备 open 成功，FSM 状态 = `"Idle"` |
| `[hw_cycle][self_check_pass]` | 自检流程 | `dispatch(EvScheduleStart{at_home=true})`，3s 内 FSM = `"CleanFwd"`，电机开始转动 |
| `[hw_cycle][one_pass_no_pid]` | 无 PID 完整一趟 | PID 关闭；Idle→SelfCheck→CleanFwd→(前限位)→CleanReturn→(后限位)→Charging；≤120s；打印各阶段耗时 |
| `[hw_cycle][one_pass_with_pid]` | 有 PID 完整一趟 | PID 开启；同上流程；整趟 yaw 漂移 < 10°；打印偏差统计 |
| `[hw_cycle][fault_p0_estop]` | P0 故障急停 | 清扫行进中 `fault->report(P0,...)`，5s 内 FSM = `"Fault"`，`walk_group` 速度 = 0 |
| `[hw_cycle][low_battery_return]` | 低电返回 | 清扫中 `dispatch(EvLowBattery)`，FSM→`Returning`，后限位触发后→`Charging` |
| `[hw_cycle][bms_valid]` | BMS 贯穿有效 | 清扫过程中 BMS `data.valid==true`，`soc_pct ∈ [0,100]`，`voltage_v > 0` |
| `[hw_cycle][imu_valid]` | IMU 贯穿有效 | 清扫过程中 IMU `data.valid==true`，`yaw_deg` 非 NaN（PID 开启场景必要条件） |
| `[hw_cycle][watchdog_alive]` | 看门狗不超时 | 正常 30s 运行中，watchdog 未触发 timeout callback |

---

## 7. `test/CMakeLists.txt` 修改

在 `hw_tests` 的源文件列表中追加：

```cmake
add_executable(hw_tests
  ...
  integration/hardware/limit_switch_hw_test.cc      # 新增
  integration/hardware/walk_motor_group_hw_test.cc  # 新增
  integration/hardware/clean_cycle_hw_test.cc       # 新增
  ...
)
```

---

## 8. 运行方法

```bash
# 部署到目标板后
./hw_tests "[hw_limit]"                    # 限位传感器全部测试
./hw_tests "[hw_walk]"                     # 行走电机组全部测试
./hw_tests "[hw_walk][fwd_no_pid]"         # 仅运行指定测试段
./hw_tests "[hw_cycle]"                    # 全流程集成测试
./hw_tests "[hw_cycle][one_pass_with_pid]" # 仅 PID 开启完整一趟
```

---

## 9. 安全约束

1. 所有行走测试速度 ≤ 30 RPM（`kTestSpeedRpm`）
2. 每个测试段 teardown 时调用 `disable_all()` + `close()`
3. `FullSystemFixture` 析构自动调用 `motion->emergency_stop()`
4. BrushMotor 使用 `MockModbusMaster`，不发送真实 Modbus 帧
5. `WatchdogMgr` 构造时 hw_watchdog 路径留空（`""`），不操作 `/dev/watchdog`
