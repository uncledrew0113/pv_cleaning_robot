# PV 清扫机器人固件

RK3576 / PREEMPT_RT Linux / aarch64 / C++17。当前主分支聚焦实际在用的运行链路：行走电机、滚刷、IMU、GPS、BMS、限位、ThingsBoard、健康遥测与任务状态机。

## 当前运行边界

- 主程序入口是 [main.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/main.cc)。
- 运行配置使用两份文件：
  - `config/config.fixed.json`
  - `config/config.runtime.json`
- 运行时待生效和备份文件由 `ConfigService` 自动派生：
  - `config/config.runtime.pending.json`
  - `config/config.runtime.backup.json`
- 默认产品构建包含 `pv_cleaning_robot`、`unit_tests`、`hw_tests`。
- OTA 由独立系统进程负责，不属于 `pv_cleaning_robot` 控制进程。

## 构建

前置条件：

1. 安装 aarch64 交叉编译工具链。
2. 设置 `RK3576_SDK_PATH`。

```bash
source .env.example
cmake --preset rk3576-cross-linux
cmake --build --preset rk3576-build
```

产物：

- `build/aarch64/bin/pv_cleaning_robot`
- `build/aarch64/bin/unit_tests`
- `build/aarch64/bin/hw_tests`

说明：

- 这些产物是 `aarch64` 可执行文件。
- 在 `x86_64` 主机直接运行会出现 `Exec format error`，这是正常现象。

## 当前配置模型

固定配置 [config.fixed.json](/home/tronlong/pv_cleaning_robot/config/config.fixed.json)：

- 日志目录与级别
- CAN / UART / GPIO / GPS / MQTT / LoRaWAN 基础接线和端口
- 诊断上报开关与本地日志参数
- 设备静态版本信息

运行配置 [config.runtime.json](/home/tronlong/pv_cleaning_robot/config/config.runtime.json)：

- `scheduler.windows`
- `robot.repeat_count`
- `robot.primary_dock`
- `robot.clean_speed_rpm`
- `robot.return_speed_rpm`
- `robot.brush_rpm`
- `robot.min_battery_soc`
- `robot.charge_stop_soc`
- `robot.heading_pid_en`
- `robot.pid.*`

加载规则：

1. 主程序优先读取 `/opt/robot/config/config.runtime.json` 与 `/opt/robot/config/config.fixed.json`
2. 失败后回退到仓库内 `config/` 目录
3. `get()` 先读 runtime，再回退到 fixed
4. runtime 主文件损坏时，可从 `*.runtime.backup.json` 回退启动

## 硬件映射

| 接口 | 当前配置键 | 说明 |
|---|---|---|
| `can0` | `can.interface` | 行走电机组 |
| `/dev/ttyS1` | `serial.imu` | IMU |
| `/dev/ttyACM0` | `serial.brush` | ODrive ASCII 滚刷 |
| `/dev/ttyS8` | `serial.bms` | BMS |
| `/dev/ttyS2` 或 `gpsd` | `gps.*` | GPS 串口或 gpsd |
| `gpiochip5/0` | `gpio.left_limit` | 左侧限位 |
| `gpiochip5/1` | `gpio.right_limit` | 右侧限位 |

说明：

- 当前代码统一使用 `left_limit` / `right_limit` 命名。
- RK3576 默认按 `gpio.use_irq=false` 的轮询模式运行。

## 启动与主循环

主程序当前流程：

1. 加载 split config
2. 初始化日志和信号处理
3. 构造 CAN、串口、GPIO、GPS、CloudService、ThingsBoardControlPlane、RobotController
4. 注册 shared attributes 和 RPC
5. 建立网络连接，请求 shared attributes 快照，发布启动属性
6. 启动 `walk_ctrl`、`nav`、`bms`、`cloud` 线程
7. 主循环内执行：
   - `scheduler.tick()`
   - 根据 `RobotController` 状态切换 health/business telemetry 的 active/idle 周期
   - 将安全监控、看门狗、恢复动作结果统一投递到控制器事件队列
8. `SIGINT` / `SIGTERM` 时优雅关闭所有线程和设备

## 当前状态机语义

设备状态来自 `RobotController`，当前核心状态为：

- `Idle`
- `SelfChecking`
- `ExecutingMission`
- `SettlingEndpoint`
- `Recovering`
- `Charging`
- `FaultStopped`

业务语义：

- 配置任务只允许从合法端点启动；低电量在启动前拒绝。
- RPC 定向清扫允许从可信位置启动，到目标端点停止。
- RPC `stop` 只允许运行任务时触发，停止电机和滚刷后回到 `Idle`。
- P0/恢复失败等锁存故障进入 `FaultStopped`，只能通过云端故障复位回到 `Idle`。

## 测试

纯软件回归：

```bash
cmake --build build --target unit_tests
./build/aarch64/bin/unit_tests
```

目标板硬件测试：

```bash
cmake --build build --target hw_tests
./build/aarch64/bin/hw_tests "[hw_system]"
```

真实 ThingsBoard 联调：

```bash
TB_REAL_TEST=1 ./build/aarch64/bin/unit_tests "[integration][thingsboard][real]"
```

## 文档

- [doc/API_REFERENCE.md](/home/tronlong/pv_cleaning_robot/doc/API_REFERENCE.md)
- [doc/thingsboard_config.md](/home/tronlong/pv_cleaning_robot/doc/thingsboard_config.md)

仓库中的 `docs/superpowers/` 是设计和实施记录，不是运行时事实文档。
