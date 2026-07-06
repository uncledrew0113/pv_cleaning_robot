# 配置参考

本文档说明当前主程序使用的配置文件职责和字段含义。JSON 配置文件不写注释；
参数说明、单位和修改建议统一维护在本文档。

## 配置文件职责

| 文件 | 职责 | 生效方式 |
|---|---|---|
| `config/config.fixed.json` | 固定硬件、系统、通信、安全恢复和安装参数 | 进程启动时读取，通常修改后重启 |
| `config/config.runtime.json` | 任务、调度、速度、PID 和 SOC 策略 | active runtime 配置 |
| `config/config.runtime.pending.json` | 云端下发后等待下一次任务生效的 runtime 补丁 | 下一次任务启动前 promote |
| `config/config.runtime.backup.json` | runtime 最近一次备份 | runtime 损坏时回退 |

读取规则：

- `ConfigService::get()` 先读 runtime，再回退 fixed。
- `ConfigService::get_fixed()` 只读 fixed。
- 当前 fixed 参数不做范围校验，依靠默认值、配置文件和人工核对。
- 协议常量、状态枚举、状态机语义和线程调度优先级不放入 JSON。

## fixed 配置

| 参数 | 默认值 | 单位 | 功能 | 修改建议 | 重启 |
|---|---:|---|---|---|---|
| `logging.log_dir` | `data/pv_robot/logs` | 路径 | 日志目录 | 部署路径变化时修改 | 是 |
| `logging.level` | `info` | 枚举 | 日志等级 | 调试时可改 | 是 |
| `logging.console` | `true` | bool | 是否输出控制台日志 | 现场可按服务管理方式调整 | 是 |
| `can.interface` | `can0` | 设备名 | 行走电机 CAN 接口 | 仅硬件接口变化时修改 | 是 |
| `can.walk_motor.motor_id` | `1` | ID | 行走电机组起始 ID | 硬件拓扑变化时修改 | 是 |
| `can.walk_motor.comm_timeout_ms` | `500` | ms | 行走电机通信超时 | 谨慎修改 | 是 |
| `can.walk_motor.feedback_period_ms` | `10` | ms | 行走电机主动反馈周期 | 谨慎修改，影响诊断和控制新鲜度 | 是 |
| `can.walk_motor.termination_init_enabled` | `true` | bool | 启动时配置终端电阻 | 硬件接法变化时修改 | 是 |
| `can.walk_motor.termination_init_retry_count` | `3` | 次 | 终端电阻配置重试次数 | 通信不稳定时可调整 | 是 |
| `can.walk_motor.termination_motor_id` | `2` | ID | 终端电阻所在电机 ID | 硬件拓扑变化时修改 | 是 |
| `serial.imu.port` | `/dev/ttyS1` | 路径 | IMU 串口 | 设备路径变化时修改 | 是 |
| `serial.imu.baudrate` | `9600` | bps | IMU 波特率 | 与设备配置一致 | 是 |
| `serial.imu.output_rate_hz` | `100` | Hz | IMU 输出频率 | 谨慎修改，影响纠偏和诊断 | 是 |
| `serial.brush.port` | `/dev/ttyACM0` | 路径 | 滚刷串口 | 设备路径变化时修改 | 是 |
| `serial.brush.baudrate` | `115200` | bps | 滚刷串口波特率 | 与驱动器配置一致 | 是 |
| `serial.brush.axis` | `0` | 轴号 | 滚刷 ODrive 轴号 | 硬件接线变化时修改 | 是 |
| `serial.bms.port` | `/dev/ttyS8` | 路径 | BMS 串口 | 设备路径变化时修改 | 是 |
| `serial.bms.baudrate` | `9600` | bps | BMS 波特率 | 与 BMS 协议一致 | 是 |
| `gps.source` | `gpsd` | 枚举 | GPS 来源，`gpsd` 或 `serial` | 现场 GPS 接入方式变化时修改 | 是 |
| `gps.serial.port` | `/dev/ttyS2` | 路径 | 串口 GPS 设备 | 使用串口 GPS 时修改 | 是 |
| `gps.serial.baudrate` | `115200` | bps | 串口 GPS 波特率 | 与 GPS 模块一致 | 是 |
| `gps.gpsd.host` | `127.0.0.1` | 地址 | gpsd 地址 | gpsd 不在本机时修改 | 是 |
| `gps.gpsd.port` | `2947` | 端口 | gpsd 端口 | gpsd 配置变化时修改 | 是 |
| `gps.gpsd.watch` | `?WATCH=...` | 字符串 | gpsd watch 命令 | 通常不改 | 是 |
| `gpio.left_limit.chip/line` | `gpiochip5/0` | GPIO | A 端主限位 | 接线变化时修改 | 是 |
| `gpio.right_limit.chip/line` | `gpiochip5/1` | GPIO | B 端主限位 | 接线变化时修改 | 是 |
| `gpio.left_attitude_limit.chip/line` | `gpiochip5/2` | GPIO | 左下姿态限位 | 接线变化时修改 | 是 |
| `gpio.right_attitude_limit.chip/line` | `gpiochip5/3` | GPIO | 右下姿态限位 | 接线变化时修改 | 是 |
| `gpio.lock_motor.open.chip/line` | `gpiochip5/8` | GPIO | 锁止电机打开输出 | 接线变化时修改 | 是 |
| `gpio.lock_motor.close.chip/line` | `gpiochip5/9` | GPIO | 锁止电机关闭输出 | 接线变化时修改 | 是 |
| `gpio.lock_motor.pulse_ms` | `200` | ms | 锁止电机输出脉冲宽度 | 执行器变化时修改 | 是 |
| `gpio.lock_motor.settle_ms` | `8000` | ms | 锁止动作等待时间 | 执行器动作时间变化时修改 | 是 |
| `gpio.use_irq` | `false` | bool | 主限位是否使用 IRQ | RK3576 gpiochip5 当前使用轮询 | 是 |
| `gpio.input_debounce_ms` | `2` | ms | 主限位和姿态限位软件防抖 | 接近开关抖动明显时修改 | 是 |
| `safety.limit_settle_stable_ms` | `30` | ms | 主限位触发稳定确认时间 | 安全参数，谨慎修改 | 是 |
| `safety.limit_release_stable_ms` | `30` | ms | 主限位释放重新 armed 稳定时间 | 安全参数，谨慎修改 | 是 |
| `gps_stuck.min_fix_quality` | `2` | 枚举 | GPS 基本定位质量门槛 | 谨慎修改 | 是 |
| `gps_stuck.min_satellites_used` | `6` | 个 | GPS 可用卫星数门槛 | 现场遮挡严重时人工评估 | 是 |
| `gps_stuck.max_hdop` | `1.2` | HDOP | GPS 水平精度门槛 | 谨慎修改 | 是 |
| `gps_stuck.max_pdop` | `2.5` | PDOP | GPS 三维精度门槛 | 谨慎修改 | 是 |
| `gps_stuck.moving_speed_mps` | `0.03` | m/s | 判定移动的速度门槛 | 谨慎修改 | 是 |
| `gps_stuck.moving_speed_confirm_samples` | `2` | 次 | 连续移动确认样本数 | 谨慎修改 | 是 |
| `gps_stuck.stuck_timeout_ms` | `8000` | ms | GPS 卡滞持续时间 | 安全停机参数 | 是 |
| `gps_stuck.sample_stale_timeout_ms` | `3000` | ms | GPS 样本陈旧超时 | GPS 频率变化时人工核对 | 是 |
| `recovery.attitude_center.lower_rpm` | `10.0` | rpm | 姿态限位回中下轮速度 | 现场恢复效果调试时修改 | 是 |
| `recovery.attitude_center.stable_samples_required` | `2` | 次 | 姿态释放稳定样本数 | 谨慎修改 | 是 |
| `recovery.attitude_center.timeout_ms` | `30000` | ms | 姿态回中整体超时 | 安全恢复参数 | 是 |
| `recovery.attitude_center.tick_ms` | `20` | ms | 姿态回中轮询/命令周期 | 谨慎修改 | 是 |
| `recovery.reverse.duration_ms` | `2000` | ms | 反向恢复运动时长 | 现场恢复效果调试时修改 | 是 |
| `recovery.reverse.tick_ms` | `20` | ms | 反向恢复命令刷新周期 | 谨慎修改 | 是 |
| `error_manager.period_ms` | `100` | ms | 错误处理线程周期 | 通常不改 | 是 |
| `error_manager.consecutive_error_limit` | `10` | 次 | 通信错误连续递增阈值 | 故障策略参数 | 是 |
| `error_manager.stream_timeout_ms` | `3000` | ms | 数据流停滞超时 | 故障策略参数 | 是 |
| `error_manager.walk_stall_duration_ms` | `2500` | ms | 行走堵转持续时间 | 故障策略参数 | 是 |
| `error_manager.attitude_repeat_gap_ms` | `8000` | ms | 姿态恢复后重复触发计数窗口 | 故障策略参数 | 是 |
| `error_manager.attitude_reverse_attempt_count` | `3` | 次 | 第几次姿态重复触发后追加反向恢复 | 故障策略参数 | 是 |
| `error_manager.attitude_fault_count` | `4` | 次 | 第几次姿态重复触发后故障停机 | 故障策略参数 | 是 |
| `bms.poll_interval_ms` | `500` | ms | BMS 采样周期 | 通常不改 | 是 |
| `bms.charging_current_threshold_a` | `0.05` | A | 判定充电中的正向电流门槛 | 只影响 `charging`，不影响满电 | 是 |
| `brush.poll_interval_ms` | `500` | ms | 滚刷诊断采样周期 | 通常不改 | 是 |
| `installation.brush_direction_sign` | `1` | `1/-1` | 滚刷安装方向乘子 | 滚刷现场反装时改为 `-1` | 是 |
| `network.transport_mode` | `mqtt_only` | 枚举 | 网络模式 | 部署方式变化时修改 | 是 |
| `network.mqtt.*` | 见 JSON | 多种 | ThingsBoard MQTT/TLS 参数 | 部署和证书变化时修改 | 是 |
| `network.lorawan.*` | 见 JSON | 多种 | LoRaWAN 串口和入网参数 | 使用 LoRaWAN 时修改 | 是 |
| `storage.cache_path` | `data/pv_robot/telemetry_cache.jsonl` | 路径 | 断网上报缓存 | 部署路径变化时修改 | 是 |
| `system.hw_watchdog` | `/dev/watchdog` | 路径 | 硬件看门狗设备 | 平台变化时修改 | 是 |
| `diagnostics.mode` | `development` | 枚举 | 诊断上报模式 | 生产可改 `production` | 是 |
| `diagnostics.cloud_upload` | `true` | bool | 是否云端上报诊断 | 现场可改 | 是 |
| `diagnostics.local_log` | `true` | bool | 是否本地保存诊断 | 现场可改 | 是 |
| `diagnostics.collect_interval_ms` | `50` | ms | DiagnosticsCollector 采集周期 | 运行诊断负载相关 | 是 |
| `diagnostics.publish_interval_active_ms` | `1000` | ms | 运动中云端上报周期 | 现场可改 | 是 |
| `diagnostics.publish_interval_idle_ms` | `300000` | ms | 非运动云端上报周期 | 现场可改 | 是 |
| `diagnostics.local_log_interval_active_ms` | `50` | ms | 运动中本地诊断保存周期 | 调试相关，注意磁盘空间 | 是 |
| `diagnostics.local_log_interval_idle_ms` | `300000` | ms | 非运动本地诊断保存周期 | 现场可改 | 是 |
| `diagnostics.local_log_path` | `data/pv_robot/logs/telemetry.jsonl` | 路径 | 本地诊断 JSONL 路径 | 部署路径变化时修改 | 是 |
| `diagnostics.local_log_max_bytes` | `10485760` | byte | 单个诊断日志文件大小 | 磁盘策略相关 | 是 |
| `diagnostics.local_log_max_files` | `5` | 个 | 诊断日志滚动文件数 | 磁盘策略相关 | 是 |
| `device.software_version` | `1.0.0` | 字符串 | 上报软件版本 | 发版时更新 | 是 |
| `device.hardware_version` | `1.0` | 字符串 | 上报硬件版本 | 硬件版本变化时更新 | 是 |
| `device.model` | `pv_cleaning_robot` | 字符串 | 上报设备型号 | 通常不改 | 是 |

## runtime 配置

| 参数 | 默认值 | 单位 | 功能 | 生效方式 |
|---|---:|---|---|---|
| `scheduler.windows` | `08:00`, `14:30` | 时间 | 定时任务窗口 | 云端修改立即同步 |
| `robot.repeat_count` | `1` | 次 | 完整任务次数 | pending，下一任务生效 |
| `robot.dock_mode` | `single_dock` | 枚举 | 单/双停机位模式 | active runtime |
| `robot.primary_dock` | `A` | 端点 | 主停机端 | pending，下一任务生效 |
| `robot.clean_speed_rpm` | `25.0` | rpm | 清扫行走速度 | pending，下一任务生效 |
| `robot.return_speed_rpm` | `25.0` | rpm | 回主停机端速度 | pending，下一任务生效 |
| `robot.brush_rpm` | `600` | rpm | 滚刷目标转速 | pending，下一任务生效 |
| `robot.heading_pid_en` | `true` | bool | 是否启用航向纠偏 | active runtime |
| `robot.min_battery_soc` | `30.0` | % | 低电量启动门槛 | pending，下一任务生效 |
| `robot.charge_stop_soc` | `95.0` | % | 满电 SOC 阈值 | pending，下一任务生效 |

### PID 配置

| 参数 | 默认值 | 单位 | 功能 |
|---|---:|---|---|
| `robot.pid.uds_path` | `/tmp/pv_edge_tracker.sock` | 路径 | 边缘跟踪 UDS socket |
| `robot.pid.reconnect_interval_ms` | `500` | ms | UDS 重连间隔 |
| `robot.pid.result_timeout_ms` | `500` | ms | UDS 结果超时 |
| `robot.pid.min_confidence` | `0.5` | 置信度 | 有效视觉结果门槛 |
| `robot.pid.deadband_yaw_deg` | `0` | deg | 航向误差死区 |
| `robot.pid.kp/ki/kd` | `4.0/0.0/0.0` | PID | 航向 PID 参数 |
| `robot.pid.integral_limit` | `1.0` | 输出 | 积分限幅 |
| `robot.pid.max_output` | `10.0` | rpm | 最大纠偏输出 |
| `robot.pid.min_effective_output` | `0.5` | rpm | 最小有效纠偏输出 |
| `robot.pid.yaw_alpha` | `0.9` | 系数 | 航向滤波系数 |
| `robot.pid.output_sign` | `1.0` | 符号 | 输出方向修正 |
| `robot.pid.angle_source` | `fused_uds_gyro` | 枚举 | 纠偏角来源 |
| `robot.pid.wheel_strategy` | `all_wheels` | 枚举 | 纠偏轮组策略 |
| `robot.pid.slow_on_error` | `false` | bool | 视觉错误时是否降速 |
| `robot.pid.slow_base_rpm` | `20.0` | rpm | 降速基础速度 |
| `robot.pid.yaw_slow_threshold_deg` | `1.0` | deg | 触发降速的航向误差 |
| `robot.pid.fusion.process_noise_angle` | `0.05` | 方差 | 融合角过程噪声 |
| `robot.pid.fusion.process_noise_bias` | `0.001` | 方差 | 陀螺零偏过程噪声 |
| `robot.pid.fusion.measurement_noise_uds` | `0.5` | 方差 | UDS 测量噪声 |
| `robot.pid.fusion.initial_angle_variance` | `1.0` | 方差 | 初始角方差 |
| `robot.pid.fusion.initial_bias_variance` | `1.0` | 方差 | 初始零偏方差 |
| `robot.pid.fusion.max_gyro_only_ms` | `300` | ms | 允许仅靠陀螺预测的最长时间 |

## 不进入 JSON 的固定参数

以下参数当前集中为代码顶部 `constexpr`，不作为现场配置：

- 线程实时优先级和 CPU 亲和性。
- `walk_ctrl` 控制周期。
- `group_recv` CAN 接收线程参数。
- `imu_read` 线程串口读超时。
- `safety_mon` 轮询周期。
- `watchdog_mon` 监控周期和硬件 watchdog 默认超时。
- CAN ID、Modbus/BMS/IMU/ODrive 协议常量。
- 错误码、状态枚举和状态机核心逻辑。

