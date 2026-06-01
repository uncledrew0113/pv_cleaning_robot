# PV 清扫机器人 API 参考

本文档只描述当前主分支代码中真实存在、且主程序实际使用的接口与运行契约。

## 1. 可执行目标

当前构建产物：

- `pv_cleaning_robot`
- `unit_tests`
- `hw_tests`

相关定义：

- 主程序目标：[pv_cleaning_robot/CMakeLists.txt](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/CMakeLists.txt:17)
- 测试目标：[test/CMakeLists.txt](/home/tronlong/pv_cleaning_robot/test/CMakeLists.txt:63)

## 2. 配置服务

配置服务类：`robot::service::ConfigService`
头文件：[config_service.h](/home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/service/config_service.h:25)

当前配置模型：

- runtime：当前生效的业务配置
- fixed：固定硬件/系统配置
- pending：下一次任务启动前生效的业务补丁
- backup：runtime 的最近一次备份

核心接口：

- `bool load()`
- `template <typename T> T get(const std::string& path, const T& default_val = T{}) const`
- `rapidjson::Document snapshot() const`
- `bool replace_and_save(const rapidjson::Value& new_root)`
- `bool save_pending(const rapidjson::Value& pending_root) const`
- `std::optional<rapidjson::Document> load_pending() const`
- `bool clear_pending() const`

当前路径语义：

- `config.runtime.json`
- `config.fixed.json`
- `config.runtime.pending.json`
- `config.runtime.backup.json`

加载规则：

- `get()` 先读 runtime，再回退到 fixed
- `load()` 先读 runtime，失败后尝试 backup

实现见：[config_service.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/config_service.cc:54)

## 3. 主程序装配

主程序入口：[main.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/main.cc:173)

当前 `main` 负责：

- 构造 `WalkMotorGroup`、`BrushMotor`、`BMS`、`ImuDevice`、`GpsDevice`
- 构造 `CloudService`、`ThingsBoardConfigManager`、`ThingsBoardControlPlane`
- 构造 `RobotController`、`FaultPolicy`、`FaultDetector`、`WatchdogMgr`
- 启动 `walk_ctrl`、`nav`、`bms`、`cloud` 线程
- 动态切换 telemetry 周期

当前线程分工：

- `walk_ctrl`：`MotionService`
- `nav`：`NavService`
  负责输出兼容 `Pose` 与新增 `FusedOdometry` 两套导航快照；内部采用“上下轮组积分 + GPS 轨道投影低频校正 + IMU 陀螺/加速度短时约束”的轻量融合实现，悬空检测逻辑也保留在该线程内。
- `bms`：`BMS::update()` + `BrushMotor::update()`
- `cloud`：`HealthService` + `ThingsBoardControlPlane::publish_business_telemetry()` + `CloudService::update()`

相关逻辑见：[main.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/main.cc:447) 和 [main.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/main.cc:590)

## 4. 运动与状态机

### 4.1 MotionService

头文件：[motion_service.h](/home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/service/motion_service.h)

主程序当前使用的接口：

- `start_segment()`：按业务任务段启动清扫运动。
- `stop_cleaning()`：停止滚刷、停止行走并关闭纠偏。
- `emergency_stop()`：刷停 + 行走电机 override 到 0。
- `update()`：周期心跳和视觉纠偏。

关键语义：

- 运动方向由目标端点换算，不由上层传电机正负号。
- 向主停机端运行使用 `return_speed_rpm`，向对端运行使用 `clean_speed_rpm`。
- 当前主流程只有清扫段；无刷返航不再作为业务状态机路径。

### 4.2 RobotController

头文件：[robot_controller.h](/home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/app/robot_controller.h)

当前状态：

- `Idle`
- `SelfChecking`
- `ExecutingMission`
- `SettlingEndpoint`
- `Recovering`
- `Charging`
- `FaultStopped`

核心接口与规则：

- `submit_command()` 是 RPC、调度和本地业务命令的统一入口。
- `StartConfiguredMission` 只允许从合法端点启动，并在启动前检查低电量。
- `CleanTowardOppositeEndpoint` / `CleanTowardPrimaryDock` 允许从可信位置启动，到目标端点停止。
- `Stop` 只允许任务运行中触发，停止后清空任务并回到 `Idle`。
- `FaultReset` 只允许 `FaultStopped`，清除锁存故障后直接回 `Idle`。
- 限位、安全、看门狗、恢复结果通过控制器事件队列串行处理。

实现见：[robot_controller.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/app/robot_controller.cc)

## 5. ThingsBoard 控制面

头文件：[thingsboard_control_plane.h](/home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/service/thingsboard_control_plane.h:118)

### 5.1 Shared Attributes

当前支持字段：

- `repeat_count`
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `primary_dock`
- `min_battery_soc`
- `charge_stop_soc`
- `schedules`

当前生效语义：

- `schedules` 立即写入 active，并同步调度器
- 其它字段写入 pending
- 新任务启动前，如果有 pending，则先 `promote_pending_to_active()`

实现见：[thingsboard_control_plane.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/thingsboard_control_plane.cc:358)

### 5.2 RPC

当前注册 RPC：

- `clean_to_return`
- `clean_to_parking`
- `start_configured`
- `stop`
- `fault_reset`

规则：

- `clean_to_return`
  - 从可信位置启动，向返机端方向清扫，到达后停止
- `clean_to_parking`
  - 从可信位置启动，向主停机端方向清扫，到达后停止
- `start_configured`
  - 只允许从配置任务合法端点启动
  - 电量需满足 `min_battery_soc`
- `stop`
  - 只允许运行任务时触发
- `fault_reset`
  - 只允许 `FaultStopped`，清除锁存故障后回到 `Idle`

实现见：[thingsboard_control_plane.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/thingsboard_control_plane.cc:883)

### 5.3 ThingsBoard 上行 payload 家族

1. `startup attributes`
2. `RPC reply`
3. `business telemetry`

编码器类：`ThingsBoardJsonCodec`
定义见：[thingsboard_control_plane.h](/home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/service/thingsboard_control_plane.h:86)

#### startup attributes

字段：

- `software_version`
- `hardware_version`
- `device_model`
- `device_id`

#### business telemetry

字段：

- `device_state`
- `task_state`
- `target_passes`
- `completed_passes`
- `clean_count`
- `active_config_version`
- `active_config`
- `pending_config`
- `active_command`
- `last_command`

编码见：[thingsboard_control_plane.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/thingsboard_control_plane.cc:772)

## 6. CloudService

头文件：[cloud_service.h](/home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/service/cloud_service.h:27)

当前职责：

- 上报 telemetry / attributes
- 路由 ThingsBoard RPC
- 路由 shared attributes push 与 snapshot response
- 网络离线时把 telemetry 写入 `DataCache`
- 周期性 `flush_cache()`

当前默认 Topics：

- `v1/devices/me/telemetry`
- `v1/devices/me/attributes`
- `v1/devices/me/attributes/request/{id}`
- `v1/devices/me/attributes/response/+`
- `v1/devices/me/rpc/request/+`
- `v1/devices/me/rpc/response/{id}`

实现见：[cloud_service.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/cloud_service.cc:63)

## 7. HealthService

头文件：[health_service.h](/home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/service/health_service.h:49)

当前模式：

- `Mode::HEALTH`
- `Mode::DIAGNOSTICS`

当前统一输出格式：

```json
{
  "ts": 1778481758995,
  "values": {
    "...": "..."
  }
}
```

### 7.1 health payload

当前字段族：

- 四轮：`lt_rpm/lt_cur/lt_err`、`rt_*`、`lb_*`、`rb_*`
- 滚刷：`br_rpm`、`br_run`、`br_err`
- 电池：`bat_soc`、`bat_vol`、`bat_chg`、`bat_alm`
- IMU：`imu_p`、`imu_r`、`imu_y`
- GPS：`gps_lat`、`gps_lon`、`gps_fix`

### 7.2 diagnostics payload

当前字段族：

- 四轮完整诊断：`*_rpm`、`*_tgt`、`*_cur`、`*_err`、`*_ec`、`*_on`、`*_ce`
- 行走组：`walk_cf`、`walk_ce`
- 滚刷：`br_rpm`、`br_tgt`、`br_cur`、`br_vol`、`br_tmp`、`br_stl`、`br_ce`
- BMS：`bat_soc`、`bat_vol`、`bat_cur`、`bat_tmp`、`bat_cmax`、`bat_cmin`、`bat_rah`、`bat_cyc`、`bat_alm`
- IMU：`imu_ax`、`imu_ay`、`imu_az`、`imu_gx`、`imu_gy`、`imu_gz`、`imu_p`、`imu_r`、`imu_y`、`imu_fr`、`imu_pe`
- GPS：`gps_lat`、`gps_lon`、`gps_alt`、`gps_spd`、`gps_sat`、`gps_hdp`、`gps_fix`、`gps_sent`

编码见：[health_service.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/health_service.cc:159)

运行开关：

- `diagnostics.mode`
- `diagnostics.cloud_upload`
- `diagnostics.local_log`
- `diagnostics.publish_interval_active_ms`
- `diagnostics.publish_interval_idle_ms`
- `diagnostics.local_log_path`

## 8. 当前未接入主程序的代码

仓库中仍有一些代码存在，但不属于当前 `main` 的默认运行路径：

- 距离传感器模块：当前 `main` 未实例化
- LoRaWAN：仅在 `network.transport_mode` 切到对应模式时参与网络层

文档更新时，应以“主程序当前真实装配链路”为准，不应把这些模块写成默认在线功能。
