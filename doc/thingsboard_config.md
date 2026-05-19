# ThingsBoard 接入说明

本文档只描述当前主分支已经实现并接入主程序的 ThingsBoard 行为，不保留历史兼容语义。

## 1. 当前接入范围

当前设备通过 MQTT over TLS 接入 ThingsBoard，主程序实际使用的能力有三类：

- Shared Attributes 下发运行参数
- RPC 下发控制命令
- Telemetry / Attributes 上行设备状态、命令事件和健康数据

主入口与接线见：

- [main.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/main.cc:447)
- [thingsboard_control_plane.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/thingsboard_control_plane.cc:883)
- [cloud_service.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/cloud_service.cc:26)

## 2. MQTT 与证书配置

当前 MQTT 固定配置位于 [config.fixed.json](/home/tronlong/pv_cleaning_robot/config/config.fixed.json:41) 的 `network.mqtt`：

- `broker_uri`
- `client_id`
- `username`
- `password`
- `tls_enabled`
- `ca_cert_path`
- `client_cert_path`
- `client_key_path`
- `insecure_skip_server_name_check`
- `keep_alive_s`
- `qos`

当前代码启动后会：

1. 建立 MQTT 连接
2. 订阅：
   - `v1/devices/me/rpc/request/+`
   - `v1/devices/me/attributes`
   - `v1/devices/me/attributes/response/+`
3. 发布启动属性
4. 主动请求一次共享属性快照

说明：

- `insecure_skip_server_name_check=true` 只适合证书 CN / SAN 与 `broker_uri` 不匹配的调试环境。
- 正式部署建议改为证书和域名完全匹配，并将该开关改回 `false`。

## 3. Shared Attributes

### 3.1 当前支持字段

当前只支持这些共享属性：

- `passes`
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `return_brush_rpm`
- `parking_side`
- `start_battery_soc`
- `charge_start_soc`
- `charge_stop_soc`
- `schedules`

解析与生效逻辑见 [thingsboard_control_plane.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/thingsboard_control_plane.cc:358)。

### 3.2 生效规则

- `schedules`
  - 立即写入 active 配置
  - 同步更新调度器窗口
  - 同时保存到 runtime 主配置
- 其它业务参数
  - 先写入 `config.runtime.pending.json`
  - 当前任务不打断
  - 在下一次任务启动前提升为 active

配置文件关系：

- `config.runtime.json`：当前 active 业务配置
- `config.runtime.pending.json`：待下一个任务生效的补丁
- `config.runtime.backup.json`：runtime 的最近一次备份

### 3.3 快照拉取语义

设备每次连上云端后会主动请求一次共享属性快照，目的是覆盖“离线期间平台改过配置，但设备重连后平台未主动补推”的情况。

当前没有字段级合并机制，语义是：

- 通过校验的更新直接应用
- 后到的合法更新覆盖先到的合法更新
- 最终以设备最后一次成功保存的值为准

## 4. RPC

### 4.1 当前支持命令

当前只注册四个 RPC：

- `start`
- `stop`
- `return`
- `reset`

启动属性里的 `supported_rpc_methods` 也与此一致。

### 4.2 设备侧语义

#### `start`

- 允许状态：`Idle`、`Charging`、`Stopped`
- 要求 `position_valid=true`
- 要求当前电量满足 `robot.start_battery_soc`
- 这是特权启动
  - 不要求当前位于停机位
  - 从当前位置开始执行完整任务
  - 正向阶段进入 `CleanFwd`
  - 到对侧后自动进入 `CleanReturn`
  - 返程继续带刷清扫

#### `stop`

- 允许状态：`CleanFwd`、`CleanReturn`、`Returning`
- 执行后进入 `Stopped`
- 立即停行走轮和滚刷

#### `return`

- 允许状态：`CleanFwd`、`CleanReturn`、`Stopped`、`Idle`
- 如果当前已经在配置停机位一侧，则拒绝
- 执行后进入 `Returning`
- 调用的是 `start_returning_no_brush()`
- 返程不带刷

#### `reset`

- 当前实现为立即接受
- 发布 RPC 响应后异步执行系统重启

RPC 处理逻辑见 [thingsboard_control_plane.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/thingsboard_control_plane.cc:883)。

## 5. 上行数据

### 5.1 启动属性

设备连接成功后会发布启动属性，字段为：

- `software_version`
- `hardware_version`
- `device_model`
- `device_id`
- `supported_rpc_methods`
- `config_schema_version`

### 5.2 业务遥测

业务遥测由 `ThingsBoardControlPlane::publish_business_telemetry()` 发布，当前字段为：

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

其中：

- `device_state` 直接反映 FSM 状态
- `task_state` 是给云端看的任务态抽象，如 `IdleTask`、`RunningTask`、`ReturningTask`
- `active_config` / `pending_config` 是当前和待生效业务配置快照

### 5.3 命令事件

设备会额外发布命令生命周期事件，用于平台审计与调试。当前事件包括：

- `command_accepted`
- `command_completed`
- `command_rejected`

事件体包含：

- `event`
- `command_id`
- `command_name`
- `request_id`
- `phase`
- `reason`
- `accepted_at_ms`
- `finished_at_ms`

### 5.4 健康遥测

健康上报由 `HealthService` 负责，统一使用 ThingsBoard 推荐格式：

```json
{
  "ts": 1778481758995,
  "values": {
    "lt_rpm": 0.0,
    "rt_rpm": 0.0
  }
}
```

不是下面这种扁平格式：

```json
{
  "ts": 1778481758995,
  "lt_rpm": 0.0,
  "rt_rpm": 0.0
}
```

当前健康数据分两种模式：

- `production`
  - 精简 health 字段
- `development`
  - 完整 diagnostics 字段

诊断配置位于 [config.fixed.json](/home/tronlong/pv_cleaning_robot/config/config.fixed.json:71)：

- `diagnostics.mode`
- `diagnostics.cloud_upload`
- `diagnostics.local_log`
- `diagnostics.publish_interval_active_ms`
- `diagnostics.publish_interval_idle_ms`
- `diagnostics.local_log_path`
- `diagnostics.local_log_max_bytes`
- `diagnostics.local_log_max_files`

当前上报周期规则：

- 运行态：使用 `publish_interval_active_ms`
- 空闲态：使用 `publish_interval_idle_ms`

当前主程序会将 `Idle`、`Stopped`、`Charging` 视为空闲态，因此 idle 时可以降到较低上报频率，例如 5 分钟一次。

### 5.5 当前 health / diagnostics 字段

`production` health 当前主要字段：

- 行走轮：
  - `lt_rpm`、`rt_rpm`、`lb_rpm`、`rb_rpm`
  - `lt_cur`、`rt_cur`、`lb_cur`、`rb_cur`
  - `lt_err`、`rt_err`、`lb_err`、`rb_err`
- 滚刷：
  - `br_rpm`
  - `br_run`
  - `br_err`
- 电池：
  - `bat_soc`
  - `bat_vol`
  - `bat_chg`
  - `bat_alm`
- IMU：
  - `imu_p`
  - `imu_r`
  - `imu_y`
- GPS：
  - `gps_lat`
  - `gps_lon`
  - `gps_fix`

`development` diagnostics 会在此基础上继续包含：

- 四个行走轮目标速度、电流、错误码、在线状态、通讯错误
- 行走电机组故障统计
- 滚刷目标转速、电流、电压、温度、堵转状态
- BMS 细项
- IMU 三轴加速度、角速度、姿态、帧率
- GPS 速度、海拔、卫星数、HDOP 等

完整字段见 [health_service.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/health_service.cc:39)。

## 6. 本地缓存与断网恢复

当 `diagnostics.cloud_upload=true` 时，遥测通过 `CloudService` 进入发送队列；未成功确认的数据会写入本地缓存：

- 默认路径：`data/pv_robot/telemetry_cache.jsonl`

缓存记录格式是本地发送队列记录，不是 ThingsBoard 原始协议体。例如：

```json
{
  "op": "push",
  "id": 7,
  "topic": "v1/devices/me/telemetry",
  "payload": "{\"ts\":1778481758995,\"values\":{\"lt_rpm\":0.0}}",
  "ts_ms": 1778481758995
}
```

其中 `payload` 字段内部仍然是发往 ThingsBoard 的标准 JSON 字符串。

## 7. 平台侧联调建议

### 7.1 先准备的 Shared Attributes

建议先只使用这些合法字段：

```json
{
  "passes": 1,
  "clean_speed_rpm": 20,
  "return_speed_rpm": 20,
  "brush_rpm": 2000,
  "return_brush_rpm": 3000,
  "parking_side": "right",
  "start_battery_soc": 0.0,
  "charge_start_soc": 0.0,
  "charge_stop_soc": 1.0,
  "schedules": [
    { "hour": 8, "minute": 0 },
    { "hour": 14, "minute": 30 }
  ]
}
```

### 7.2 推荐观察面

平台侧至少观察这些键：

- `device_state`
- `task_state`
- `target_passes`
- `completed_passes`
- `active_config`
- `pending_config`
- `active_command`
- `last_command`
- 关键 health 字段，如 `lt_rpm`、`rb_err`、`imu_p`、`bat_soc`

### 7.3 推荐联调顺序

1. 启动设备，确认：
   - MQTT connected
   - 启动属性发布成功
   - attributes 快照响应成功
2. 修改 `schedules`
   - 确认设备立即保存 active 配置
3. 修改 `passes` 或 `brush_rpm`
   - 确认进入 pending，而不是直接打断当前任务
4. 发送 RPC `start`
   - 确认接受并进入 `CleanFwd`
5. 发送 RPC `stop`
   - 确认进入 `Stopped`
6. 发送 RPC `return`
   - 确认进入 `Returning`，且返程不带刷
7. 发送 RPC `reset`
   - 确认设备响应后重启

## 8. 当前明确不在接入范围内的内容

当前 ThingsBoard 文档只覆盖主程序已经接线的配置、控制和上报链路，不扩展到以下方向：

- 未接入主程序的历史控制面
- 未使用的旧业务字段
- LoRaWAN 控制面
- 更复杂的调度策略

这些内容当前不属于主程序默认运行事实。
