# ThingsBoard 首版接入说明

本文档只描述当前代码已经实现并验证过的 ThingsBoard 能力，不包含未上线功能设想。

## 1. 当前已支持的能力

### 1.1 RPC 控制

通过 ThingsBoard RPC 下发，当前支持：

- `start`
  - `Idle` / `Charging` 下启动新任务
  - `Paused` 下恢复已暂停任务
- `stop`
  - 仅在 `CleanFwd` / `CleanReturn` 下有效
  - 执行后进入 `Paused`
- `return`
  - 仅在 `CleanFwd` / `CleanReturn` / `Paused` 下有效
  - 执行后进入 `Returning`
- `terminate`
  - 仅在 `CleanFwd` / `CleanReturn` / `Paused` / `Returning` 下有效
  - 执行后进入 `Terminated`
- `reset`
  - 仅在 `Fault` / `Terminated` 且 `at_home=true` 下有效
  - 执行后进入 `Idle`

### 1.2 遥测

通过 telemetry 上报，当前已实现：

- health / diagnostics telemetry
  - 由 `HealthService` 单独上报
  - 与 business telemetry 不是同一份 payload
  - `production` 模式下为精简 health 字段
  - `development` 模式下为完整 diagnostics 字段
- startup attributes
  - `software_version`
  - `hardware_version`
  - `device_model`
  - `device_id`
  - `supported_rpc_methods`
  - `config_schema_version`
- status / command event
  - `shared_attr_update`
  - `config_backup_fallback`
  - `command_accepted`
  - `command_completed`
  - `command_rejected`
- business telemetry
  - `device_state`
  - `task_state`
  - `target_half_passes`
  - `completed_half_passes`
  - `clean_count`
  - `active_config`
  - `pending_config`
  - `active_config_version`
  - `last_command`

当前上报频率规则：

- active 状态
  - 使用 `diagnostics.publish_interval_active_ms`
  - 未配置时回退到 `diagnostics.publish_interval_ms`
- idle 状态
  - 使用 `diagnostics.publish_interval_idle_ms`
  - 默认 `300000ms`，即 `5 分钟`

当前 `RobotSupervisor` 视为 active 的状态包括：

- `CleanFwd`
- `CleanReturn`
- `Returning`
- `Paused`

也就是说，`Paused` 当前仍然走 active 上报周期，不走 idle 周期。

### 1.3 云端配置（Shared Attributes）

当前支持：

- `passes`
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `parking_policy`
- `charging_side`
- `schedules`

当前生效规则：

- `schedules`
  - 立即写入 `active`
  - 同时同步到 `pending`
- 任务相关参数
  - 写入 `pending`
  - 在下一次任务启动前由 `promote_pending_to_active()` 提升

当前上线后的拉取与覆盖语义：

- 设备 MQTT 连接成功后，会主动请求一次当前 release 关心的 shared attributes 快照
- 这次快照的目的，是补齐“设备离线期间平台改过配置，但平台重连后不一定自动补推”的场景
- 当前没有 shared attributes 的版本合并或字段级冲突解决机制
- 语义是：
  - 先到先应用
  - 后到后覆盖
  - 最终以最后一条成功通过校验的 shared attributes 更新为准
- 因此，如果设备刚上线拉取快照的同时，平台侧又手动改了配置：
  - 若平台保存后的新值已经进入快照响应，设备会直接拿到新值
  - 若设备先收到旧快照，随后平台再推新修改，则后到的新修改会覆盖旧快照结果

## 2. 当前明确不支持的内容

以下内容在首版里明确排除：

- `passes=0.5`
- `parking_policy=both`
- 无限趟
- 按星期几的 schedule
- OTA
- LoRaWAN
- 欠压阈值云端配置
- 温度保护云端配置
- 过流保护云端配置
- 任务超时保护云端配置
- 速度百分比语义
  - 当前使用的是 `*_rpm`，不是 `0~100%`

## 3. 配置示例

当前 MQTT/TLS 配置示例：

```json
{
  "network": {
    "mqtt": {
      "broker_uri": "ssl://124.222.209.230:8883",
      "client_id": "pv_robot_001",
      "username": "",
      "password": "",
      "tls_enabled": true,
      "ca_cert_path": "/root/pv_cleaning_robot/bin/config/tb_server_ca.pem",
      "client_cert_path": "/root/pv_cleaning_robot/bin/config/pv_device_test1.pem",
      "client_key_path": "/root/pv_cleaning_robot/bin/config/pv_device_test1.key",
      "insecure_skip_server_name_check": true,
      "keep_alive_s": 60,
      "qos": 1
    }
  }
}
```

说明：

- `insecure_skip_server_name_check=true`
  - 仅用于当前调试环境
  - 适用于服务端证书和 `broker_uri` 中使用的 IP/主机名不匹配的情况
- 正式上线建议：
  - 改用证书匹配的域名
  - 或重签服务端证书，把目标 IP 加入 SAN
  - 最终将该开关恢复为 `false`

## 4. ThingsBoard 平台侧需要准备的内容

### 4.1 设备认证

- 创建目标设备
- 配置 MQTT over TLS
- 导入或绑定当前客户端证书身份
- 确认设备连接到正确的 broker / device profile

### 4.2 Shared Attributes 准备

平台侧需要可手动修改：

- `passes`
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `parking_policy`
- `charging_side`
- `schedules`

推荐首轮联调只使用合法值，例如：

```json
{
  "passes": 2,
  "clean_speed_rpm": 320,
  "return_speed_rpm": 280,
  "brush_rpm": 1000,
  "parking_policy": "terminal_a_only",
  "charging_side": "terminal_a",
  "schedules": [
    { "hour": 8, "minute": 0 }
  ]
}
```

### 4.3 RPC 准备

平台侧需要能手动发送：

- `start`
- `stop`
- `return`
- `terminate`
- `reset`

并同时观察：

- RPC response
- command event
- business telemetry 中的 `device_state` / `task_state`

### 4.4 遥测观察面

平台侧建议至少建出这些可视化项：

- `device_state`
- `task_state`
- `target_half_passes`
- `completed_half_passes`
- `active_config`
- `pending_config`
- `active_config_version`
- `last_command`

## 5. 真实联调测试

### 5.1 真实云端集成测试

手动启用：

```bash
TB_REAL_TEST=1 ./unit_tests "[integration][thingsboard][real]"
```

已覆盖：

- mutual TLS 连接
- startup attributes publish
- business telemetry publish
- shared attributes 合法更新
- shared attributes 非法值拒绝

### 5.2 真实硬件 + 真实云端联调测试

手动启用：

```bash
TB_REAL_TEST=1 ./hw_tests "[hw_system][tb_rpc_runtime]"
TB_REAL_TEST=1 ./hw_tests "[hw_system][tb_terminate_reset]"
```

平台侧动作要求：

- `tb_rpc_runtime`
  - 依次发送 `start -> stop -> return`
  - `return` 后人工触发回家限位，使状态进入 `Charging`
- `tb_terminate_reset`
  - 依次发送 `start -> stop -> terminate -> reset`
  - `reset` 前确保 `at_home=true`

## 6. 快速上线判定

当前版本满足快速上线的最低要求，前提是以下 4 条都已通过真实板子联调：

- mutual TLS 连接成功
- startup attributes / telemetry 可上报
- shared attributes 可下发，并符合当前 release 规则
- RPC 能驱动真实状态变化，并在云端看到对应 telemetry / event

如果上述 4 条没有完整跑通，不建议直接上线。
