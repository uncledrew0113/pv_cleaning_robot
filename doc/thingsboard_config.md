# ThingsBoard 接入说明

本文档只描述当前 MVP 已接入主程序的 ThingsBoard 行为，不保留历史兼容语义。

## 1. 当前接入范围

当前设备通过 MQTT over TLS 接入 ThingsBoard，主程序实际使用三类能力：

- Shared Attributes 下发运行参数。
- RPC 下发控制命令。
- Telemetry / Attributes 上行设备状态和健康数据。

当前接线由 `RobotApplication` 完成，进程入口 `main.cc` 只负责创建应用并调用 `run()`。

## 2. MQTT 与证书配置

MQTT 固定配置位于 `config.fixed.json` 的 `network.mqtt`：

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

启动后会：

1. 建立 MQTT 连接。
2. 订阅 RPC、shared attributes push、shared attributes response。
3. 发布启动属性。
4. 主动请求一次共享属性快照。

## 3. Shared Attributes

当前支持字段：

- `repeat_count`
- `clean_speed_rpm`
- `return_speed_rpm`
- `brush_rpm`
- `primary_dock`
- `min_battery_soc`
- `charge_stop_soc`
- `schedules`

生效规则：

- `schedules` 立即写入 active 配置，并同步调度器窗口。
- 其它业务参数写入 `config.runtime.pending.json`。
- 下一次任务启动前，pending 配置提升为 active。

## 4. RPC

当前注册五个 RPC：

- `clean_to_return`
- `clean_to_parking`
- `start_configured`
- `stop`
- `fault_reset`

语义：

- `clean_to_return`：从可信位置向返机端方向清扫。
- `clean_to_parking`：从可信位置向主停机端方向清扫。
- `start_configured`：按配置任务从主停机端语义启动。
- `stop`：只在任务运行中接受，执行后回到 `Idle`。
- `fault_reset`：只在 `FaultStopped` 接受，清除锁存故障后回到 `Idle`。

RPC 回复格式：

```json
{"code":"accepted"}
```

被拒绝时 `code` 为具体拒绝原因。

## 5. 上行数据

### 5.1 启动属性

字段：

- `software_version`
- `hardware_version`
- `device_model`
- `device_id`

### 5.2 业务遥测

业务遥测由 `ThingsBoardControlPlane::publish_business_telemetry()` 发布。

当前字段：

- `state`
- `fault`
- `cfg_ver`
- `repeat_count`
- `completed_cycles`

示例：

```json
{
  "state": "ExecutingMission",
  "fault": 0,
  "cfg_ver": 123456,
  "repeat_count": 1,
  "completed_cycles": 0
}
```

### 5.3 健康遥测

健康上报由 `HealthService` 负责，使用 ThingsBoard 推荐格式：

```json
{
  "ts": 1778481758995,
  "values": {
    "lt_rpm": 0.0,
    "rt_rpm": 0.0
  }
}
```

模式：

- `production`：精简 health 字段。
- `development`：完整 diagnostics 字段。

配置位于 `config.fixed.json` 的 `diagnostics`：

- `mode`
- `cloud_upload`
- `local_log`
- `collect_interval_ms`
- `publish_interval_active_ms`
- `publish_interval_idle_ms`
- `local_log_interval_active_ms`
- `local_log_interval_idle_ms`
- `local_log_path`
- `local_log_max_bytes`
- `local_log_max_files`

上报周期：

- 运行态使用 `publish_interval_active_ms`。
- 空闲态使用 `publish_interval_idle_ms`。
- 本地日志运行态使用 `local_log_interval_active_ms`。
- 本地日志空闲态使用 `local_log_interval_idle_ms`。

## 6. 本地缓存与断网恢复

当 `diagnostics.cloud_upload=true` 时，遥测通过 `CloudService` 进入发送队列。
未成功确认的数据会写入本地缓存。

默认路径：

- `data/pv_robot/telemetry_cache.jsonl`

缓存由 `CloudService::flush_cache()` 在网络恢复后继续发送。

## 7. 旧业务遥测说明

当前业务遥测只以上文列出的五个字段为准。历史版本中的扩展任务字段、命令字段和
配置快照字段不再作为当前 MVP 接口；如云端仪表盘仍依赖旧字段，需要按产品需求单独恢复，
不应在旧残留清理中隐式保留。
