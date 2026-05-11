# Health Telemetry Ts Values Design

## Goal

把 `HealthService` 产出的 `health` / `diagnostics` 遥测统一改成 ThingsBoard 推荐的时序格式，并压缩键名，减少上行流量。

本次只做以下事情：

- `health` 和 `diagnostics` 都改成：
  ```json
  {"ts": 1710000000123, "values": {...}}
  ```
- `ts` 使用 Unix 毫秒时间戳
- `values` 内使用扁平、可读缩写键
- `health` 的 walk 字段改为四轮分别上报 rpm / 电流 / 错误
- `health` 的 imu 字段至少包含 `pitch / roll / yaw`
- 改完后按典型业务流程估算月上行 payload 流量

本次不做以下事情：

- 不改 `ThingsBoardControlPlane` 的 business telemetry / status event / command event 格式
- 不改 attributes / RPC response / shared attribute request 的格式
- 不把键名压到不可读的极限缩写
- 不统计 MQTT/TLS 协议额外开销，只统计上行 payload 体积

## Current Problem

当前项目里只有 `HealthService` 产出的 `health` / `diagnostics` 遥测带 `ts`，但格式不是 ThingsBoard 推荐格式。

当前格式是：

```json
{"ts":"2026-05-11T12:34:56Z", ...}
```

存在三个问题：

1. `ts` 是 ISO8601 字符串，不是 ThingsBoard 常用的毫秒时间戳
2. 没有使用推荐的 `{"ts":...,"values":{...}}` 包裹结构
3. `health` 中 `walk` 只发四轮平均值，不利于定位单轮问题

另外，当前键名较长、层级较深，单帧 payload 体积偏大，不利于长期上行流量控制。

## Target Format

### 1. Unified Envelope

`health` 和 `diagnostics` 都统一为：

```json
{"ts": 1710000000123, "values": {...}}
```

约束：

- `ts` 为 Unix epoch 毫秒整数
- 所有业务字段都放入 `values`
- 顶层不再出现其他业务字段

### 2. Flat Keys

`values` 不保留嵌套对象，统一改成扁平键。

采用“可读缩写”策略，不走极限压缩。

键名风格示例：

- 行走轮
  - `lt_rpm` `rt_rpm` `lb_rpm` `rb_rpm`
  - `lt_cur` `rt_cur` `lb_cur` `rb_cur`
  - `lt_err` `rt_err` `lb_err` `rb_err`
- 滚刷
  - `br_rpm`
  - `br_tgt`
  - `br_cur`
  - `br_vol`
  - `br_tmp`
  - `br_run`
  - `br_err`
- 电池
  - `bat_soc`
  - `bat_vol`
  - `bat_cur`
  - `bat_tmp`
  - `bat_chg`
  - `bat_alm`
- IMU
  - `imu_p`
  - `imu_r`
  - `imu_y`
  - diagnostics 下再补更细项
- GPS
  - `gps_lat`
  - `gps_lon`
  - `gps_fix`

## Health Payload Design

`health` 定位为“生产环境的轻量健康帧”。

### Required Fields

#### Walk

不再发送平均 rpm / 平均 torque。

改为四轮分别发送：

- `lt_rpm` `rt_rpm` `lb_rpm` `rb_rpm`
- `lt_cur` `rt_cur` `lb_cur` `rb_cur`
- `lt_err` `rt_err` `lb_err` `rb_err`

说明：

- `rpm` 直接使用当前轮速
- `cur` 直接使用当前电流/力矩电流字段
- `err` 用布尔值表达该轮是否故障

#### Brush

保留最核心健康信息：

- `br_run`
- `br_err`

可选保留：

- `br_rpm`

#### Battery

保留轻量字段：

- `bat_soc`
- `bat_vol`
- `bat_chg`
- `bat_alm`

#### IMU

`health` 中 IMU 至少包含：

- `imu_p`
- `imu_r`
- `imu_y`

#### GPS

保留轻量定位字段：

- `gps_lat`
- `gps_lon`
- `gps_fix`

## Diagnostics Payload Design

`diagnostics` 定位为“开发/联调时的详细诊断帧”。

它也统一为 `ts + values`，但字段比 `health` 更完整。

### Walk

保留四轮逐项诊断能力：

- `lt_rpm` `rt_rpm` `lb_rpm` `rb_rpm`
- `lt_tgt` `rt_tgt` `lb_tgt` `rb_tgt`
- `lt_cur` `rt_cur` `lb_cur` `rb_cur`
- `lt_err` `rt_err` `lb_err` `rb_err`
- `lt_ec` `rt_ec` `lb_ec` `rb_ec`  （error code）
- `lt_on` `rt_on` `lb_on` `rb_on`

总控字段保留：

- `walk_cf`  （ctrl frame count）
- `walk_ce`  （ctrl error count）

### Brush

保留现有详细诊断能力，但键名缩短：

- `br_rpm`
- `br_tgt`
- `br_cur`
- `br_vol`
- `br_tmp`
- `br_stl`
- `br_ce`

### BMS

保留：

- `bat_soc`
- `bat_vol`
- `bat_cur`
- `bat_tmp`
- `bat_cmax`
- `bat_cmin`
- `bat_rah`
- `bat_cyc`
- `bat_alm`

### IMU

保留姿态与原始量：

- `imu_ax` `imu_ay` `imu_az`
- `imu_gx` `imu_gy` `imu_gz`
- `imu_p`
- `imu_r`
- `imu_y`
- `imu_fr`
- `imu_pe`

### GPS

保留：

- `gps_lat`
- `gps_lon`
- `gps_alt`
- `gps_spd`
- `gps_sat`
- `gps_hdp`
- `gps_fix`
- `gps_sent`

## Timestamp Rule

`ts` 改为 Unix 毫秒时间戳。

实现要求：

- `health` 和 `diagnostics` 都统一使用同一套毫秒时间戳生成逻辑
- 不再生成 ISO8601 字符串
- 本地日志 JSONL 与上云 payload 保持一致

## Scope

预计涉及：

- [include/pv_cleaning_robot/service/health_service.h](/home/tronlong/pv_cleaning_robot/include/pv_cleaning_robot/service/health_service.h)
- [pv_cleaning_robot/service/health_service.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/health_service.cc)
- 相关 `HealthService` / 硬件集成测试
- 必要时更新 README 中关于 telemetry 示例的说明

本次不涉及：

- [pv_cleaning_robot/service/thingsboard_control_plane.cc](/home/tronlong/pv_cleaning_robot/pv_cleaning_robot/service/thingsboard_control_plane.cc)
- `business telemetry`
- `startup attributes`
- `command/status events`

## Traffic Estimate

改完后需要给出“典型业务流程”的月上行 payload 估算。

口径固定为：

- 只算上行上传 payload
- 不算 MQTT/TLS/重传等协议开销
- 每天 1 次调度启动
- 每次任务 `passes = 1`
- 单次运行约 20 分钟
- 运行态上报周期：1 秒 1 帧
- 空闲态上报周期：5 分钟 1 帧

输出内容应包括：

- 单帧 `health` 大小估算
- 单帧 `diagnostics` 大小估算
- 每日上行量估算
- 每月上行量估算

## Testing

至少覆盖：

- `health` payload 改为 `{"ts":<int64>,"values":{...}}`
- `diagnostics` payload 改为 `{"ts":<int64>,"values":{...}}`
- `ts` 为整数毫秒，不是字符串
- `health` 中四轮 rpm / 电流 / 错误字段存在
- `health` 中 `imu_p / imu_r / imu_y` 存在
- `diagnostics` 中保留详细字段，但键名已缩短
- 本地 JSONL 与上云 payload 一致

## Risks

主要风险有两个：

1. 云端现有规则链或仪表盘如果依赖旧键名，会需要同步调整
2. 扁平键一旦命名不稳定，后续统计和报警规则会频繁改动

因此本次设计强调：

- 只改 `HealthService`
- 一次性定好缩写键名
- 不再保留旧格式与新格式双写，避免长期维护两套遥测合同
