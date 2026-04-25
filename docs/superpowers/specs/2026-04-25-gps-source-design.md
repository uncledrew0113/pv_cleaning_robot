# GPS 数据源扩展设计文档

**日期：** 2026-04-25  
**状态：** 已确认  
**范围：** GPS 设备层与启动配置扩展，业务服务接口保持不变

---

## 1. 背景与目标

当前项目中的 `GpsDevice` 只支持一种工作方式：

- 通过 `LibSerialPort` 从 `/dev/ttyS*` 串口读取 NMEA 0183 文本
- 在 `GpsDevice` 内部使用 `NmeaParser` 完成解析
- 将最新 `GpsData` 和 `Diagnostics` 提供给 `HealthService`、`NavService` 等上层服务

现在 GPS 数据解析已经由嵌入式 Linux 的 `gpsd` 服务接管，需要新增第二种数据接入方式：

- 继续支持原有串口读取
- 新增通过标准 `gpsd` TCP 服务读取定位数据
- 通过配置显式选择数据源，不做自动回退

本次设计目标：

1. 保持上层服务继续依赖 `GpsDevice`，不感知底层数据来源变化
2. 将“串口 + NMEA”和“gpsd + JSON/TCP”拆分为可独立维护的后端
3. 在 `gpsd` 模式下只实现读数据，不实现对 GPS 模组的控制命令
4. 保证现有串口模式行为不回退，并为旧配置提供平滑兼容

---

## 2. 现状与约束

### 2.1 现状

当前初始化链路位于 `pv_cleaning_robot/main.cc`：

```cpp
auto gps_serial = std::make_shared<robot::driver::LibSerialPort>(
    cfg.get<std::string>("serial.gps.port", "/dev/ttyS2"),
    robot::hal::UartConfig{cfg.get<int>("serial.gps.baudrate", 9600)});
auto gps = std::make_shared<robot::device::GpsDevice>(gps_serial);
```

当前 `GpsDevice` 同时承担了多种职责：

- 设备生命周期管理
- 串口读取线程
- 行缓冲
- NMEA 解析
- 诊断统计
- 最新数据缓存

这种实现只适合单一串口来源；继续在同一个类中直接塞入 `gpsd` TCP、JSON 解析和重连逻辑，会让职责进一步混杂。

### 2.2 已确认约束

- 数据源选择方式：**仅通过配置显式选择**
- `gpsd` 接入方式：**标准 gpsd TCP 服务**
- `gpsd` 模式下：**不实现 `set_output_rate()` / `hot_restart()` / `cold_restart()`**
- 上层服务接口：**保持现有 `GpsDevice` 的读取接口不变**

---

## 3. 方案比较

### 方案 A：保留 `GpsDevice` 门面，内部拆分数据源后端

做法：

- `GpsDevice` 继续对外暴露现有接口
- 内部新增 GPS 数据源抽象
- 分别实现串口后端和 `gpsd` 后端

优点：

- 上层调用点改动最小
- 串口和 `gpsd` 的实现边界清晰
- 单元测试更容易覆盖不同来源
- 后续如果要增加第三种来源，扩展成本较低

缺点：

- 需要新增抽象层和少量 glue code

### 方案 B：拆成 `SerialGpsDevice` / `GpsdGpsDevice` 两个完整设备类

优点：

- 类型边界最清楚

缺点：

- `HealthService`、`NavService`、`main.cc` 都要改成依赖新接口
- 改动面大于本次需求

### 方案 C：继续只保留一个 `GpsDevice`，内部按模式分支

优点：

- 表面上改动最少

缺点：

- 一个类同时负责串口、TCP、NMEA、JSON、重连、统计
- 测试与维护成本高
- 后续继续扩展时会快速失控

### 推荐方案

采用**方案 A：统一门面 + 可插拔数据源后端**。

---

## 4. 总体设计

### 4.1 结构划分

保留 `GpsDevice` 作为统一门面，新增内部数据源抽象，例如：

```cpp
class IGpsSource {
public:
    virtual ~IGpsSource() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual DeviceError set_output_rate(int hz) = 0;
    virtual DeviceError hot_restart() = 0;
    virtual DeviceError cold_restart() = 0;
};
```

实现两个后端：

- `SerialGpsSource`
  - 依赖 `hal::ISerialPort`
  - 复用现有 `NmeaParser`
  - 负责串口读取线程与 NMEA 行处理
- `GpsdTcpSource`
  - 依赖 Linux TCP socket
  - 连接 `gpsd` 服务
  - 发送 `WATCH` 指令
  - 消费 `TPV` / `SKY` JSON 报文

`GpsDevice` 自身负责：

- 依据配置构造正确的数据源后端
- 提供线程安全的 `get_latest()` / `get_diagnostics()`
- 统一维护最新 `GpsData`
- 统一维护诊断计数

### 4.2 职责边界

职责分配固定为：

- 后端负责“从哪里读”和“如何解析原始输入”
- `GpsDevice` 负责“如何缓存”和“如何暴露给业务层”

这样 `HealthService`、`NavService`、诊断上报都不需要知道数据到底来自串口还是 `gpsd`。

---

## 5. 配置设计

### 5.1 新配置结构

新增顶层 `gps` 配置节点：

```json
"gps": {
  "source": "serial",
  "serial": {
    "port": "/dev/ttyS2",
    "baudrate": 115200
  },
  "gpsd": {
    "host": "127.0.0.1",
    "port": 2947,
    "watch": "?WATCH={\"enable\":true,\"json\":true};"
  }
}
```

含义如下：

- `gps.source`
  - `"serial"`：从物理串口直接读取
  - `"gpsd"`：从本机或远端 `gpsd` TCP 服务读取
- `gps.serial.*`
  - 串口模式参数
- `gps.gpsd.*`
  - `gpsd` 主机、端口、默认 `WATCH` 命令

### 5.2 兼容策略

为避免现网配置一次性失效，本次实现保留一版向后兼容：

1. 若存在 `gps.source`，优先按新配置解析
2. 若 `gps.source` 缺失，则回退读取旧配置 `serial.gps.*`
3. 回退场景默认等价于：

```json
"gps": {
  "source": "serial",
  "serial": {
    "port": "<serial.gps.port>",
    "baudrate": "<serial.gps.baudrate>"
  }
}
```

### 5.3 启动代码调整

`main.cc` 不再固定构造 `LibSerialPort + GpsDevice(serial)`，而是改为：

- 从配置读取 `gps.source`
- `serial` 模式构造串口后端
- `gpsd` 模式构造 TCP 后端
- 最终仍只向上层暴露一个 `std::shared_ptr<GpsDevice>`

---

## 6. `gpsd` 协议接入设计

### 6.1 接入方式

`GpsdTcpSource` 按标准 `gpsd` socket 协议接入：

1. TCP 连接 `host:port`
2. 读取连接建立后 `gpsd` 自动返回的 `VERSION`
3. 发送：

```text
?WATCH={"enable":true,"json":true};
```

4. 持续接收以 `CR-LF` 结束的 JSON 报文
5. 根据 `class` 区分 `TPV`、`SKY`、`WATCH`、`VERSION`、`ERROR`

本项目只消费：

- `TPV`
- `SKY`

其余报文：

- `VERSION` / `WATCH`：忽略，仅用于启动握手
- `ERROR`：记日志并计入错误统计

### 6.2 字段映射

`gpsd` JSON 到现有 `GpsData` 的映射固定如下：

| `gpsd` 字段 | `GpsData` 字段 | 说明 |
|---|---|---|
| `TPV.lat` | `latitude` | 十进制度 |
| `TPV.lon` | `longitude` | 十进制度 |
| `TPV.speed` | `speed_m_s` | `gpsd` 已为 m/s |
| `TPV.track` | `course_deg` | 地面航向角 |
| `TPV.altMSL` | `altitude_m` | 优先使用 |
| `TPV.altHAE` | `altitude_m` | 当 `altMSL` 缺失时回退 |
| `TPV.time` | `utc_timestamp_ms` | ISO8601 UTC 转 epoch ms |
| `SKY.hdop` | `hdop` | 与现有字段语义一致 |
| `SKY.pdop` | `pdop` | 与现有字段语义一致 |
| `SKY.vdop` | `vdop` | 与现有字段语义一致 |
| `SKY.nSat` | `satellites_in_view` | 优先使用，缺失时回退 `satellites.size()` |
| `count(satellites[].used == true)` | `satellites_used` | 仅在 `satellites` 数组存在且 `used` 可统计时更新 |

### 6.3 定位状态映射

`gpsd` 的 `TPV.mode` 为：

- `0`：unknown
- `1`：no fix
- `2`：2D
- `3`：3D

项目内已有字段 `fix_quality` 来自 NMEA GGA，语义不完全等价。为了兼容现有上层使用方式，定义如下映射：

| `TPV.mode` | `valid` | `fix_quality` |
|---|---|---|
| `0` / `1` | `false` | `0` |
| `2` | `true` | `1` |
| `3` | `true` | `2` |

这里的 `fix_quality` 不再表达真实 NMEA 质量值，而是表达**兼容性的“可用定位等级”**。

### 6.4 增量更新策略

`gpsd` 的 `TPV` 与 `SKY` 通常分开发送，且字段缺失时不是 `null`，而是直接省略。因此缓存更新规则固定为：

- 报文只更新自己携带的字段
- 缺失字段保持上次值，不清零
- 只有在收到明确 `TPV.mode < 2` 时，才将 `valid` 置 `false`
- `satellites_used` 只在 `SKY.satellites[].used` 可统计时更新，否则保持上次值

这样可避免：

- 因 `SKY` 报文没有位置字段而覆盖位置为 0
- 因 `TPV` 报文没有卫星视图而丢失 `SKY` 统计

---

## 7. 控制接口行为

保留 `GpsDevice` 现有控制接口：

- `set_output_rate(int hz)`
- `hot_restart()`
- `cold_restart()`

行为定义如下：

- 串口模式：保持现有实现，直接向 GPS 模组发送控制命令
- `gpsd` 模式：统一返回 `DeviceError::NOT_SUPPORTED`

这里不做“空实现后返回成功”，原因是：

- `gpsd` 已接管底层设备，不应伪装仍能直接控制模组
- 明确返回不支持更容易让调用方定位模式差异

---

## 8. 错误处理与重连策略

### 8.1 串口模式

串口模式沿用现有思路：

- 读超时：继续轮询
- 无效 NMEA：累计解析错误
- 关闭设备时安全退出线程

### 8.2 `gpsd` 模式

`GpsdTcpSource` 的错误处理策略如下：

- `open()`
  - 完成 `socket()`、`connect()`、初始握手
  - 任何一步失败则返回 `false`
- 读线程运行中
  - `recv()` 返回 EOF：视为连接断开
  - `recv()` 返回系统错误：视为连接断开
  - 读到无法组成完整 JSON 的损坏片段：记错误并丢弃当前行
- 连接断开后
  - 在线程内等待固定退避时间后重连，建议 1 秒
  - 重连成功后重新发送 `WATCH`
- `close()`
  - 先将 `running_` 置为 false
  - 再 `shutdown()` / `close()` socket，主动打断阻塞 `recv()`
  - 最后 `join()` 读线程

该策略保证：

- 启动阶段可用 `open()` 的返回值快速判断初始接入是否成功
- 运行阶段出现短暂服务重启时可以自恢复
- 停机流程不会因为阻塞读而卡死

---

## 9. 诊断统计语义

沿用现有 `Diagnostics` 结构：

```cpp
struct Diagnostics : GpsData {
    uint32_t sentence_count;
    uint32_t parse_error_count;
    uint32_t fix_loss_count;
};
```

但不同来源下的计数语义统一定义如下：

- `sentence_count`
  - 串口模式：成功收到一条完整 NMEA 行时加一
  - `gpsd` 模式：成功解析一条完整 JSON 报文时加一
- `parse_error_count`
  - 串口模式：校验失败、过长行、无效句子时加一
  - `gpsd` 模式：JSON 解析失败、字段类型错误、`ERROR` 报文、非法 `class` 时加一
- `fix_loss_count`
  - 上一次 `valid=true`，本次更新后 `valid=false` 时加一

这样 `HealthService` 不需要修改字段名，只需要接受其在不同来源下的统一统计语义。

---

## 10. 测试设计

### 10.1 单元测试

新增或调整以下测试：

- 串口模式回归
  - 继续保留现有 `test/device/gps_device_test.cc`
  - 验证串口模式 open/close、NMEA 解析、诊断统计不回退
- `gpsd` JSON 映射测试
  - `TPV` 坐标/速度/航向映射
  - `SKY` DOP/卫星统计映射
  - `TPV.mode` 到 `valid/fix_quality` 的映射
  - `TPV.time` 到 `utc_timestamp_ms` 的解析
  - 字段缺失时的增量更新策略
- 控制接口测试
  - `gpsd` 模式下三个控制接口统一返回 `NOT_SUPPORTED`

### 10.2 数据源层测试

为 `gpsd` 后端新增可注入的 socket 读写抽象或最小 mock，使以下行为可测试：

- 连接成功后发送 `WATCH`
- 收到 `TPV` / `SKY` 后更新缓存
- 收到断线或 EOF 后进入重连
- `close()` 可打断阻塞读

### 10.3 集成回归

保持现有系统集成测试对 GPS 的依赖方式不变，目标是验证：

- `HealthService` 和 `NavService` 不需要因数据源扩展而修改接口
- GPS 改为 `gpsd` 来源后，诊断输出 JSON 结构保持兼容

---

## 11. 改动边界

本次设计允许的改动：

- `GpsDevice` 设备层重构
- 新增 GPS 数据源抽象与 `gpsd` 后端实现
- `main.cc` 的 GPS 初始化逻辑
- `config/config.json` 配置结构扩展
- GPS 相关测试补充

本次设计明确不包含：

- 自动回退策略（如 `gpsd` 失败后自动切串口）
- 基于 `libgps` 的客户端实现
- `gpsd` 模式下的设备配置和复位命令透传
- 上层业务服务的接口改造
- 非 GPS 设备的统一数据源抽象

---

## 12. 实施建议

建议实现顺序：

1. 抽出 `GpsDevice` 的缓存与诊断更新逻辑
2. 将当前串口实现迁移到 `SerialGpsSource`
3. 引入 `gps` 新配置并在 `main.cc` 完成来源选择
4. 实现 `GpsdTcpSource` 的 TCP 读取、`WATCH` 握手和 JSON 映射
5. 补齐 `gpsd` 单元测试与串口回归测试

这样可以保证在每一步都保留可运行的串口版本，降低重构风险。

---

## 13. 最终结论

本次 GPS 扩展采用：

- **统一 `GpsDevice` 对外接口**
- **内部按配置选择 `SerialGpsSource` 或 `GpsdTcpSource`**
- **`gpsd` 按标准 TCP/JSON 协议接入**
- **`gpsd` 模式只支持读数据，不支持模组控制命令**

该方案在满足新需求的同时，最大限度复用现有业务层接口，并将串口与 `gpsd` 的复杂度隔离在设备层内部。
