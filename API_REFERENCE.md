# PV 清扫机器人 — 完整接口参考文档

> **目标平台**：RK3576 · aarch64-linux-gnu · C++17  
> **构建系统**：CMake 3.22 + 交叉编译 Toolchain  
> **核心依赖**：Boost.SML · spdlog · paho-mqtt-cpp · libmodbus · libserialport · libgpiod · nlohmann/json · sqlite3 · OpenSSL

---

## 目录

1. [架构总览](#1-架构总览)
2. [HAL 硬件抽象层](#2-hal-硬件抽象层)
3. [Driver 驱动层](#3-driver-驱动层)
4. [Protocol 协议层](#4-protocol-协议层)
5. [Device 设备层](#5-device-设备层)
6. [Middleware 中间件层](#6-middleware-中间件层)
7. [Service 服务层](#7-service-服务层)
8. [App 应用层](#8-app-应用层)
9. [错误码系统](#9-错误码系统)
10. [线程模型与调度](#10-线程模型与调度)
11. [配置文件结构](#11-配置文件结构)
12. [启动流程](#12-启动流程)

---

## 1. 架构总览

```
┌─────────────────────────────────────────────────────────────────┐
│                        App Layer（应用层）                        │
│  RobotFsm(Boost.SML) │ CleanTask │ FaultHandler │ WatchdogMgr  │
├─────────────────────────────────────────────────────────────────┤
│                      Service Layer（服务层）                      │
│ MotionService │ NavService │ CloudService │ FaultService        │
│ HealthService │ DiagnosticsCollector │ SchedulerService         │
│ ConfigService                                                   │
├─────────────────────────────────────────────────────────────────┤
│                    Middleware Layer（中间件层）                    │
│ EventBus │ ThreadExecutor │ Logger │ SafetyMonitor              │
│ NetworkManager │ MqttTransport │ LoRaWANTransport               │
│ DataCache │ OtaManager                                          │
├─────────────────────────────────────────────────────────────────┤
│                      Device Layer（设备层）                       │
│ WalkMotor(CAN) │ BrushMotor(Modbus) │ BMS(Modbus)              │
│ ImuDevice(UART) │ GpsDevice(UART) │ LimitSwitch(GPIO)          │
├─────────────────────────────────────────────────────────────────┤
│                     Protocol Layer（协议层）                      │
│ WalkMotorCanCodec │ NmeaParser │ ImuProtocol                    │
├─────────────────────────────────────────────────────────────────┤
│                      Driver Layer（驱动层）                       │
│ LinuxCanSocket │ LibSerialPort │ LibGpiodPin │ LibModbusMaster   │
├─────────────────────────────────────────────────────────────────┤
│                       HAL Layer（抽象接口）                       │
│    ICanBus │ ISerialPort │ IGpioPin │ IModbusMaster             │
└─────────────────────────────────────────────────────────────────┘
```

**设计原则**：
- 每层只依赖下层接口（HAL 抽象），测试时用 Mock 替换底层实现
- 所有共享状态通过 `std::mutex` 保护，对外暴露的状态读取均为缓存值（无 I/O 阻塞）
- 跨模块通信优先使用 `EventBus` 发布/订阅（松耦合），同步调用仅用于强时序依赖
- 配置文件（`config.json`）驱动全部运行时参数，零硬编码

---

## 2. HAL 硬件抽象层

命名空间：`robot::hal`  
头文件路径：`include/pv_cleaning_robot/hal/`

所有 HAL 接口均为纯虚类，驱动层提供具体实现，测试时用 Mock 替换。

---

### 2.1 `IModbusMaster` — Modbus RTU 主站

```cpp
class IModbusMaster {
public:
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;
    virtual int  read_registers(int slave_id, int addr, int count, uint16_t* out) = 0;
    virtual int  write_register(int slave_id, int addr, uint16_t val) = 0;
    virtual int  write_registers(int slave_id, int addr, int count, const uint16_t* vals) = 0;
    virtual void set_timeout_ms(int ms) = 0;
};
```

| 方法 | 返回值 | 说明 |
|------|--------|------|
| `open()` | `bool` | 打开并配置 RS485 串口，波特率在构造时指定 |
| `close()` | `void` | 关闭并释放所有资源 |
| `is_open()` | `bool` | 查询当前是否已打开 |
| `read_registers(slave_id, addr, count, out)` | `≥0`=成功读取数量；`-1`=失败 | 功能码 0x03（保持寄存器读取），`out` 由调用方保证大小 ≥ count |
| `write_register(slave_id, addr, val)` | `0`=成功；`-1`=失败 | 功能码 0x06（单寄存器写入）|
| `write_registers(slave_id, addr, count, vals)` | `0`=成功；`-1`=失败 | 功能码 0x10（多寄存器写入）|
| `set_timeout_ms(ms)` | `void` | 设置请求超时，默认 500ms |

**错误语义分层**：
- HAL 层（Level-1）：返回 `-1` 表示任意通信失败（超时/CRC/无响应）
- 设备层（Level-3）：解析响应寄存器中的业务错误码，映射为 `DeviceError`

---

### 2.2 `ISerialPort` — 串口抽象

```cpp
class ISerialPort {
public:
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;
    virtual int  write(const uint8_t* buf, size_t len) = 0;
    virtual int  read(uint8_t* buf, size_t max_len, int timeout_ms) = 0;
    virtual bool flush_input() = 0;
    virtual bool flush_output() = 0;
    virtual int  bytes_available() = 0;
};
```

| 方法 | 返回值 | 说明 |
|------|--------|------|
| `write(buf, len)` | 实际写入字节数；`-1`=错误 | 非阻塞写入，RS485 方向切换由硬件自动处理 |
| `read(buf, max_len, timeout_ms)` | 实际读取字节数；`0`=超时；`-1`=错误 | 阻塞最多 `timeout_ms` 毫秒 |
| `bytes_available()` | 待读字节数 | 可用于协议解析前判断是否有足够数据 |

---

### 2.3 `IGpioPin` — GPIO 数字输入

```cpp
class IGpioPin {
public:
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;
    virtual bool read_value() = 0;
    virtual void set_edge_callback(GpioEdge edge, std::function<void()> cb) = 0;
    virtual void start_monitoring() = 0;
    virtual void stop_monitoring() = 0;
};

enum class GpioEdge { RISING, FALLING, BOTH };
```

支持两种使用模式：
1. **主动轮询**：调用 `read_value()` 获取当前电平
2. **事件驱动**：`set_edge_callback()` + `start_monitoring()` 启动后台监听线程

⚠️ 边沿回调在专用监听线程中执行，**不能阻塞**，应仅置位原子标志或投递消息。

---

### 2.4 `ICanBus` — CAN 总线抽象

```cpp
struct CanFrame {
    uint32_t id;       // CAN ID（标准帧11位，扩展帧29位）
    uint8_t  len;      // DLC 0~8
    uint8_t  data[8];
    bool     is_ext;   // true=扩展帧
    bool     is_rtr;   // true=远程帧
};

class ICanBus {
public:
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;
    virtual bool send(const CanFrame& frame) = 0;
    virtual bool recv(CanFrame& frame, int timeout_ms) = 0;
    virtual bool set_filter(uint32_t id, uint32_t mask) = 0;
    virtual bool clear_filter() = 0;
};
```

| 方法 | 说明 |
|------|------|
| `send(frame)` | 发送一帧，成功返回 `true` |
| `recv(frame, timeout_ms)` | 阻塞接收，超时或错误返回 `false` |
| `set_filter(id, mask)` | 只接收满足 `frame.id & mask == id & mask` 的帧 |

---

## 3. Driver 驱动层

命名空间：`robot::driver`  
头文件路径：`include/pv_cleaning_robot/driver/`

| 类 | 接口实现 | 底层库 | 说明 |
|----|----------|--------|------|
| `LibModbusMaster` | `IModbusMaster` | libmodbus | RS485 Modbus RTU 主站 |
| `LibSerialPort` | `ISerialPort` | libserialport | UART 串口（IMU/GPS/LoRaWAN）|
| `LibGpiodPin` | `IGpioPin` | libgpiod v1.6 | GPIO 数字输入，支持边沿事件 |
| `LinuxCanSocket` | `ICanBus` | linux/can.h SocketCAN | 行走电机 CAN 通信 |

### 构造示例

```cpp
// Modbus RTU（串口路径 + 波特率）
auto modbus = std::make_shared<LibModbusMaster>("/dev/ttyS3", 115200);

// 串口
auto serial = std::make_shared<LibSerialPort>("/dev/ttyS1", 921600);

// GPIO（chip名 + 引脚号）
auto gpio = std::make_shared<LibGpiodPin>("gpiochip0", 10);

// CAN（接口名）
auto can = std::make_shared<LinuxCanSocket>("can0");
```

---

## 4. Protocol 协议层

命名空间：`robot::protocol`  
头文件路径：`include/pv_cleaning_robot/protocol/`

---

### 4.1 `WalkMotorCanCodec` — 行走电机 CAN 编解码

负责将速度命令编码为 CAN 帧，将状态 CAN 帧解码为 `WalkMotor::Status`。

```cpp
class WalkMotorCanCodec {
public:
    WalkMotorCanCodec(uint32_t cmd_id, uint32_t status_id);
    hal::CanFrame encode_speed(float rpm) const;
    hal::CanFrame encode_enable(bool en) const;
    hal::CanFrame encode_emergency_stop() const;
    bool decode_status(const hal::CanFrame& frame, WalkMotorStatus& out) const;
};
```

---

### 4.2 `NmeaParser` — GPS NMEA 0183 解析

流式解析 `$GPRMC` / `$GPGGA` 句子，输出 `GpsData`。

```cpp
struct GpsData {
    double   latitude;       // 纬度（度，正=北）
    double   longitude;      // 经度（度，正=东）
    float    altitude_m;     // 海拔（米）
    float    speed_knots;    // 速度（节）
    float    course_deg;     // 航向（度）
    uint8_t  satellites;     // 可见卫星数
    bool     fix_valid;      // 定位有效
    uint64_t utc_ms;         // UTC 时间（ms）
};

class NmeaParser {
public:
    bool feed_line(const std::string& line);  // 返回 true=成功解析
    const GpsData& latest() const;
};
```

---

### 4.3 `ImuProtocol` — WIT Motion IMU 协议

流式解析 WIT Motion 二进制帧（加速度/角速度/磁场/四元数），输出 `ImuDevice::ImuData`。

```cpp
class ImuProtocol {
public:
    // 喂入原始字节流，内部缓冲帧头/校验
    void feed(const uint8_t* buf, size_t len);
    bool has_new_frame() const;
    ImuDevice::ImuData pop_frame();
    // 生成配置命令帧
    std::vector<uint8_t> make_set_rate_cmd(int hz) const;
    std::vector<uint8_t> make_cali_gyro_cmd() const;
};
```

---

## 5. Device 设备层

命名空间：`robot::device`  
头文件路径：`include/pv_cleaning_robot/device/`

**通用设计模式**：
- 内部维护本地缓存（`Diagnostics` 结构），所有外部读取方法均返回缓存值（无 I/O，不阻塞）
- `update()` 方法由 `ThreadExecutor` 以固定周期调用，在线程内执行 I/O 并刷新缓存
- 缓存由 `mutable std::mutex mtx_` 保护，支持任意线程并发读取

---

### 5.1 `WalkMotor` — 行走电机（CAN）

```cpp
class WalkMotor {
public:
    struct Status {
        float   actual_rpm;
        float   current_a;
        float   bus_voltage_v;
        int64_t encoder_count;  // 累计编码器脉冲（里程计基础数据）
        bool    enabled;
        bool    fault;
        uint8_t fault_code;
    };
    struct Diagnostics : Status {
        float    target_rpm;
        uint32_t can_err_count;
        uint32_t overload_count;
        uint64_t total_run_ms;
    };

    DeviceError open();
    void        close();
    DeviceError enable();
    DeviceError disable();
    DeviceError set_speed(float rpm);       // 正=正转，负=反转
    DeviceError emergency_stop();           // 直写 CAN 帧，延迟 <1ms
    DeviceError clear_fault();
    Status      get_status() const;         // 线程安全，无 I/O
    Diagnostics get_diagnostics() const;    // 线程安全，无 I/O
    void        update();                   // ThreadExecutor 10ms 调用
};
```

**调度**：`walk_ctrl_thread`，`SCHED_FIFO` 优先级 80，周期 10ms

---

### 5.2 `BrushMotor` — 辊刷电机（Modbus RTU）

```cpp
class BrushMotor {
public:
    struct Status {
        int      actual_rpm;
        float    current_a;
        bool     running;
        bool     fault;
        uint16_t fault_code;
    };
    struct Diagnostics : Status {
        float    temperature_c;
        float    bus_voltage_v;
        int      target_rpm;
        uint32_t stall_count;
        uint32_t comm_error_count;
    };

    DeviceError start();
    DeviceError stop();
    DeviceError set_rpm(int rpm);
    DeviceError clear_fault();
    Status      get_status() const;
    Diagnostics get_diagnostics() const;
    void        update();   // ThreadExecutor 50ms 调用
};
```

**关键寄存器地址**（需按实际驱动器手册调整）：

| 符号 | 地址 | 方向 | 含义 |
|------|------|------|------|
| `REG_TARGET_RPM` | 0x1000 | 写 | 目标转速 |
| `REG_ENABLE` | 0x1001 | 写 | 使能（1=启动，0=停止）|
| `REG_CLR_FAULT` | 0x1002 | 写 | 清故障 |
| `REG_ACT_RPM` | 0x2000 | 读 | 实际转速 |
| `REG_STATUS` | 0x2004 | 读 | 状态字 |
| `REG_FAULT_CODE` | 0x2005 | 读 | 故障码 |

---

### 5.3 `BMS` — 电池管理系统（Modbus RTU）

```cpp
class BMS {
public:
    struct BatteryData {
        float    soc_pct;          // 电量（%）
        float    voltage_v;        // 总压（V）
        float    current_a;        // 电流（正=放电，负=充电）
        float    temperature_c;    // 最高温度（℃）
        uint16_t alarm_flags;      // 告警标志位
        bool     charging;
        bool     fully_charged;    // SOC≥阈值 且 |I|<0.5A 持续 3s
        bool     low_battery;      // SOC≤低电量阈值
        bool     valid;
    };
    struct Diagnostics : BatteryData {
        float    cell_voltage_max_v;
        float    cell_voltage_min_v;
        float    remaining_capacity_ah;
        float    full_capacity_ah;
        uint32_t cycle_count;
        uint16_t protect_flags;
    };

    BMS(shared_ptr<IModbusMaster> modbus, int slave_id,
        float full_soc = 95.0f, float low_soc = 15.0f);

    bool        read_all();            // 同步读取（初始化时调用）
    BatteryData get_data() const;      // 线程安全，无 I/O
    Diagnostics get_diagnostics() const;
    bool        is_fully_charged() const;
    bool        is_low_battery() const;
    bool        has_alarm() const;
    void        update();              // ThreadExecutor 500ms 调用
};
```

**告警位定义**：

| 常量 | 位 | 含义 |
|------|----|------|
| `ALARM_OVERTEMP` | bit0 | 过温 |
| `ALARM_UNDERVOLT` | bit1 | 欠压 |
| `ALARM_OVERVOLT` | bit2 | 过压 |
| `ALARM_OVERCURR` | bit3 | 过流 |

---

### 5.4 `ImuDevice` — 九轴 IMU（UART）

```cpp
class ImuDevice {
public:
    struct ImuData {
        float    accel[3];      // 加速度 m/s² [x,y,z]
        float    gyro[3];       // 角速度 rad/s [x,y,z]
        float    mag[3];        // 磁场 uT [x,y,z]
        float    quat[4];       // 四元数 [w,x,y,z]
        float    roll_deg;
        float    pitch_deg;
        float    yaw_deg;
        uint64_t timestamp_us;
        bool     valid;
    };

    bool        open();
    void        close();
    DeviceError set_output_rate(int hz);
    DeviceError calibrate_gyro();        // 需设备静止
    DeviceError save_config();           // 保存至 Flash
    DeviceError reset();                 // 软件复位
    ImuData     get_latest() const;      // 线程安全，无 I/O
};
```

内部维护读取线程（由 `open()` 启动），流式解析 WIT Motion 二进制帧。

---

### 5.5 `GpsDevice` — GPS（UART NMEA）

```cpp
class GpsDevice {
public:
    bool        open();
    void        close();
    DeviceError set_output_rate(int hz);
    DeviceError hot_restart();
    DeviceError cold_restart();
    GpsData     get_latest() const;         // 线程安全，无 I/O
    Diagnostics get_diagnostics() const;
};
```

内部维护读取线程，按行缓冲 NMEA 句子并解析。

---

### 5.6 `LimitSwitch` — 感应式限位开关（GPIO）

```cpp
class LimitSwitch {
public:
    using TriggerCallback = std::function<void(LimitSide)>;

    bool open();
    void close();
    void start_monitoring();
    void stop_monitoring();
    void set_trigger_callback(TriggerCallback cb);  // SafetyMonitor 注册
    bool is_triggered() const;                       // 原子读，无锁
    void clear_trigger();

    LimitSide side() const;  // FRONT / REAR
};
```

**触发路径**（端到端目标 ≤50ms）：
```
GPIO 下降沿 → libgpiod 事件 → TriggerCallback → SafetyMonitor::on_limit_hit()
           → WalkMotor::emergency_stop() → EventBus FaultEvent(P0)
```

---

## 6. Middleware 中间件层

命名空间：`robot::middleware`  
头文件路径：`include/pv_cleaning_robot/middleware/`

---

### 6.1 `EventBus` — 类型安全事件总线

```cpp
class EventBus {
public:
    template <typename EventT>
    int subscribe(std::function<void(const EventT&)> handler);

    void unsubscribe(int subscription_id);

    template <typename EventT>
    void publish(const EventT& event);
};
```

| 特性 | 说明 |
|------|------|
| 类型安全 | 基于 `std::type_index` + `std::any`，编译期检查事件类型 |
| 线程安全 | `subscribe`/`unsubscribe`/`publish` 均持锁 |
| 同步分发 | `publish()` 在调用方线程同步执行所有回调 |
| 回调隔离 | 读取回调列表后释放锁，避免回调内死锁 |

**使用示例**：
```cpp
EventBus bus;
int sub_id = bus.subscribe<FaultEvent>([](const FaultEvent& e) {
    // 处理故障事件
});
bus.publish(FaultEvent{FaultEvent::Level::P0, 0x0001, "过流"});
bus.unsubscribe(sub_id);
```

---

### 6.2 `ThreadExecutor` — 固定周期线程执行器

```cpp
class ThreadExecutor {
public:
    struct Config {
        std::string name;           // 线程名（最多15字符）
        int         period_ms{100};
        int         sched_policy{0};   // SCHED_OTHER=0, SCHED_FIFO=1, SCHED_RR=2
        int         sched_priority{0}; // RT 优先级
    };

    explicit ThreadExecutor(Config cfg);
    void add_runnable(std::shared_ptr<IRunnable> runnable);
    bool start();
    void stop();
    bool is_running() const;
};

class RunnableAdapter : public IRunnable {
public:
    explicit RunnableAdapter(std::function<void()> fn);
    void update() override;
};
```

**实现机制**：
- 每次循环计算实际执行时间，扣除后 `sleep_for` 剩余时间，维持周期精度
- 支持一个 Executor 携带多个 `IRunnable`（同一线程顺序调用）
- SCHED_FIFO/SCHED_RR 需 root 权限或 `CAP_SYS_NICE` 能力

---

### 6.3 `Logger` — 全局日志器

```cpp
class Logger {
public:
    struct Config {
        std::string log_dir{"logs"};
        std::string file_name{"robot"};
        size_t      max_file_size{10 * 1024 * 1024};  // 10MB
        int         max_files{5};                       // 保留5个滚动文件
        std::string level{"info"};                      // trace/debug/info/warn/error/critical
        bool        console_output{true};
    };

    static void init(const Config& cfg);
    static std::shared_ptr<spdlog::logger> get();
};
```

**特性**：
- `spdlog` 旋转文件 + 控制台双 sink
- 多次调用 `init()` 只有首次生效（单例）
- 全局通过 `Logger::get()` 获取 logger 实例
- 支持结构化日志（spdlog 格式化字符串）

**使用示例**：
```cpp
auto log = middleware::Logger::get();
log->info("[BMS] SOC={}% V={:.2f}V", soc, voltage);
log->error("[WalkMotor] 通信超时，错误计数={}", err_cnt);
```

---

### 6.4 `SafetyMonitor` — 安全监控器

```cpp
class SafetyMonitor {
public:
    SafetyMonitor(shared_ptr<WalkMotor>    walk,
                  shared_ptr<LimitSwitch>  front,
                  shared_ptr<LimitSwitch>  rear,
                  EventBus&               bus);
    void start();
    void stop();
};
```

**职责**：
- 注册两个限位开关的触发回调
- 触发后立即调用 `walk->emergency_stop()`（P0 安全路径，目标 ≤50ms）
- 同时向 `EventBus` 发布 `FaultEvent{P0, code, "限位触发"}`
- 独立于调度线程，基于 GPIO 事件中断驱动

---

### 6.5 `INetworkTransport` / `MqttTransport` / `LoRaWANTransport`

```cpp
class INetworkTransport {
public:
    using MessageCallback = std::function<void(const std::string& topic,
                                               const std::string& payload)>;
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool is_connected() const = 0;
    virtual bool publish(const std::string& topic, const std::string& payload) = 0;
    virtual bool subscribe(const std::string& topic, MessageCallback cb) = 0;
};
```

**`MqttTransport`** （paho-mqtt-cpp，QoS=1，持久会话）：

```cpp
struct Config {
    std::string broker_uri;           // "tcp://host:1883" 或 "ssl://host:8883"
    std::string client_id;
    std::string username;
    std::string password;
    bool        tls_enabled{false};
    std::string ca_cert_path;
    int         keep_alive_sec{30};
    int         connect_timeout_sec{10};
    int         qos{1};
};
```

**`LoRaWANTransport`**（AT 指令，OTAA 入网）：

```cpp
struct Config {
    std::string dev_eui;             // 16字符十六进制
    std::string app_key;             // 32字符十六进制
    int         port{2};             // LoRaWAN FPort
    int         min_interval_sec{30};// 占空比保护
    int         join_timeout_sec{60};// OTAA 入网超时
};
```

---

### 6.6 `NetworkManager` — 网络管理器

```cpp
class NetworkManager {
public:
    enum class Mode { MQTT_ONLY, LORAWAN_ONLY, DUAL_PARALLEL };

    NetworkManager(shared_ptr<INetworkTransport> mqtt,
                   shared_ptr<INetworkTransport> lorawan,
                   Mode mode);

    bool connect();
    void disconnect();
    bool publish(const string& topic, const string& payload);
    bool subscribe(const string& topic, INetworkTransport::MessageCallback cb);
    bool is_connected() const;
    Mode mode() const;
};
```

`DUAL_PARALLEL` 模式下，`publish()` 同时向 MQTT 和 LoRaWAN 投递，任一成功即返回 `true`。

---

### 6.7 `DataCache` — 离线遥测缓存（SQLite3 WAL）

```cpp
class DataCache {
public:
    struct Record {
        int64_t     id;
        std::string topic;
        std::string payload;
        uint64_t    ts_ms;
    };

    explicit DataCache(string db_path, size_t max_rows = 10000);

    bool open();
    void close();
    bool push(const string& topic, const string& payload, uint64_t ts_ms = 0);
    vector<Record> pop_batch(int max_count = 50);
    void confirm_sent(const vector<int64_t>& ids);
    size_t size();
};
```

**工作机制**：
1. 网络断开时，`CloudService` 调用 `push()` 将遥测写入 SQLite
2. 网络恢复后，`flush_cache()` 循环 `pop_batch()` → 上报 → `confirm_sent()`
3. 超过 `max_rows` 自动清理最旧记录，防止磁盘溢出
4. **WAL 模式**：写入不阻塞读取（SQLite3 `PRAGMA journal_mode=WAL`）

---

### 6.8 `OtaManager` — OTA 固件更新

```cpp
class OtaManager {
public:
    enum class State { IDLE, DOWNLOADING, VERIFYING, WRITING_FLAG, PENDING_REBOOT, FAILED };

    struct Progress {
        State       state{State::IDLE};
        uint32_t    bytes_written{0};
        uint32_t    total_bytes{0};
        std::string error_msg;
    };

    using ProgressCallback = std::function<void(const Progress&)>;

    OtaManager(string partition_b_path, string flag_path);

    bool     start(const string& firmware_data, const string& expected_md5);
    bool     apply_and_reboot();
    Progress get_progress() const;
    State    state() const;
    void     set_progress_callback(ProgressCallback cb);
};
```

**A/B 分区更新流程**：
```
cloud → 固件数据 → start() → 写入 B 分区 → MD5校验 → 写入分区标志 → apply_and_reboot()
```

---

## 7. Service 服务层

命名空间：`robot::service`  
头文件路径：`include/pv_cleaning_robot/service/`

---

### 7.1 `ConfigService` — 全局配置服务

```cpp
class ConfigService {
public:
    explicit ConfigService(string config_path);
    bool load();

    template <typename T>
    T get(const string& path, const T& default_val = T{}) const;

    json get_subtree(const string& path) const;
    bool is_loaded() const;
};
```

**路径语法**：`.` 分隔的嵌套键，例如：
```cpp
cfg.get<string>("network.mqtt.broker_uri", "tcp://localhost:1883")
cfg.get<float>("robot.clean_speed_rpm", 300.0f)
cfg.get<bool>("network.mqtt.tls_enabled", false)
```

异常安全：路径不存在或类型不匹配时返回 `default_val`，不抛出异常。

---

### 7.2 `MotionService` — 运动控制服务

```cpp
class MotionService : public IRunnable {
public:
    struct Config {
        float clean_speed_rpm{300.0f};
        float return_speed_rpm{500.0f};
        int   brush_rpm{1200};
    };

    bool start_cleaning();       // 使能行走 + 辊刷
    void stop_cleaning();        // 停辊刷，行走归零
    bool start_returning();      // 以返回速度反向行进
    void emergency_stop();       // 原地急停

    bool set_walk_speed(float rpm);
    bool is_moving() const;
    bool is_brush_running() const;

    void update() override;      // ThreadExecutor 10ms 调用
};
```

---

### 7.3 `NavService` — 导航与里程计服务

```cpp
class NavService : public IRunnable {
public:
    struct Pose {
        double distance_m{0.0};  // 累计位移（米）
        float  pitch_deg{0.0f};  // 纵坡角（IMU，正=上坡）
        float  roll_deg{0.0f};   // 横坡角
        float  speed_mps{0.0f};  // 当前速度（m/s）
        bool   valid{false};
    };

    void reset_odometry();
    Pose get_pose() const;
    bool is_slope_too_steep(float threshold_deg = 15.0f) const;
    void update() override;      // ThreadExecutor 10ms 调用
};
```

算法：编码器脉冲差分 × 车轮周长 = 增量位移；IMU 四元数提取 pitch/roll。

---

### 7.4 `CloudService` — 云端通信服务

```cpp
class CloudService : public IRunnable {
public:
    using RpcHandler = std::function<std::string(const std::string& params)>;

    struct Topics {
        string telemetry{"v1/devices/me/telemetry"};
        string attributes{"v1/devices/me/attributes"};
        string rpc_request{"v1/devices/me/rpc/request/+"};
        string rpc_response_prefix{"v1/devices/me/rpc/response/"};
    };

    bool publish_telemetry(const string& json_payload);
    bool publish_attributes(const string& json_payload);
    void register_rpc(const string& method, RpcHandler handler);
    void flush_cache();          // 手动触发积压数据上传
    void update() override;
};
```

**离线容错机制**：
- 网络在线：直接通过 `NetworkManager::publish()`
- 网络离线：写入 `DataCache`（SQLite）
- 网络恢复：`update()` 中调用 `flush_cache()` 批量回传

---

### 7.5 `FaultService` — 故障管理服务

```cpp
class FaultService {
public:
    struct FaultEvent {
        enum class Level { P0, P1, P2, P3 };
        Level       level;
        uint32_t    code;
        string      description;
        uint64_t    timestamp_ms;
    };

    void report(FaultEvent::Level level, uint32_t code, const string& description);
    bool has_active_fault(FaultEvent::Level min_level = FaultEvent::Level::P2) const;
    const FaultEvent& last_fault() const;
};
```

**故障等级定义**：

| 等级 | 处理策略 | 典型场景 |
|------|----------|----------|
| P0 | 立即停机，置 Fault 状态 | 限位触发、通信完全失联、看门狗超时 |
| P1 | 停止清扫，安全返回停机位 | 单次通信失败、设备过温预警 |
| P2 | 降速继续，告警上报 | BMS 告警、轻微通信抖动 |
| P3 | 仅记录日志 | 非关键统计事件 |

---

### 7.6 `HealthService` / `DiagnosticsCollector` — 上报服务

两者均实现 `IRunnable`，区别在于上报数据量：

| 服务 | 数据量 | 适用场景 | 推荐频率 |
|------|--------|----------|----------|
| `HealthService` | 精简（Status 结构） | 生产/低带宽 | 1 Hz |
| `DiagnosticsCollector` | 完整（Diagnostics 结构） | 开发/调试 | 可配置 |

通过 `config.json` 的 `diagnostics.mode` 字段在运行时切换：`"production"` 或 `"development"`。

---

### 7.7 `SchedulerService` — 清扫调度服务

```cpp
class SchedulerService {
public:
    struct TimeWindow {
        int hour{8};
        int minute{0};
    };

    void add_window(TimeWindow w);
    void set_on_task_start(TaskCallback cb);
    void set_on_task_end(TaskCallback cb);
    void tick();  // 由 1Hz 循环调用
};
```

支持配置多个时间窗口（如 `08:00`、`14:00`），到达时触发 FSM `EvStart` 事件。每个窗口默认持续 1 小时（`kWindowDurationSec = 3600`）。

---

## 8. App 应用层

命名空间：`robot::app`  
头文件路径：`include/pv_cleaning_robot/app/`

---

### 8.1 `RobotFsm` — 机器人有限状态机（Boost.SML）

#### 状态定义

| 状态 | 说明 |
|------|------|
| `StateInit` | 初始化中 |
| `StateIdle` | 待机 |
| `StateHoming` | 归零（回到轨道起点）|
| `StateCleanFwd` | 正向清扫 |
| `StateCleanReturn` | 反向清扫（往复中）|
| `StateReturning` | 返回停机位 |
| `StateCharging` | 充电中 |
| `StateFault` | 故障停机 |

#### 事件定义

| 事件 | 触发来源 | 说明 |
|------|----------|------|
| `EvInitDone` | `main()` | 系统初始化完成 |
| `EvStart` | `SchedulerService` / 远程 RPC | 开始清扫任务 |
| `EvStop` | 远程 RPC / 手动 | 停止清扫，返回停机位 |
| `EvHomeDone` | `NavService` | 归零完成 |
| `EvReachEnd` | `CleanTask` | 到达轨道端头 |
| `EvReachHome` | `CleanTask` | 到达轨道起点 |
| `EvChargeDone` | `BMS` | 电池充满 |
| `EvFaultP0` | `FaultHandler` | P0 级故障 |
| `EvFaultP1` | `FaultHandler` | P1 级故障 |
| `EvFaultReset` | 远程 RPC | 故障复位 |
| `EvLowBattery` | `main()` 轮询 | BMS 低电量 |

#### 状态转换矩阵

```
Init       --EvInitDone-->   Idle
Idle       --EvStart-->      Homing
Homing     --EvHomeDone-->   CleanFwd
CleanFwd   --EvReachEnd-->   CleanReturn
CleanReturn--EvReachHome-->  CleanFwd         (往复继续)
CleanFwd   --EvStop-->       Returning
CleanReturn--EvStop-->       Returning
Returning  --EvReachHome-->  Charging
Charging   --EvChargeDone--> Idle
CleanFwd   --EvLowBattery--> Returning
CleanReturn--EvLowBattery--> Returning
[任意清扫/返回状态] --EvFaultP0--> Fault
[任意清扫状态]      --EvFaultP1--> Returning
Fault      --EvFaultReset--> Idle
```

```cpp
class RobotFsm {
public:
    template <typename Event>
    void dispatch(Event e);         // 线程安全

    string current_state() const;   // 线程安全
};
```

---

### 8.2 `CleanTask` — 单次清扫任务

```cpp
class CleanTask {
public:
    struct Config {
        float track_length_m{1000.0f};  // 轨道全长（米）
        int   passes{1};                // 清扫趟数
    };

    bool start();
    void pause();
    void stop();

    bool reached_end() const;       // 当前方向是否到达端头
    bool all_passes_done() const;   // 是否完成全部趟数
    void switch_direction();        // 切换方向（下一趟）

    int  current_pass() const;
    bool is_paused() const;
};
```

---

### 8.3 `FaultHandler` — 故障响应器

```cpp
class FaultHandler {
public:
    using FsmDispatchFn = std::function<void(FaultService::FaultEvent)>;

    FaultHandler(shared_ptr<MotionService> motion,
                 EventBus&               bus,
                 FsmDispatchFn           dispatch_fn);

    void start_listening();  // 订阅 EventBus 的 FaultEvent
};
```

处理逻辑：
- P0 → `motion->emergency_stop()` + `dispatch(EvFaultP0{})`
- P1 → `motion->stop_cleaning()` + `dispatch(EvFaultP1{})`
- P2 → `motion->set_walk_speed(reduced_speed)` + 日志记录
- P3 → 仅日志记录

---

### 8.4 `WatchdogMgr` — 软件看门狗管理器

```cpp
class WatchdogMgr {
public:
    explicit WatchdogMgr(string hw_watchdog_path = "");
    bool start();
    void stop();

    int  register_thread(const string& name, int timeout_ms);
    void heartbeat(int ticket_id);
    void set_timeout_callback(TimeoutCallback cb);
};
```

**双层看门狗机制**：

| 层级 | 实现 | 超时动作 |
|------|------|----------|
| 软件看门狗 | `WatchdogMgr` 内部监控线程 | 调用 `on_timeout` 回调（触发 P0 故障）|
| 硬件看门狗 | `/dev/watchdog` 喂狗信号 | 系统强制重启（对抗内核级卡死）|

---

## 9. 错误码系统

命名空间：`robot::device`  
头文件：`include/pv_cleaning_robot/device/device_error.h`

```cpp
enum class DeviceError : int {
    OK             = 0,   // 操作成功
    COMM_TIMEOUT   = 1,   // 通信超时
    COMM_CRC       = 2,   // CRC 校验失败
    COMM_NO_RESP   = 3,   // 无响应
    EXEC_FAILED    = 4,   // 设备执行失败（状态寄存器报错）
    OVERCURRENT    = 5,   // 过流保护
    OVERVOLTAGE    = 6,   // 过压
    UNDERVOLTAGE   = 7,   // 欠压
    OVERTEMP       = 8,   // 过温
    STALL          = 9,   // 堵转
    NOT_OPEN       = 10,  // 设备未打开
    INVALID_PARAM  = 11,  // 参数越界
    NOT_SUPPORTED  = 12,  // 功能不支持
};

const char* device_error_str(DeviceError e);  // 转为可读字符串
```

**分层错误模型**：

```
HAL 层返回值  →  Device 层 DeviceError  →  Service 层 FaultEvent.Level  →  FSM 状态转换
  (-1 / ≥0)       (COMM_* / EXEC_*)        (P0 / P1 / P2 / P3)          (Fault / Returning)
```

---

## 10. 线程模型与调度

### 线程清单

| 线程名 | 调度策略 | 优先级 | 周期 | 职责 |
|--------|----------|--------|------|------|
| `walk_ctrl` | SCHED_FIFO | 80 | 10ms | WalkMotor CAN 帧发送/接收 + MotionService |
| `nav` | SCHED_OTHER | - | 10ms | NavService 里程计更新 |
| `bms` | SCHED_OTHER | - | 500ms | BMS Modbus 轮询 |
| `cloud` | SCHED_OTHER | - | 配置 | HealthService/Diagnostics + CloudService |
| `safety_monitor` | 事件驱动 | - | - | SafetyMonitor + LimitSwitch GPIO 中断 |
| `imu_read` | SCHED_OTHER | - | 流式 | ImuDevice UART 读取线程 |
| `gps_read` | SCHED_OTHER | - | 流式 | GpsDevice UART 读取线程 |
| `watchdog` | SCHED_OTHER | - | 100ms | WatchdogMgr 超时检测 |
| `main` | SCHED_OTHER | - | 100ms | BMS 低电量轮询 + FSM 事件分发 |

### 关键路径延迟目标

| 路径 | 目标延迟 |
|------|----------|
| 限位触发 → 急停指令 | ≤ 50ms |
| CAN 速度指令 → 电机执行 | ≤ 10ms（走线 FIFO 80 保证）|
| 故障事件 → FSM 状态切换 | ≤ 20ms |
| 云端 RPC 下行 → 本机执行 | ≤ 500ms（网络延迟主导）|

---

## 11. 配置文件结构

路径：`/opt/robot/config/config.json`（生产）或 `config/config.json`（开发）

```json
{
  "logging": {
    "log_dir": "/var/log/robot",
    "level": "info",
    "console": false
  },
  "can": {
    "interface": "can0",
    "walk_motor": { "cmd_id": 1537, "status_id": 1409 }
  },
  "serial": {
    "imu":   { "port": "/dev/ttyS1", "baudrate": 921600 },
    "gps":   { "port": "/dev/ttyS2", "baudrate": 9600 },
    "brush": { "port": "/dev/ttyS3", "baudrate": 115200, "slave_id": 1 },
    "bms":   { "port": "/dev/ttyS4", "baudrate": 9600, "slave_id": 2 }
  },
  "gpio": {
    "front_limit": { "chip": "gpiochip0", "line": 10 },
    "rear_limit":  { "chip": "gpiochip0", "line": 11 }
  },
  "network": {
    "transport_mode": "mqtt_only",
    "mqtt": {
      "broker_uri": "tcp://192.168.1.100:1883",
      "client_id": "pv_robot_001",
      "tls_enabled": false
    },
    "lorawan": {
      "port": "/dev/ttyS5",
      "baudrate": 9600,
      "dev_eui": "0000000000000001",
      "app_key": "00000000000000000000000000000001"
    }
  },
  "storage": {
    "cache_db": "/var/robot/telemetry.db"
  },
  "robot": {
    "track_length_m": 1000.0,
    "passes": 2,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 500.0,
    "brush_rpm": 1200,
    "battery_full_soc": 95.0,
    "battery_low_soc": 15.0
  },
  "diagnostics": {
    "mode": "production",
    "publish_interval_ms": 1000
  },
  "system": {
    "hw_watchdog": "/dev/watchdog"
  }
}
```

---

## 12. 启动流程

```
main()
  │
  ├─ 1. ConfigService::load()         ← /opt/robot/config/config.json
  ├─ 2. Logger::init()
  ├─ 3. signal(SIGINT/SIGTERM)
  ├─ 4. 构造 HAL/Driver/Device 对象   ← 依赖注入（shared_ptr）
  ├─ 5. 设备 open() / 初始化
  ├─ 6. SafetyMonitor::start()        ← 立即启动（安全第一）
  ├─ 7. NetworkManager::connect()
  ├─ 8. DataCache::open()
  ├─ 9. 构造 Service / App 对象
  ├─ 10. RobotFsm::dispatch(EvInitDone)
  ├─ 11. FaultHandler::start_listening()
  ├─ 12. WatchdogMgr::start()
  ├─ 13. ThreadExecutor × 4 start()
  │
  └─ 主循环（100ms）
       ├─ watchdog.heartbeat()
       ├─ BMS 低电量检测 → dispatch(EvLowBattery)
       └─ 等待 g_running = false (SIGINT/SIGTERM)

优雅关闭：
  停止所有 ThreadExecutor → SafetyMonitor → WatchdogMgr
  → emergency_stop() → close() → disconnect() → 退出
```

---

*文档生成日期：2026-03-16*  
*对应代码版本：pv_cleaning_robot @ RK3576 aarch64*
