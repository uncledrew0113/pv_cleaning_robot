# PV 清扫机器人 — 完整接口参考文档

> **目标平台**：RK3576 · aarch64-linux-gnu · C++17  
> **构建系统**：CMake 3.22 + 交叉编译 Toolchain  
> **核心依赖**：Boost.SML · spdlog · paho-mqtt-cpp · libmodbus · libserialport · libgpiod · nlohmann/json · OpenSSL

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
13. [模块调用关系图](#13-模块调用关系图)
14. [数据流路径](#14-数据流路径)
15. [实时性分析](#15-实时性分析)
16. [冗余与优化分析](#16-冗余与优化分析)

---

## 1. 架构总览

```
┌─────────────────────────────────────────────────────────────────┐
│                        App Layer（应用层）                        │
│  RobotFsm(Boost.SML) │ CleanTask │ FaultHandler │ WatchdogMgr  │
├─────────────────────────────────────────────────────────────────┤
│                      Service Layer（服务层）                      │
│ MotionService │ NavService │ CloudService │ FaultService        │
│ HealthService(HEALTH/DIAGNOSTICS双模式) │ SchedulerService     │
│ ConfigService                                                   │
├─────────────────────────────────────────────────────────────────┤
│                    Middleware Layer（中间件层）                    │
│ EventBus(PiMutex) │ ThreadExecutor │ Logger │ SafetyMonitor     │
│ NetworkManager │ MqttTransport │ LoRaWANTransport               │
│ DataCache │ OtaManager                                          │
├─────────────────────────────────────────────────────────────────┤
│                      Device Layer（设备层）                       │
│ WalkMotorGroup(CAN,4路) │ BrushMotor(Modbus)                    │
│ BMS(Modbus) │ ImuDevice(UART) │ GpsDevice(UART)                 │
│ LimitSwitch(GPIO)                                               │
├─────────────────────────────────────────────────────────────────┤
│                     Protocol Layer（协议层）                      │
│ WalkMotorCanCodec │ NmeaParser │ ImuProtocol                    │
│ BmsProtocol                                                       │
├─────────────────────────────────────────────────────────────────┤
│                      Driver Layer（驱动层）                       │
│ LinuxCanSocket │ LibSerialPort │ LibGpiodPin │ LibModbusMaster   │
├─────────────────────────────────────────────────────────────────┤
│                       HAL Layer（抽象接口）                       │
│    ICanBus │ ISerialPort │ IGpioPin │ IModbusMaster │ PiMutex   │
└─────────────────────────────────────────────────────────────────┘
```

**设计原则**：
- 每层只依赖下层接口（HAL 抽象），测试时用 Mock 替换底层实现
- 所有共享状态通过 `std::mutex` / `PiMutex` 保护，对外暴露的状态读取均为缓存值（无 I/O 阻塞）
- 跨模块通信优先使用 `EventBus` 发布/订阅（松耦合），同步调用仅用于强时序依赖
- 配置文件（`config.json`）驱动全部运行时参数，零硬编码
- `PiMutex`（PTHREAD_PRIO_INHERIT）用于可能被 RT 线程持有的互斥量，防止优先级反转

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

### 5.2 `BrushMotor` — 滚刷电机（Modbus RTU）

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
    void        update();   // bms_exec (SCHED_OTHER, 500ms) 周期调用
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
GPIO 下降沿 → libgpiod 事件 → TriggerCallback → SafetyMonitor::on_limit_trigger()
           → WalkMotorGroup::emergency_override(0.0f) → EventBus LimitTriggerEvent
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
| 类型安全 | 基于 `std::type_index`，编译期检查事件类型 |
| 线程安全 | `subscribe`/`unsubscribe`/`publish` 均持锁 |
| 同步分发 | `publish()` 在调用方线程同步执行所有回调 |
| 零堆分配 | 回调签名为 `void(const void*)`，`static_cast` 还原类型；`publish()` 热路径无堆分配，RT 线程安全 |
| 回调隔离 | 读取回调列表后释放锁，避免回调内死锁 |
| 优先级安全 | 互斥量使用 `PiMutex`（`PTHREAD_PRIO_INHERIT`），防止 SCHED_FIFO 95 GPIO 线程调用 `publish()` 时发生优先级反转 |

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
    SafetyMonitor(shared_ptr<WalkMotorGroup> walk_group,
                  shared_ptr<LimitSwitch>    front,
                  shared_ptr<LimitSwitch>    rear,
                  EventBus&                 bus);
    bool start();
    void stop();
    bool is_estop_active() const;
    void reset_estop();
};
```

**职责**：
- 注册两个限位开关的触发回调（GPIO 监控线程，SCHED_FIFO **95**）
- 触发后立即调用 `walk_group->emergency_override(0.0f)`（同时停全部4轮+锁定心跳，P0 安全路径，目标 ≤50ms）
- 同时向 `EventBus` 发布 `LimitTriggerEvent{side}`，通知 FSM 切换状态
- `monitor_loop()` 独立线程运行（SCHED_FIFO **94**，5ms 周期，CPU 4 亲和），定期检查 estop 状态
- `is_estop_active()` / `reset_estop()` 供上层（FaultHandler / RPC）查询和手动解除急停

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

### 6.7 `DataCache` — 离线遥测缓存（JSONL 文件持久化）

```cpp
class DataCache {
public:
    static constexpr size_t kDefaultMaxRows = 500;

    struct Record {
        int64_t     id;
        std::string topic;
        std::string payload;
        uint64_t    ts_ms;
    };

    explicit DataCache(std::string file_path, size_t max_rows = kDefaultMaxRows);

    bool open();   // 创建父目录；从已有 JSONL 文件加载未确认记录（断电恢复）
    void close();  // 无操作
    bool push(const std::string& topic, const std::string& payload, uint64_t ts_ms = 0);
    std::vector<Record> pop_batch(int max_count = 50);
    void confirm_sent(const std::vector<int64_t>& ids);
    size_t size();
};
```

**工作机制**：
1. 网络断开时，`CloudService` 调用 `push()` 将遥测追加到内存队列，并**原子重写** JSONL 文件（`.tmp` → `rename`）
2. 网络恢复后，`flush_cache()` 循环 `pop_batch()` → 上报 → `confirm_sent()`，每次 `confirm_sent()` 同样原子重写文件
3. 超过 `max_rows`（默认 500）时自动丢弃最旧记录，防止磁盘溢出（500 条 × ~300 B ≈ 150 KB）
4. **断电安全**：`open()` 重新加载文件中所有未确认记录，重启后自动续传；无 SQLite 依赖

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
        float clean_speed_rpm{300.0f};   ///< 清扫行进速度（RPM）
        float return_speed_rpm{500.0f};  ///< 返回速度（RPM）
        int   brush_rpm{1200};           ///< 滚刷转速
        float edge_reverse_rpm{0.0f};    ///< 边缘触发反转速度（0=原地停）
        bool  heading_pid_en{true};      ///< 是否使能航向 PID
        device::WalkMotorGroup::HeadingPidParams pid{};  ///< PID 参数
    };

    // 构造：需传入 WalkMotorGroup（4轮统一控制）+ ImuDevice（PID yaw 来源）
    MotionService(std::shared_ptr<device::WalkMotorGroup> group,
                  std::shared_ptr<device::BrushMotor>     brush,
                  std::shared_ptr<device::ImuDevice>      imu,
                  middleware::EventBus&                   bus,
                  Config                                  cfg);

    bool start_cleaning();       // 使能行走 + 滚刷；锁定当前 yaw 为 PID 目标
    void stop_cleaning();        // 停滚刷，行走归零，禁用 PID
    bool start_returning();      // 以返回速度反向行进（禁用 PID）
    void emergency_stop();       // group->emergency_override(0) + disable_all

    // 边缘触发接口（硬件限位或障碍物）
    void on_edge_triggered();    // group->emergency_override(edge_reverse_rpm) + 停刷
    void cancel_edge_override(); // group->clear_override()，恢复正常控制

    bool set_walk_speed(float rpm);
    bool is_moving() const;
    bool is_brush_running() const;
    bool is_edge_override_active() const;

    void update() override;      // ThreadExecutor 50ms 调用
                                 // 读 imu_->get_latest().yaw_deg，传入 group_->update(yaw)
};
```

**实现原理**：
- `start_cleaning()` 时以当前 IMU yaw 锁定航向目标，WalkMotorGroup 在每次 `update()` 内依据 yaw 误差差速修正左右轮
- `emergency_stop()` 调用 `group_->emergency_override(0)` 后设置 `override_active_`，阻止 `update()` 心跳帧覆盖停车指令，直到 `cancel_edge_override()` 解除
- `update()` 读取 ImuDevice 的缓存值（无 I/O），再将 `yaw_deg` 传入 WalkMotorGroup

**⚠️ 架构注意**：`SafetyMonitor` 直接操作 `WalkMotorGroup`（4轮统一急停），不调用 `MotionService::on_edge_triggered()`。
详见 [第 16 节冗余分析](#16-冗余与优化分析)。

---

### 7.3 `NavService` — 导航与里程计服务

```cpp
class NavService : public IRunnable {
public:
    struct Pose {
        double distance_m{0.0};        // 累计位移（米）
        float  pitch_deg{0.0f};        // 纵坡角（IMU，正=上坡）
        float  roll_deg{0.0f};         // 横坡角
        float  speed_mps{0.0f};        // 当前速度（m/s）
        bool   spin_free_detected{false}; // 悬空/空转检测触发标志
        bool   valid{false};
    };

    NavService(std::shared_ptr<device::WalkMotorGroup> walk_group,
               std::shared_ptr<device::ImuDevice> imu,
               std::shared_ptr<device::GpsDevice> gps,
               float wheel_circumference_m = 0.3f);

    void reset_odometry();         // 重置 distance_m / speed_mps / spin 计数器
    void clear_spin_detection();   // 仅清除悬空检测计数器，保留里程数据
    Pose get_pose() const;
    bool is_slope_too_steep(float threshold_deg = 15.0f) const;
    void update() override;        // ThreadExecutor 10ms 调用
};
```

**实现原理（航位推算）**：
```
// 四轮平均：(左前 + 右前 - 左后 - 右后) / 4（差速抵消原地转）
avg_rpm = (LT_rpm + RT_rpm - LB_rpm - RB_rpm) / 4.0f
// 坡度修正：水平分量 = 转速 × cos(pitch)
delta_m = (avg_rpm / 60.0f) × wheel_circ_m × std::cos(pitch_rad) × kDt
distance_m += delta_m
speed_mps = (avg_rpm / 60.0f) × wheel_circ_m
pitch_deg/roll_deg ← imu_->get_latest()（缓存，无 I/O）
```

**悬空检测算法（spin-free detection）**：
```
// 每 10ms 在 update() 中运行
if (|avg_rpm| > kSpinCmdThreshold && |commanded_rpm| < kSpinStopThreshold)
    spin_free_ticks_++
else
    spin_free_ticks_ = 0
if (spin_free_ticks_ >= kSpinMaxTicks)  // 连续 500ms
    pose_.spin_free_detected = true
```
检测到悬空后，`main.cc` 调用 `clear_spin_detection()` 清除标志位，同时上报 P0 故障并停机。  
`clear_spin_detection()` 不重置 `distance_m`，保留已行走里程。

**GPS 说明**：固定轨道场景下 GPS 精度（~5 m CEP）不足以替代限位开关；GPS 数据仅上报遥测，不参与里程计算。

**⚠️ 注意**：`kDt` 硬编码为 `0.010f`（10 ms），若 nav 线程周期调整需同步修改。

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
- 网络离线：写入 `DataCache`（JSONL 文件，断电可恢复）
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

### 7.6 `HealthService` — 遥测上报服务（双模式）

实现 `IRunnable`，通过构造时传入的 `Mode` 枚举切换上报详细程度（无需两个独立类）：

```cpp
class HealthService : public IRunnable {
public:
    enum class Mode { HEALTH, DIAGNOSTICS };
    HealthService(shared_ptr<WalkMotorGroup>, shared_ptr<BrushMotor>,
                  shared_ptr<BmsDevice>, shared_ptr<ImuDevice>,
                  shared_ptr<GpsDevice>, shared_ptr<CloudService>,
                  Mode        mode            = Mode::HEALTH,
                  std::string local_log_path  = "");  ///< 本地 JSONL 路径（可选）
};
```

| 模式 | 数据量 | JSON 字段 | 适用场景 | 推荐频率 |
|------|--------|-----------|----------|----------|
| `Mode::HEALTH` | 精简 | 4轮均值 rpm/current/fault + temp | 生产/低带宽 | 1 Hz |
| `Mode::DIAGNOSTICS` | 完整 | 每轮独立 lt/rt/lb/rb + ctrl_frames/ctrl_err | 开发/调试 | 可配置 |

通过 `config.json` 的 `diagnostics.mode` 字段选择模式：`"production"` → `Mode::HEALTH`，`"development"` → `Mode::DIAGNOSTICS`。

**本地 JSONL 落盘**（`local_log_path` 参数）：
- 非空时，`update()` 每帧额外将 JSON payload 以 `\n` 结尾追加写入本地文件
- 完全独立于 MQTT/LoRaWAN；离线测试环境直接 `cat` 或 `jq` 查看
- 构造时自动调用 `std::filesystem::create_directories()` 创建父目录
- 配置键：`diagnostics.local_path`，例如 `"/data/pv_robot/logs/telemetry.jsonl"`
- 生产环境将 `local_path` 置空（默认值）即可禁用，无性能损耗

> **注意**：`DiagnosticsCollector` 已废弃，其功能完全合并到 `HealthService::Mode::DIAGNOSTICS`。
> 头文件保留为迁移存根，新代码应直接使用 `HealthService`。

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

### 10.1 线程清单

| 线程名 | 调度策略 | 优先级 | CPU 核心 | 周期 | 职责 |
|--------|----------|--------|----------|------|------|
| `safety_monitor` | SCHED_FIFO | **94** | **4** | 5ms 轮询（备份） | monitor_loop 备份轮询 + 急停兜底 |
| `gpio_front` | SCHED_FIFO | **95** | **4** | 事件驱动 | front LimitSwitch GPIO edge 监控 |
| `gpio_rear` | SCHED_FIFO | **95** | **4** | 事件驱动 | rear LimitSwitch GPIO edge 监控 |
| `walk_ctrl` | SCHED_FIFO | **80** | **5** | **50ms** | MotionService::update() → WalkMotorGroup::update(yaw) |
| `group_recv` | SCHED_FIFO | **82** | **5** | 流式 | WalkMotorGroup CAN 接收线程（4轮组） |
| `imu_read` | SCHED_FIFO | **68** | **6** | 流式 | ImuDevice UART 读取线程 |
| `nav` | SCHED_FIFO | **65** | **6** | 10ms | NavService 里程计更新（WalkMotorGroup + IMU） |
| `watchdog_mon` | SCHED_FIFO | **50** | **7** | 200ms | WatchdogMgr 超时检测 + 喂硬件狗 |
| `bms` | SCHED_OTHER | - | **0-3** | 500ms | BMS Modbus 轮询 |
| `cloud` | SCHED_OTHER | - | **0-3** | 配置 | HealthService/Diagnostics + CloudService |
| `gps_read` | SCHED_OTHER | - | 任意 | 流式 | GpsDevice UART 读取线程 |
| `main` | SCHED_OTHER | - | 任意 | 100ms | BMS 低电量轮询 + FSM 事件分发 |

> **注**：`walk_recv`（单电机 WalkMotor CAN 接收）已随 WalkMotor 从生产路径完全移除（见 §16.7）。

### 10.2 RK3576 CPU 拓扑与核心分配

RK3576 为大小核异构架构：

| 核心编号 | 微架构 | 类型 | 频率 | 分配策略 |
|----------|--------|------|------|----------|
| CPU 0-3 | Cortex-A55 | LITTLE | ~1.8 GHz | 后台非 RT 任务（bms / cloud） |
| CPU 4-7 | Cortex-A76 | BIG | ~2.2 GHz | 实时任务（RT）|

**BIG 核心分工：**

| CPU | 专用任务 | 原因 |
|-----|----------|------|
| **CPU 4** | gpio_front/rear(95) + safety_monitor(94) | 边沿中断路径优先；轮询线程仅做兜底，不反向压制 GPIO |
| **CPU 5** | group_recv(82) + walk_ctrl(80) | 先接收后控制，降低 50ms 控制拍读取到陈旧 CAN 状态的概率 |
| **CPU 6** | imu_read(68) + nav(65) | 导航感知组：IMU 读取为 nav 的上游，同核数据局部性好 |
| **CPU 7** | watchdog_mon(50) | 管理任务，低优先级但需 SCHED_FIFO 以保证能抢占所有 SCHED_OTHER |

**内核级隔离建议（需修改 GRUB 启动参数）：**

```
isolcpus=4,5,6,7 nohz_full=4,5,6,7 rcu_nocbs=4,5,6,7
```

代码层已通过 `pthread_setaffinity_np` 将所有 RT 线程绑定到对应核心，但未添加内核级 `isolcpus`。若追求极致抖动，需在目标板修改 `/etc/default/grub` 后重启。

### 10.3 同步原语安全性（Priority Inheritance Mutex）

**优先级反转风险（已全部修复）：**

| 被保护数据 | 持锁高优先级线程 | 原锁类型 | 修复后 |
|------------|-----------------|----------|--------|
| `WalkMotorGroup::mtx_` | walk_ctrl(FIFO 80) 读状态 | `std::mutex` | `hal::PiMutex` ✅ |
| `ImuDevice::mtx_` | walk_ctrl(FIFO 80) 读姿态 | `std::mutex` | `hal::PiMutex` ✅ |
| `RobotFsm::mtx_` | GPIO 线程(FIFO 95) 触发 dispatch | `std::mutex` | `hal::PiMutex` ✅ |
| `FaultService::mtx_` | 安全路径调用 | `std::mutex` | `hal::PiMutex` ✅ |
| `WatchdogMgr::tickets_mtx_` | walk_ctrl(FIFO 80) 调用 heartbeat() | `std::mutex` | `hal::PiMutex` ✅ |
| `NavService::mtx_` | nav(FIFO 65) 写位姿 + cloud/SCHED_OTHER 读位姿 | `std::shared_mutex` | `hal::PiMutex` ✅ |

`hal::PiMutex` 底层使用 `PTHREAD_PRIO_INHERIT` + `PTHREAD_MUTEX_ROBUST`，确保高优先级线程不因低优先级持锁者而被阻塞超时。

**已正确使用的同步原语：**

- `EventBus::sub_mutex_`—`hal::PiMutex` ✅（RT 路径发布）
- `LibGpiodPin::cb_mutex_`—`hal::PiMutex` ✅
- `NavService::mtx_`—`hal::PiMutex` ✅（RT 写路径避免被低优先级读者反转）
- 所有 `running_` 标志—`std::atomic<bool>` ✅

**潜在死锁注意（当前不触发）：**

EventBus 持 `sub_mutex_` → subscriber 函数内 → `fsm->dispatch()` 持 `RobotFsm::mtx_`。  
若 FSM 状态转换函数内部再调用 `EventBus::publish()`，则构成死锁。  
当前 FSM 所有状态函数均不调用 `EventBus::publish()`，约束已满足。  
> **设计约束**：禁止在任何 `EventBus` subscriber 回调内调用 `EventBus::publish()`。

### 10.4 内存锁定（mlockall）

```cpp
// main.cc — 日志初始化后立即执行
if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    log->warn("[Main] mlockall 失败: {} (建议以 root 运行)", strerror(errno));
} else {
    log->info("[Main] 内存已全部锁定，缺页中断风险消除");
}
```

在 PREEMPT_RT 系统上，缺页中断（page fault）是 RT 延迟抖动的最大来源之一。`mlockall(MCL_CURRENT | MCL_FUTURE)` 将进程所有虚拟内存页（含未来分配）锁定在物理内存中，彻底消除运行期缺页。需以 `root` 或具有 `CAP_IPC_LOCK` 能力的用户运行。

### 10.5 关键路径延迟目标

| 路径 | 目标延迟 | 实测估算 |
|------|----------|----------|
| 限位触发 → 4轮同步急停 | ≤ 50ms | **~2ms** ✅（CPU 4 独占，GPIO 95 优先于轮询 94）|
| 限位触发 → FSM 确认停车 | ≤ 50ms | **<8ms** ✅（同步事件链）|
| CAN 速度指令 → 电机执行 | ≤ 10ms | <1ms ✅ |
| 故障事件 → FSM 状态切换 | ≤ 20ms | ~1ms ✅（PiMutex，无优先级反转）|
| 云端 RPC 下行 → 本机执行 | ≤ 500ms | 网络延迟主导 |

### 10.6 全项目同步语义统计（锁 / 原子量 / 条件变量）

统计范围：`include/pv_cleaning_robot/**` + `pv_cleaning_robot/**`（不含 `test/`、`doc/`）。

| 类别 | 是否使用 | 典型模块 | 结论 |
|------|----------|----------|------|
| `hal::PiMutex` | ✅ | EventBus, WalkMotorGroup, ImuDevice, RobotFsm, FaultService, WatchdogMgr, NavService | 用于 RT 交叉路径，已启用优先级继承 |
| `std::mutex` | ✅ | BrushMotor, BMS, CloudService, ConfigService, LinuxCanSocket(filter_mutex) | 用于非 RT 或局部短临界区 |
| `std::shared_mutex` | ✅ | LibSerialPort(port_rwlock), LibGpiodPin(io_mutex) | 用于 I/O 句柄读多写少场景 |
| `std::atomic<T>` | ✅ | 运行标志、连接状态、错误码、计数器 | 用于无锁状态标志和轻量统计 |
| `std::condition_variable` | ❌ | - | 当前项目未使用条件变量 |

> 结论：项目同步模型以“PiMutex + atomic + 局部 mutex/shared_mutex”为主，无条件变量依赖。

### 10.7 分层同步语义分析（HAL → Driver → Protocol → Device → Middleware → Service → App）

| 层级 | 关键模块 | 主要同步语义 | 选择是否最优 | 风险与建议 |
|------|----------|--------------|--------------|------------|
| HAL | `PiMutex` | `PTHREAD_PRIO_INHERIT` + `PTHREAD_MUTEX_ROBUST` | ✅ | 满足 RT 优先级继承；持锁区必须保持短小 |
| Driver | LinuxCanSocket / LibSerialPort / LibGpiodPin / LibModbusMaster | `atomic` + `mutex/shared_mutex/PiMutex` | ✅ | I/O 资源保护完整；`shared_mutex` 不带 PI，仅用于非 RT 句柄管理 |
| Protocol | CAN codec / 序列化编解码 | 无共享状态（纯函数/栈对象） | ✅ | 无锁设计正确 |
| Device | WalkMotorGroup / ImuDevice / LimitSwitch / GPS/BMS/Brush | PiMutex + mutex + atomic | ✅（主体） | `WalkMotor`（遗留）仍是 `std::mutex`，建议仅测试保留不进入生产路径 |
| Middleware | EventBus / ThreadExecutor / SafetyMonitor / Network | PiMutex + atomic | ✅ | **约束必须遵守**：subscriber 回调内禁止再 publish，防重入死锁 |
| Service | Motion/Nav/Cloud/Fault/Config/Health | PiMutex 或 mutex | ✅ | Nav 已改 PiMutex；Cloud RPC 互斥仅在 SCHED_OTHER，可接受 |
| App | RobotFsm / WatchdogMgr / CleanTask | PiMutex + atomic | ✅ | Watchdog 票据锁已 PI，RT 心跳路径安全 |

**跨层调用链重点检查：**

1. GPIO(FIFO95) → LimitSwitch → SafetyMonitor → EventBus → RobotFsm：
    PiMutex 链路完整；`safety_monitor` 轮询线程已降为 FIFO94，确保不压制 GPIO 事件线程。

2. group_recv(FIFO82) → WalkMotorGroup::mtx_ → walk_ctrl(FIFO80)：
    接收线程优先于控制线程，保证控制拍更大概率读到最新 CAN 状态。

3. imu_read(FIFO68) → ImuDevice::mtx_ → nav(FIFO65)：
    上游采集优先级高于下游计算，符合“先采样后控制”实时链路。

4. nav(FIFO65) ↔ cloud(SCHED_OTHER) 读写位姿：
    已由 `std::shared_mutex` 迁移到 PiMutex，避免低优先级读者导致写者（RT）等待的不确定性。

**当前确认的死锁风险：**

- EventBus 回调重入 publish（逻辑重入风险）
- 当前代码未触发，但属于架构级约束，需代码评审持续检查。

**需要同步但目前未缺失的点：**

- 关键共享状态（电机组状态、IMU 缓存、FSM 状态、故障列表、看门狗票据）均已加锁。
- 线程存活/连接状态均使用原子量；未发现“明确应加锁但未加锁”的生产路径数据竞争点。

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
    "walk_motor": { "motor_id": 1, "comm_timeout_ms": 200 }
  },
  "serial": {
    "imu":   { "port": "/dev/ttyS1", "baudrate": 921600 },
    "gps":   { "port": "/dev/ttyS2", "baudrate": 9600 },
    "brush": { "port": "/dev/ttyS3", "baudrate": 9600, "slave_id": 1 },
    "bms":   { "port": "/dev/ttyS4", "baudrate": 9600 }
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
    "cache_path": "/data/pv_robot/telemetry_cache.jsonl"
  },
  "robot": {
    "track_length_m": 1000.0,
    "passes": 1,
    "clean_speed_rpm": 300.0,
    "return_speed_rpm": 500.0,
    "brush_rpm": 1200,
    "edge_reverse_rpm": 0.0,
    "heading_pid_en": true,
    "battery_full_soc": 95.0,
    "battery_low_soc": 15.0,
    "wheel_circ_m": 0.3
  },
  "diagnostics": {
    "mode": "development",
    "publish_interval_ms": 1000
  },
  "system": {
    "hw_watchdog": "/dev/watchdog"
  },
  "device": {
    "fw_version": "1.0.0",
    "hw_version": "1.0"
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

*文档生成日期：2026-03-25*  
*对应代码版本：pv_cleaning_robot @ RK3576 aarch64*

---

## 13. 模块调用关系图

### 13.1 完整调用依赖矩阵

```
调用方 ──────────────────► 被调用方                        调用方式
═══════════════════════════════════════════════════════════════════════
main.cc          ──► ConfigService                         直接调用
main.cc          ──► Logger::init()                        直接调用
main.cc          ──► LinuxCanSocket/LibSerialPort/...      构造+open()
main.cc          ──► WalkMotor.open()                      设备初始化
main.cc          ──► WalkMotorGroup.open()                 设备初始化（新增）
main.cc          ──► SafetyMonitor.start()                 启动安全监控
main.cc          ──► MotionService(Config)                 构造，传 WalkMotorGroup+ImuDevice
main.cc          ──► NavService(WalkMotor,IMU,GPS)         构造
main.cc          ──► CloudService(NetworkManager,DataCache)构造
main.cc          ──► RobotFsm.dispatch(EvInitDone)         初始化
main.cc          ──► FaultHandler.start_listening()        订阅 FaultEvent
main.cc          ──► WatchdogMgr.start()                   启动看门狗
main.cc          ──► ThreadExecutor × 4 .start()           线程启动
main.cc          ──► EventBus.subscribe<LimitTriggerEvent> 订阅限位事件→FSM

───────────────────────────────────────────────────────────────────────
RobotFsm         ──► MotionService.start_cleaning()        dispatch(EvStart) 内
RobotFsm         ──► MotionService.start_returning()       dispatch(EvStop/EvReachEnd) 内
RobotFsm         ──► MotionService.stop_cleaning()         dispatch(EvReachHome→Charging) 内
RobotFsm         ──► MotionService.emergency_stop()        dispatch(EvFaultP0) 内
RobotFsm         [内部] Boost.SML 状态机驱动              dispatch() 调用均加 mtx_ 保护

───────────────────────────────────────────────────────────────────────
FaultHandler     ──► MotionService.emergency_stop()        P0 故障时
FaultHandler     ──► MotionService.stop_cleaning()         P1 故障时
FaultHandler     ──► MotionService.start_returning()       P1 故障时
FaultHandler     ──► EventBus.subscribe<FaultEvent>        订阅注册

───────────────────────────────────────────────────────────────────────
CleanTask        ──► MotionService.start_cleaning()        start()/switch_direction()
CleanTask        ──► MotionService.stop_cleaning()         stop()/pause()
CleanTask        ──► MotionService.set_walk_speed()        switch_direction 反向
CleanTask        ──► NavService.get_pose()                 里程计终点检测
                 ⚠️ CleanTask 在 main.cc 中被创建但从未被 FSM 使用（死代码）

───────────────────────────────────────────────────────────────────────
MotionService    ──► WalkMotorGroup.enable_all()           start_cleaning()
MotionService    ──► WalkMotorGroup.set_speeds(×4)         start/set_walk_speed
MotionService    ──► WalkMotorGroup.enable_heading_control start_cleaning()
MotionService    ──► WalkMotorGroup.set_target_heading()   start_cleaning()
MotionService    ──► WalkMotorGroup.disable_all()          stop_cleaning()/emergency_stop()
MotionService    ──► WalkMotorGroup.emergency_override()   emergency_stop()+on_edge_triggered()
MotionService    ──► WalkMotorGroup.clear_override()       cancel_edge_override()
MotionService    ──► WalkMotorGroup.update(yaw_deg)        update() 50ms 周期
MotionService    ──► BrushMotor.start/stop/set_rpm         清扫控制
                 ─── BrushMotor.update() 已移至 bms_exec（SCHED_OTHER, 500ms）
MotionService    ──► ImuDevice.get_latest().yaw_deg        update() 读缓存

───────────────────────────────────────────────────────────────────────
WalkMotorGroup   ──► WalkMotorCanCodec（×4 实例）          编码 CAN 帧
WalkMotorGroup   ──► ICanBus.send()                        发控制帧/查询帧
WalkMotorGroup   ──► ICanBus.recv()                        接收状态帧（后台线程）
WalkMotorGroup   [内部] PID 差速: calc_pid_correction(),   update(yaw_deg)
                 [内部] 温度轮询: encode_query(TEMP),      每 1000ms
                 [内部] override: atomic<bool>,            emergency_override()

───────────────────────────────────────────────────────────────────────
NavService       ──► WalkMotorGroup.get_group_status()     读4轮状态缓存（无 I/O）
NavService       ──► ImuDevice.get_latest()                读缓存（无 I/O）
NavService       ──► GpsDevice.get_latest()                【未使用！GPS 字段未融合进 Pose】

───────────────────────────────────────────────────────────────────────
SafetyMonitor    ──► LimitSwitch(front/rear).start_monitoring()
SafetyMonitor    ──► LimitSwitch.set_trigger_callback()    注册边缘回调
SafetyMonitor    ──► WalkMotorGroup.emergency_override(0.0f)   REAR 触发：1帧停4轮+锁定心跳
SafetyMonitor    ──► EventBus.publish(LimitTriggerEvent)   通知 main.cc 订阅者
SafetyMonitor    [内部] monitor_loop (SCHED_FIFO 94, 5ms) 备份轮询

───────────────────────────────────────────────────────────────────────
HealthService    ──► WalkMotorGroup.get_group_status()     读4轮状态缓存（HEALTH模式均值）
HealthService    ──► WalkMotorGroup.get_group_diagnostics() 读完整诊断缓存（DIAGNOSTICS模式）
HealthService    ──► BrushMotor.get_status()               读缓存
HealthService    ──► BMS.get_data()                        读缓存
HealthService    ──► ImuDevice.get_latest()                读缓存
HealthService    ──► GpsDevice.get_latest()                读缓存
HealthService    ──► CloudService.publish_telemetry()      上报
                     Mode::HEALTH → 4轮均值 rpm/current/fault/temp
                     Mode::DIAGNOSTICS → 每轮独立 lt/rt/lb/rb + ctrl 统计

───────────────────────────────────────────────────────────────────────
CloudService     ──► NetworkManager.publish()              在线上报
CloudService     ──► DataCache.push()                      离线缓存
CloudService     ──► DataCache.pop_batch()/confirm_sent()  恢复上报（flush_cache()）
CloudService     ──► NetworkManager.subscribe(rpc_request) RPC 下行

FaultService     ──► EventBus.publish(FaultEvent)          report() 内同步发布

WatchdogMgr      ──► /dev/watchdog（ioctl WDIOC_SETTIMEOUT/write "1"）
                 ── 已注册 walk_ctrl/nav/bms/cloud 线程票据并周期 heartbeat()
```

---

### 13.2 事件总线事件流

```
事件类型                  发布者                   订阅者（处理）
════════════════════════════════════════════════════════════════════
LimitTriggerEvent(FRONT)  SafetyMonitor            main.cc → fsm->dispatch(EvReachEnd)
LimitTriggerEvent(REAR)   SafetyMonitor            main.cc → fsm->dispatch(EvReachHome)
FaultEvent(P0)            FaultService::report()   FaultHandler → emergency_stop() + EvFaultP0
FaultEvent(P1)            FaultService::report()   FaultHandler → stop_cleaning() + EvFaultP1
FaultEvent(P2/P3)         FaultService::report()   FaultHandler → 仅日志
```

---

## 14. 数据流路径

### 14.1 控制指令流（下行）

```
用户/云端 RPC
    │ CloudService.on_rpc_message() 解析 method/params
    │ rpc_handlers_["start"]() 用户注册回调
    ▼
FSM 事件分发 dispatch(EvStart)
    │ RobotFsm::dispatch<EvStart> 持 mtx_ 锁
    ▼
MotionService::start_cleaning()
    │ group_->set_heading_pid_params(cfg_.pid)
    │ group_->set_target_heading(imu_->get_latest().yaw_deg)  ← 当前 yaw 锁定
    │ group_->enable_heading_control(true)
    │ group_->enable_all()        → CAN 0x105 (ENABLE × 4)
    │ group_->set_speeds(spd×4)   → CAN 0x32 (1帧，4轮)
    │ brush_->set_rpm() + start() → Modbus 写寄存器
    ▼
WalkMotorGroup::update(yaw_deg)  ← 每 50ms，walk_ctrl 线程 SCHED_FIFO 80
    │ calc_pid_correction(yaw, dt_s) → correction（RPM 差速）
    │ encode_group_speed(id_base, lt+corr, rt-corr, lb+corr, rb-corr)
    │ ICanBus::send(ctrl_frame)    → SocketCAN → 总线 → 电机
    │ [每 1000ms] ICanBus::send(query_temp_frame) → 0x107 温度查询
    ▼
M1502E_111 电机组（CAN 0x32，4轮同步）
```

### 14.2 安全紧急停车流（关键路径，≤50ms）

```
物理边缘 / 限位开关触发
    │ libgpiod GPIO 中断  ( 硬件 → poll() 唤醒 )
    │ [GPIO 监控线程, SCHED_FIFO 95]
    ▼
LimitSwitch::on_edge()
    │ triggered_.store(true)               ← 原子置位
    │ PiMutex 加锁（无竞争 ~0μs）
    │ callback_(side)                      ← SafetyMonitor 注册的回调
    ▼
SafetyMonitor::on_limit_trigger(side)
    │ [REAR] estop_active_.store(true)
    │ [REAR] walk_group_->emergency_override(0.0f)  → CAN 0x32 停4轮同步 ✅ <1ms
    │                                                 + override_active_=true 锁定心跳帧
    │ [ALL] EventBus.publish(LimitTriggerEvent{side})  ← 同步，PiMutex
    ▼
main.cc EventBus 订阅回调（同一线程栈执行）
    │ [REAR] fsm->dispatch(EvReachHome{}) ← 持 std::mutex（非 PiMutex，见§16注意）
    ▼
RobotFsm::dispatch<EvReachHome>
    │ was_clean_return ? start_cleaning() : stop_cleaning()
    │ [通常] motion_->stop_cleaning()
    ▼
MotionService::stop_cleaning()
    │ group_->enable_heading_control(false)
    │ group_->set_speed_uniform(0.0f)      → CAN 0x32（4轮同步停止）✅
    │ group_->disable_all()               → CAN 0x105 DISABLE（4轮）
    │                                       全程在同一 RT 线程中同步执行
    │                                       （emergency_override 已幅等，检查 override_active_ 跳过心跳）
    ▼
全部 4 台电机停止  总估算延迟：~2ms ✅（emergency_override 已在首帧完成）

备份路径（若 GPIO 回调丢失）：
    SafetyMonitor::monitor_loop (SCHED_FIFO 94, 5ms轮询)
  → triggered_ 为 true → 再次调用 on_limit_trigger()
  最坏延迟：5ms 轮询 + 上述路径 ≈ 20ms ✅
```

### 14.3 传感器状态上报流（遥测）

```
ImuDevice 后台线程（流式 UART）
    │ LibSerialPort::read() → ImuProtocol::feed() → ImuData 帧缓存
    │ 更新 diag_（加 mtx_ 锁）
    ▼ (异步，后台持续运行)

WalkMotorGroup 后台接收线程（group_recv）
    │ ICanBus::recv() → 解析 0x97~0x9E 状态帧
    │ 更新 diag_[i].speed_rpm/torque_a/position_deg/fault （加 mtx_ 锁）
    │ [0x107 温度响应] 提取 temperature_deg_（合理性校验 -40~150℃）
    ▼ (异步，100Hz 主动上报)

cloud 线程（SCHED_OTHER, 1000ms）
    │ HealthService::update()
    │     walk->get_group_status()  ← 读 WalkMotorGroup 缓存（无 I/O）·4轮均值
    │     bms->get_data()           ← 读 BMS 缓存
    │     imu->get_latest()         ← 读 IMU 缓存
    │     gps->get_latest()         ← 读 GPS 缓存
    │     build_payload() → JSON（键预分配，无堆分配）
    │     cloud_->publish_telemetry(json)
    ▼
CloudService::publish_telemetry()
    │ [在线] NetworkManager::publish(topic, payload)
    │         → MqttTransport::publish() → paho-mqtt-cpp → MQTT Broker
    │ [离线] DataCache::push(topic, payload) → SQLite3 WAL
    ▼
[恢复在线] CloudService::update() → flush_cache() → pop_batch(50)
    → publish() 批量回传 → confirm_sent()
```

### 14.4 里程计与导航流

```
WalkMotorGroup 后台接收线程（group_recv）
    │ ICanBus::recv() → 解析 0x97~0x9E → 更新 diag_[i].speed_rpm（4轮）
    ▼

nav 线程（SCHED_FIFO 65, 10ms, CPU 6）
    │ NavService::update()
    │     auto gs = walk_->get_group_status()   ← WalkMotorGroup 缓存（无 I/O）
    │     float avg_rpm = 0.0f;
    │     for (auto& w : gs.wheel) avg_rpm += w.speed_rpm;
    │     avg_rpm /= WalkMotorGroup::kWheelCount  ← 4轮平均速度
    │     imu_->get_latest().pitch/roll   ← IMU 缓存
    │     delta_m = (avg_rpm/60) * circ * 0.01s
    │     pose_.distance_m += delta_m    ← 加 PiMutex 互斥
    ▼

CleanTask::reached_end()              ← 暂未被 FSM 使用（见 §16.3）
    │ nav_->get_pose()                 ← PiMutex 读缓存
    │ 比较 distance_m ≥ track_length_m
```

### 14.5 故障处理流

```
任意模块检测到故障
    │ FaultService::report(level, code, description)
    │ 更新 last_fault_（加 mtx_ 锁）
    │ EventBus.publish(FaultEvent{level, code, ...})  ← 同步
    ▼
FaultHandler::on_fault(evt)  (在 publish() 的调用线程中执行)
    │ [P0] motion_->emergency_stop() → group_->emergency_override(0)
    │       + dispatch_fn_(evt) → fsm->dispatch(EvFaultP0)
    │ [P1] motion_->stop_cleaning() + start_returning()
    │       + dispatch_fn_(evt) → fsm->dispatch(EvFaultP1)
    │ [P2/P3] 仅日志
```

---

## 15. 实时性分析

### 15.1 安全关键路径（目标 ≤50ms）

**路径 A：GPIO 边缘 → 4轮同步急停（首选路径，emergency_override）**

| 步骤 | 组件 | 延迟估算 | 机制 |
|------|------|----------|------|
| 硬件中断 → poll() 唤醒 | 内核 + libgpiod | ~0.1ms | GPIO IRQ → eventfd |
| SCHED_FIFO 95 线程抢占 | OS 调度 | ~0ms | RT 最高优先级，立即执行 |
| `on_edge()` → `on_limit_trigger()` | LimitSwitch → SafetyMonitor | ~0.01ms | 直接函数调用 + PiMutex |
| `walk_group_->emergency_override(0.0f)` → CAN 发帧 | WalkMotorGroup → ICanBus | < 1ms | SocketCAN 直写，4轮同步，无队列 |
| override_active_ = true，锁定心跳帧 | WalkMotorGroup | ~0ms | 原子标志，阻止50ms周期重发 |
| **4轮全部急停路径总延迟** | | **< 2ms ✅** | |

**路径 B：FSM 链冗余路径（随后执行，确认停止状态）**

| 步骤 | 组件 | 延迟估算 |
|------|------|----------|
| A 路径（emergency_override 已完成，4轮已停） | | ~2ms |
| EventBus.publish(LimitTriggerEvent) | PiMutex 同步回调 | ~0.1ms |
| main.cc 订阅者执行 | 同步，同一线程 | ~0.1ms |
| fsm->dispatch(EvReachHome) → std::mutex | 若无竞争：~0；有竞争（最坏）：~5ms |
| stop_cleaning() → group_->disable_all() | override_active_ 幂等，无重复CAN帧 | ~0.1ms |
| **FSM 链完成总延迟** | | **< 8ms ✅** |

**路径 C：备份轮询路径（GPIO 边缘丢失时）**

| 步骤 | 延迟估算 |
|------|----------|
| monitor_loop 5ms 轮询唤醒 | 0~5ms |
| + B 路径 | ~10ms |
| **备份路径最坏延迟** | **< 15ms ✅** |

**结论**：三条路径均满足 ≤50ms 目标，系统具备良好冗余。

---

### 15.2 潜在实时性风险点

#### ✅ 已修复：FSM 互斥量优先级反转

```
SafetyMonitor 回调链（GPIO SCHED_FIFO 95）经 EventBus 同步到达 fsm->dispatch()
fsm->dispatch() 互斥量已迁移为 hal::PiMutex（带优先级继承）。

RT 路径不会因低优先级线程持锁而出现非受控阻塞。
```

#### ✅ 已修复：关键线程看门狗覆盖

```
WatchdogMgr 已注册 walk_ctrl/nav/bms/cloud 线程票据，
每个执行线程 update() 周期结束后 heartbeat()。
```

#### ✅ 已正确处理：EventBus 使用 PiMutex

```
EventBus::publish() 被 SafetyMonitor/GPIO RT 路径直接调用，
mtx_ 使用 PiMutex (PTHREAD_PRIO_INHERIT) 避免持锁的低优先级线程阻塞 RT 路径。
```

#### ✅ 已正确处理：LimitSwitch 回调路径极短

```
on_edge() 中只有：atomic::store() + PiMutex + 函数调用
无内存分配、无日志、无 I/O，满足 ISR-like 最短路径要求。
```

#### ✅ 已正确处理：WalkMotorGroup override 防止心跳覆盖

```
emergency_override() 设置 override_active_ = true
update() 首先检测此标志，若为 true 则跳过心跳帧发送，
防止 50ms 周期心跳在紧急停车后重新驱动电机。
```

---

### 15.3 各模块实时性汇总

| 模块/路径 | 目标 | 实现方案 | 评估 |
|-----------|------|----------|------|
| GPIO → 4轮同步急停 | ≤50ms | SCHED_FIFO 95, CPU 4 + emergency_override | ✅ **~2ms** |
| GPIO → FSM确认停车 | ≤50ms | 同步事件链 + group::disable_all（幂等）| ✅ **<8ms** |
| 行走控制心跳 | 50ms | SCHED_FIFO 80, CPU 5, ThreadExecutor | ✅ |
| group_recv CAN 接收 | <5ms（消费延迟）| SCHED_FIFO 82, CPU 5 | ✅（优先于 walk_ctrl）|
| imu_read UART 接收 | <5ms（消费延迟）| SCHED_FIFO 68, CPU 6 | ✅（与 nav 同核）|
| 里程计更新 | 10ms | SCHED_FIFO 65, CPU 6 | ✅（升级为强 RT）|
| BMS 采集 | 500ms | SCHED_OTHER, CPU 0-3（LITTLE 核）| ✅ |
| 云端上报 | 1000ms | SCHED_OTHER, CPU 0-3（LITTLE 核）| ✅ |
| FSM 状态切换 | ≤20ms | PiMutex（PTHREAD_PRIO_INHERIT）| ✅（优先级反转已消除）|
| 温度轮询 | 1000ms | 在 update() 内计时发帧 | ✅ |
| 通信超时触发自停 | 200ms | 电机硬件看门狗 | ✅ 兜底 |
| 看门狗超时检测 | 200ms | SCHED_FIFO 50, CPU 7 | ✅（可抢占所有 SCHED_OTHER）|

---

## 16. 冗余与优化分析

### 16.1 ✅ 已修复：SafetyMonitor 已改用 WalkMotorGroup::emergency_override()

**原问题**：SafetyMonitor 只调用 `WalkMotor.disable()`，仅停单轮；其余 3 轮依赖 EventBus → FSM 延迟链。

**已修复**：SafetyMonitor 构造时接受 `WalkMotorGroup`，REAR 触发时直接调用：

```cpp
// safety_monitor.cc
walk_group_->emergency_override(0.0f);  // 1帧停4轮 + override_active_=true
```

效果：4轮在 <2ms 内全部停止，`override_active_` 锁定 50ms 心跳帧防止重新驱动。FSM 链（stop_cleaning）不再是安全偿付关键路径。

---

### 16.2 ✅ 已修复：HealthService 已内置双模式，DiagnosticsCollector 已废弃

**原问题**：`HealthService` 与 `DiagnosticsCollector` 代码高度重复。

**已修复**：`HealthService` 通过 `Mode` 枚举直接支持双模式，解决重复问题：
- `Mode::HEALTH` → 4轮均值精简上报
- `Mode::DIAGNOSTICS` → 每轮独立 lt/rt/lb/rb + ctrl 统计完整上报

`DiagnosticsCollector` 头文件保留为迁移存根，新代码应直接使用 `HealthService`。

---

### 16.3 🟡 中度：CleanTask 实例化但从未被 FSM 使用

**问题描述**：
```cpp
// main.cc：
auto clean_task = std::make_shared<robot::app::CleanTask>(motion, nav, task_cfg);
// 之后 clean_task 从未被调用，也未被 FSM 引用
```

`RobotFsm` 直接通过 `MotionService` 控制运动（`start_cleaning/start_returning`），
`CleanTask` 的里程计终点判断（`reached_end()`）和趟数统计从未在正式路径中执行。

FSM 对轨道端点的感知完全依赖 `LimitTriggerEvent`（物理限位开关），
而非 CleanTask 的里程计判断。

**建议**：
- 若多趟往复清扫需要里程计判断：在 FSM 中集成 CleanTask，或在 NavService 轮询
- 若不需要：删除 `CleanTask` 实例化，减少无效依赖

---

### 16.4 🟡 中度：SchedulerService 头文件包含但未使用

**问题描述**：
```cpp
// main.cc 第 53 行：
#include "pv_cleaning_robot/service/scheduler_service.h"
// 但 main.cc 中没有 SchedulerService 实例
```

时间窗口调度功能未接入。FSM 的 `EvStart` 只能通过 RPC 下行触发。

**建议**：在 main.cc 中实例化 `SchedulerService`，并通过
`set_on_task_start([&fsm]() { fsm->dispatch(EvStart{}); })` 连接。

---

### 16.5 🟡 中度：NavService 未使用 GPS 数据

**问题描述**：`NavService` 构造接受 `GpsDevice` 指针，但 `update()` 中从不调用 `gps_->get_latest()`。GPS 数据只在 `HealthService/DiagnosticsCollector` 中用于上报，没有参与里程计校正。

**建议**：
- 短期：将 GPS 参数从 NavService 构造移除（或标注为 reserved）
- 长期：实现 GPS 航迹定期校正里程计累积误差（适合轨道时长 >5分钟的场景）

---

### 16.6 ✅ 已修复：NavService 里程计已改用 WalkMotorGroup

**原问题**：NavService 使用单轮 `WalkMotor` 做速度积分。

**已修复**：NavService 构造接受 `WalkMotorGroup`，`update()` 取 4 轮平均速度：

```cpp
auto gs = walk_->get_group_status();
float avg_rpm = 0.0f;
for (auto& w : gs.wheel) avg_rpm += w.speed_rpm;
avg_rpm /= static_cast<float>(device::WalkMotorGroup::kWheelCount);
```

单轮打滑对里程计的影响由 4 轮平均收敛。

---

### 16.7 ✅ 已修复：CAN 接收竞争已解决，WalkMotor 已完全移除

**原问题**：`WalkMotor` 和 `WalkMotorGroup` 各自持有一个后台接收线程，在同一个 CAN fd 上竞争，潜在丢帧。

**已修复**：`WalkMotor` 实例已从 main.cc 完全移除，生产路径只保留 `group_recv` 一个接收线程，竞争问题彻底消除。

> `WalkMotor` 头文件仅在单元测试中保留使用。

---

### 16.8 🟢 轻微：WatchdogMgr 监控覆盖不全

**问题描述**：仅 `walk_ctrl` 线程注册了软件看门狗，`nav`/`bms`/`cloud` 线程死锁时只有硬件看门狗（30s 超时）兜底。

**建议**：为所有关键线程注册看门狗票据，并在每个 `update()` 结束后调用 `heartbeat()`。

---

### 16.9 冗余分析一览表

| 编号 | 问题 | 严重度 | 状态 |
|------|------|--------|------|
| 16.1 | SafetyMonitor 不调用 WalkMotorGroup::emergency_override() | 🔴 | ✅ 已修复 |
| 16.2 | HealthService / DiagnosticsCollector 代码重复 | 🟡 | ✅ 已修复（Mode 枚举双模式）|
| 16.3 | CleanTask 无效实例（FSM 不使用） | 🟡 | ⏳ 待接入 FSM |
| 16.4 | SchedulerService 未实例化（include 但不用） | 🟡 | ⏳ 待接入定时调度 |
| 16.5 | NavService.gps_ 未使用 | 🟡 | ⏳ 待移除参数或实现 GPS 校正 |
| 16.6 | NavService 用单 WalkMotor 而非 WalkMotorGroup | 🟡 | ✅ 已修复（4轮平均）|
| 16.7 | 双重 CAN 接收线程竞争 | 🟢 | ✅ 已修复（WalkMotor 已移除）|
| 16.8 | WatchdogMgr 监控不全 | 🟢 | ⏳ 各线程注册心跳票据 |
| 16.9 | group_recv/imu_read/nav/watchdog_mon 缺乏 RT 调度与 CPU 亲和性 | 🔴 | ✅ 已修复（SCHED_FIFO + CPU 绑定）|
| 16.10 | 优先级反转：std::mutex 在 RT 与非 RT 线程间共享 | 🔴 | ✅ 已修复（全部迁移至 PiMutex）|
| 16.11 | 进程缺少 mlockall，缺页中断引入 RT 延迟抖动 | 🔴 | ✅ 已修复（main.cc 启动时调用）|
| 16.12 | safety_monitor(99) 高于 GPIO(95) 可能反向压制边沿事件 | 🟡 | ✅ 已修复（safety_monitor 调整为 FIFO 94）|
| 16.13 | group_recv 低于 walk_ctrl 可能导致控制拍读取旧 CAN 状态 | 🟡 | ✅ 已修复（group_recv 调整为 FIFO 82）|
| 16.14 | NavService 使用 shared_mutex 存在 RT 写者等待低优先级读者风险 | 🟡 | ✅ 已修复（迁移至 PiMutex）|

---

*文档生成日期：2026-03-26*  
*对应代码版本：pv_cleaning_robot @ RK3576 aarch64（WalkMotor 全面替换为 WalkMotorGroup，DiagnosticsCollector 合并至 HealthService，RT 调度/CPU 亲和性/PiMutex/mlockall 全面补齐）*
