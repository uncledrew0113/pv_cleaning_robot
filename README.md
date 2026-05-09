# PV 清扫机器人固件

RK3576 / PREEMPT\_RT Linux / aarch64，C++17。光伏板面清扫机器人固件——主轨道清扫、故障自检、云端遥测。

- OTA 支持当前处于 dormant 状态，并且默认不参与产品构建。
- 仓库仍保留 `OtaManager` 代码，作为后续集成的归档实现基础。

## 硬件要求

| 接口 | 设备 | 配置路径 |
|------|------|----------|
| CAN (`can0`) | 行走电机组 × 4（M1502E_111，motor_id 1–4） | `can.interface` |
| UART `/dev/ttyS1` | IMU（WIT Motion 9轴，9600 baud） | `serial.imu` |
| UART `/dev/ttyS2` / TCP `127.0.0.1:2947` | GPS（NMEA 0183 或 gpsd，可选） | `gps.source="serial"` 使用 `gps.serial`；`gps.source="gpsd"` 使用 `gps.gpsd` |
| UART `/dev/ttyS3` | 滚刷电机（ODrive 3.6 UART ASCII） | `serial.brush` |
| UART `/dev/ttyS8` | BMS（嘉佰达通用协议 V4，9600 baud） | `serial.bms` |
| GPIO `gpiochip5/0` | 前限位传感器（接近开关，NPN 常开） | `gpio.front_limit` |
| GPIO `gpiochip5/1` | 后限位传感器（接近开关，NPN 常开） | `gpio.rear_limit` |

> **重要**：RK3576 的 `gpiochip5` 不支持 GPIO 中断（`GPIO_GET_LINEEVENT_IOCTL` 返回 `ENODEV`），
> 必须在 `config.json` 中设置 `"gpio": { "use_irq": false }` 启用轮询模式（默认已配置）。

## 构建前置条件

1. 安装 aarch64 交叉编译工具链（`aarch64-linux-gnu-gcc`）
2. 设置 RK3576 SDK 路径环境变量：

```bash
source .env.example        # 或 export RK3576_SDK_PATH=/your/sdk/path
```

3. 配置并编译：

```bash
cmake --preset rk3576-cross-linux
cmake --build --preset rk3576-build
```

构建产物：
- 主程序：`build/aarch64/bin/pv_cleaning_robot`
- 单元测试：`build/aarch64/bin/unit_tests`（可在 QEMU 或目标板运行，**不需要真实硬件**）
- 硬件集成测试：`build/aarch64/bin/hw_tests`（**必须在目标板上运行，需要真实硬件**）

> `build/aarch64/bin/*` 是目标机 `aarch64` 产物。若在 x86 开发机直接执行，出现 `Exec format error` 是正常现象。

## 运行测试

### 单元测试（Mock 硬件，无需真实设备）

```bash
# 全部单元测试
./unit_tests

# 按标签筛选
./unit_tests "[protocol]"                         # 协议层解码测试
./unit_tests "[device][bms]"                      # BMS 设备层
./unit_tests "[integration]"                      # 系统集成测试（mock 硬件）
./unit_tests "[integration][system][health]"      # HealthService JSONL 落盘验证
./unit_tests "[integration][system][combined]"    # 联合 Safety/Health/Watchdog 300ms
```

### 硬件集成测试（目标板 + 真实硬件）

> **安全须知**：运行前确保机器人在停机位，清扫轨道前方无障碍物。
> 电机测试限速 30 RPM。完整任务链测试需前后限位传感器均已接线。

```bash
# 限位传感器回调链路（需在提示后 5 秒内手动触发传感器）
./hw_tests "[hw_limit][callback_front]"
./hw_tests "[hw_limit][callback_rear]"

# 行走电机组（前进/反转/急停/恢复）
./hw_tests "[hw_walk]"

# BMS 数据读取验证
./hw_tests "[hw_bms]"

# IMU 数据读取验证
./hw_tests "[hw_imu]"

# 全栈初始化（推荐先运行）
./hw_tests "[hw_system][full_init]"

# 实时健康数据采集验证
./hw_tests "[hw_system][health_real_data]"

# N=1 完整任务链（CleanFwd → CleanReturn → Charging）+ 持续健康数据采集
# 结束后 JSONL 文件保留在 /data/pv_robot/logs/
./hw_tests "[hw_system][n1_clean_cycle]"

# 完整循环测试（同上，全程持续采集）
./hw_tests "[hw_system][combined]"

# 运行全部硬件系统测试
./hw_tests "[hw_system]"
```

### 目标机推荐测试顺序

1. 在开发机交叉编译：

```bash
source .env.example
cmake --preset rk3576-cross-linux
cmake --build --preset rk3576-build
```

2. 把以下内容拷到目标机同一目录，例如 `/opt/pv_robot/tests/`：

```bash
build/aarch64/bin/unit_tests
build/aarch64/bin/hw_tests
config/
certs/   # 如果 MQTT TLS 证书放在仓库内
```

3. 在目标机进入测试目录，先跑不依赖真实云和硬件的回归：

```bash
./unit_tests "[service][config]"
./unit_tests "[service][tb_control_plane]"
./unit_tests "[app][robot_supervisor]"
./unit_tests "[integration][task_chain]"
```

4. 需要验证真实 ThingsBoard 时，再开启真实云测试：

```bash
export TB_REAL_TEST=1
./unit_tests "[integration][thingsboard][real]"
```

5. 需要验证真实硬件链路时，先准备 `hw_test_config.json`，再按风险从低到高执行：

```bash
export HW_TEST_CONFIG=/path/to/hw_test_config.json
./hw_tests "[hw_system][full_init]"
./hw_tests "[hw_system][health_real_data]"
./hw_tests "[hw_system][n1_clean_cycle]"
```

6. 查看硬件测试产物：

```bash
ls -lh /data/pv_robot/logs/
```

说明：
- `TB_REAL_TEST=1` 只在真实 ThingsBoard 联调时需要。
- `HW_TEST_CONFIG` 不设时，测试会按 `CWD/hw_test_config.json`，再退回内嵌默认值。
- `n1_clean_cycle` 前必须确认机器人已停在停机位，轨道无遮挡，前后限位传感器和电机线束都已接好。

## 配置文件

运行时配置：`/opt/robot/config/config.json`（生产）/ `config/config.json`（开发回退）。

**运维文件语义**：

- `config.json`：当前生效配置。设备启动时优先加载它；运行中的立即生效配置也写回这里。
- `config.pending.json`：待下次任务提升的配置。ThingsBoard 下发的“下次任务生效”参数会先落在这里。
- `config.backup.json`：主配置回滚副本。主配置写入前会先更新它；若 `config.json` 损坏，启动时会尝试从这里恢复。

**关键配置项**：

| 键 | 说明 |
|----|------|
| `gpio.use_irq` | `false`=轮询模式（**RK3576 gpiochip5 必须设为 false**）；`true`=中断模式（其他平台） |
| `gps.source` | GPS 数据源选择：`"serial"` / `"gpsd"` |
| `gps.serial.port` / `gps.serial.baudrate` | 串口 GPS 参数 |
| `gps.gpsd.host` / `gps.gpsd.port` / `gps.gpsd.watch` | gpsd TCP 服务参数 |
| `network.transport_mode` | `"mqtt_only"` / `"lorawan_only"` / `"dual_parallel"` |
| `network.mqtt.tls_enabled` | 是否启用 TLS（生产环境应为 `true`） |
| `network.mqtt.ca_cert_path` | CA 证书路径 |
| `diagnostics.mode` | `"production"` → HEALTH 精简模式；`"development"` → DIAGNOSTICS 完整模式 |
| `diagnostics.local_path` | 本地 JSONL 落盘路径（空字符串禁用） |
| `robot.passes` | 清扫趟数（首版仅支持整数趟；`1.0`=往返一趟） |
| `robot.heading_pid_en` | 是否启用 IMU 航向 PID 差速补偿（需 IMU 已接线） |

兼容说明：若 `gps.source` 缺失，程序会临时回退读取旧配置 `serial.gps.*`。

## 参考文档

- `doc/API_REFERENCE.md` — 完整 API 接口文档（含协议格式、JSONL 载荷格式、配置说明）
- `doc/dev-guide/CONCURRENCY.md` — 并发契约与线程关闭顺序
- `docs/superpowers/specs/` — 架构优化设计规格（历史记录）
