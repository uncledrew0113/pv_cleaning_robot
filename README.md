# PV 清扫机器人固件

RK3576 / PREEMPT\_RT Linux / aarch64，C++17。光伏板面清扫机器人固件——主轨道清扫、故障自检、云端遥测。

## 硬件要求

| 接口 | 设备 | 配置路径 |
|------|------|----------|
| CAN (`can0`) | 行走电机组 × 4（M1502E_111，motor_id 1–4） | `can.interface` |
| UART `/dev/ttyS1` | IMU（WIT Motion 9轴，9600 baud） | `serial.imu` |
| UART `/dev/ttyS2` | GPS（NMEA 0183，可选，暂无硬件） | `serial.gps` |
| UART `/dev/ttyS3` | 辊刷电机（Modbus RTU，slave_id=1） | `serial.brush` |
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

## 配置文件

运行时配置：`/opt/robot/config/config.json`（生产）/ `config/config.json`（开发回退）。

**关键配置项**：

| 键 | 说明 |
|----|------|
| `gpio.use_irq` | `false`=轮询模式（**RK3576 gpiochip5 必须设为 false**）；`true`=中断模式（其他平台） |
| `network.transport_mode` | `"mqtt_only"` / `"lorawan_only"` / `"dual_parallel"` |
| `network.mqtt.tls_enabled` | 是否启用 TLS（生产环境应为 `true`） |
| `network.mqtt.ca_cert_path` | CA 证书路径 |
| `diagnostics.mode` | `"production"` → HEALTH 精简模式；`"development"` → DIAGNOSTICS 完整模式 |
| `diagnostics.local_path` | 本地 JSONL 落盘路径（空字符串禁用） |
| `robot.passes` | 清扫趟数（`0.5`=单程，`1.0`=往返一趟） |
| `robot.heading_pid_en` | 是否启用 IMU 航向 PID 差速补偿（需 IMU 已接线） |

## 参考文档

- `doc/API_REFERENCE.md` — 完整 API 接口文档（含协议格式、JSONL 载荷格式、配置说明）
- `doc/dev-guide/CONCURRENCY.md` — 并发契约与线程关闭顺序
- `docs/superpowers/specs/` — 架构优化设计规格（历史记录）
