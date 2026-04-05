# PV 清扫机器人固件

RK3576 / PREEMPT\_RT Linux / aarch64，C++17。光伏板面清扫机器人固件——主轨道清扫、故障自检、云端遥测。

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
- 单元测试：`build/aarch64/bin/unit_tests`
- 硬件集成测试：`build/aarch64/bin/hw_tests`

## 运行测试

测试须在 aarch64 目标板上运行（或 QEMU 模拟）：

```bash
# 所有单元测试
./build/aarch64/bin/unit_tests

# 指定测试段
./build/aarch64/bin/unit_tests --section "safety_monitor"
```

## 配置文件

运行时配置：`/opt/robot/config/config.json`（部署）/ `config/config.json`（开发回退）。

关键配置项：
- `network.mqtt.tls_enabled` — 是否启用 TLS（默认 `true`）
- `network.mqtt.ca_cert_path` — CA 证书路径
- `diagnostics.mode` — `"production"` 或 `"development"`
- `robot.clean_speed_rpm` — 清扫速度
- `network.transport_mode` — `"mqtt_only"` / `"lorawan_only"` / `"dual_parallel"`

## 参考文档

- `doc/API_REFERENCE.md` — 完整 API 文档
- `doc/dev-guide/CONCURRENCY.md` — 并发契约与关闭顺序
- `docs/superpowers/specs/` — 优化设计规格
