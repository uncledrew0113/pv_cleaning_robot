# PV Cleaning Robot systemd 自启动设计

## 目标

将 `pv_cleaning_robot` 交由 systemd 管理，实现：

- 工控机开机后自动启动；
- EC20N 拨号服务和视觉服务启动后，再启动机器人主程序；
- 主程序异常退出后有限重启；
- 通过 `systemctl` 完成启动、停止、重启和状态查看；
- `systemctl stop` 使用程序现有的 `SIGTERM` 优雅退出流程；
- 启动失败和运行日志可以通过 journal 明确查询。

## 当前运行事实

- 可执行文件：
  `/root/pv_cleaning_robot/bin_new_0724/pv_cleaning_robot`
- 工作目录：
  `/root/pv_cleaning_robot/bin_new_0724`
- 程序配置加载顺序保持现状：
  1. 优先加载 `/opt/robot/config/config.runtime.json` 和
     `/opt/robot/config/config.fixed.json`；
  2. 加载失败时回退到工作目录下的 `config/config.runtime.json` 和
     `config/config.fixed.json`。
- 当前 `/opt/robot/config` 未部署配置，因此实际使用
  `/root/pv_cleaning_robot/bin_new_0724/config/` 下的配置。
- 程序需要访问 CAN、串口、GPIO、硬件看门狗，并创建实时调度线程和锁定内存页。
- 程序已经处理 `SIGINT` 和 `SIGTERM`，收到信号后会停止线程、运动和硬件资源。

## 方案

### 基础服务

新增 `pv-cleaning-robot.service`，核心设置如下：

- `Type=simple`：主进程保持前台运行，由 systemd 直接跟踪；
- `User=root`：延续当前运行权限，满足硬件访问、实时调度和内存锁定需求；
- `WorkingDirectory=/root/pv_cleaning_robot/bin_new_0724`；
- `ExecStart=/root/pv_cleaning_robot/bin_new_0724/pv_cleaning_robot`；
- `Restart=on-failure`，正常停止不自动拉起；
- `RestartSec=5s`；
- 60 秒内最多尝试重启 3 次，避免配置或硬件持续错误造成无限重启；
- `KillSignal=SIGTERM`；
- `TimeoutStopSec=30s`，给现有优雅关闭流程留出时间；
- `LimitMEMLOCK=infinity` 和 `LimitRTPRIO=99`，满足内存锁定和实时线程需求；
- 标准输出和标准错误进入 systemd journal。

不在 service 中复制配置文件，也不改变程序现有配置优先级。若两套配置都无法加载，
主程序返回非零，systemd 按有限重启策略处理，并在 journal 中保留失败原因。

### EC20N 和视觉服务依赖

两个现有依赖 unit 为：

- EC20N 拨号服务：`ec20-qmi.service`
- 视觉服务：`pv_edge_tracker.service`

机器相关依赖使用独立 drop-in：

`/etc/systemd/system/pv-cleaning-robot.service.d/dependencies.conf`

```ini
[Unit]
Requires=ec20-qmi.service pv_edge_tracker.service
After=ec20-qmi.service pv_edge_tracker.service
```

安装 drop-in 并执行 `systemctl daemon-reload` 后，才启用机器人主程序的开机自启动。
这样基础 unit 保持通用，工控机上的依赖关系则明确绑定到实际服务。

`After=` 保证 unit 启动顺序，`Requires=` 保证依赖启动失败时机器人服务不会启动。
它们是否代表“业务已就绪”取决于 EC20N 和视觉服务自身的 `Type`：

- 若依赖服务只有完成拨号或初始化后才进入 active，以上配置足够；
- 若依赖服务为 `Type=simple`，active 只代表进程已经创建，不能证明网络或视觉数据已经
  可用。此时应另行定义确定性的就绪检查，并设置最大等待时间，不能用无限循环等待。

第一版不猜测网络接口、端口、socket 或共享内存名称，因此不加入未经确认的
`ExecStartPre` 就绪探测。

## 运维接口

现场使用 systemd 原生命令，不额外维护单用途启动脚本：

```bash
systemctl start pv-cleaning-robot
systemctl stop pv-cleaning-robot
systemctl restart pv-cleaning-robot
systemctl status pv-cleaning-robot
journalctl -u pv-cleaning-robot -f
```

安装基础 service 和依赖 drop-in 后，使用：

```bash
systemctl enable --now pv-cleaning-robot
```

## 验证

实施后依次验证：

1. `systemd-analyze verify` 检查 unit 和 drop-in 语法；
2. 手动启动后确认主进程、工作目录和实际配置加载日志；
3. `systemctl stop` 后确认程序完成优雅关闭，且没有自动重启；
4. 模拟一次非零异常退出，确认 5 秒后自动重启；
5. 制造持续启动失败，确认 60 秒内超过 3 次后停止重试并明确报错；
6. 重启工控机，确认 EC20N、视觉服务先启动，机器人主程序随后启动；
7. 查看 `systemctl status`、`journalctl` 和 `systemd-analyze critical-chain`，
   确认启动顺序与错误信息。

硬件实机验证不得在锁止机构、行走轮或滚刷处于不安全状态时模拟异常。

## 回退

若自启动影响现场调试，执行：

```bash
systemctl disable --now pv-cleaning-robot
```

禁用后仍可回到原来的目录手动运行主程序。删除 unit 或 drop-in 前先禁用服务并确认主
进程已经退出。
