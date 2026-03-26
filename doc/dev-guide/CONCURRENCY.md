**并发契约清单（Concurrency Contracts）**

本文件列出仓库中关键对象的线程/互斥契约与建议的 shutdown 顺序，便于开发者在修改驱动与服务时遵循统一规则。

**总体原则**
- 所有公开的 stop/request_stop/close 接口必须幂等。
- 停止顺序遵循：请求停止 → 等待线程退出（join）→ 关闭/释放资源。
- 持有 `PiMutex` 时不要 sleep 或长时间阻塞；回调中尽量避免再次获取相同的锁。

**组件契约（按文件/类）**

- **`robot::hal::PiMutex`** ([include/pv_cleaning_robot/hal/pi_mutex.h](include/pv_cleaning_robot/hal/pi_mutex.h))
  - 合约：实现 BasicLockable；`lock()` 在持有者线程返回；若出现 `EOWNERDEAD` 抛异常，调用者负责恢复/重建受保护状态；`unlock()` 必须由获取锁的同一线程调用。
  - 注意：不要在持锁期间做长阻塞或调用会导致阻塞的外部 API。

- **`LinuxCanSocket`** ([include/pv_cleaning_robot/driver/linux_can_socket.h](include/pv_cleaning_robot/driver/linux_can_socket.h))
  - 合约：`send()`/`recv()` 设计为可并发调用（内部使用原子 FD 快照、保护过滤器的 mutex）。
  - 关闭：在调用 `close()` 前必须先请求停止相关 IO 线程并 `join()`，否则会出现 FD 竞争、EPIPE 或未定义行为。
  - 恢复：`recover()` 应在 IO 停止或已保证无并发访问时调用。

- **`LibGpiodPin` / GPIO 监控** ([pv_cleaning_robot/driver/libgpiod_pin.cc](pv_cleaning_robot/driver/libgpiod_pin.cc))
  - 合约：监控在独立线程执行；回调在 `cb_mutex_`（现为 `PiMutex`）保护下触发。
  - 回调约束：回调须尽快返回，不可直接销毁或关闭底层 line；如需销毁，回调应通知管理线程完成后再释放资源。
  - 关闭：`stop()`/`request_stop()` 应设置停止标志并 `join()` 监控线程，等待线程安全退出后再释放 line。

- **`LibSerialPort`** ([pv_cleaning_robot/driver/libserialport_port.cc](pv_cleaning_robot/driver/libserialport_port.cc))
  - 合约：所有对 `sp_port*` 的调用需由 `io_mutex_` 串行化，防止 TOCTOU 或并发关闭。
  - 关闭：在释放 port 前应确认没有其他线程在使用（通过互斥或先 `request_stop()` 并 `join()`）。

- **`LibModbusMaster`** ([pv_cleaning_robot/driver/libmodbus_master.cc](pv_cleaning_robot/driver/libmodbus_master.cc))
  - 合约：交易由 `bus_mutex_` 串行化；重试/等待期间已释放互斥以避免阻塞其他事务。
  - 使用约定：避免在持有会等待 modbus 响应的锁时再调用 modbus API，以防死锁。
  - 关闭：先禁止新事务（标志）→ 等待当前事务完成 → 释放资源。

- **`EventBus`** ([include/pv_cleaning_robot/middleware/event_bus.h](include/pv_cleaning_robot/middleware/event_bus.h))
  - 合约：订阅/退订在内部互斥保护下执行；发布应复制订阅者列表并在释放锁后调用回调（避免在持锁时执行回调）。
  - 回调约束：回调必须快速返回，避免在回调中做会导致死锁或长阻塞的操作。

**建议的 Shutdown 顺序（简洁版）**
1. 请求停止高层调度与服务（如 `SafetyMonitor`、任务调度器）。
2. 停止产生事件的组件（发布者），防止新任务入队。
3. 停止监控线程（GPIO / `LibGpiodPin`）：发停止请求并 `join()`。
4. 停止通信 IO（CAN / 串口 / Modbus）：发停止请求并 `join()`。
5. 停止线程池 / 执行器（`ThreadExecutor`），等待任务排空并 `join()`。
6. 关闭并释放硬件句柄（socket、serial port、modbus context、gpiod line）。
7. 销毁全局资源（日志、临时文件、全局互斥）。
8. 最后停止 watchdog 或将其置为安全状态（watchdog 留到最后以便记录崩溃点）。

**Pre-shutdown 快捷检查清单**
- 已对所有生产者发送停止请求吗？
- 所有 monitor/IO 线程是否已 `join()`？（或在超时后记录并强制清理）
- 在释放句柄前，是否确认没有其他线程在使用该句柄？
- 是否在捕获 `EOWNERDEAD` 后执行明确恢复策略而不是吞掉错误？

**补充说明**
- 对于需要硬件才可验证的集成测试（真实 CAN、RS485/Modbus），请在目标设备上按上述关闭顺序手动或通过 CI 脚本验证。
- 若需要，我可以把本文件的关键信息同步到仓库根 README 或生成 PR。

—— End
