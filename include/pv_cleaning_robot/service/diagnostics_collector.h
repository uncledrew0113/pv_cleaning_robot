#pragma once
// DiagnosticsCollector 已废弃，功能已合并到 HealthService::Mode::DIAGNOSTICS。
// 请改用：
//   #include "pv_cleaning_robot/service/health_service.h"
//   auto svc = std::make_shared<HealthService>(..., HealthService::Mode::DIAGNOSTICS);
