#include "pv_cleaning_robot/middleware/logger.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <mutex>

namespace robot::middleware {

static std::once_flag         g_init_flag;
static std::shared_ptr<spdlog::logger> g_logger;

void Logger::init(const Config& cfg)
{
    std::call_once(g_init_flag, [&cfg]() {
        std::vector<spdlog::sink_ptr> sinks;

        // 旋转文件 sink
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            cfg.log_dir + "/" + cfg.file_name + ".log",
            cfg.max_file_size,
            cfg.max_files);
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%t] %v");
        sinks.push_back(file_sink);

        // 控制台 sink
        if (cfg.console_output) {
            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console_sink->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
            sinks.push_back(console_sink);
        }

        g_logger = std::make_shared<spdlog::logger>("robot", sinks.begin(), sinks.end());
        g_logger->set_level(spdlog::level::from_str(cfg.level));
        g_logger->flush_on(spdlog::level::warn);
        spdlog::register_logger(g_logger);
        spdlog::set_default_logger(g_logger);
    });
}

std::shared_ptr<spdlog::logger> Logger::get()
{
    if (!g_logger) {
        // 未初始化时返回默认 logger
        return spdlog::default_logger();
    }
    return g_logger;
}

} // namespace robot::middleware
