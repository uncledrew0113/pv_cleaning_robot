#pragma once
#include <memory>
#include <string>

// spdlog 前向声明
namespace spdlog { class logger; }

namespace robot::middleware {

/// @brief 全局日志器封装（spdlog，旋转文件 + 控制台双 sink）
///
/// 调用 Logger::init() 后，在任意模块通过 Logger::get() 访问 spdlog::logger。
/// 不重复创建：多次调用 init() 只有首次生效。
class Logger {
public:
    struct Config {
        std::string log_dir{"logs"};       ///< 日志文件目录
        std::string file_name{"robot"};    ///< 日志基础文件名
        size_t      max_file_size{10 * 1024 * 1024};  ///< 单文件最大字节数（10 MiB）
        int         max_files{5};          ///< 保留旋转文件数
        bool        console_output{true};  ///< 是否同时输出到控制台
        std::string level{"info"};         ///< 日志级别：trace/debug/info/warn/error/critical
    };

    /// 初始化（线程安全，幂等）
    static void init(const Config& cfg);

    /// 获取 spdlog logger 实例
    static std::shared_ptr<spdlog::logger> get();

    Logger() = delete;
};

} // namespace robot::middleware
