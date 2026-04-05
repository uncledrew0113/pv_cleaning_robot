/*
 * @Author: UncleDrew
 * @Date: 2026-03-14 16:02:26
 * @LastEditors: UncleDrew
 * @LastEditTime: 2026-03-15 16:11:46
 * @FilePath: /pv_cleaning_robot/include/pv_cleaning_robot/middleware/ota_manager.h
 * @Description:
 *
 * Copyright (c) 2026 by UncleDrew, All Rights Reserved.
 */
#pragma once
#include <cstdint>
#include <functional>
#include <istream>
#include <mutex>
#include <string>

namespace robot::middleware {

/// @brief OTA 固件更新管理器（A/B 分区）
///
/// 工作流程：
///   1. cloud 回调通知固件包 URL/topic
///   2. OtaManager 分块接收固件到非活动分区 B（流式 4KB chunk）
///   3. SHA-256 校验（OpenSSL EVP，兼容 1.x/3.x）
///   4. 写入 bootloader 分区标志（切换至 B）
///   5. 安全状态检查通过后触发重启
class OtaManager {
   public:
    enum class State { IDLE, DOWNLOADING, VERIFYING, WRITING_FLAG, PENDING_REBOOT, FAILED };

    struct Progress {
        State state{State::IDLE};
        uint32_t bytes_written{0};
        uint32_t total_bytes{0};
        std::string error_msg;
    };

    using ProgressCallback = std::function<void(const Progress&)>;
    /// 安全门卫回调：返回 true 表示可以重启，false 阻止重启
    using SafetyCheckFn = std::function<bool()>;

    /// @param partition_b_path  固件写入目标路径（B 分区块设备或文件）
    /// @param flag_path         bootloader 分区标志文件路径
    OtaManager(std::string partition_b_path, std::string flag_path);

    /// 开始 OTA（流式写入，峰值内存 ~8KB）
    /// @param firmware_stream   固件数据流（istream，支持 istringstream/ifstream）
    /// @param expected_bytes    固件总字节数（用于进度上报，传 0 则跳过进度计算）
    /// @param expected_sha256   固件 SHA-256（64字符十六进制，cloud 端提供）
    bool start_stream(std::istream& firmware_stream,
                      uint64_t expected_bytes,
                      const std::string& expected_sha256);

    /// 兼容旧接口：内部包装为 istringstream 调用 start_stream
    /// @deprecated 优先使用 start_stream；此接口会将全量数据载入内存
    bool start(const std::string& firmware_data, const std::string& expected_sha256);

    /// 写入分区切换标志并触发重启（start/start_stream 成功后调用）
    /// 若已通过 set_safety_check() 注入安全门卫，则仅在门卫返回 true 时执行重启
    bool apply_and_reboot();

    Progress get_progress() const;
    State state() const;
    void set_progress_callback(ProgressCallback cb);
    /// 注入安全门卫（建议在 main.cc 中注入，确保机器人在安全状态才允许重启）
    void set_safety_check(SafetyCheckFn fn);

   private:
    bool verify_sha256(std::istream& data, const std::string& expected_hex);
    void set_state(State s, const std::string& err = "");

    std::string partition_b_path_;
    std::string flag_path_;
    Progress progress_;
    ProgressCallback on_progress_;
    SafetyCheckFn safety_check_;
    mutable std::mutex mtx_;
};

}  // namespace robot::middleware
