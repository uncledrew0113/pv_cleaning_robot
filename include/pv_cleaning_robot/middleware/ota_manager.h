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
#include <mutex>
#include <string>

namespace robot::middleware {

/// @brief OTA 固件更新管理器（A/B 分区）
///
/// 工作流程：
///   1. cloud 回调通知固件包 URL/topic
///   2. OtaManager 分块接收固件到非活动分区 B（文件流）
///   3. MD5 校验
///   4. 写入 bootloader 分区标志（切换至 B）
///   5. 触发重启
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

    /// @param partition_b_path  固件写入目标路径（B 分区块设备或文件）
    /// @param flag_path         bootloader 分区标志文件路径
    OtaManager(std::string partition_b_path, std::string flag_path);

    /// 开始 OTA（下载并写入固件）
    /// @param expected_md5  固件包 MD5（32字符十六进制，cloud 端提供）
    bool start(const std::string& firmware_data, const std::string& expected_md5);

    /// 写入分区切换标志并触发重启（start() 成功后调用）
    bool apply_and_reboot();

    Progress get_progress() const;
    State state() const;
    void set_progress_callback(ProgressCallback cb);

   private:
    bool verify_md5(const std::string& data, const std::string& expected_md5);
    void set_state(State s, const std::string& err = "");

    std::string partition_b_path_;
    std::string flag_path_;
    Progress progress_;
    ProgressCallback on_progress_;
    mutable std::mutex mtx_;
};

}  // namespace robot::middleware
