#include <fstream>
#include <iomanip>
#include <linux/reboot.h>
#include <mutex>
#include <openssl/evp.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <sys/syscall.h>
#include <unistd.h>

#include "pv_cleaning_robot/middleware/ota_manager.h"

namespace robot::middleware {

OtaManager::OtaManager(std::string partition_b_path, std::string flag_path)
    : partition_b_path_(std::move(partition_b_path)), flag_path_(std::move(flag_path)) {}

bool OtaManager::start_stream(std::istream& firmware_stream,
                               uint64_t expected_bytes,
                               const std::string& expected_sha256) {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        progress_.state = State::DOWNLOADING;
        progress_.total_bytes = static_cast<uint32_t>(expected_bytes);
        progress_.bytes_written = 0;
        progress_.error_msg.clear();
    }
    if (on_progress_) on_progress_(get_progress());

    // 流式写入 B 分区（峰值内存 ~4KB）
    {
        std::ofstream ofs(partition_b_path_, std::ios::binary | std::ios::trunc);
        if (!ofs) {
            set_state(State::FAILED, "cannot open partition: " + partition_b_path_);
            return false;
        }
        constexpr size_t kChunk = 4096;
        char buf[kChunk];
        while (firmware_stream.read(buf, kChunk) || firmware_stream.gcount() > 0) {
            auto n = static_cast<std::streamsize>(firmware_stream.gcount());
            ofs.write(buf, n);
            {
                std::lock_guard<std::mutex> lk(mtx_);
                progress_.bytes_written += static_cast<uint32_t>(n);
            }
            if (on_progress_) on_progress_(get_progress());
        }
        ofs.flush();
    }

    // SHA-256 校验（使用 EVP API，兼容 OpenSSL 1.x/3.x，MD5 已废弃）
    set_state(State::VERIFYING);
    if (on_progress_) on_progress_(get_progress());

    // 重置流位置以便校验
    firmware_stream.clear();
    firmware_stream.seekg(0);
    if (!verify_sha256(firmware_stream, expected_sha256)) {
        set_state(State::FAILED, "SHA-256 mismatch");
        return false;
    }

    set_state(State::WRITING_FLAG);
    if (on_progress_) on_progress_(get_progress());

    set_state(State::PENDING_REBOOT);
    if (on_progress_) on_progress_(get_progress());

    return true;
}

bool OtaManager::start(const std::string& firmware_data,
                        const std::string& expected_sha256) {
    std::istringstream iss(firmware_data);
    return start_stream(iss, firmware_data.size(), expected_sha256);
}

bool OtaManager::apply_and_reboot() {
    // 安全门卫：机器人必须处于 Idle 或 Charging 状态才允许重启
    if (safety_check_ && !safety_check_()) {
        spdlog::error("[OTA] Reboot blocked: robot not in safe state (Idle/Charging required)");
        set_state(State::FAILED, "blocked by safety check");
        return false;
    }

    // 写入分区切换标志（简单标志文件：写入 "B\n"）
    {
        std::ofstream ofs(flag_path_, std::ios::trunc);
        if (!ofs) return false;
        ofs << "B\n";
    }
    sync();  // 确保写入落盘

    spdlog::info("[OTA] Rebooting to apply firmware update...");
    // 触发系统重启（需 root 权限）
    syscall(
        SYS_reboot, LINUX_REBOOT_MAGIC1, LINUX_REBOOT_MAGIC2, LINUX_REBOOT_CMD_RESTART, nullptr);
    return true;  // 不可达，仅作类型要求
}

OtaManager::Progress OtaManager::get_progress() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return progress_;
}

OtaManager::State OtaManager::state() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return progress_.state;
}

void OtaManager::set_progress_callback(ProgressCallback cb) {
    on_progress_ = std::move(cb);
}

void OtaManager::set_safety_check(SafetyCheckFn fn) {
    safety_check_ = std::move(fn);
}

bool OtaManager::verify_sha256(std::istream& data, const std::string& expected_hex) {
    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (!ctx) return false;

    EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr);
    char buf[4096];
    while (data.read(buf, sizeof(buf)) || data.gcount() > 0) {
        EVP_DigestUpdate(ctx, buf, static_cast<size_t>(data.gcount()));
    }
    unsigned char digest[32];
    unsigned int len = 0;
    EVP_DigestFinal_ex(ctx, digest, &len);
    EVP_MD_CTX_free(ctx);

    // 转为小写十六进制
    std::ostringstream oss;
    for (unsigned int i = 0; i < len; ++i)
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(digest[i]);

    return oss.str() == expected_hex;
}

void OtaManager::set_state(State s, const std::string& err) {
    std::lock_guard<std::mutex> lk(mtx_);
    progress_.state = s;
    progress_.error_msg = err;
}

}  // namespace robot::middleware
