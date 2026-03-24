#include <fstream>
#include <iomanip>
#include <linux/reboot.h>
#include <mutex>
#include <openssl/md5.h>
#include <sstream>
#include <sys/syscall.h>
#include <unistd.h>

#include "pv_cleaning_robot/middleware/ota_manager.h"

namespace robot::middleware {

OtaManager::OtaManager(std::string partition_b_path, std::string flag_path)
    : partition_b_path_(std::move(partition_b_path)), flag_path_(std::move(flag_path)) {}

bool OtaManager::start(const std::string& firmware_data, const std::string& expected_md5) {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        progress_.state = State::DOWNLOADING;
        progress_.total_bytes = static_cast<uint32_t>(firmware_data.size());
        progress_.bytes_written = 0;
        progress_.error_msg.clear();
    }
    if (on_progress_)
        on_progress_(get_progress());

    // 写入 B 分区
    {
        std::ofstream ofs(partition_b_path_, std::ios::binary | std::ios::trunc);
        if (!ofs) {
            set_state(State::FAILED, "cannot open partition: " + partition_b_path_);
            return false;
        }
        constexpr size_t kChunk = 4096;
        size_t offset = 0;
        while (offset < firmware_data.size()) {
            size_t to_write = std::min(kChunk, firmware_data.size() - offset);
            ofs.write(firmware_data.data() + offset, static_cast<std::streamsize>(to_write));
            offset += to_write;
            {
                std::lock_guard<std::mutex> lk(mtx_);
                progress_.bytes_written = static_cast<uint32_t>(offset);
            }
            if (on_progress_)
                on_progress_(get_progress());
        }
        ofs.flush();
    }

    // MD5 校验
    set_state(State::VERIFYING);
    if (on_progress_)
        on_progress_(get_progress());

    if (!verify_md5(firmware_data, expected_md5)) {
        set_state(State::FAILED, "MD5 mismatch");
        return false;
    }

    set_state(State::WRITING_FLAG);
    if (on_progress_)
        on_progress_(get_progress());

    set_state(State::PENDING_REBOOT);
    if (on_progress_)
        on_progress_(get_progress());

    return true;
}

bool OtaManager::apply_and_reboot() {
    // 写入分区切换标志（简单标志文件：写入 "B\n"）
    {
        std::ofstream ofs(flag_path_, std::ios::trunc);
        if (!ofs)
            return false;
        ofs << "B\n";
    }
    sync();  // 确保写入落盘

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

bool OtaManager::verify_md5(const std::string& data, const std::string& expected_md5) {
    unsigned char digest[MD5_DIGEST_LENGTH];
    MD5(reinterpret_cast<const unsigned char*>(data.data()), data.size(), digest);

    std::ostringstream oss;
    for (auto b : digest)
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);

    return oss.str() == expected_md5;
}

void OtaManager::set_state(State s, const std::string& err) {
    std::lock_guard<std::mutex> lk(mtx_);
    progress_.state = s;
    progress_.error_msg = err;
}

}  // namespace robot::middleware
