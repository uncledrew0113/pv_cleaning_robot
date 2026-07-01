/*
 * NMEA 0183 GPS 语句解析器接口。
 *
 * 本文件只处理 NMEA 语句解析和字段累计，不访问串口/GPSD，也不决定 GPS 卡滞或通信恢复策略。
 */
#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot::protocol {

/// @brief 解析后的 GPS 定位数据。
struct GpsData {
    double latitude;             ///< 纬度（度，负=南纬）
    double longitude;            ///< 经度（度，负=西经）
    float altitude_m;            ///< 海拔（米，来自 GGA 句子）
    float speed_m_s;             ///< 地速（m/s，来自 RMC 句子）
    float course_deg;            ///< 航向角（度，来自 RMC 句子）
    float hdop;                  ///< 水平精度因子
    uint8_t satellites_used;     ///< 参与定位的卫星数
    uint8_t satellites_in_view;  ///< 视野内卫星总数
    uint8_t fix_quality;         ///< 定位质量（0=无效,1=GPS,2=差分,4=RTK固定）
    float pdop;                  ///< 位置精度因子
    float vdop;                  ///< 垂直精度因子
    bool valid;                  ///< 数据是否有效
    uint64_t utc_timestamp_ms;   ///< UTC 毫秒时间戳（从 epoch 起算）
};

/// @brief NMEA 0183 语句解析器。
///
/// 支持以下语句类型：
///   $GxGGA — 全球定位数据（位置/高程/定位质量/卫星数/HDOP）
///   $GxRMC — 推荐最小导航信息（速度/航向/日期）
///   $GxGSA — DOP 和有效卫星
///   $GxGSV — 可见卫星详情（用于卫星总数统计）
///
/// 其中 x = N (NAVSTAR/GPS), C (北斗), A (综合)
class NmeaParser {
   public:
    /// 解析单行 NMEA 句子（含或不含 \r\n）
    /// 返回 true 表示该句子语法有效且已被成功解析；
    /// 返回 false 表示格式/校验/句型不合法，不更新统计调用方可据此计入解析错误。
    /// 每次成功调用后通过 get_data() 获取最新值。
    bool parse_sentence(const std::string& sentence);

    /// 获取当前积累的 GPS 数据；线程安全由调用方保证。
    const GpsData& get_data() const {
        return data_;
    }

    /// 重置解析累计数据。
    void reset();

   private:
    bool parse_gga(const std::string& s);
    bool parse_rmc(const std::string& s);
    bool parse_gsa(const std::string& s);
    bool parse_gsv(const std::string& s);

    static double parse_nmea_coord(const std::string& raw, const std::string& hemi);
    static bool validate_checksum(const std::string& sentence);
    static std::vector<std::string> split(const std::string& s, char delim);

    GpsData data_{};
};

}  // namespace robot::protocol
