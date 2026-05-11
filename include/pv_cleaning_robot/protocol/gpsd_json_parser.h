#pragma once

/// @file gpsd_json_parser.h
/// @brief 解析从 gpsd 输出的 JSON 格式 GPS 数据。

#include <cstdint>
#include <string_view>

#include "pv_cleaning_robot/protocol/nmea_parser.h"

namespace robot::protocol {

enum class GpsdMessageClass : uint8_t {
    NONE = 0,
    TPV,
    SKY,
    VERSION,
    WATCH,
    ERROR_MESSAGE,
    UNKNOWN,
};

struct GpsdJsonParseResult {
    /// GPSD JSON 消息类型
    GpsdMessageClass message_class{GpsdMessageClass::NONE};
    /// 是否更新了定位解算结果
    bool updated_fix{false};
    /// 是否更新了卫星信息
    bool updated_satellites{false};
    /// 是否发生了解析错误
    bool parse_error{false};
    /// 时间字段是否解析失败
    bool time_parse_error{false};
};

class GpsdJsonParser {
   public:
    /// @brief 解析单行 gpsd JSON 输出并更新 GPS 数据。
    /// @param line 输入 JSON 行
    /// @param data 解析后填充的 GPS 数据
    /// @param result 附加解析结果状态
    /// @return true 表示解析成功并获得有效数据，否则 false
    static bool parse_line(std::string_view line,
                           GpsData& data,
                           GpsdJsonParseResult& result) noexcept;
};

}  // namespace robot::protocol
