#pragma once

#include "pv_cleaning_robot/protocol/nmea_parser.h"

#include <cstdint>
#include <string_view>

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
    GpsdMessageClass message_class{GpsdMessageClass::NONE};
    bool updated_fix{false};
    bool updated_satellites{false};
    bool parse_error{false};
    bool time_parse_error{false};
};

class GpsdJsonParser {
public:
    static bool parse_line(std::string_view line,
                           GpsData& data,
                           GpsdJsonParseResult& result) noexcept;
};

}  // namespace robot::protocol
