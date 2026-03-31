/**
 * NMEA 0183 协议解析器单元测试
 * [protocol][nmea]
 */
#include <catch2/catch.hpp>

#include "pv_cleaning_robot/protocol/nmea_parser.h"

using robot::protocol::NmeaParser;

// ────────────────────────────────────────────────────────────────
// 辅助：构造合法校验和后缀 "*HH\r\n"
// ────────────────────────────────────────────────────────────────
static std::string append_checksum(const std::string& sentence) {
    // 从 '$' 之后到 '*' 之前计算异或校验和
    uint8_t cs = 0;
    for (size_t i = 1; i < sentence.size(); ++i)
        cs ^= static_cast<uint8_t>(sentence[i]);
    char buf[8];
    snprintf(buf, sizeof(buf), "*%02X\r\n", cs);
    return sentence + buf;
}

TEST_CASE("NMEA: GGA 句子——有效定位数据", "[protocol][nmea]") {
    NmeaParser p;
    // 标准 GPGGA 句子（武汉附近位置）
    std::string s =
        append_checksum("$GPGGA,120000.00,3032.0000,N,11414.0000,E,1,08,1.2,30.0,M,0.0,M,,");
    p.parse_sentence(s);

    const auto& d = p.get_data();
    REQUIRE(d.valid);
    REQUIRE(d.latitude == Approx(30.5333333).epsilon(1e-4));
    REQUIRE(d.longitude == Approx(114.2333333).epsilon(1e-4));
    REQUIRE(d.altitude_m == Approx(30.0f).epsilon(0.1f));
    REQUIRE(d.satellites_used == 8);
    REQUIRE(d.fix_quality == 1);
    REQUIRE(d.hdop == Approx(1.2f).epsilon(0.01f));
}

TEST_CASE("NMEA: GGA 南纬西经转换", "[protocol][nmea]") {
    NmeaParser p;
    std::string s =
        append_checksum("$GPGGA,120000.00,3032.0000,S,11414.0000,W,1,06,1.5,100.0,M,,M,,");
    p.parse_sentence(s);

    const auto& d = p.get_data();
    REQUIRE(d.latitude < 0.0);   // 南纬为负
    REQUIRE(d.longitude < 0.0);  // 西经为负
}

TEST_CASE("NMEA: GGA 无效定位质量 0 → valid=false", "[protocol][nmea]") {
    NmeaParser p;
    std::string s = append_checksum("$GPGGA,120000.00,3032.0000,N,11414.0000,E,0,00,,,M,,M,,");
    p.parse_sentence(s);

    REQUIRE_FALSE(p.get_data().valid);
}

TEST_CASE("NMEA: RMC 句子——速度与航向", "[protocol][nmea]") {
    NmeaParser p;
    // 先解析一条 GGA 建立有效基准
    p.parse_sentence(
        append_checksum("$GPGGA,120000.00,3032.0000,N,11414.0000,E,1,08,1.0,30.0,M,,M,,"));
    // 3.0 节 = 1.543 m/s，航向 90°
    p.parse_sentence(
        append_checksum("$GPRMC,120000.00,A,3032.0000,N,11414.0000,E,3.0,90.0,010125,,,A"));

    const auto& d = p.get_data();
    REQUIRE(d.speed_m_s == Approx(3.0f * 0.5144f).epsilon(0.01f));
    REQUIRE(d.course_deg == Approx(90.0f).epsilon(0.1f));
}

TEST_CASE("NMEA: RMC 无效状态 V → valid 不被置真", "[protocol][nmea]") {
    NmeaParser p;
    p.parse_sentence(
        append_checksum("$GPRMC,120000.00,V,0000.0000,N,00000.0000,E,0.0,0.0,010125,,,N"));
    // RMC 状态 V → 不应标记 valid
    REQUIRE_FALSE(p.get_data().valid);
}

TEST_CASE("NMEA: GSA 句子——PDOP/VDOP 解析", "[protocol][nmea]") {
    NmeaParser p;
    p.parse_sentence(append_checksum("$GPGSA,A,3,01,02,03,04,05,06,07,08,,,,,1.5,1.2,0.9"));
    const auto& d = p.get_data();
    REQUIRE(d.pdop == Approx(1.5f).epsilon(0.01f));
    REQUIRE(d.hdop == Approx(1.2f).epsilon(0.01f));
    REQUIRE(d.vdop == Approx(0.9f).epsilon(0.01f));
}

TEST_CASE("NMEA: GSV 句子——视野内卫星数", "[protocol][nmea]") {
    NmeaParser p;
    // GSV: 消息总数=2, 本为第1条, 总卫星数=8
    p.parse_sentence(
        append_checksum("$GPGSV,2,1,08,01,40,030,45,02,50,120,43,03,30,200,40,04,60,310,50"));
    const auto& d = p.get_data();
    REQUIRE(d.satellites_in_view == 8);
}

TEST_CASE("NMEA: 错误校验和被拒绝", "[protocol][nmea]") {
    NmeaParser p;
    // 构造错误校验和
    std::string bad = "$GPGGA,120000.00,3032.0000,N,11414.0000,E,1,08,1.0,30.0,M,,M,,*FF\r\n";
    p.parse_sentence(bad);
    // 校验失败：valid 应保持 false 或数据不更新
    REQUIRE_FALSE(p.get_data().valid);
}

TEST_CASE("NMEA: BDGGA（北斗）前缀也能解析", "[protocol][nmea]") {
    NmeaParser p;
    std::string s =
        append_checksum("$BDGGA,120000.00,3032.5000,N,11415.0000,E,1,12,0.8,50.0,M,,M,,");
    p.parse_sentence(s);
    REQUIRE(p.get_data().valid);
    REQUIRE(p.get_data().satellites_used == 12);
}

TEST_CASE("NMEA: reset() 清空所有字段", "[protocol][nmea]") {
    NmeaParser p;
    p.parse_sentence(
        append_checksum("$GPGGA,120000.00,3032.0000,N,11414.0000,E,1,08,1.0,30.0,M,,M,,"));
    REQUIRE(p.get_data().valid);
    p.reset();
    REQUIRE_FALSE(p.get_data().valid);
    REQUIRE(p.get_data().latitude == Approx(0.0));
    REQUIRE(p.get_data().longitude == Approx(0.0));
}

TEST_CASE("NMEA: 空字符串不崩溃", "[protocol][nmea]") {
    NmeaParser p;
    REQUIRE_NOTHROW(p.parse_sentence(""));
    REQUIRE_NOTHROW(p.parse_sentence("$"));
    REQUIRE_NOTHROW(p.parse_sentence("not_nmea_at_all"));
}

TEST_CASE("NMEA: 连续多句累积", "[protocol][nmea]") {
    NmeaParser p;
    p.parse_sentence(
        append_checksum("$GPGGA,120000.00,3032.0000,N,11414.0000,E,1,08,1.0,30.0,M,,M,,"));
    p.parse_sentence(
        append_checksum("$GPRMC,120000.00,A,3032.0000,N,11414.0000,E,5.0,180.0,010125,,,A"));
    p.parse_sentence(append_checksum("$GPGSA,A,3,01,02,03,04,05,06,07,08,,,,,2.0,1.8,1.0"));

    const auto& d = p.get_data();
    REQUIRE(d.valid);
    REQUIRE(d.speed_m_s > 0.0f);
    REQUIRE(d.course_deg == Approx(180.0f).epsilon(0.1f));
    REQUIRE(d.pdop == Approx(2.0f).epsilon(0.01f));
}

TEST_CASE("NMEA: GNGGA（联合）前缀解析", "[protocol][nmea]") {
    NmeaParser p;
    std::string s =
        append_checksum("$GNGGA,120000.00,3032.0000,N,11414.0000,E,1,10,0.9,25.0,M,,M,,");
    p.parse_sentence(s);
    REQUIRE(p.get_data().valid);
}
