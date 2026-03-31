/**
 * GpsDevice 设备层单元测试（依赖 MockSerialPort）
 * [device][gps]
 */
#include <catch2/catch.hpp>
#include <chrono>
#include <thread>

#include "../mock/mock_serial_port.h"
#include "pv_cleaning_robot/device/gps_device.h"

using robot::device::GpsDevice;

// ────────────────────────────────────────────────────────────────
// 辅助：构造合法 NMEA 校验和后缀
// ────────────────────────────────────────────────────────────────
static std::string nmea_with_cs(const std::string& s) {
    uint8_t cs = 0;
    for (size_t i = 1; i < s.size(); ++i)
        cs ^= static_cast<uint8_t>(s[i]);
    char buf[8];
    snprintf(buf, sizeof(buf), "*%02X\r\n", cs);
    return s + buf;
}

// ────────────────────────────────────────────────────────────────
// 构建辅助
// ────────────────────────────────────────────────────────────────
struct GpsFixture {
    std::shared_ptr<MockSerialPort> serial{std::make_shared<MockSerialPort>()};
    GpsDevice gps;

    GpsFixture() : gps(serial) {
        serial->open_result = true;
    }
};

// ────────────────────────────────────────────────────────────────
// open() / close()
// ────────────────────────────────────────────────────────────────
TEST_CASE("GpsDevice: open() 打开串口并返回 true", "[device][gps]") {
    GpsFixture f;
    REQUIRE(f.gps.open());
    f.gps.close();
}

TEST_CASE("GpsDevice: open() 串口失败时返回 false", "[device][gps]") {
    GpsFixture f;
    f.serial->open_result = false;
    REQUIRE_FALSE(f.gps.open());
}

TEST_CASE("GpsDevice: close() 可重复调用不崩溃", "[device][gps]") {
    GpsFixture f;
    f.gps.open();
    REQUIRE_NOTHROW(f.gps.close());
    REQUIRE_NOTHROW(f.gps.close());
}

// ────────────────────────────────────────────────────────────────
// 初始状态：get_latest() 返回 valid=false
// ────────────────────────────────────────────────────────────────
TEST_CASE("GpsDevice: 未收到任何数据时 get_latest() valid=false", "[device][gps]") {
    GpsFixture f;
    f.gps.open();
    // 串口没有注入数据，立即读取
    auto d = f.gps.get_latest();
    f.gps.close();
    REQUIRE_FALSE(d.valid);
}

// ────────────────────────────────────────────────────────────────
// 注入 GGA 句子：后台线程解析后 get_latest().valid == true
// ────────────────────────────────────────────────────────────────
TEST_CASE("GpsDevice: 注入有效 GGA 后 valid=true 且坐标正确", "[device][gps]") {
    GpsFixture f;

    // 构造一条有效 GPGGA + 换行
    std::string gga =
        nmea_with_cs("$GPGGA,120000.00,3032.0000,N,11414.0000,E,1,08,1.0,30.0,M,,M,,");

    // MockSerialPort 的 rx_data 会在 read() 时返回
    f.serial->rx_data.insert(f.serial->rx_data.end(), gga.begin(), gga.end());

    f.gps.open();
    // 等后台读取线程处理
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto d = f.gps.get_latest();
    f.gps.close();

    REQUIRE(d.valid);
    REQUIRE(d.latitude == Approx(30.5333333).epsilon(1e-4));
    REQUIRE(d.longitude == Approx(114.2333333).epsilon(1e-4));
}

// ────────────────────────────────────────────────────────────────
// 输入无效句子：parse_error_count 增加
// ────────────────────────────────────────────────────────────────
TEST_CASE("GpsDevice: 注入非法句子后 parse_error_count 累加", "[device][gps]") {
    GpsFixture f;

    std::string bad = "$GPGGA,999,BAD,DATA*ZZ\r\n";
    f.serial->rx_data.insert(f.serial->rx_data.end(), bad.begin(), bad.end());

    f.gps.open();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto diag = f.gps.get_diagnostics();
    f.gps.close();

    // 解析了并失败，应有错误计数（valid=false）
    REQUIRE_FALSE(diag.valid);
}

// ────────────────────────────────────────────────────────────────
// get_diagnostics() 包含 sentence_count
// ────────────────────────────────────────────────────────────────
TEST_CASE("GpsDevice: 处理完句子后 sentence_count > 0", "[device][gps]") {
    GpsFixture f;

    std::string gga =
        nmea_with_cs("$GPGGA,120000.00,3032.0000,N,11414.0000,E,1,06,1.5,50.0,M,,M,,");
    f.serial->rx_data.insert(f.serial->rx_data.end(), gga.begin(), gga.end());

    f.gps.open();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto diag = f.gps.get_diagnostics();
    f.gps.close();

    REQUIRE(diag.sentence_count > 0);
}
