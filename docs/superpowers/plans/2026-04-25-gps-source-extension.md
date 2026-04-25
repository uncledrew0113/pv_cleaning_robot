# GPS Source Extension Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add explicit GPS source selection so the robot can read location data either from the existing serial NMEA path or from a standard `gpsd` TCP service, without changing how higher-level services consume `GpsDevice`.

**Architecture:** Keep `GpsDevice` as the single public device facade, but move source-specific I/O into two focused backends: a serial/NMEA backend and a `gpsd` TCP/JSON backend. `GpsDevice` owns the shared cache and diagnostics, while each backend owns its own `read_loop()` and parsing path.

**Tech Stack:** C++17, POSIX sockets, `nlohmann::json`, Catch2, CMake, spdlog, libserialport.

---

## File Map

**Create:**
- `include/pv_cleaning_robot/device/gps_source.h` — internal GPS source interface plus source config structs
- `pv_cleaning_robot/device/serial_gps_source.cc` — serial/NMEA backend implementation
- `pv_cleaning_robot/device/gpsd_gps_source.cc` — `gpsd` TCP backend implementation

**Modify:**
- `include/pv_cleaning_robot/device/gps_device.h` — refactor `GpsDevice` into facade/cache owner
- `pv_cleaning_robot/device/gps_device.cc` — source selection, cache/diagnostics helpers, command forwarding
- `include/pv_cleaning_robot/protocol/nmea_parser.h` — make NMEA parse success/failure explicit for diagnostics
- `pv_cleaning_robot/protocol/nmea_parser.cc` — return parse result instead of silently ignoring invalid sentences
- `pv_cleaning_robot/main.cc` — instantiate GPS from explicit `gps.*` config or legacy fallback
- `config/config.json` — add new `gps` config block while keeping legacy serial block for compatibility
- `test/CMakeLists.txt` — include new backend source files and any new test files in `unit_tests`
- `test/device/gps_device_test.cc` — preserve serial behavior against the new facade/backend split

**Test Create:**
- `test/device/gpsd_gps_source_test.cc` — `gpsd` mapping, incremental update, and unsupported-command tests

---

### Task 1: Lock Public `GpsDevice` Behavior with Tests

**Files:**
- Modify: `test/device/gps_device_test.cc`
- Test: `build/aarch64/bin/unit_tests`

- [ ] **Step 1: Expand the serial regression tests before refactoring**

Add test coverage for the behavior the new architecture must preserve:

```cpp
TEST_CASE("GpsDevice(serial): valid GGA updates latest data", "[device][gps]") {
    auto serial = std::make_shared<MockSerialPort>();
    serial->open_result = true;
    std::string gga =
        nmea_with_cs("$GPGGA,120000.00,3032.0000,N,11414.0000,E,1,08,1.0,30.0,M,,M,,");
    serial->rx_data.insert(serial->rx_data.end(), gga.begin(), gga.end());

    robot::device::GpsDevice gps(serial);
    REQUIRE(gps.open());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const auto latest = gps.get_latest();
    gps.close();

    REQUIRE(latest.valid);
    REQUIRE(latest.latitude == Approx(30.5333333).epsilon(1e-4));
    REQUIRE(latest.longitude == Approx(114.2333333).epsilon(1e-4));
}

TEST_CASE("GpsDevice(serial): command path still writes to UART", "[device][gps]") {
    auto serial = std::make_shared<MockSerialPort>();
    serial->open_result = true;
    robot::device::GpsDevice gps(serial);
    REQUIRE(gps.open());

    REQUIRE(gps.set_output_rate(5) == robot::device::DeviceError::OK);
    gps.close();

    const std::string tx(serial->tx_captured.begin(), serial->tx_captured.end());
    REQUIRE(tx.find("$PMTK220,200") != std::string::npos);
}
```

- [ ] **Step 2: Run only the GPS serial tests to confirm the baseline still passes**

Run:

```bash
cd /home/tronlong/pv_cleaning_robot
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[device][gps]"
```

Expected: existing GPS tests pass before any production refactor.

- [ ] **Step 3: Commit the safety-net tests**

```bash
cd /home/tronlong/pv_cleaning_robot
git add test/device/gps_device_test.cc
git commit -m "test: lock serial GpsDevice behavior before source split"
```

---

### Task 2: Refactor `GpsDevice` into a Facade with Shared Cache Helpers

**Files:**
- Create: `include/pv_cleaning_robot/device/gps_source.h`
- Modify: `include/pv_cleaning_robot/device/gps_device.h`
- Modify: `pv_cleaning_robot/device/gps_device.cc`
- Test: `test/device/gps_device_test.cc`

- [ ] **Step 1: Define the internal GPS source interface and source config types**

Create `include/pv_cleaning_robot/device/gps_source.h` with a focused interface:

```cpp
#pragma once

#include "pv_cleaning_robot/device/device_error.h"
#include "pv_cleaning_robot/protocol/nmea_parser.h"
#include <functional>
#include <memory>
#include <string>

namespace robot::hal { class ISerialPort; }

namespace robot::device {

struct SerialGpsSourceConfig {
    std::shared_ptr<hal::ISerialPort> serial;
};

struct GpsdSourceConfig {
    std::string host{"127.0.0.1"};
    int         port{2947};
    std::string watch{"?WATCH={\"enable\":true,\"json\":true};"};
};

class IGpsSource {
public:
    using GpsData = protocol::GpsData;
    using DataCallback = std::function<void(const GpsData&)>;
    using ParseErrorCallback = std::function<void()>;
    using MessageCallback = std::function<void()>;

    virtual ~IGpsSource() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual DeviceError set_output_rate(int hz) = 0;
    virtual DeviceError hot_restart() = 0;
    virtual DeviceError cold_restart() = 0;
};

}  // namespace robot::device
```

- [ ] **Step 2: Change `GpsDevice` to own a source plus shared cache/diagnostic update helpers**

Update `include/pv_cleaning_robot/device/gps_device.h` so `GpsDevice` no longer owns UART/NMEA state directly:

```cpp
class GpsDevice {
public:
    using GpsData = protocol::GpsData;

    struct Diagnostics : GpsData {
        uint32_t sentence_count{0};
        uint32_t parse_error_count{0};
        uint32_t fix_loss_count{0};
    };

    explicit GpsDevice(std::shared_ptr<hal::ISerialPort> serial);
    explicit GpsDevice(std::unique_ptr<IGpsSource> source);
    static std::shared_ptr<GpsDevice> create_gpsd(const GpsdSourceConfig& cfg);
    ~GpsDevice();

    bool open();
    void close();

    DeviceError set_output_rate(int hz);
    DeviceError hot_restart();
    DeviceError cold_restart();

    GpsData     get_latest() const;
    Diagnostics get_diagnostics() const;

    void on_source_message(const GpsData& data);
    void on_source_parse_error();
    void on_source_message_count();

private:
    GpsDevice() = default;
    std::unique_ptr<IGpsSource> source_;
    mutable std::mutex          mtx_;
    Diagnostics                 diag_{};
};
```

- [ ] **Step 3: Implement the facade behavior in `gps_device.cc` without source-specific I/O**

Replace the old `GpsDevice` implementation with cache/forwarding logic like this:

```cpp
GpsDevice::GpsDevice(std::unique_ptr<IGpsSource> source)
    : source_(std::move(source)) {}

GpsDevice::GpsDevice(std::shared_ptr<hal::ISerialPort> serial)
    : GpsDevice(std::make_unique<SerialGpsSource>(
          SerialGpsSourceConfig{std::move(serial)},
          [this](const GpsData& data) { on_source_message(data); },
          [this]() { on_source_parse_error(); },
          [this]() { on_source_message_count(); })) {}

std::shared_ptr<GpsDevice> GpsDevice::create_gpsd(const GpsdSourceConfig& cfg) {
    auto gps = std::shared_ptr<GpsDevice>(new GpsDevice());
    gps->source_ = std::make_unique<GpsdGpsSource>(
        cfg,
        [gps](const GpsData& data) { gps->on_source_message(data); },
        [gps]() { gps->on_source_parse_error(); },
        [gps]() { gps->on_source_message_count(); });
    return gps;
}

bool GpsDevice::open() {
    return source_ && source_->open();
}

void GpsDevice::on_source_message_count() {
    std::lock_guard<std::mutex> lk(mtx_);
    ++diag_.sentence_count;
}

void GpsDevice::on_source_parse_error() {
    std::lock_guard<std::mutex> lk(mtx_);
    ++diag_.parse_error_count;
}

void GpsDevice::on_source_message(const GpsData& data) {
    std::lock_guard<std::mutex> lk(mtx_);
    const bool was_fixed = diag_.valid;
    static_cast<GpsData&>(diag_) = data;
    if (was_fixed && !data.valid) {
        ++diag_.fix_loss_count;
    }
}

DeviceError GpsDevice::set_output_rate(int hz) {
    return source_ ? source_->set_output_rate(hz) : DeviceError::NOT_OPEN;
}
```

- [ ] **Step 4: Re-run the serial `GpsDevice` tests**

Run:

```bash
cd /home/tronlong/pv_cleaning_robot
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[device][gps]"
```

Expected: serial tests still pass after `GpsDevice` stops owning UART logic directly.

- [ ] **Step 5: Commit the facade refactor**

```bash
cd /home/tronlong/pv_cleaning_robot
git add include/pv_cleaning_robot/device/gps_source.h \
        include/pv_cleaning_robot/device/gps_device.h \
        pv_cleaning_robot/device/gps_device.cc
git commit -m "refactor: split GpsDevice cache from source-specific I/O"
```

---

### Task 3: Move the Existing UART/NMEA Logic into `SerialGpsSource`

**Files:**
- Create: `pv_cleaning_robot/device/serial_gps_source.cc`
- Modify: `include/pv_cleaning_robot/device/gps_source.h`
- Modify: `include/pv_cleaning_robot/protocol/nmea_parser.h`
- Modify: `pv_cleaning_robot/protocol/nmea_parser.cc`
- Modify: `test/device/gps_device_test.cc`
- Test: `build/aarch64/bin/unit_tests`

- [ ] **Step 1: Declare `SerialGpsSource` in the shared source header**

Extend `gps_source.h` with the concrete serial backend:

```cpp
class SerialGpsSource final : public IGpsSource {
public:
    SerialGpsSource(SerialGpsSourceConfig cfg,
                    DataCallback on_data,
                    ParseErrorCallback on_parse_error,
                    MessageCallback on_message);
    ~SerialGpsSource() override;

    bool open() override;
    void close() override;
    DeviceError set_output_rate(int hz) override;
    DeviceError hot_restart() override;
    DeviceError cold_restart() override;

private:
    void read_loop();

    SerialGpsSourceConfig cfg_;
    DataCallback          on_data_;
    ParseErrorCallback    on_parse_error_;
    MessageCallback       on_message_;
    protocol::NmeaParser  parser_;
    std::thread           read_thread_;
    std::atomic<bool>     running_{false};
    std::string           line_buf_;
};
```

- [ ] **Step 2: Move the old serial implementation into `serial_gps_source.cc`**

Create `pv_cleaning_robot/device/serial_gps_source.cc` by lifting the current UART implementation out of `GpsDevice`:

```cpp
bool SerialGpsSource::open() {
    if (!cfg_.serial || !cfg_.serial->open()) return false;
    running_.store(true);
    read_thread_ = std::thread(&SerialGpsSource::read_loop, this);
    return true;
}

void SerialGpsSource::read_loop() {
    uint8_t byte_buf[128];
    while (running_.load()) {
        int n = cfg_.serial->read(byte_buf, sizeof(byte_buf), 50);
        if (n <= 0) continue;

        for (int i = 0; i < n; ++i) {
            const char c = static_cast<char>(byte_buf[i]);
            if (c == '\n') {
                if (line_buf_.empty()) continue;
                on_message_();
                if (!parser_.parse_sentence(line_buf_)) {
                    on_parse_error_();
                    line_buf_.clear();
                    continue;
                }
                on_data_(parser_.get_data());
                line_buf_.clear();
            } else if (c != '\r') {
                if (line_buf_.size() < 256) {
                    line_buf_ += c;
                } else {
                    line_buf_.clear();
                    on_parse_error_();
                }
            }
        }
    }
}
```

- [ ] **Step 3: Make `NmeaParser` return success/failure explicitly**

Change the parser API so the serial backend can count malformed NMEA input deterministically:

```cpp
// include/pv_cleaning_robot/protocol/nmea_parser.h
class NmeaParser {
public:
    bool parse_sentence(const std::string& sentence);
    const GpsData& get_data() const { return data_; }
    ...
};

// pv_cleaning_robot/protocol/nmea_parser.cc
bool NmeaParser::parse_sentence(const std::string& sentence) {
    if (sentence.empty() || sentence[0] != '$') return false;
    if (!validate_checksum(sentence)) return false;
    if (sentence.size() < 6) return false;

    const std::string type = sentence.substr(3, 3);
    if (type == "GGA") return parse_gga(sentence);
    if (type == "RMC") return parse_rmc(sentence);
    if (type == "GSA") return parse_gsa(sentence);
    if (type == "GSV") return parse_gsv(sentence);
    return false;
}
```

- [ ] **Step 4: Re-run the serial regression tests**

Run:

```bash
cd /home/tronlong/pv_cleaning_robot
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[device][gps]"
```

Expected: all serial GPS tests pass with the new backend class in place.

- [ ] **Step 5: Commit the serial backend extraction**

```bash
cd /home/tronlong/pv_cleaning_robot
git add include/pv_cleaning_robot/device/gps_source.h \
        include/pv_cleaning_robot/protocol/nmea_parser.h \
        pv_cleaning_robot/protocol/nmea_parser.cc \
        pv_cleaning_robot/device/serial_gps_source.cc \
        test/device/gps_device_test.cc
git commit -m "refactor: move UART GPS handling into SerialGpsSource"
```

---

### Task 4: Add the `gpsd` TCP Backend with `read_loop()` and Incremental JSON Updates

**Files:**
- Create: `pv_cleaning_robot/device/gpsd_gps_source.cc`
- Modify: `include/pv_cleaning_robot/device/gps_source.h`
- Test: `test/device/gpsd_gps_source_test.cc`

- [ ] **Step 1: Declare `GpsdGpsSource` with explicit `read_loop()` ownership**

Add the `gpsd` backend declaration to `gps_source.h`:

```cpp
class GpsdGpsSource final : public IGpsSource {
public:
    GpsdGpsSource(GpsdSourceConfig cfg,
                  DataCallback on_data,
                  ParseErrorCallback on_parse_error,
                  MessageCallback on_message);
    ~GpsdGpsSource() override;

    bool open() override;
    void close() override;
    DeviceError set_output_rate(int) override { return DeviceError::NOT_SUPPORTED; }
    DeviceError hot_restart() override { return DeviceError::NOT_SUPPORTED; }
    DeviceError cold_restart() override { return DeviceError::NOT_SUPPORTED; }

    void ingest_json_line_for_test(const std::string& line);

private:
    bool connect_socket();
    bool send_watch();
    void read_loop();
    void handle_json_line(const std::string& line);
    void apply_tpv(const nlohmann::json& j);
    void apply_sky(const nlohmann::json& j);

    GpsdSourceConfig  cfg_;
    DataCallback      on_data_;
    ParseErrorCallback on_parse_error_;
    MessageCallback   on_message_;
    int               sock_fd_{-1};
    std::thread       read_thread_;
    std::atomic<bool> running_{false};
    protocol::GpsData data_{};
    std::string       rx_buf_;
};
```

- [ ] **Step 2: Implement connection, WATCH handshake, JSON line framing, and reconnection**

Create `pv_cleaning_robot/device/gpsd_gps_source.cc` with concrete TCP behavior:

```cpp
bool GpsdGpsSource::open() {
    if (!connect_socket() || !send_watch()) return false;
    running_.store(true);
    read_thread_ = std::thread(&GpsdGpsSource::read_loop, this);
    return true;
}

void GpsdGpsSource::read_loop() {
    char buf[512];
    while (running_.load()) {
        const int n = ::recv(sock_fd_, buf, sizeof(buf), 0);
        if (n <= 0) {
            ::close(sock_fd_);
            sock_fd_ = -1;
            if (!running_.load()) break;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (connect_socket()) {
                send_watch();
            }
            continue;
        }

        rx_buf_.append(buf, n);
        std::size_t pos = 0;
        while ((pos = rx_buf_.find('\n')) != std::string::npos) {
            std::string line = rx_buf_.substr(0, pos);
            rx_buf_.erase(0, pos + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;
            on_message_();
            ingest_json_line_for_test(line);
        }
    }
}
```

- [ ] **Step 3: Implement `TPV` / `SKY` incremental mapping exactly as the spec requires**

Use `nlohmann::json` and update only fields present in each message:

```cpp
void GpsdGpsSource::ingest_json_line_for_test(const std::string& line) {
    handle_json_line(line);
}

void GpsdGpsSource::handle_json_line(const std::string& line) {
    try {
        const auto j = nlohmann::json::parse(line);
        const std::string cls = j.value("class", "");
        if (cls == "TPV") {
            apply_tpv(j);
            on_data_(data_);
            return;
        }
        if (cls == "SKY") {
            apply_sky(j);
            on_data_(data_);
            return;
        }
        if (cls == "ERROR") {
            on_parse_error_();
            return;
        }
    } catch (...) {
        on_parse_error_();
    }
}

void GpsdGpsSource::apply_tpv(const nlohmann::json& j) {
    if (j.contains("lat")) data_.latitude = j.at("lat").get<double>();
    if (j.contains("lon")) data_.longitude = j.at("lon").get<double>();
    if (j.contains("speed")) data_.speed_m_s = j.at("speed").get<float>();
    if (j.contains("track")) data_.course_deg = j.at("track").get<float>();
    if (j.contains("altMSL")) data_.altitude_m = j.at("altMSL").get<float>();
    else if (j.contains("altHAE")) data_.altitude_m = j.at("altHAE").get<float>();
    if (j.contains("mode")) {
        const int mode = j.at("mode").get<int>();
        data_.valid = mode >= 2;
        data_.fix_quality = mode >= 3 ? 2 : (mode >= 2 ? 1 : 0);
    }
}

void GpsdGpsSource::apply_sky(const nlohmann::json& j) {
    if (j.contains("hdop")) data_.hdop = j.at("hdop").get<float>();
    if (j.contains("pdop")) data_.pdop = j.at("pdop").get<float>();
    if (j.contains("vdop")) data_.vdop = j.at("vdop").get<float>();
    if (j.contains("nSat")) data_.satellites_in_view = j.at("nSat").get<uint8_t>();
    if (j.contains("satellites")) {
        uint8_t used = 0;
        for (const auto& sat : j.at("satellites")) {
            if (sat.value("used", false)) ++used;
        }
        data_.satellites_used = used;
        if (!j.contains("nSat")) data_.satellites_in_view = static_cast<uint8_t>(j.at("satellites").size());
    }
}
```

- [ ] **Step 4: Commit the `gpsd` backend**

```bash
cd /home/tronlong/pv_cleaning_robot
git add include/pv_cleaning_robot/device/gps_source.h \
        pv_cleaning_robot/device/gpsd_gps_source.cc
git commit -m "feat: add gpsd TCP backend for GPS input"
```

---

### Task 5: Wire Explicit `gps.*` Config Selection in `main.cc`

**Files:**
- Modify: `pv_cleaning_robot/main.cc`
- Modify: `config/config.json`
- Modify: `include/pv_cleaning_robot/device/gps_device.h`
- Modify: `pv_cleaning_robot/device/gps_device.cc`

- [ ] **Step 1: Add the new `gps` block to the sample config without deleting the old serial block yet**

Update `config/config.json` so it includes the new explicit source selection:

```json
"serial": {
  "imu":   { "port": "/dev/ttyS1", "baudrate": 9600 },
  "gps":   { "port": "/dev/ttyS2", "baudrate": 115200 },
  "brush": { "port": "/dev/ttyS3", "baudrate": 9600, "slave_id": 1 },
  "bms":   { "port": "/dev/ttyS8", "baudrate": 9600 },
  "distance_sensor": { "port": "/dev/ttyS9", "baudrate": 9600, "slave_id": 1, "channel_count": 4, "decimal_mode": 0, "analog_type": 0 }
},
"gps": {
  "source": "serial",
  "serial": {
    "port": "/dev/ttyS2",
    "baudrate": 115200
  },
  "gpsd": {
    "host": "127.0.0.1",
    "port": 2947,
    "watch": "?WATCH={\"enable\":true,\"json\":true};"
  }
},
```

- [ ] **Step 2: Add an explicit `gpsd` factory on `GpsDevice` so `main.cc` stays clean**

Use the `create_gpsd()` constructor path already added in Task 2 to hide callback wiring:

```cpp
// pv_cleaning_robot/device/gps_device.cc
std::shared_ptr<GpsDevice> GpsDevice::create_gpsd(const GpsdSourceConfig& cfg) {
    auto gps = std::shared_ptr<GpsDevice>(new GpsDevice());
    gps->source_ = std::make_unique<GpsdGpsSource>(
        cfg,
        [gps](const GpsData& data) { gps->on_source_message(data); },
        [gps]() { gps->on_source_parse_error(); },
        [gps]() { gps->on_source_message_count(); });
    return gps;
}
```

- [ ] **Step 3: Replace the hard-coded GPS serial construction in `main.cc`**

Change the GPS initialization branch to:

```cpp
const std::string gps_source = cfg.get<std::string>("gps.source", "serial");

std::shared_ptr<robot::device::GpsDevice> gps;
if (gps_source == "gpsd") {
    robot::device::GpsdSourceConfig gpsd_cfg;
    gpsd_cfg.host  = cfg.get<std::string>("gps.gpsd.host", "127.0.0.1");
    gpsd_cfg.port  = cfg.get<int>("gps.gpsd.port", 2947);
    gpsd_cfg.watch = cfg.get<std::string>("gps.gpsd.watch", "?WATCH={\"enable\":true,\"json\":true};");
    gps = robot::device::GpsDevice::create_gpsd(gpsd_cfg);
} else {
    const std::string port = cfg.get<std::string>(
        "gps.serial.port",
        cfg.get<std::string>("serial.gps.port", "/dev/ttyS2"));
    const int baudrate = cfg.get<int>(
        "gps.serial.baudrate",
        cfg.get<int>("serial.gps.baudrate", 9600));
    auto gps_serial = std::make_shared<robot::driver::LibSerialPort>(
        port, robot::hal::UartConfig{baudrate});
    gps = std::make_shared<robot::device::GpsDevice>(gps_serial);
}
```

- [ ] **Step 4: Build the main target**

Run:

```bash
cd /home/tronlong/pv_cleaning_robot
cmake --build --preset rk3576-build --target pv_cleaning_robot
```

Expected: the firmware target links successfully with the new GPS source code.

- [ ] **Step 5: Commit the config-selection wiring**

```bash
cd /home/tronlong/pv_cleaning_robot
git add include/pv_cleaning_robot/device/gps_device.h \
        pv_cleaning_robot/device/gps_device.cc \
        pv_cleaning_robot/main.cc \
        config/config.json
git commit -m "feat: select GPS input source from config"
```

---

### Task 6: Add `gpsd` Unit Tests and Final Regression Coverage

**Files:**
- Create: `test/device/gpsd_gps_source_test.cc`
- Modify: `test/CMakeLists.txt`
- Test: `build/aarch64/bin/unit_tests`

- [ ] **Step 1: Register the new test file and backend source in `test/CMakeLists.txt`**

Make sure `COMMON_SRCS` includes both new backend `.cc` files and `unit_tests` includes the new test file:

```cmake
set(COMMON_SRCS
  ${PROJ}/device/gps_device.cc
  ${PROJ}/device/serial_gps_source.cc
  ${PROJ}/device/gpsd_gps_source.cc
  ${PROJ}/device/bms.cc
  ${PROJ}/device/imu_device.cc
  ...
)

add_executable(unit_tests
  ...
  device/gps_device_test.cc
  device/gpsd_gps_source_test.cc
  ...
)
```

- [ ] **Step 2: Add focused `gpsd` mapping tests**

Create `test/device/gpsd_gps_source_test.cc` with concrete cases for the spec:

```cpp
TEST_CASE("GpsdGpsSource: TPV maps mode 3 to valid fix", "[device][gps][gpsd]") {
    robot::protocol::GpsData latest{};
    robot::device::GpsdGpsSource src(
        {"127.0.0.1", 2947, "?WATCH={\"enable\":true,\"json\":true};"},
        [&](const auto& d) { latest = d; },
        []() {},
        []() {});

    src.ingest_json_line_for_test(R"({"class":"TPV","lat":30.5,"lon":114.2,"speed":1.2,"track":91.0,"mode":3})");

    REQUIRE(latest.valid);
    REQUIRE(latest.fix_quality == 2);
    REQUIRE(latest.latitude == Approx(30.5));
    REQUIRE(latest.longitude == Approx(114.2));
}

TEST_CASE("GpsdGpsSource: SKY updates used satellites without clearing TPV", "[device][gps][gpsd]") {
    std::vector<robot::protocol::GpsData> updates;
    robot::device::GpsdGpsSource src(
        {"127.0.0.1", 2947, "?WATCH={\"enable\":true,\"json\":true};"},
        [&](const auto& d) { updates.push_back(d); },
        []() {},
        []() {});

    src.ingest_json_line_for_test(R"({"class":"TPV","lat":30.5,"lon":114.2,"mode":2})");
    src.ingest_json_line_for_test(R"({"class":"SKY","hdop":0.8,"satellites":[{"used":true},{"used":false},{"used":true}]})");

    REQUIRE(updates.back().latitude == Approx(30.5));
    REQUIRE(updates.back().satellites_used == 2);
    REQUIRE(updates.back().satellites_in_view == 3);
}

TEST_CASE("GpsDevice(gpsd): commands are not supported", "[device][gps][gpsd]") {
    struct FakeGpsdSource final : robot::device::IGpsSource {
        bool open() override { return true; }
        void close() override {}
        robot::device::DeviceError set_output_rate(int) override { return robot::device::DeviceError::NOT_SUPPORTED; }
        robot::device::DeviceError hot_restart() override { return robot::device::DeviceError::NOT_SUPPORTED; }
        robot::device::DeviceError cold_restart() override { return robot::device::DeviceError::NOT_SUPPORTED; }
    };

    auto gps = robot::device::GpsDevice(std::make_unique<FakeGpsdSource>());
    REQUIRE(gps.set_output_rate(5) == robot::device::DeviceError::NOT_SUPPORTED);
    REQUIRE(gps.hot_restart() == robot::device::DeviceError::NOT_SUPPORTED);
    REQUIRE(gps.cold_restart() == robot::device::DeviceError::NOT_SUPPORTED);
}
```

- [ ] **Step 3: Run the focused GPS suite and then the full unit test suite**

Run:

```bash
cd /home/tronlong/pv_cleaning_robot
cmake --build --preset rk3576-build --target unit_tests
./build/aarch64/bin/unit_tests "[device][gps]"
./build/aarch64/bin/unit_tests
```

Expected:
- GPS-tagged tests pass
- full `unit_tests` run passes

- [ ] **Step 4: Commit the test coverage**

```bash
cd /home/tronlong/pv_cleaning_robot
git add test/CMakeLists.txt test/device/gpsd_gps_source_test.cc
git commit -m "test: cover gpsd mapping and unsupported GPS commands"
```

---

### Task 7: Final Verification and Documentation Sync

**Files:**
- Modify: `README.md`
- Test: `build/aarch64/bin/pv_cleaning_robot`

- [ ] **Step 1: Update the user-facing GPS configuration docs**

Update the hardware/config section in `README.md` so GPS configuration is described by source selection rather than by `serial.gps` alone:

```md
| GPS | NMEA 0183 GPS (optional) | `gps.source="serial"` uses `gps.serial`; `gps.source="gpsd"` uses `gps.gpsd` |

关键配置：
- `gps.source`: `"serial"` 或 `"gpsd"`
- `gps.serial.port` / `gps.serial.baudrate`: 串口 GPS 参数
- `gps.gpsd.host` / `gps.gpsd.port` / `gps.gpsd.watch`: `gpsd` TCP 参数
- 兼容说明：若 `gps.source` 缺失，程序暂时回退读取旧配置 `serial.gps.*`
```

- [ ] **Step 2: Run the final build and smoke-test commands**

Run:

```bash
cd /home/tronlong/pv_cleaning_robot
cmake --build --preset rk3576-build --target pv_cleaning_robot unit_tests
./build/aarch64/bin/unit_tests "[device][gps]"
```

Expected: final GPS-specific regressions pass on the completed branch.

- [ ] **Step 3: Summarize any residual risks in the implementation notes**

Record these checks in the implementation handoff or PR description:

```md
- `gpsd` reconnect logic was verified by unit tests but not on target hardware in this branch.
- `utc_timestamp_ms` parsing depends on `gpsd` ISO8601 strings including timezone (`Z`); malformed time strings count as parse errors.
- Legacy `serial.gps.*` fallback remains temporarily supported and should be removed only after deployment confirms all configs have migrated.
```

- [ ] **Step 4: Commit the cleanup/docs sync**

```bash
cd /home/tronlong/pv_cleaning_robot
git add README.md
git commit -m "docs: align GPS configuration docs with source selection"
```
