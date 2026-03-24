#include "pv_cleaning_robot/protocol/nmea_parser.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <sstream>

namespace robot::protocol {

void NmeaParser::parse_sentence(const std::string& sentence) {
    if (sentence.empty() || sentence[0] != '$') return;
    if (!validate_checksum(sentence)) return;

    // 提取句子类型（去掉 $Gx 前缀后的3字符，如 GGA/RMC/GSA/GSV）
    if (sentence.size() < 6) return;
    std::string type = sentence.substr(3, 3);  // e.g. "GGA"

    if      (type == "GGA") parse_gga(sentence);
    else if (type == "RMC") parse_rmc(sentence);
    else if (type == "GSA") parse_gsa(sentence);
    else if (type == "GSV") parse_gsv(sentence);
}

void NmeaParser::reset() { data_ = GpsData{}; }

// ── 内部辅助 ─────────────────────────────────────────────────────────────────

std::vector<std::string> NmeaParser::split(const std::string& s, char delim) {
    std::vector<std::string> result;
    std::string token;
    for (char c : s) {
        if (c == delim) {
            result.push_back(token);
            token.clear();
        } else if (c != '\r' && c != '\n' && c != '*') {
            token += c;
        } else if (c == '*') {
            result.push_back(token);
            break;
        }
    }
    if (!token.empty()) result.push_back(token);
    return result;
}

bool NmeaParser::validate_checksum(const std::string& sentence) {
    // $....*XX 格式验证
    auto star = sentence.rfind('*');
    if (star == std::string::npos || star + 3 > sentence.size()) return false;

    uint8_t expected = 0;
    for (size_t i = 1; i < star; ++i)
        expected ^= static_cast<uint8_t>(sentence[i]);

    std::string hex = sentence.substr(star + 1, 2);
    uint8_t actual  = static_cast<uint8_t>(std::stoul(hex, nullptr, 16));
    return expected == actual;
}

double NmeaParser::parse_nmea_coord(const std::string& raw,
                                    const std::string& hemi) {
    if (raw.empty()) return 0.0;
    // NMEA 格式：DDDMM.MMMM 或 DDMM.MMMM
    auto dot = raw.find('.');
    if (dot == std::string::npos || dot < 2) return 0.0;

    int    deg_digits = static_cast<int>(dot) - 2;  // 2位分钟整数
    double degrees    = std::stod(raw.substr(0, deg_digits));
    double minutes    = std::stod(raw.substr(deg_digits));
    double result     = degrees + minutes / 60.0;
    if (hemi == "S" || hemi == "W") result = -result;
    return result;
}

bool NmeaParser::parse_gga(const std::string& s) {
    auto f = split(s, ',');
    // $GxGGA,hhmmss.ss,ddmm.mm,N,dddmm.mm,E,q,nn,h.h,a.a,M,g.g,M,D.D,rrrr*hh
    if (f.size() < 12) return false;

    data_.latitude       = parse_nmea_coord(f[2], f[3]);
    data_.longitude      = parse_nmea_coord(f[4], f[5]);
    data_.fix_quality    = f[6].empty() ? 0 : static_cast<uint8_t>(std::stoi(f[6]));
    data_.satellites_used= f[7].empty() ? 0 : static_cast<uint8_t>(std::stoi(f[7]));
    data_.hdop           = f[8].empty() ? 0.f : std::stof(f[8]);
    data_.altitude_m     = f[9].empty() ? 0.f : std::stof(f[9]);
    data_.valid          = (data_.fix_quality > 0);
    return true;
}

bool NmeaParser::parse_rmc(const std::string& s) {
    auto f = split(s, ',');
    // $GxRMC,hhmmss.ss,A,ddmm.mm,N,dddmm.mm,E,n.n,x.x,ddmmyy,,,A*hh
    if (f.size() < 9) return false;
    if (f[2] != "A") { data_.valid = false; return false; }  // V=无效

    data_.latitude  = parse_nmea_coord(f[3], f[4]);
    data_.longitude = parse_nmea_coord(f[5], f[6]);
    data_.speed_m_s = f[7].empty() ? 0.f : std::stof(f[7]) * 0.5144f;  // knots→m/s
    data_.course_deg= f[8].empty() ? 0.f : std::stof(f[8]);
    data_.valid     = true;
    return true;
}

bool NmeaParser::parse_gsa(const std::string& s) {
    auto f = split(s, ',');
    // $GxGSA,A,3,sv1..sv12,PDOP,HDOP,VDOP*hh
    if (f.size() < 18) return false;
    data_.pdop = f[15].empty() ? 0.f : std::stof(f[15]);
    data_.hdop = f[16].empty() ? 0.f : std::stof(f[16]);
    data_.vdop = f[17].empty() ? 0.f : std::stof(f[17]);
    return true;
}

bool NmeaParser::parse_gsv(const std::string& s) {
    auto f = split(s, ',');
    // $GxGSV,nmsg,msg,nsv,...
    if (f.size() < 4) return false;
    // 只从第一条 GSV 中取卫星总数
    if (f[2] == "1" && !f[3].empty())
        data_.satellites_in_view = static_cast<uint8_t>(std::stoi(f[3]));
    return true;
}

}  // namespace robot::protocol
