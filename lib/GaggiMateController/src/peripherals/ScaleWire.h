// Wire format of the scale-board serial link (fork). Shared word for word by the Pro firmware
// (ScaleLink) and the Waveshare scale-board sketch, so both sides agree on every field.
//
// Same framing as the meter link (LineProtocol.h): "$<payload>*XX\r\n", 115200 8N1.
//
//   scale board -> Pro
//     $W,<grams>,<cell1_g>,<cell2_g>,<valid>,<seq>*XX   about 10 per second, one per published reading
//         valid: bit 0 = cell 1 good, bit 1 = cell 2 good. 0 with grams = -9999 means "no scale"
//         (HX711 missing or faulted), the same sentinel the stock HardwareScale sends.
//     $H,<uptime_s>*XX                                   1 per second
//   Pro -> scale board
//     $Z*XX                                              tare (display hold, shot start, shot saved, web tare)
//     $B,<0|1>*XX                                        brew valve open or pump running: pauses idle zero
//                                                        tracking and enables the brew-time outlier filter
//     $C,<factor1>,<factor2>,<sps>,<idle_alpha>,<active_alpha>*XX
//                                                        hardware-scale settings from the display
//     $H,<uptime_s>*XX                                   1 per second
//
// STL-only on purpose: the native unit tests include this header directly.

#ifndef SCALEWIRE_H
#define SCALEWIRE_H

#include "LineProtocol.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace scalewire {

constexpr float UNAVAILABLE_GRAMS = -9999.0f; // HARDWARE_SCALE_UNAVAILABLE in the stock firmware
constexpr float MAX_GRAMS = 5000.0f;          // anything beyond is a broken line, not a cup

struct Reading {
    float grams = UNAVAILABLE_GRAMS;
    float cell1 = 0.0f;
    float cell2 = 0.0f;
    bool cell1Valid = false;
    bool cell2Valid = false;
    uint32_t seq = 0;
    bool available() const { return cell1Valid && cell2Valid; }
};

struct Config {
    float factor1 = -2500.0f; // stock HardwareScale defaults
    float factor2 = 2500.0f;
    uint16_t sps = 10;
    float idleAlpha = 0.80f;
    float activeAlpha = 0.80f;
};

// $W -> Reading. False for anything that is not a well-formed weight line.
inline bool parseWeight(const lineproto::Message &m, Reading &out) {
    if (m.type[0] != 'W' || m.type[1] != '\0' || m.count < 5)
        return false;
    const float grams = m.f(0, NAN);
    const float c1 = m.f(1, NAN);
    const float c2 = m.f(2, NAN);
    const long valid = m.i(3, -1);
    if (!std::isfinite(grams) || valid < 0 || valid > 3)
        return false;
    Reading r;
    r.cell1Valid = (valid & 1) != 0;
    r.cell2Valid = (valid & 2) != 0;
    r.seq = static_cast<uint32_t>(m.i(4, 0));
    if (r.available()) {
        if (!std::isfinite(c1) || !std::isfinite(c2) || std::fabs(grams) > MAX_GRAMS)
            return false;
        r.grams = grams;
        r.cell1 = c1;
        r.cell2 = c2;
    } else {
        // Not both cells: report "no scale" exactly like the stock code does (sentinel, zero cells, flags false).
        r.grams = UNAVAILABLE_GRAMS;
        r.cell1Valid = r.cell2Valid = false;
    }
    out = r;
    return true;
}

// Reading -> "$W" payload (without framing). Returns the snprintf length.
inline int formatWeight(const Reading &r, char *out, size_t size) {
    const int valid = (r.cell1Valid ? 1 : 0) | (r.cell2Valid ? 2 : 0);
    if (valid != 3)
        return snprintf(out, size, "W,-9999,0,0,0,%lu", static_cast<unsigned long>(r.seq));
    return snprintf(out, size, "W,%.2f,%.2f,%.2f,%d,%lu", r.grams, r.cell1, r.cell2, valid, static_cast<unsigned long>(r.seq));
}

// $C -> Config. False if a field is missing or not a number; range checks are the scale's job.
inline bool parseConfig(const lineproto::Message &m, Config &out) {
    if (m.type[0] != 'C' || m.type[1] != '\0' || m.count < 5)
        return false;
    Config c;
    c.factor1 = m.f(0, NAN);
    c.factor2 = m.f(1, NAN);
    const long sps = m.i(2, -1);
    c.idleAlpha = m.f(3, NAN);
    c.activeAlpha = m.f(4, NAN);
    if (!std::isfinite(c.factor1) || !std::isfinite(c.factor2) || !std::isfinite(c.idleAlpha) ||
        !std::isfinite(c.activeAlpha) || sps <= 0 || sps > 1000)
        return false;
    c.sps = static_cast<uint16_t>(sps);
    out = c;
    return true;
}

inline int formatConfig(const Config &c, char *out, size_t size) {
    return snprintf(out, size, "C,%.3f,%.3f,%u,%.3f,%.3f", c.factor1, c.factor2, static_cast<unsigned>(c.sps), c.idleAlpha,
                    c.activeAlpha);
}

inline bool sameConfig(const Config &a, const Config &b) {
    auto near = [](float x, float y, float tol) { return std::fabs(x - y) <= tol; };
    return near(a.factor1, b.factor1, 0.0015f) && near(a.factor2, b.factor2, 0.0015f) && a.sps == b.sps &&
           near(a.idleAlpha, b.idleAlpha, 0.0005f) && near(a.activeAlpha, b.activeAlpha, 0.0005f);
}

} // namespace scalewire

#endif // SCALEWIRE_H
