// Line protocol shared by the controller and the Waveshare boards on the J4/J5 serial links.
//
//   $<TYPE>[,<field>...]*XX\n
//
// XX is the 8-bit XOR of every character between '$' and '*' (both excluded), as two
// upper-case hex digits (the NMEA-0183 convention). Anything that is not a complete,
// checksummed line is dropped. Fields are plain ASCII, comma separated, no quoting.
//
// STL-only on purpose: the native unit tests include this header directly.

#ifndef LINEPROTOCOL_H
#define LINEPROTOCOL_H

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace lineproto {

constexpr size_t MAX_LINE = 96;   // longest line incl. '$' and '*XX', excluding the newline
constexpr size_t MAX_FIELDS = 10; // type + up to 9 values

inline uint8_t checksum(const char *payload, size_t len) {
    uint8_t x = 0;
    for (size_t i = 0; i < len; i++)
        x ^= static_cast<uint8_t>(payload[i]);
    return x;
}

inline int hexVal(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

// Builds "$<payload>*XX\r\n" into out. Returns the length written (0 if out is too small).
inline size_t frame(const char *payload, char *out, size_t outSize) {
    const size_t len = strlen(payload);
    if (len + 7 > outSize) // '$' + payload + '*' + 2 hex + "\r\n" + NUL
        return 0;
    const uint8_t x = checksum(payload, len);
    static const char hex[] = "0123456789ABCDEF";
    size_t n = 0;
    out[n++] = '$';
    memcpy(out + n, payload, len);
    n += len;
    out[n++] = '*';
    out[n++] = hex[x >> 4];
    out[n++] = hex[x & 0x0F];
    out[n++] = '\r';
    out[n++] = '\n';
    out[n] = '\0';
    return n;
}

struct Message {
    char type[4] = {0};        // "F", "T", "H", "L", "Z", "G", "W", "B", "C"
    size_t count = 0;          // number of value fields (excluding the type)
    const char *field[MAX_FIELDS - 1] = {nullptr};

    float f(size_t i, float fallback = 0.0f) const {
        if (i >= count || field[i] == nullptr || *field[i] == '\0')
            return fallback;
        char *end = nullptr;
        const float v = strtof(field[i], &end);
        return end == field[i] ? fallback : v;
    }
    long i(size_t i, long fallback = 0) const {
        if (i >= count || field[i] == nullptr || *field[i] == '\0')
            return fallback;
        char *end = nullptr;
        const long v = strtol(field[i], &end, 10);
        return end == field[i] ? fallback : v;
    }
};

// Validates and splits one complete line (without the newline). `line` is modified in place
// (commas and the '*' become NULs). Returns false for anything malformed or with a bad checksum.
inline bool parse(char *line, size_t len, Message &msg) {
    if (len < 5 || line[0] != '$')
        return false;
    if (line[len - 3] != '*')
        return false;
    const int hi = hexVal(line[len - 2]);
    const int lo = hexVal(line[len - 1]);
    if (hi < 0 || lo < 0)
        return false;
    const size_t payloadLen = len - 4; // between '$' and '*'
    if (payloadLen == 0)
        return false;
    if (checksum(line + 1, payloadLen) != static_cast<uint8_t>((hi << 4) | lo))
        return false;
    line[len - 3] = '\0';

    msg.count = 0;
    char *p = line + 1;
    char *comma = strchr(p, ',');
    size_t typeLen = comma ? static_cast<size_t>(comma - p) : strlen(p);
    if (typeLen == 0 || typeLen >= sizeof(msg.type))
        return false;
    memcpy(msg.type, p, typeLen);
    msg.type[typeLen] = '\0';
    while (comma != nullptr) {
        *comma = '\0';
        p = comma + 1;
        if (msg.count >= MAX_FIELDS - 1)
            return false;
        msg.field[msg.count++] = p;
        comma = strchr(p, ',');
    }
    return true;
}

// Byte-at-a-time line assembler. Feed every received byte; when a full line arrives and
// parses, returns true with msg filled. Garbage (boot text, noise, overlong lines) is skipped.
class LineReader {
  public:
    bool feed(char c, Message &msg) {
        if (c == '\n' || c == '\r') {
            bool ok = false;
            if (_len > 0) {
                _buf[_len] = '\0';
                ok = parse(_buf, _len, msg);
                if (ok)
                    _good++;
                else
                    _bad++;
            }
            _len = 0;
            return ok;
        }
        if (c == '$') { // a new start resynchronises after a partial line
            _len = 0;
        }
        if (_len == 0 && c != '$')
            return false; // ignore everything until a line start
        if (_len < MAX_LINE) {
            _buf[_len++] = c;
        } else {
            _len = 0; // overlong -> drop
            _bad++;
        }
        return false;
    }
    uint32_t goodLines() const { return _good; }
    uint32_t badLines() const { return _bad; }

  private:
    char _buf[MAX_LINE + 1] = {0};
    size_t _len = 0;
    uint32_t _good = 0;
    uint32_t _bad = 0;
};

} // namespace lineproto

#endif // LINEPROTOCOL_H
