#include "StderrDskyHardware.h"

#include <cstdio>

namespace dskymatic {

StderrDskyHardware::StderrDskyHardware(bool only_on_change)
    : m_only_on_change(only_on_change)
{}

bool StderrDskyHardware::init()  { std::fprintf(stderr, "StderrDskyHardware: init\n"); return true; }
void StderrDskyHardware::shutdown() { std::fprintf(stderr, "StderrDskyHardware: shutdown\n"); }

bool StderrDskyHardware::store_if_changed(const std::string &name, int value) {
    for (int i = 0; i < m_cache_count; ++i) {
        if (m_cache[i].name == name) {
            if (m_cache[i].value == value) return false;
            m_cache[i].value = value;
            return true;
        }
    }
    if (m_cache_count < kCapacity) {
        m_cache[m_cache_count++] = {name, value};
    }
    return true;
}

void StderrDskyHardware::set_digit(const std::string &field, int value) {
    if (m_only_on_change && !store_if_changed(field, value)) return;
    if (value < 0)  std::fprintf(stderr, "DSKY %s: _\n", field.c_str());
    else            std::fprintf(stderr, "DSKY %s: %d\n", field.c_str(), value);
}

void StderrDskyHardware::set_sign(const std::string &field, int value) {
    if (m_only_on_change && !store_if_changed(field, value)) return;
    const char *s = (value > 0) ? "+" : (value < 0) ? "-" : " ";
    std::fprintf(stderr, "DSKY %s: %s\n", field.c_str(), s);
}

void StderrDskyHardware::set_light(const std::string &name, bool on) {
    if (m_only_on_change && !store_if_changed(name, on ? 1 : 0)) return;
    std::fprintf(stderr, "DSKY %s: %s\n", name.c_str(), on ? "ON" : "off");
}

void StderrDskyHardware::set_flashing(const std::string &group, bool flashing) {
    if (m_only_on_change && !store_if_changed("FLASH_" + group, flashing ? 1 : 0)) return;
    std::fprintf(stderr, "DSKY FLASH-%s: %s\n", group.c_str(), flashing ? "ON" : "off");
}

} // namespace dskymatic
