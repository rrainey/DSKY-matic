#pragma once

#include "IDskyHardware.h"

namespace dskymatic {

/*
 * No-hardware driver that just prints each sink update to stderr.
 * Useful for debugging the DSKY dispatch path when no physical
 * DSKY-matic hardware is attached.  Also the default driver for
 * dskymatic-headless until a real Pi driver is wired in.
 */
class StderrDskyHardware : public IDskyHardware {
public:
    explicit StderrDskyHardware(bool only_on_change = true);

    bool init() override;
    void shutdown() override;

    void set_digit(const std::string &field, int value) override;
    void set_sign(const std::string &field, int value) override;
    void set_light(const std::string &name, bool on) override;
    void set_flashing(const std::string &group, bool flashing) override;

    int  poll_keycode() override { return -1; }

private:
    bool m_only_on_change;
    // Per-field last value caches, so we don't spam identical writes.
    // Simple flat maps — DSKY has ~18 digits + 3 signs + ~8 lights + 2 flashes.
    struct Field {
        std::string name;
        int value;
    };
    static constexpr int kCapacity = 64;
    Field m_cache[kCapacity];
    int   m_cache_count = 0;

    bool store_if_changed(const std::string &name, int value);
};

} // namespace dskymatic
