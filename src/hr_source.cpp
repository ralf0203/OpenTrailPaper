#include "hr_source.h"

#include <Arduino.h>
#include <atomic>

#include "ride_state.h"

namespace {

constexpr uint32_t kFreshMs = 15000;

struct Slot {
    std::atomic<uint8_t> bpm{0xFF};
    std::atomic<uint32_t> atMs{0};
};

Slot ble;
Slot ant;
std::atomic<hr_source::Preference> requested{hr_source::Preference::AUTO};

// Accessed only in Arduino loop() by tick(). Never mutate from radio callbacks.
hr_source::Source lastSource = hr_source::Source::NONE;
uint8_t lastBpm = 0xFF;
uint32_t lastAtMs = 0;

Slot* slotFor(hr_source::Source source) {
    switch (source) {
        case hr_source::Source::BLE: return &ble;
        case hr_source::Source::ANT: return &ant;
        default: return nullptr;
    }
}

struct Reading {
    uint8_t bpm = 0xFF;
    uint32_t atMs = 0;
    uint32_t ageMs = 0;
    bool fresh = false;
};

Reading read(const Slot& slot, uint32_t now) {
    Reading r;
    // Publishers store bpm first, then the timestamp with release semantics.
    r.atMs = slot.atMs.load(std::memory_order_acquire);
    r.bpm = slot.bpm.load(std::memory_order_relaxed);
    r.ageMs = now - r.atMs; // unsigned subtraction tolerates millis() wraparound
    r.fresh = r.atMs != 0 && r.bpm > 0 && r.bpm != 0xFF && r.ageMs <= kFreshMs;
    return r;
}

struct Snapshot {
    hr_source::Preference preference;
    hr_source::Source active;
    Reading bleReading;
    Reading antReading;
};

Snapshot snapshot() {
    const uint32_t now = millis();
    Snapshot s{};
    s.preference = requested.load(std::memory_order_relaxed);
    s.bleReading = read(ble, now);
    s.antReading = read(ant, now);

    // AUTO and ANT: ANT is preferred. BLE: BLE is preferred.
    // The other source always takes over when the preferred one becomes stale.
    if (s.preference == hr_source::Preference::BLE) {
        s.active = s.bleReading.fresh ? hr_source::Source::BLE
                 : s.antReading.fresh ? hr_source::Source::ANT
                                      : hr_source::Source::NONE;
    } else {
        s.active = s.antReading.fresh ? hr_source::Source::ANT
                 : s.bleReading.fresh ? hr_source::Source::BLE
                                      : hr_source::Source::NONE;
    }
    return s;
}

const char* sourceName(hr_source::Source source) {
    switch (source) {
        case hr_source::Source::BLE: return "BLE";
        case hr_source::Source::ANT: return "ANT+";
        default: return "NONE";
    }
}

const char* preferenceName(hr_source::Preference pref) {
    switch (pref) {
        case hr_source::Preference::BLE: return "ble (BLE > ANT+)";
        case hr_source::Preference::ANT: return "ant (ANT+ > BLE)";
        default: return "auto (ANT+ > BLE)";
    }
}

} // namespace

namespace hr_source {

void ingest(Source source, uint8_t bpm, uint32_t atMs) {
    Slot* slot = slotFor(source);
    if (!slot) return;
    // A zero or 0xFF HR value is not a live pulse sample.
    if (bpm == 0 || bpm == 0xFF || atMs == 0) return;
    slot->bpm.store(bpm, std::memory_order_relaxed);
    slot->atMs.store(atMs, std::memory_order_release);
}

void clear(Source source) {
    Slot* slot = slotFor(source);
    if (!slot) return;
    slot->atMs.store(0, std::memory_order_release);
    slot->bpm.store(0xFF, std::memory_order_relaxed);
}

void setPreference(Preference pref) {
    requested.store(pref, std::memory_order_relaxed);
}

Preference preference() {
    return requested.load(std::memory_order_relaxed);
}

// UI reads the very same arbitration snapshot as the console diagnostics.
Status status() {
    const Snapshot s = snapshot();
    Status out;
    out.active = s.active;
    out.bleFresh = s.bleReading.fresh;
    out.bleBpm = s.bleReading.bpm;
    out.antFresh = s.antReading.fresh;
    out.antBpm = s.antReading.bpm;
    return out;
} // HR_UI_V1

void tick() {
    const Snapshot s = snapshot();
    const Reading* r = s.active == Source::ANT ? &s.antReading
                     : s.active == Source::BLE ? &s.bleReading : nullptr;
    const uint8_t bpm = r ? r->bpm : 0xFF;
    const uint32_t atMs = r ? r->atMs : 0;

    if (s.active == lastSource && bpm == lastBpm && atMs == lastAtMs) return;

    // Central arbitration is the sole authority for the HR fields used by
    // dashboard, BLE phone updates and FIT recording.
    g_state.with([&](RideState& state) {
        state.heartRateBpm = bpm;
        state.hrMs = atMs;
        state.hrConnected = r != nullptr;
        state.hrSource = static_cast<uint8_t>(s.active);  // HR_UI_SOURCE_V1
    });
    lastSource = s.active;
    lastBpm = bpm;
    lastAtMs = atMs;
}

void report() {
    const Snapshot s = snapshot();
    Serial.printf("[HR] priority=%s, active=%s (session only)\n",
                  preferenceName(s.preference), sourceName(s.active));
    if (s.bleReading.fresh)
        Serial.printf("[HR] BLE: %u bpm, age %lu ms\n",
                      (unsigned)s.bleReading.bpm,
                      (unsigned long)s.bleReading.ageMs);
    else
        Serial.println("[HR] BLE: no fresh sample");
    if (s.antReading.fresh)
        Serial.printf("[HR] ANT+: %u bpm, age %lu ms\n",
                      (unsigned)s.antReading.bpm,
                      (unsigned long)s.antReading.ageMs);
    else
        Serial.println("[HR] ANT+: no fresh sample");
}

} // namespace hr_source
