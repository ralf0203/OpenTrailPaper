#pragma once

#include <stdint.h>

// BLE and ANT+ both publish samples here; this module is the ONLY writer of
// RideState::heartRateBpm / hrMs / hrConnected. No preference is persisted.
namespace hr_source {

enum class Source : uint8_t { NONE, BLE, ANT };
enum class Preference : uint8_t { AUTO, BLE, ANT };

// Read-only snapshot for the sensor UI (no radio operations, no NVS).
struct Status {
    Source active = Source::NONE;
    bool bleFresh = false;
    uint8_t bleBpm = 0xFF;
    bool antFresh = false;
    uint8_t antBpm = 0xFF;
}; // HR_UI_V1
Status status();


// Safe to call from BLE/ANT task callbacks, but NOT from an ISR.
void ingest(Source source, uint8_t bpm, uint32_t atMs);
void clear(Source source);

void setPreference(Preference preference);
Preference preference();

// Call regularly from Arduino loop() (the only RideState HR writer).
// Freshness window: 15 seconds, matching OpenTrailPaper's existing policy.
void tick();

// Console diagnostics; may be called from the UI task.
void report();

} // namespace hr_source
