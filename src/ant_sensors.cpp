#include "ant_sensors.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <atomic>

#include "ant_node.h"
#include "diag.h"
#include "power_mgmt.h"
#include "ride_state.h"

namespace {

ant_node_t node{};

std::atomic<bool> started{false};
std::atomic<uint8_t> latestHr{0xFF};
std::atomic<uint32_t> latestHrMs{0};
std::atomic<uint16_t> latestDevice{0};
std::atomic<int8_t> latestRssi{0};

void onAntData(
    ant_node_t*,
    const ant_node_rx_t* rx,
    const uint8_t page[8],
    void*)
{
    if (rx->device_type != ANTPLUS_DEVTYPE_HRM)
        return;

    antplus_hrm_data_t hr{};

    if (!antplus_hrm_decode(page, &hr))
        return;

    latestHr.store(hr.computed_heart_rate);
    latestHrMs.store(millis());
    latestDevice.store(rx->device_num);
    latestRssi.store(rx->rssi);
}

void onAntPaired(
    ant_node_t*,
    uint8_t channel,
    const ant_node_device_t* dev,
    bool remembered,
    void*)
{
    diag::log(
        "ANT+: ch%u HRM #%u %s",
        channel,
        dev->device_num,
        remembered ? "paired" : "known"
    );
}

} // namespace


namespace ant_sensors {

bool begin()
{
    if (started.load())
        return true;

    /*
     * Coexist mode uses NimBLE's scan windows for ANT reception.
     * NimBLE itself must already be initialised.
     */
    NimBLEScan* scan = NimBLEDevice::getScan();

    if (scan->isScanning())
        scan->stop();

    scan->setActiveScan(false);

    // Continuous passive scan. ANT+ receives through these windows.
    if (!scan->start(0, false, true)) {
        diag::log("ANT+: failed to start passive BLE scan");
        return false;
    }

    ant_node_config_t cfg{};
    cfg.on_data = onAntData;
    cfg.on_paired = onAntPaired;
    cfg.store = &ant_node_store_nvs;
    cfg.proximity_rssi = -70;
    cfg.coexist = true;

    const ant_espphy_status_t status =
        ant_node_start(&node, &cfg);

    diag::log(
        "ANT+: node start %s",
        ant_espphy_status_str(status)
    );

    if (status != ANT_ESPPHY_OK) {
        scan->stop();
        return false;
    }

    const uint8_t rc =
        ant_node_open_antplus_slave(
            &node,
            0,
            ANTPLUS_DEVTYPE_HRM,
            0,
            ANTPLUS_PERIOD_HRM
        );

    if (rc != 0) {
        diag::log("ANT+: HRM channel open failed 0x%02X", rc);
        ant_node_stop(&node);
        scan->stop();
        return false;
    }

    /*
     * esp32-ant keeps a continuous RX activity alive.
     * Don't allow automatic light sleep during this first bring-up.
     */
    power_mgmt::busyAcquire();

    started.store(true);

    diag::log("ANT+: HRM receiver running");
    return true;
}


void task(void*)
{
    uint32_t lastPublished = 0;
    uint32_t lastLog = 0;

    for (;;) {

        if (started.load()) {
            const uint32_t at = latestHrMs.load();

            if (at != 0 && at != lastPublished) {
                lastPublished = at;

                const uint8_t bpm = latestHr.load();

                g_state.with([&](RideState& s) {
                    s.heartRateBpm = bpm;
                    s.hrMs = at;
                    s.hrConnected = true;
                });
            }

            const uint32_t now = millis();

            // Same 15 s freshness rule as BLE.
            if (lastPublished &&
                now - lastPublished > 15000) {

                g_state.with([](RideState& s) {
                    s.heartRateBpm = 0xFF;
                    s.hrConnected = false;
                });
            }

            if (lastPublished &&
                now - lastLog >= 10000) {

                lastLog = now;

                diag::log(
                    "ANT+: HRM #%u %u bpm RSSI %d",
                    latestDevice.load(),
                    latestHr.load(),
                    latestRssi.load()
                );
            }
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}


void stop()
{
    if (!started.exchange(false))
        return;

    ant_node_stop(&node);

    power_mgmt::busyRelease();

    NimBLEScan* scan = NimBLEDevice::getScan();
    if (scan->isScanning())
        scan->stop();

    g_state.with([](RideState& s) {
        s.heartRateBpm = 0xFF;
        s.hrConnected = false;
    });

    diag::log("ANT+: stopped");
}


bool running()
{
    return started.load();
}

}