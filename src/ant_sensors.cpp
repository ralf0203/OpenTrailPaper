#include "ant_sensors.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <atomic>

#include "ant_node.h"
#include "diag.h"
#include "power_mgmt.h"
#include "ride_state.h"
#include "hr_source.h"

#include "ble_sensors.h"

TaskHandle_t antTaskHandle = nullptr;

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

bool startManual()
{
    if (started.load())
        return true;

    diag::log("ANT+: manual start requested");
    // A previous run's last packet must never count as a new reception.
    latestHr.store(0xFF);
    latestHrMs.store(0);
    hr_source::clear(hr_source::Source::ANT);

    /*
     * Stop OpenTrailPaper from managing BLE scans while ANT+ owns
     * the scan windows.
     */
    ble_sensors::reserveScanForAnt(true);

    /*
     * Keep automatic light sleep out of the bring-up completely.
     */
    power_mgmt::busyAcquire();

    NimBLEScan* scan = NimBLEDevice::getScan();

    if (scan->isScanning())
        scan->stop();

    /*
     * IMPORTANT:
     *
     * Install ANT coexist hooks BEFORE starting the passive scan.
     * The previous implementation did this in the opposite order.
     */
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
        power_mgmt::busyRelease();
        ble_sensors::reserveScanForAnt(false);
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
        power_mgmt::busyRelease();
        ble_sensors::reserveScanForAnt(false);

        return false;
    }

    /*
     * Only NOW start the scan whose radio windows ANT hooks.
     */
    scan->setActiveScan(false);
    scan->setInterval(160);
    scan->setWindow(48);

    if (!scan->start(0, false, true)) {
        diag::log("ANT+: passive BLE scan start failed");

        ant_node_stop(&node);
        power_mgmt::busyRelease();
        ble_sensors::reserveScanForAnt(false);

        return false;
    }

    started.store(true);

    if (!antTaskHandle) {
        xTaskCreatePinnedToCore(
            task,
            "ant",
            3072,
            nullptr,
            2,
            &antTaskHandle,
            1
        );
    }

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

                // Only publish into the ANT+ source slot, not RideState.
                hr_source::ingest(hr_source::Source::ANT, bpm, at);
            }

            const uint32_t now = millis();

            // Freshness and failover are handled centrally, including when
            // the ANT task isn't running. Log only genuinely fresh reception.
            if (lastPublished && now - lastPublished <= 15000 &&
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

    NimBLEScan* scan = NimBLEDevice::getScan();
    if (scan->isScanning())
        scan->stop();

    power_mgmt::busyRelease();

    ble_sensors::reserveScanForAnt(false);

    latestHrMs.store(0);
    latestHr.store(0xFF);
    // Do not clear the BLE source; the arbiter immediately falls back to it.
    hr_source::clear(hr_source::Source::ANT);

    diag::log("ANT+: stopped");
}
bool running()
{
    return started.load();
}

} // namespace ant_sensors