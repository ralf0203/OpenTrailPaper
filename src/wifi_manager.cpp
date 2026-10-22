#include "wifi_manager.h"

#include <Arduino.h>
#include <WiFi.h>

#include "diag.h"
#include "settings.h"

namespace {

bool wasConnected = false;

void startConnection() {
    const char* ssid = settings::wifiSsid();

    if (!settings::wifiEnabled() || !ssid || !ssid[0]) {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    // Never log the password.
    diag::log("wifi: connecting to '%s'", ssid);

    WiFi.begin(
        settings::wifiSsid(),
        settings::wifiPassword()
    );
}

} // namespace


namespace wifi_manager {

void begin() {
    if (!settings::wifiEnabled()) {
        WiFi.mode(WIFI_OFF);
        diag::log("wifi: disabled");
        return;
    }

    if (!settings::wifiSsid()[0]) {
        WiFi.mode(WIFI_OFF);
        diag::log("wifi: enabled but no SSID configured");
        return;
    }

    startConnection();
}


bool connected() {
    return WiFi.status() == WL_CONNECTED;
}


int rssi() {
    return connected() ? WiFi.RSSI() : 0;
}


void task(void*) {
    uint32_t lastRetry = 0;

    for (;;) {
        const bool enabled = settings::wifiEnabled();
        const bool configured = settings::wifiSsid()[0] != '\0';
        const bool isConnected = connected();

        if (!enabled || !configured) {
            if (WiFi.getMode() != WIFI_OFF) {
                WiFi.disconnect(true);
                WiFi.mode(WIFI_OFF);
            }

            wasConnected = false;
        }
        else {
            if (isConnected && !wasConnected) {
                diag::log(
                    "wifi: connected '%s' ip=%s rssi=%d",
                    settings::wifiSsid(),
                    WiFi.localIP().toString().c_str(),
                    WiFi.RSSI()
                );
            }

            if (!isConnected &&
                millis() - lastRetry >= 30000) {

                lastRetry = millis();
                diag::log(
                    "wifi: reconnecting '%s'",
                    settings::wifiSsid()
                );

                WiFi.mode(WIFI_STA);
                WiFi.begin(
                    settings::wifiSsid(),
                    settings::wifiPassword()
                );
            }

            wasConnected = isConnected;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

}