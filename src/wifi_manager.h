#pragma once

#include <cstdint>

namespace wifi_manager {

void begin();
void task(void* arg);

bool connected();
int rssi();

}