#pragma once

namespace ant_sensors {

bool begin();
void task(void* arg);
void stop();
bool running();

}