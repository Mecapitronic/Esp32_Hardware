#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <Adafruit_INA219.h>
#include <Wire.h>

#include "ESP32_Hardware.h"

namespace PowerMonitor
{
    constexpr float maxVoltage_V = 12.6; // Maximum voltage of the battery
    constexpr float minVoltage_V = 11.0;  // Minimum voltage of the battery

    void Initialisation(void);

    float getBusVoltage_V();
    float getShuntVoltage_mV();
    float getCurrent_mA();
    float getPower_mW();

} // namespace PowerMonitor

#endif
