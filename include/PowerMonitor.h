#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <Adafruit_INA219.h>
#include <Wire.h>

#include "ESP32_Helper.h"

namespace PowerMonitor
{
    void Initialisation(void);

    float getBusVoltage_V();
    float getShuntVoltage_mV();
    float getCurrent_mA();
    float getPower_mW();

} // namespace PowerMonitor

#endif
