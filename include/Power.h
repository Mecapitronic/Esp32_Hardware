#ifndef POWER_H
#define POWER_H

#include <Adafruit_INA219.h>
#include <Wire.h>

#include "ESP32_Hardware.h"

namespace Power
{
    constexpr float lowVoltage_V = 11.0;  // Low Voltage warning

    void Initialisation(void);
    void Update();

    void DisablePower();
    void EnablePower();
    void UpdatePower();
    bool isPowerON();

    float getBusVoltage_V();
    float getShuntVoltage_mV();
    float getCurrent_mA();
    float getPower_mW();
    bool isLowVoltage();

} // namespace Power

#endif
