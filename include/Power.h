#ifndef POWER_H
#define POWER_H

#include <Adafruit_INA219.h>
#include <Wire.h>

#include "ESP32_Hardware.h"

namespace Power
{
    constexpr float maxVoltage_V = 12.6; // Maximum voltage of the battery
    constexpr float lowVoltage_V = 11.0;  // Low Voltage warning
    constexpr float minVoltage1_V = 10.0;  // Minimum voltage of the battery to shut down power
    constexpr float minVoltage2_V = 10.5;  // Minimum voltage of the battery to power up

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
