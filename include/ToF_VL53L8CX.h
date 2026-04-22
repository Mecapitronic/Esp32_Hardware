#ifndef ToF_VL53L8CX_H
#define ToF_VL53L8CX_H

#include "ESP32_Hardware.h"
#include <vl53l8cx.h>  //https://github.com/stm32duino/VL53L8CX

namespace ToF_VL53L8CX
{
    constexpr uint16_t DEFAULT_RESOLUTION = 64;  // 8x8 resolution
    constexpr uint8_t RANGING_FREQUENCY_HZ = 5;  // 5Hz measurement rate

    // Initialization
    void Initialisation();

    // Data access
    void Update();

    // Getters for sensor data
    const VL53L8CX_ResultsData& getSensorData();
    uint8_t getImageWidth();
    uint8_t getImageResolution();
    bool isError();

    // Utility functions
    void printProcessing();
    void printFormattedOutput();
    void printCSVOutput();

} // namespace ToF_VL53L8CX

#endif
