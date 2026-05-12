/**
 * @file ESP32_Hardware.h
 * @author Mecapitronic (mecapitronic@gmail.com)
 * @brief Master header
 * @date 2026-03-06
 */
#ifndef ESP32_HARDWARE_H
#define ESP32_HARDWARE_H

#include "ESP32_Helper.h"

#if __has_include("../../../include/Hardware_Config.h") && !defined(ESP32_HARDWARE_LIB)
#include "../../../include/Hardware_Config.h"
#else
#include "Hardware_Config_Base.h"
#endif

#include "IHM.h"
#include "Match.h"
#include "ScreenSSD1306.h"
#include "ServoAX12.h"
#include "Power.h"
#include "ToF_VL53L8CX.h"

namespace Hardware
{
    using CallbackFunction_t = void (*)();

    /**
     * Initialize all hardware modules and start their internal tasks.
     */
    void Initialisation(bool _useToF);

    /**
     * Register a callback called from the Hardware task loop.
     * Can be used to run external I2C sensor updates in the same task context.
     */
    void SetExternalI2CUpdateCallback(CallbackFunction_t callback);

    /**
     * Task function to periodically update hardware modules.
     * This should be run in a FreeRTOS task.
     */
    void TaskUpdateHardware(void *pvParameters);
}

#endif
