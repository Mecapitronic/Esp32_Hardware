/**
 * @file ESP32_Hardware.h
 * @author Mecapitronic (mecapitronic@gmail.com)
 * @brief Master header
 * @date 2026-03-06
 */
#ifndef ESP32_HARDWARE_H
#define ESP32_HARDWARE_H

#include "ESP32_Helper.h"

#if __has_include("../../../include/Hardware_Config.h")
#include "../../../include/Hardware_Config.h"
#else
#include "Hardware_Config_Base.h"
#endif

#include "IHM.h"
#include "Match.h"
#include "ScreenSSD1306.h"
#include "ServoAX12.h"
#include "PowerMonitor.h"
#include "ToF_VL53L8CX.h"

#endif
