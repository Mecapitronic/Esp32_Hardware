#ifndef SCREENSSD1306_H
#define SCREENSSD1306_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#include "ESP32_Hardware.h"
#ifndef HARDWARE_CONFIG_H
#include "Hardware_Config_Base.h"
#endif

using namespace Hardware_Config;

namespace Screen
{
    void Init(void);
    void Text(String &text, int size = 1, int cursorX = 0, int cursorY = 0, int color = SSD1306_WHITE);
    void Logo(void);
} // namespace Screen

#endif