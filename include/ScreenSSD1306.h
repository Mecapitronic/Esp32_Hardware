#ifndef SCREENSSD1306_H
#define SCREENSSD1306_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "ESP32_Hardware.h"
#ifndef HARDWARE_CONFIG_H
#include "Hardware_Config_Base.h"
#endif

using namespace Hardware_Config;

namespace Screen
{
    void init(void);
    void testtext(void);
    void testdrawbitmap(void);
}

#endif