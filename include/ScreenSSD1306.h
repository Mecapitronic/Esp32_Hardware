#ifndef SCREENSSD1306_H
#define SCREENSSD1306_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#include "ESP32_Hardware.h"
#include "Logo.h"

using namespace Hardware_Config;

namespace Screen
{
    void Init(void);
    void ClearDisplay(void);
    void Display(void);
    void Text(const String &text, int size = 1, int cursorX = 0, int cursorY = 0, int color = SSD1306_WHITE);
    void Logo(void);
    void ShowIHM();

} // namespace Screen

#endif