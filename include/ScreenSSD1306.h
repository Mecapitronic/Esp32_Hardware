#ifndef SCREENSSD1306_H
#define SCREENSSD1306_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#include "ESP32_Hardware.h"
#include "Logo.h"

/* Représention ASCII de l'afficheur
    MODE ?        BAU         STATE
    COLOR ?       BAU          0:00

    X  200                     AX12
    Y 1500                   1: 123
    A  180                   2: 123

    BAT 12.00V 0,25mA             WIFI
*/

namespace Screen
{
    void Initialisation(void);
    void ClearDisplay(void);
    void Display(void);
    void Text(const String &text, int size = 1, int cursorX = 0, int cursorY = 0, int color = SSD1306_WHITE);
    void Logo(void);
    void TaskUpdateScreen(void *pvParameters);

} // namespace Screen

#endif