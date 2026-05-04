#ifndef SCREENSSD1306_H
#define SCREENSSD1306_H

#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <Wire.h>

#include "ESP32_Hardware.h"

/* Représentation ASCII de l'afficheur (128x64, 21 chars max/ligne, 8 lignes)
   |0         1         2
   |123456789012345678901
___|_____________________
 0 |TEST     BAU  0 BOOT
 1 |JAUNE           000 s
 2 | 
 3 |X  200          AX12
 4 |Y 1500        1:123.4
 5 |A  180        2:123.4
 6 | 
 7 |BAT 12.30V 12mA  WIFI
*/

namespace Screen
{
    void Initialisation(void);
    void ClearDisplay(void);
    void Display(void);
    void Text(const String &text, uint8_t row = 0, uint8_t col = 0);
    void Logo(void);
    void TaskUpdateScreen(void *pvParameters);

} // namespace Screen

#endif