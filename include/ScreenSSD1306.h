#ifndef SCREENSSD1306_H
#define SCREENSSD1306_H

#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <Wire.h>

#include "ESP32_Hardware.h"

/* Représentation ASCII de l'afficheur (128x64, 21 chars max/ligne, 8 lignes)
   |0         1         2|
   |123456789012345678901|
___|_____________________|
 0 |TEST     BAU  0 BOOT |
 1 |JAUNE  ! Low !   000s|
 2 |Obs --------         |
 3 |X 200 X  200    AX12 |
 4 |Y 1500Y 1500 Bras:123|
 5 |A 180 A 180  VL53:123|
 6 |                     |
 7 |ON  12.30V 500mA WF-X|
*/

namespace Screen
{
    void Initialisation(void);
    void ClearDisplay(void);
    void Text(const String &text, uint8_t row = 0, uint8_t col = 0);
    void Logo(void);
    void Update();
    void SetPose(const Pose newPose);
    void SetTarget(const Pose newTarget);
    void SetHold(bool hold);
    void SetObstacle(int id, int num);

} // namespace Screen

#endif