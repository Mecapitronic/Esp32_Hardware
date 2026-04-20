/*
    Start the serial communication at default baud rate speed : 921600
    SERIAL_DEBUG is the Serial port used for communication with the PC
    You can redefine it to change the Serial port used
*/

#include "ESP32_Hardware.h"
using namespace Printer;

void setup(void)
{
    ESP32_Helper::Initialisation();
    Wire.begin(SDA, SCL, 400000UL);
    Screen::Initialisation();
}

void loop(void)
{
    delay(1000);
}
