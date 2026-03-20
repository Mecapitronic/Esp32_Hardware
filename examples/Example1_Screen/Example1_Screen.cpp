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
    Screen::Init();
}

int32_t cpt = 0;
void loop(void)
{
    for (size_t i = 1; i < 5; i++)
    {
        Screen::Text("Mecapi", i);
        delay(1000);
    }
    delay(1000);
    Screen::Logo();
    delay(1000);
}
