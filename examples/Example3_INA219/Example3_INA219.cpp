/*
    Start the serial communication at default baud rate speed : 921600
    SERIAL_DEBUG is the Serial port used for communication with the PC
    You can redefine it to change the Serial port used
*/

#include "ESP32_Hardware.h"
using namespace Printer;


Adafruit_INA219 ina219;
void setup(void)
{
    ESP32_Helper::Initialisation();
    Wire.begin(SDA, SCL, 400000UL);
    PowerMonitor::Initialisation();
}

void loop(void)
{
  println("Bus Voltage:   %f V",PowerMonitor::getBusVoltage_V());
  println("Shunt Voltage: %f mV",PowerMonitor::getShuntVoltage_mV());
  println("Current:       %f mA",PowerMonitor::getCurrent_mA());
  println("Power:         %f mW",PowerMonitor::getPower_mW());
  println("");

  delay(500);
}