#include "ESP32_Hardware.h"
using namespace Printer;
using namespace Hardware_Config;

void setup(void)
{
    ESP32_Helper::Initialisation();
    Hardware::Initialisation();
    ServoAX12::AddServo(Hardware_Config::ServoID::Test, "Test", Hardware_Config::ServoPosition::TestMin, Hardware_Config::ServoPosition::TestMax);
    

}

void loop(void)
{
    /*
        println("Bus Voltage:   %f V",Power::getBusVoltage_V());
        println("Shunt Voltage: %f mV",Power::getShuntVoltage_mV());
        println("Current:       %f mA",Power::getCurrent_mA());
        println("Power:         %f mW",Power::getPower_mW());
        println("");
    */
    /*
    ServoAX12::SetServoPosition(Hardware_Config::ServoID::Test, Hardware_Config::ServoPosition::TestMin);
    while (ServoAX12::IsServoMoving(Hardware_Config::ServoID::Test))
    {
        ServoAX12::TeleplotAllPosition();
        vTaskDelay(100);
    }
    */
    // ToF_VL53L8CX::printProcessing();

    //if (ToF_VL53L8CX::isError())
    //{
    //    printError("Sensor error detected!");
    //}

    delay(500);
}
