#include "ESP32_Hardware.h"
using namespace Printer;
using namespace Hardware_Config;
using namespace ServoAX12;

void setup(void)
{
    ESP32_Helper::Initialisation();
    Hardware::Initialisation(false);
    Power::EnablePower();

    //ServoConfig testConfig = ServoConfig((uint8_t)ServoID::Test,
    //    std::array<int32_t, MAX_SERVO_POSITIONS>{0, 145, 290}, 3);
    //AddServo(ServoID::Test, "Test", testConfig);
}

void loop(void)
{
    /*
        println("Bus Voltage   %f V",Power::getBusVoltage_V());
        println("Shunt Voltage %f mV",Power::getShuntVoltage_mV());
        println("Current       %f mA",Power::getCurrent_mA());
        println("Power         %f mW",Power::getPower_mW());
        println("");
    */
    /*
    SetServoPosition(ServoID::Test, 0.0f);
    while (IsServoMoving(ServoID::Test))
    {
        TeleplotAllPosition();
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
