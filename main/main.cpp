#include "ESP32_Hardware.h"
using namespace Printer;
using namespace Hardware_Config;

void setup(void)
{
    ESP32_Helper::Initialisation();
    println("ESP32 Hardware Lib");
    Wire.begin(SDA, SCL, 400000UL);
    PowerMonitor::Initialisation();
    Screen::Initialisation();
    IHM::Initialisation();
    Match::Initialisation();

    ServoAX12::Initialisation(SERIAL_SERVO, RX_SERVO, TX_SERVO, PIN_SERVO_DIR);
    //ServoAX12::Scan(ServoAX12::DxlProtocolVersion::PROTOCOL_1, BaudRate::BAUD_RATE_1000000);
    ServoAX12::AddServo(Hardware_Config::ServoID::Test, "Test", Hardware_Config::ServoPosition::TestMin, Hardware_Config::ServoPosition::TestMax);
    ESP32_Helper::RegisterCommandHandler("AX12", ServoAX12::HandleCommand, ServoAX12::PrintCommandHelp);
    
    ToF_VL53L8CX::Initialisation();
}

void loop(void)
{
    /*
        println("Bus Voltage:   %f V",PowerMonitor::getBusVoltage_V());
        println("Shunt Voltage: %f mV",PowerMonitor::getShuntVoltage_mV());
        println("Current:       %f mA",PowerMonitor::getCurrent_mA());
        println("Power:         %f mW",PowerMonitor::getPower_mW());
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
