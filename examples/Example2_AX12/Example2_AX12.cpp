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
    int8_t rxPin = 26;
    int8_t txPin = 27;
    int8_t dirPin = 14;

    ServoAX12::Initialisation(Serial1, rxPin, txPin, dirPin);

    ServoAX12::Scan(ServoAX12::DxlProtocolVersion::PROTOCOL_1, BaudRate::BAUD_RATE_1000000);
    
    ServoAX12::AddServo(ServoID::Test, "Test", ServoPosition::TestMin, ServoPosition::TestMax);
}

int32_t cpt = 0;

void loop(void)
{
    ServoAX12::SetServoPosition(ServoID::Test, 10);
    while (ServoAX12::IsServoMoving(ServoID::Test))
    {
        ServoAX12::TeleplotAllPosition();
        vTaskDelay(10);
    }
    while (ServoAX12::IsServoMoving(ServoID::Test))
    {
        ServoAX12::TeleplotAllPosition();
        vTaskDelay(10);
    }

    ServoAX12::SetServoPosition(ServoID::Test, 290);
    while (ServoAX12::AreAllServoMoving())
    {
        ServoAX12::TeleplotAllPosition();
        vTaskDelay(10);
    }
}