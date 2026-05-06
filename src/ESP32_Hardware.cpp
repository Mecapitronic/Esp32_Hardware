#include "ESP32_Hardware.h"

using namespace Printer;
using namespace Hardware_Config;

namespace Hardware
{
    void Initialisation()
    {
        println();
        println("-- Starting Hardware Initialisation --");

        Wire.begin(SDA, SCL, 400000UL);

        Power::Initialisation();
        Screen::Initialisation();
        IHM::Initialisation();
        Match::Initialisation();

        ServoAX12::Initialisation(SERIAL_SERVO, RX_SERVO, TX_SERVO, PIN_SERVO_DIR);
        //ServoAX12::AddServo(Hardware_Config::ServoID::Test, "Test", Hardware_Config::ServoPosition::TestMin, Hardware_Config::ServoPosition::TestMax);
        ESP32_Helper::RegisterCommandHandler("AX12", ServoAX12::HandleCommand, ServoAX12::PrintCommandHelp);

        ToF_VL53L8CX::Initialisation();
        println("-- End of Hardware Initialisation --");
        println();
    }
} // namespace Hardware