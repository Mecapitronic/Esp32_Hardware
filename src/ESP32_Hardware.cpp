#include "ESP32_Hardware.h"

using namespace Printer;
using namespace Hardware_Config;

namespace Hardware
{
    namespace
    {
        TaskThread taskUpdateHardware;
        Timeout powerUpdateTimeout;
        Timeout ihmUpdateTimeout;
        Timeout screenUpdateTimeout;
        Timeout tofUpdateTimeout;

        constexpr uint32_t kPowerUpdatePeriodMs = 100;
        constexpr uint32_t kIhmUpdatePeriodMs = 20;
        constexpr uint32_t kScreenUpdatePeriodMs = 200;
        constexpr uint32_t kTofUpdatePeriodMs = 500;
    }
    void Initialisation()
    {
        println();
        println("-- Starting Hardware Initialisation --");

        Wire.begin(SDA, SCL, 400000UL);

        Power::Initialisation();
        Screen::Initialisation();
        IHM::Initialisation();
        // ToF disabled temporarily while validating random reset stability.
        ToF_VL53L8CX::Initialisation();

        powerUpdateTimeout.Start(kPowerUpdatePeriodMs);
        ihmUpdateTimeout.Start(kIhmUpdatePeriodMs);
        screenUpdateTimeout.Start(kScreenUpdatePeriodMs);
        tofUpdateTimeout.Start(kTofUpdatePeriodMs);

        taskUpdateHardware = TaskThread(TaskUpdateHardware, "TaskUpdateHardware", 20000, 10, 0);

        Match::Initialisation();

        ServoAX12::Initialisation(SERIAL_SERVO, RX_SERVO, TX_SERVO, PIN_SERVO_DIR);
        //ServoAX12::AddServo(Hardware_Config::ServoID::Test, "Test", Hardware_Config::ServoPosition::TestMin, Hardware_Config::ServoPosition::TestMax);
        ESP32_Helper::RegisterCommandHandler("AX12", ServoAX12::HandleCommand, ServoAX12::PrintCommandHelp);

        println("-- End of Hardware Initialisation --");
        println();
    }

    void TaskUpdateHardware(void *pvParameters)
    {
        println("Start Task Update Hardware");
        Chrono chrono("Hardware", 1000);
        while (true)
        {
            chrono.Start();
            try
            {
                if (powerUpdateTimeout.IsTimeOut())
                {
                    Power::Update();
                    powerUpdateTimeout.Start(kPowerUpdatePeriodMs);
                }

                if (ihmUpdateTimeout.IsTimeOut())
                {
                    IHM::Update();
                    ihmUpdateTimeout.Start(kIhmUpdatePeriodMs);
                }

                if (screenUpdateTimeout.IsTimeOut())
                {
                    Screen::Update();
                    screenUpdateTimeout.Start(kScreenUpdatePeriodMs);
                }

                if (tofUpdateTimeout.IsTimeOut())
                {
                    ToF_VL53L8CX::Update();
                    tofUpdateTimeout.Start(kTofUpdatePeriodMs);
                }
            }
            catch (const std::exception &e)
            {
                printError(e.what());
            }
            if (chrono.Check())
            {
                printChrono(chrono);
            }
            vTaskDelay(10);
        }
        println("Hardware Update Task STOPPED !");
    }
} // namespace Hardware