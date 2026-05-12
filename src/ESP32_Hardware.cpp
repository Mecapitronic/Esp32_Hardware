#include "ESP32_Hardware.h"

using namespace Printer;
using namespace Hardware_Config;

namespace Hardware
{
    namespace
    {
        TaskThread taskUpdateHardware;
        CallbackFunction_t externalI2CUpdateCallback = nullptr;

        bool useToF = false;

        Timeout externalI2CUpdateTimeout;
        Timeout powerUpdateTimeout;
        Timeout ihmUpdateTimeout;
        Timeout screenUpdateTimeout;
        Timeout tofUpdateTimeout;

        constexpr uint32_t kExternalI2CUpdatePeriodMs = 5;
        constexpr uint32_t kPowerUpdatePeriodMs = 500;
        constexpr uint32_t kIhmUpdatePeriodMs = 50;
        constexpr uint32_t kScreenUpdatePeriodMs = 500;
        constexpr uint32_t kTofUpdatePeriodMs = 500;
    }

    void SetExternalI2CUpdateCallback(CallbackFunction_t callback)
    {
        externalI2CUpdateCallback = callback;
        externalI2CUpdateTimeout.Start(kExternalI2CUpdatePeriodMs);
    }

    void Initialisation(bool _useToF)
    {
        useToF = _useToF;
        println();
        println("-- Starting Hardware Initialisation --");

        Wire.begin(SDA, SCL, 400000UL);

        Power::Initialisation();
        Screen::Initialisation();
        IHM::Initialisation();
        if (useToF)
            ToF_VL53L8CX::Initialisation();

        externalI2CUpdateTimeout.Start(kExternalI2CUpdatePeriodMs);
        powerUpdateTimeout.Start(kPowerUpdatePeriodMs);
        ihmUpdateTimeout.Start(kIhmUpdatePeriodMs);
        screenUpdateTimeout.Start(kScreenUpdatePeriodMs);
        tofUpdateTimeout.Start(kTofUpdatePeriodMs);

        taskUpdateHardware =
            TaskThread(TaskUpdateHardware, "TaskUpdateHardware", 20000, 15, 0);

        Match::Initialisation();
        ESP32_Helper::RegisterCommandHandler("Match", Match::HandleCommand, Match::PrintCommandHelp);

        ServoAX12::Initialisation(SERIAL_SERVO, RX_SERVO, TX_SERVO, PIN_SERVO_DIR);
        //ServoAX12::AddServo(Hardware_Config::ServoID::Test, "Test", Hardware_Config::ServoPosition::TestMin, Hardware_Config::ServoPosition::TestMax);
        ESP32_Helper::RegisterCommandHandler("AX12", ServoAX12::HandleCommand, ServoAX12::PrintCommandHelp);

        println("-- End of Hardware Initialisation --");
        println();
    }

    void TaskUpdateHardware(void *pvParameters)
    {
        println("Start Task Update Hardware");
        Chrono chrono("Hardware", 5000);
        while (true)
        {
            chrono.Start();
            try
            {
                if (externalI2CUpdateCallback != nullptr
                    && externalI2CUpdateTimeout.IsTimeOut())
                {
                    externalI2CUpdateCallback();
                    externalI2CUpdateTimeout.Start(kExternalI2CUpdatePeriodMs);
                }

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

                if (useToF && tofUpdateTimeout.IsTimeOut())
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
            // Keep loop latency low so external I2C callback can run at 5 ms cadence.
            vTaskDelay(1);
        }
        println("Hardware Update Task STOPPED !");
    }
} // namespace Hardware