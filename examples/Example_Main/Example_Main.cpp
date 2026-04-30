#include "ESP32_Hardware.h"
using namespace Printer;

// ── Broches AX12 ─────────────────────────────────────────────────────────────
static constexpr int8_t AX12_RX  = 26;
static constexpr int8_t AX12_TX  = 27;
static constexpr int8_t AX12_DIR = 14;

void setup(void)
{
    // ── Initialisation commune ────────────────────────────────────────────────
    ESP32_Helper::Initialisation();
    Wire.begin(SDA, SCL, 400000UL);

    // ── Example1 : Écran ─────────────────────────────────────────────────────
    Screen::Initialisation();

    // ── Example4 : IHM ───────────────────────────────────────────────────────
    IHM::Initialisation();

    // ── Example4 / Example1 : Match ──────────────────────────────────────────
    Match::Initialisation();

    // ── Example3 : Moniteur de puissance ─────────────────────────────────────
    PowerMonitor::Initialisation();

    // ── Example2 : Servomoteurs AX12 ─────────────────────────────────────────
    ServoAX12::Initialisation(Serial1, AX12_RX, AX12_TX, AX12_DIR);
    ServoAX12::Scan(ServoAX12::DxlProtocolVersion::PROTOCOL_1, BaudRate::BAUD_RATE_1000000);
    ServoAX12::AddServo(ServoID::Test, "Test", ServoPosition::TestMin, ServoPosition::TestMax);

    // ── Example5 : Capteur ToF ───────────────────────────────────────────────
    ToF_VL53L8CX::Initialisation();

    println("Resolution: %u pixels (%ux%u)",
            ToF_VL53L8CX::getImageResolution(),
            ToF_VL53L8CX::getImageWidth(),
            ToF_VL53L8CX::getImageWidth());
}

void loop(void)
{
    // ── Example3 : Puissance ─────────────────────────────────────────────────
    println("Bus Voltage:   %f V",   PowerMonitor::getBusVoltage_V());
    println("Shunt Voltage: %f mV",  PowerMonitor::getShuntVoltage_mV());
    println("Current:       %f mA",  PowerMonitor::getCurrent_mA());
    println("Power:         %f mW",  PowerMonitor::getPower_mW());
    println("");

    // ── Example2 : Mouvement AX12 ────────────────────────────────────────────
    ServoAX12::SetServoPosition(ServoID::Test, ServoPosition::TestMin);
    while (ServoAX12::IsServoMoving(ServoID::Test))
    {
        ServoAX12::TeleplotAllPosition();
        vTaskDelay(10);
    }

    ServoAX12::SetServoPosition(ServoID::Test, ServoPosition::TestMax);
    while (ServoAX12::AreAllServoMoving())
    {
        ServoAX12::TeleplotAllPosition();
        vTaskDelay(10);
    }

    // ── Example5 : ToF ───────────────────────────────────────────────────────
    ToF_VL53L8CX::printProcessing();

    if (ToF_VL53L8CX::isError())
    {
        printError("Sensor error detected!");
    }

    delay(500);
}
