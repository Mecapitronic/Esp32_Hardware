#include "PowerMonitor.h"
using namespace Printer;
using namespace Hardware_Config;

namespace PowerMonitor
{
    namespace
    {
        Adafruit_INA219 ina219;

        float busVoltage_V = 0.0f;
        float shuntVoltage_mV = 0.0f;
        float current_mA = 0.0f;
        float power_mW = 0.0f;
        bool initDone = false;
        
        // Simulated values for SIMULATOR mode
        float simulatedBusVoltage_V = 12.0f;
        float simulatedShuntVoltage_mV = 10.0f;
        float simulatedCurrent_mA = 500.0f;
        float simulatedPower_mW = 6000.0f;
    } // namespace

    void Initialisation(void)
    {
        if (simulation)
        {
            println("INA219 init OK (SIMULATED)");
            initDone = true;
        }
        else
        {
            if (!ina219.begin())
            {
                printError("Failed to find INA219 chip");
                initDone = false;
                return;
            }

            println("INA219 init OK");
            // By default the INA219 will have an I2C address of 0x40

            // To use something like the default 32V 2A range (but with 32V to 5V
            // range that has 69.6mV LSB/incrementer move the slider to the 5A (1mV
            // increment) and then divide the output by 4.
            ina219.setCalibration_32V_2A();
            initDone = true;
        }
    }

    void UpdateMeasurements(void)
    {
        if (simulation)
        {
            initDone = true;
            busVoltage_V = simulatedBusVoltage_V;
            shuntVoltage_mV = simulatedShuntVoltage_mV;
            current_mA = simulatedCurrent_mA;
            power_mW = simulatedPower_mW;
            return;
        }

        // If initialization is not validated, retry it here.
        if (!initDone)
        {
            Initialisation();
        }
        else
        {
            busVoltage_V = ina219.getBusVoltage_V();
            shuntVoltage_mV = ina219.getShuntVoltage_mV();
            current_mA = ina219.getCurrent_mA();
            power_mW = ina219.getPower_mW();
        }
    }

    float getBusVoltage_V()
    {
        if (!initDone)
            UpdateMeasurements();
        return busVoltage_V;
    }

    float getShuntVoltage_mV()
    {
        if (!initDone)
            UpdateMeasurements();
        return shuntVoltage_mV;
    }

    float getCurrent_mA()
    {
        if (!initDone)
            UpdateMeasurements();
        return current_mA;
    }

    float getPower_mW()
    {
        if (!initDone)
            UpdateMeasurements();
        return power_mW;
    }

} // namespace PowerMonitor
