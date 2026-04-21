#include "PowerMonitor.h"
using namespace Printer;

namespace PowerMonitor
{
    namespace
    {
        Adafruit_INA219 ina219;
    } // namespace

    void Initialisation(void)
    {
        if (!ina219.begin())
        {
            printError("Failed to find INA219 chip");
        }
        else
        {
            println("INA219 init OK");
            // By default the INA219 will have an I2C address of 0x40

            // To use something like the default 32V 2A range (but with 32V to 5V
            // range that has 69.6mV LSB/incrementer move the slider to the 5A (1mV
            // increment) and then divide the output by 4.

            ina219.setCalibration_32V_2A();
        }
    }

    float getBusVoltage_V()
    {
        return ina219.getBusVoltage_V();
    }

    float getShuntVoltage_mV()
    {
        return ina219.getShuntVoltage_mV();
    }

    float getCurrent_mA()
    {
        return ina219.getCurrent_mA();
    }

    float getPower_mW()
    {
        return ina219.getPower_mW();
    }

} // namespace PowerMonitor
