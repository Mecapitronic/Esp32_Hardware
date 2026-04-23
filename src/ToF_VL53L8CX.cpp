#include "ToF_VL53L8CX.h"
using namespace Printer;

namespace ToF_VL53L8CX
{
    namespace
    {
        // Private namespace for internal state
        VL53L8CX sensor(&Wire, -1);      // Create sensor object with I2C interface
        VL53L8CX_ResultsData sensorData; // Result data class structure
        uint8_t imageResolution = 0;     // Current resolution (4x4=16 or 8x8=64)
        uint8_t imageWidth = 0;          // Width for printing (4 or 8)
        uint8_t errorStatus = false;     // Error status flag
        long measurements = 0;           // Used to calculate actual output rate
        long measurementStartTime = 0;   // Used to calculate actual output rate

        TaskThread taskUpdateVL53;

        // Helper functions for sending binary arrays
        void sendInt16Array(int16_t *data, size_t len)
        {
            for (size_t i = 0; i < len; i++)
            {
                int16_t val = data[i];
                Serial.write((uint8_t)(val & 0xFF));        // LSB
                Serial.write((uint8_t)((val >> 8) & 0xFF)); // MSB
            }
        }

        void sendUint16Array(uint16_t *data, size_t len)
        {
            for (size_t i = 0; i < len; i++)
            {
                uint16_t val = data[i];
                Serial.write((uint8_t)(val & 0xFF));        // LSB
                Serial.write((uint8_t)((val >> 8) & 0xFF)); // MSB
            }
        }

        void sendInt32Array(int32_t *data, size_t len)
        {
            for (size_t i = 0; i < len; i++)
            {
                int32_t val = data[i];
                Serial.write((uint8_t)(val & 0xFF));
                Serial.write((uint8_t)((val >> 8) & 0xFF));
                Serial.write((uint8_t)((val >> 16) & 0xFF));
                Serial.write((uint8_t)((val >> 24) & 0xFF));
            }
        }

        void sendUint32Array(uint32_t *data, size_t len)
        {
            for (size_t i = 0; i < len; i++)
            {
                uint32_t val = data[i];
                Serial.write((uint8_t)(val & 0xFF));
                Serial.write((uint8_t)((val >> 8) & 0xFF));
                Serial.write((uint8_t)((val >> 16) & 0xFF));
                Serial.write((uint8_t)((val >> 24) & 0xFF));
            }
        }
    } // namespace

    void Initialisation()
    {
        println("Init VL53L8CX");
        println("Initializing sensor board. This can take up to 10s. Please wait.");

        // Time how long it takes to transfer firmware to sensor
        long startTime = millis();
        sensor.begin();
        errorStatus = sensor.init();
        long stopTime = millis();

        if (errorStatus)
        {
            printError("Sensor initialization failed !");
        }
        else
        {
            println("Sensor initialization successful !");
        }

        float timeTaken = (stopTime - startTime) / 1000.0;
        println("Firmware transfer time: %0.3f s", timeTaken);

        sensor.set_resolution(DEFAULT_RESOLUTION); // Enable all 64 pads (8x8)

        sensor.get_resolution(&imageResolution); // Query sensor for current resolution
        imageWidth = sqrt(imageResolution);      // Calculate printing width

        // Using 4x4, min frequency is 1Hz and max is 60Hz
        // Using 8x8, min frequency is 1Hz and max is 15Hz
        sensor.set_ranging_frequency_hz(RANGING_FREQUENCY_HZ);

        sensor.start_ranging();

        measurementStartTime = millis();

        taskUpdateVL53 = TaskThread(TaskUpdateVL53, "TaskUpdateVL53", 10000, 15, 0);
    }

    void TaskUpdateVL53(void *pvParameters)
    {
        println("Start Task Update VL53");
        Chrono chrono("VL53", 100);
        uint8_t newDataReady = 0;
        while (true)
        {
            chrono.Start();
            try
            {
                if (!newDataReady)
                {
                    errorStatus = sensor.check_data_ready(&newDataReady);
                }
                if ((!errorStatus) && (newDataReady != 0))
                {
                    errorStatus = sensor.get_ranging_data(&sensorData); // Read distance data
                    newDataReady = 0;
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
            vTaskDelay(100);
        }
        println("VL53 Update Task STOPPED !");
    }

    // Getters for sensor data
    const VL53L8CX_ResultsData &getSensorData()
    {
        return sensorData;
    }

    uint8_t getImageWidth()
    {
        return imageWidth;
    }

    uint8_t getImageResolution()
    {
        return imageResolution;
    }

    bool isError()
    {
        return errorStatus;
    }

    void printProcessing()
    {
        Serial.print("VL53amb");
        sendUint32Array(sensorData.ambient_per_spad, 64);

        Serial.print("VL53tar");
        Serial.write(sensorData.nb_target_detected, 64);

        Serial.print("VL53spa");
        sendUint32Array(sensorData.nb_spads_enabled, 64);

        Serial.print("VL53sps");
        sendUint32Array(sensorData.signal_per_spad, 64);

        Serial.print("VL53sig");
        sendUint16Array(sensorData.range_sigma_mm, 64);

        Serial.print("VL53dis");
        sendInt16Array(sensorData.distance_mm, 64);

        Serial.print("VL53sta");
        Serial.write(sensorData.target_status, 64);

        Serial.print("VL53ref");
        Serial.write(sensorData.reflectance, 64);
    }

    void printFormattedOutput()
    {
        // Print formatted output
        println("Full distance grid (mm):");
        // Pretty-print data with increasing y, decreasing x to reflect reality
        for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
        {
            for (int x = imageWidth - 1; x >= 0; x--)
            {
                Serial.print(sensorData.distance_mm[x + y]);
                Serial.print("\t");
            }
            Serial.println();
        }
        Serial.println();
    }

    void printCSVOutput()
    {
        // CSV format output
        for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
        {
            for (int x = imageWidth - 1; x >= 0; x--)
            {
                Serial.print(sensorData.distance_mm[x + y]);
                Serial.print(",");
            }
        }
        Serial.println();
    }

} // namespace ToF_VL53L8CX
