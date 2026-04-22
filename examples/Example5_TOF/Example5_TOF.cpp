/*
    VL53L8CX Time-of-Flight Sensor Example
    
    This example demonstrates how to use the VL53L8CX sensor for distance measurement.
    The sensor provides an 8x8 grid of distance measurements (64 pixels).
    
    The serial communication starts at 921600 baud rate.
*/

#include "ESP32_Hardware.h"
using namespace Printer;

void setup(void)
{
    ESP32_Helper::Initialisation();
    Wire.begin(SDA, SCL, 400000UL);
    ToF_VL53L8CX::Initialisation();
    
    // Update sensor data
    ToF_VL53L8CX::Update();
    
    // Display sensor resolution and status
    println("Resolution: %u pixels (%ux%u)",
            ToF_VL53L8CX::getImageResolution(),
            ToF_VL53L8CX::getImageWidth(),
            ToF_VL53L8CX::getImageWidth());
}

void loop(void)
{
    // Update sensor data
    ToF_VL53L8CX::Update();

    // Send data to be diiplay in Processing
    ToF_VL53L8CX::printProcessing();
    
    // Display raw distance data
    //const VL53L8CX_ResultsData& data = ToF_VL53L8CX::getSensorData();
    //println("Center pixel distance: %d mm", data.distance_mm[32]);  // Center of 8x8 grid
        
    // Show error status if any
    if (ToF_VL53L8CX::isError())
    {
        printError("Sensor error detected!");
    }
    
    delay(100);
}
