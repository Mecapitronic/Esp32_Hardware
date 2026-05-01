#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H
#include <cstddef>
#include <HardwareSerial.h>
#include <pins_arduino.h>
// Copy this file to your include project folder and change it's name to Hardware_Config.h
// Then you can modify the one in your project at your convenience

#ifdef SIMULATOR
constexpr bool simulation = true;
#else
constexpr bool simulation = false;
#endif

namespace Hardware_Config
{
    enum class ServoPosition
    {
        TestMin = 0,
        TestTest = 145,
        TestMax = 290,

        Min = 0,
        Max = 290
    };
    
    enum class ServoID
    {
        Test = 0,
        BroadCast = 0xFE    // Broadcast ID pour communiquer avec tous les servos
    };    

    //******************** Pins SERVO
    #define SERIAL_SERVO Serial1
    constexpr size_t RX_SERVO = 18;
    constexpr size_t TX_SERVO = 17;
    constexpr size_t PIN_SERVO_DIR = 10;
    
    //******************** Pins IHM
    constexpr size_t PIN_SWITCH = 14;
    constexpr size_t PIN_TEAM = 13;
    constexpr size_t PIN_BAU = 15;
    constexpr size_t PIN_START = 16;

    //******************** Pins LED - RGB
    constexpr size_t PIN_RGB_LED = 38;
    constexpr size_t PIN_WS2812_LED = 12;

    //******************** Pins Enable Power
    constexpr size_t PIN_EN_MCU = 3;

    //******************** Pins TwoWire I²C
    constexpr size_t PIN_SDA = SDA;
    constexpr size_t PIN_SCL = SCL;

}
#endif