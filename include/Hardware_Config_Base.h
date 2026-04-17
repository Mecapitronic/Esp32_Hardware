#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

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
        Test = 0,           // Servo pour soulever la planche
        BroadCast = 0xFE    // Broadcast ID pour communiquer avec tous les servos
    };
    
    //******************** Pins IHM
    constexpr size_t PIN_SWITCH = 14;
    constexpr size_t PIN_TEAM = 13;
    constexpr size_t PIN_BAU = 15;
    constexpr size_t PIN_START = 16;

    //******************** Pins LED - RGB
    constexpr size_t PIN_RGB_LED = 38;

    //******************** Pins Enable Power
    constexpr size_t PIN_EN_MCU = 3;

}
#endif