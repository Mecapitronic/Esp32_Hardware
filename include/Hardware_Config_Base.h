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
}
#endif