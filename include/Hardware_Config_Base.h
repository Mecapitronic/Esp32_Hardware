#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

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