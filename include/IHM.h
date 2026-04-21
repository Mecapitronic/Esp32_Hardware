#ifndef IHM_H
#define IHM_H

#include <FastLED.h>

#include "ESP32_Hardware.h"

#define BUILTIN_BRIGHTNESS 50 // Brightness of the built-in LED

namespace IHM
{
    // Jaune = 0
    // Bleu  = 1 
    enum class Team
    {
        Jaune,
        Bleu,
        None
    };

    // Bleu = 0, Jaune = 1
    extern Team team;

    // Présente = 1, Absente = 0, None = -1
    extern int tirettePresent;

    // Match = 1, TEST = 0, None = -1
    extern int switchMode;

    //  Retiré (OK) = 1, Enclenché (NOK) = 0, None = -1
    extern int bauReady;

    extern bool useBlink;

    void Initialisation();
    void TaskUpdateIHM(void *pvParameters);

    void Blink();

    void PrintAll();
    void PrintTeam();
    void PrintSwitch();
    void PrintBAU();
    void PrintStart();

} // namespace IHM
#endif