#ifndef MATCH_H
#define MATCH_H

#include "ESP32_Helper.h"
#include "ESP32_Hardware.h"

namespace Match
{
    //  BOOT à l'allumage, attend l'insertion de la tirette
    //  WAIT attente de démarrage via le retrait de la tirette
    //  RUN le match est en cours,
    //  STOP les actions sont terminées, on attend la fin du timer,
    //  END le match et le timer sont terminés
    enum class State
    {
        MATCH_BOOT,
        MATCH_WAIT,
        MATCH_RUN,
        MATCH_STOP,
        MATCH_END
    };

    // Time in ms
    static constexpr int time_start_match = 0;
    static constexpr int time_end_match = time_start_match + 100000;

    // Get the current state of the match
    extern State matchState;

    void Initialisation();

    void startMatchTimer();
    long getMatchTimeSec();
    long getMatchTimeMs();
    void CheckEndOfMatch();
    void printMatch();

    void TaskUpdateMatch(void *pvParameters);
} // namespace Match
#endif
