#include "Match.h"
using namespace Printer;

namespace Match
{
    // Global variable definition
    State matchState = State::MATCH_BOOT;

    namespace
    {
        long elapsedTime = 0;
        long startTime = 0;

        TaskThread taskUpdateMatch;

    } // namespace

    void Initialisation()
    {
        // Start the match timer task
        TaskThread taskUpdateMatch =
            TaskThread(TaskUpdateMatch, "TaskUpdateMatch", 2000, 15, 0);
    }

    void startMatchTimer()
    {
        startTime = millis();
    }

    long getMatchTimeSec()
    {
        return (getMatchTimeMs()) / 1000;
    }

    long getMatchTimeMs()
    {
        if (matchState != State::MATCH_BOOT && matchState != State::MATCH_WAIT)
            return millis() - startTime;
        else
            return 0; // Match not started yet
    }

    void CheckEndOfMatch()
    {
        if ((matchState == State::MATCH_RUN) || (matchState == State::MATCH_STOP) && IHM::switchMode == 1)
        {
            // robot still running or waiting for end of match
            elapsedTime = millis() - startTime;
            if (elapsedTime >= time_end_match)
            {
                matchState = State::MATCH_END;
            }
        }
        if(matchState == State::MATCH_STOP && IHM::switchMode == 0 && IHM::tirettePresent == 1)
        {
            matchState = State::MATCH_END;
        }
    }

    void printMatch()
    {
        print("Match State : ");
        switch (matchState)
        {
            ENUM_PRINT(State::MATCH_BOOT);
            ENUM_PRINT(State::MATCH_WAIT);
            ENUM_PRINT(State::MATCH_RUN);
            ENUM_PRINT(State::MATCH_STOP);
            ENUM_PRINT(State::MATCH_END);
        }
    }

    // Note the 1 Tick delay, this is need  so the watchdog doesn't get confused
    void TaskUpdateMatch(void *pvParameters)
    {
        println("Start Task Update Match");
        Chrono chrono("Match", 1000);
        while (true)
        {
            chrono.Start();
            try
            {
                // Etat au boot du robot, en attente de l'insertion de la tirette
                if (Match::matchState == State::MATCH_BOOT)
                {
                    if (IHM::tirettePresent == 1)
                    {
                        Match::matchState = State::MATCH_WAIT;
                        Match::printMatch();
                    }
                }

                // Tirette présente, en attente de retrait de la tirette pour démarrer le match
                if (Match::matchState == State::MATCH_WAIT)
                {
                    // Lecture do codage du numéro de PAMI
                    /*int numPamiTmp = ReadNumPami();
                    if (numPamiTmp != numPami)
                    {
                        numPami = numPamiTmp;
                        println("N° PAMI : %i", numPami);
                        Wifi_Helper::SetLocalIP("192.168.137."
                                                + String(100 + numPami + 1));
                    }*/

                    if (IHM::tirettePresent == 0)
                    {
                        Match::startMatchTimer();
                        Match::matchState = State::MATCH_RUN;
                        Match::printMatch();
                    }
                }

                // Match en cours
                if (Match::matchState == State::MATCH_RUN)
                {
                    //println("Start of Match !");
                    /*
                    int lastMatchTime = 0;
                    while(Match::getMatchTimeMs() < Match::time_start_match &&
                    IHM::switchMode == 1)
                    {
                      // Countdown to start
                      if (lastMatchTime != (int)(Match::getMatchTimeSec()))
                      {
                          println("Match Time : %i", (int)(Match::getMatchTimeSec()));
                          lastMatchTime = (int)(Match::getMatchTimeSec());
                      }
                      vTaskDelay(1);
                    }*/
/*
                    if (IHM::switchMode == 1)
                    {
                        println("Mode Match !");
                    }
                    else
                    {
                        println("Mode Test !");
                    }*/

                    CheckEndOfMatch();
                    // Fin des actions
                    //Match::matchState = State::MATCH_STOP;
                }

                // Arrêt des actions, en attente de la fin du timer pour terminer le match
                if (Match::matchState == State::MATCH_STOP)
                {
                    CheckEndOfMatch();
                    //Match::matchState = State::MATCH_END;
                }

                // Fin du match
                if (Match::matchState == State::MATCH_END)
                {
                    // Disable Motor & Servo Power
                    digitalWrite(Hardware_Config::PIN_EN_MCU, LOW);
                    IHM::useBlink = false;
                    //ServoAX12::StopAllServo();
                }
            }
            catch (std::exception const &e)
            {
                printError(e.what());
            }
            if (chrono.Check())
            {
                printChrono(chrono);
            }
            vTaskDelay(10);
        }
        println("Match Update Task STOPPED !");
    }
} // namespace Match