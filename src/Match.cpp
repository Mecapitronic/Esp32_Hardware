#include "Match.h"
using namespace Printer;

namespace Match
{
    // Global variable definition
    State matchState = State::MATCH_BOOT;
    int numPami = 0;

    namespace
    {
        long elapsedTime = 0;
        long startTime = 0;

        TaskThread taskUpdateMatch;

    } // namespace

    void Initialisation()
    {
        int pami = Preferences_Helper::LoadFromPreference("NumPami", 0);
        if(pami == 0)
        {
            printError("N° PAMI : " + String(pami));
        }
        else
        {
            println("N° PAMI : %i", pami);
        }
        SetNumPami(pami);
        // Start the match timer task
        taskUpdateMatch = TaskThread(TaskUpdateMatch, "TaskUpdateMatch", 10000, 15, 0);
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
        else if(matchState == State::MATCH_BOOT)
            return millis();
        else
            return 0; // Match not started yet
    }

    void SetNumPami(int num)
    {
        numPami = num;
        Preferences_Helper::SaveToPreference("NumPami", numPami);
        println("N° PAMI : %i", numPami);
        Wifi_Helper::SetLocalIP("192.168.43." + String(100 + numPami));
    }

    int GetNumPami()
    {
        return numPami;
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
                    Power::EnablePower();

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
                    Power::EnablePower();
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
                    Power::DisablePower();
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

    bool HandleCommand(Command cmd)
    {
        if(cmd.cmdEquals("MatchNumPami") && cmd.size == 1)
        {
            int numPamiCmd = cmd.data[0];
            if(numPamiCmd >= 0 && numPamiCmd <= 99)
            {
                SetNumPami(numPamiCmd);
                return true;
            }
            else
            {
                printError("Invalid NumPami value: " + String(numPamiCmd));
                return false;
            }
        }
        else
        {
            printError("Unknown command: " + String(cmd.cmd));
            return false;
        }
    }

    void PrintCommandHelp()
    {
        println("Available Commands:");
        println("NumPami:<value> - Set the PAMI number (1-99)");
    }
} // namespace Match