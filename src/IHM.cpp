#include "IHM.h"

using namespace Printer;
using namespace std;

namespace IHM
{
    // Global variable definitions
    Team team = Team::None;
    int tirettePresent = -1;
    int switchMode = -1;
    int bauReady = -1;
    bool useBlink = true;

    namespace
    {
        int ledState = LOW;
        Timeout ledTimeOut;

        TaskThread taskUpdateIHM;
        CLEDController *LEDcontroller;
        CRGB builtin_led;
    } // namespace


    void Initialisation()
    {
        // Tirette
        pinMode(PIN_START, INPUT_PULLUP);

        // Switch Color
        pinMode(PIN_TEAM, INPUT_PULLUP);

        // Switch Switch
        pinMode(PIN_SWITCH, INPUT_PULLUP);

        // Boutton Arret d'Urgence
        pinMode(PIN_BAU, INPUT);

        ledTimeOut.Start(1000);

        tirettePresent = !digitalRead(PIN_START);
        if (tirettePresent == 1)
        {
            println("Tirette : Présente au démarrage");
            ledTimeOut.timeOut = 500;
        }
        else if (tirettePresent == 0)
        {
            println("Tirette : Absente au démarrage");
            ledTimeOut.timeOut = 500;
        }

        LEDcontroller = &FastLED.addLeds<WS2812, Hardware_Config::PIN_RGB_LED, RGB>(&builtin_led, 1);
        builtin_led = CRGB::Black;
        LEDcontroller->showLeds(BUILTIN_BRIGHTNESS);

        taskUpdateIHM = TaskThread(TaskUpdateIHM, "TaskUpdateIHM", 2000, 15, 0);
        
        println("Vérifier que le BAU est retiré pour démarrer le robot");
        while (bauReady != 1)
        {
            println("[!] Retirer le BAU [!]");
            delay(500);
        }
    }

    void TaskUpdateIHM(void *pvParameters)
    {
        println("Start Task Update IHM");
        while (true)
        {
            try
            {
                // Lecture du bouton Team Yellow / Blue
                Team teamTmp = (Team)digitalRead(PIN_TEAM);
                if (teamTmp != team)
                {
                    team = teamTmp;
                    PrintTeam();
                }

                // Lecture du bouton Switch TEST / OK
                int switchTmp = digitalRead(PIN_SWITCH);
                if (switchTmp != switchMode)
                {
                    switchMode = switchTmp;
                    PrintSwitch();
                }

                // Lecture de la tirette
                int tiretteTmp = !digitalRead(PIN_START);
                if (tiretteTmp != tirettePresent)
                {
                    tirettePresent = tiretteTmp;
                    PrintStart();
                }

                // Lecture du BAU
                int bauTmp = digitalRead(PIN_BAU);
                if (bauTmp != bauReady)
                {
                    bauReady = bauTmp;
                    PrintBAU();
                }
                Blink();
            }
            catch (const std::exception &e)
            {
                printError(e.what());
            }
            vTaskDelay(10);
        }
        println("IHM Update Task STOPPED !");
    }

    void Blink()
    {
        CRGB teamColor;
        if (team == Team::Jaune)
        {
            teamColor = CRGB::Gold;
        }
        if (team == Team::Bleu)
        {
            teamColor = CRGB::Blue;
        }

        // no blink behavior
        if (!bauReady)
        {
            builtin_led = CRGB::Red;
        }
        else
        {
            builtin_led = teamColor;
        }

        if (useBlink)
        {
            if (ledTimeOut.IsTimeOut())
            {
                ledState = !ledState;
            }

            if (ledState)
                builtin_led = teamColor;
            else
            {
                builtin_led = bauReady ? CRGB::Black : CRGB::Red;
            }
        }
        LEDcontroller->showLeds(BUILTIN_BRIGHTNESS);
    }

    void PrintAll()
    {
        PrintTeam();
        PrintSwitch();
        PrintBAU();
        PrintStart();
    }

    void PrintTeam()
    {
        print("Team    : ");
        if (team == Team::Bleu)
            println("Bleu");
        else if (team == Team::Jaune)
            println("Jaune");
    }

    void PrintSwitch()
    {
        print("Switch  : ");
        if (switchMode == 1)
            println("MATCH");
        else
            println("TEST");
    }

    void PrintBAU()
    {
        print("BAU     : ");
        if (bauReady == 1)
            println("Retiré");
        else
            println("Enclenché");
    }

    void PrintStart()
    {
        print("Tirette : ");
        if (tirettePresent == 1)
            println("Insérée");
        else if (tirettePresent == 0)
            println("Enlevée");
    }
} // namespace IHM