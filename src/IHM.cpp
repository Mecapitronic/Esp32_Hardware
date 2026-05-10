#include "IHM.h"
using namespace Printer;
using namespace Hardware_Config;

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

        CLEDController *builtInLEDController = nullptr;
        CRGB builtin_led;
        constexpr int stripLEDCount = 9;
        constexpr uint8_t teamLedIndex = 0;
        constexpr uint8_t modeLedIndex = stripLEDCount - 1;
        CLEDController *stripLEDController = nullptr;
        CRGB strip_led[stripLEDCount];
    } // namespace

    void Initialisation()
    {
        // Tirette
        pinMode(Hardware_Config::PIN_START, INPUT_PULLUP);

        // Switch Color
        pinMode(Hardware_Config::PIN_TEAM, INPUT_PULLUP);

        // Switch Switch
        pinMode(Hardware_Config::PIN_SWITCH, INPUT_PULLUP);

        // Boutton Arret d'Urgence
        pinMode(Hardware_Config::PIN_BAU, INPUT);

        ledTimeOut.Start(1000);

        tirettePresent = !digitalRead(Hardware_Config::PIN_START);
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

        builtInLEDController = &FastLED.addLeds<WS2812, Hardware_Config::PIN_RGB_LED, RGB>(&builtin_led, 1);
        builtin_led = CRGB::Purple;
        if (builtInLEDController)
        {
            builtInLEDController->showLeds(BUILTIN_BRIGHTNESS);
        }

        stripLEDController = &FastLED.addLeds<NEOPIXEL, Hardware_Config::PIN_WS2812_LED>(strip_led, stripLEDCount);
        fill_solid(strip_led, stripLEDCount, CRGB::Purple);
        if (stripLEDController)
        {
            stripLEDController->showLeds(BUILTIN_BRIGHTNESS);
        }
    }

    void Update()
    {
        try
        {
            // Lecture du bouton Team Yellow / Blue
            Team teamTmp = (Team)digitalRead(Hardware_Config::PIN_TEAM);
            if (teamTmp != team)
            {
                team = teamTmp;
                PrintTeam();
            }

            // Lecture du bouton Switch TEST / OK
            int switchTmp = digitalRead(Hardware_Config::PIN_SWITCH);
            if (switchTmp != switchMode)
            {
                switchMode = switchTmp;
                PrintSwitch();
            }

            // Lecture de la tirette
            int tiretteTmp = !digitalRead(Hardware_Config::PIN_START);
            if (tiretteTmp != tirettePresent)
            {
                tirettePresent = tiretteTmp;
                PrintStart();
            }

            // Lecture du BAU
            int bauTmp = digitalRead(Hardware_Config::PIN_BAU);
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
    }

    void Blink()
    {
        CRGB teamColor = CRGB::Black;
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
            fill_solid(strip_led, stripLEDCount, CRGB::Red);
        }
        else
        {
            builtin_led = teamColor;
            fill_solid(strip_led, stripLEDCount, CRGB::Black);
        }

        if (useBlink)
        {
            if (ledTimeOut.IsTimeOut())
            {
                ledState = !ledState;
            }

            if (ledState)
            {
                builtin_led = teamColor;
            }
            else
            {
                builtin_led = bauReady ? CRGB::Black : CRGB::Red;
            }
        }
        renderStatusIndicators();
        if (builtInLEDController != nullptr)
            builtInLEDController->showLeds(BUILTIN_BRIGHTNESS);
        if (stripLEDController != nullptr)
            stripLEDController->showLeds(BUILTIN_BRIGHTNESS);
    }

    void renderStatusIndicators()
    {
        const CRGB teamColor = (IHM::team == IHM::Team::Jaune) ? CRGB::Gold : ((IHM::team == IHM::Team::Bleu) ? CRGB::DodgerBlue : CRGB::White);
        const CRGB modeColor = (IHM::switchMode == 1) ? CRGB::Green : ((IHM::switchMode == 0) ? CRGB::Violet: CRGB::White);

        strip_led[teamLedIndex] = teamColor;
        strip_led[modeLedIndex] = modeColor;
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