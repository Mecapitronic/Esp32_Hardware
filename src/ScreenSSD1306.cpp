#include "ScreenSSD1306.h"

using namespace Printer;

#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

namespace Screen
{
    namespace
    {
        SSD1306AsciiWire display;

        constexpr uint8_t kRows = 8;
        constexpr uint8_t kCols = 21;
        constexpr uint8_t kCharWidthPx = 6;

        struct Element
        {
            uint8_t line;
            uint8_t startCol;
            String text;
            String oldText;
        };

        TaskThread taskUpdateScreen;
        Timeout blinkTimeOut;

        Element elementMode{0, 0, "", ""};
        Element elementBau{0, 9, "", ""};
        Element elementState{0, 16, "", ""};

        Element elementColor{1, 0, "", ""};
        Element elementTime{1, 16, "", ""};

        Element elementBlankLine2{2, 0, "                     ", ""};

        Element elementPosX{3, 0, "X  200", ""};
        Element elementAx12Title{3, 16, "AX12", ""};

        Element elementPosY{4, 0, "Y 1500", ""};
        Element elementServo1{4, 14, "1:100", ""};

        Element elementPosA{5, 0, "A  180", ""};
        Element elementServo2{5, 14, "2:200", ""};

        Element elementBlankLine6{6, 0, "                     ", ""};

        Element elementBattery{7, 0, "", ""};
        Element elementWifi{7, 17, "", ""};

        String ClipElementText(const Element &element)
        {
            if (element.startCol >= kCols)
            {
                return "";
            }

            const uint8_t maxLen = kCols - element.startCol;
            String out = element.text;
            if (out.length() > maxLen)
            {
                out = out.substring(0, maxLen);
            }
            return out;
        }

        void write_element(Element &element)
        {
            if (element.line >= kRows || element.startCol >= kCols)
            {
                return;
            }

            String newText = ClipElementText(element);
            if (newText == element.oldText)
            {
                return;
            }

            display.setCursor(element.startCol * kCharWidthPx, element.line);
            display.print(newText);

            if (element.oldText.length() > newText.length())
            {
                for (size_t i = 0; i < element.oldText.length() - newText.length(); ++i)
                {
                    display.print(' ');
                }
            }

            element.oldText = newText;
        }

        void ResetElements()
        {
            elementMode.oldText = "";
            elementBau.oldText = "";
            elementState.oldText = "";
            elementColor.oldText = "";
            elementTime.oldText = "";
            elementBlankLine2.oldText = "";
            elementPosX.oldText = "";
            elementAx12Title.oldText = "";
            elementPosY.oldText = "";
            elementServo1.oldText = "";
            elementPosA.oldText = "";
            elementServo2.oldText = "";
            elementBlankLine6.oldText = "";
            elementBattery.oldText = "";
            elementWifi.oldText = "";
        }

        String MatchStateToText(Match::State state)
        {
            switch (state)
            {
            case Match::State::MATCH_BOOT:
                return "BOOT";
            case Match::State::MATCH_WAIT:
                return "WAIT";
            case Match::State::MATCH_RUN:
                return "RUN";
            case Match::State::MATCH_STOP:
                return "STOP";
            case Match::State::MATCH_END:
                return "END";
            default:
                return "?";
            }
        }

        String TeamToText(IHM::Team team)
        {
            if (team == IHM::Team::Jaune)
            {
                return "JAUNE";
            }
            if (team == IHM::Team::Bleu)
            {
                return "BLEU";
            }
            return "COLOR?";
        }

        String FormatMatchTime()
        {
            int time = Match::getMatchTimeSec();
            int min = time / 60;
            int sec = time % 60;
            return String(min) + ":" + (sec < 10 ? "0" : "") + String(sec);
        }

        String FormatTimeSec()
        {
            int timeSec = Match::getMatchTimeSec();
            if (timeSec < 0)
            {
                timeSec = 0;
            }
            if (timeSec > 999)
            {
                timeSec = 999;
            }

            String out = String(timeSec);
            while (out.length() < 3)
            {
                out = " " + out;
            }
            return out + " s";
        }

        String WifiToText4()
        {
            if (!Wifi_Helper::IsEnable())
            {
                return "WF-X";
            }
            else
            {
                if (Wifi_Helper::IsClientConnected())
                {
                    return "WIFI";
                }
                else if (Wifi_Helper::IsWifiConnected())
                {
                    return "WI-!";
                }
                else
                {
                    return "WF-?";
                }
            }
        }

        bool blinkState = false;
        String BAUToText3()
        {
            if (IHM::bauReady == 1)
            {
                return "   ";
            }
            else
            {
                if(blinkTimeOut.IsTimeOut())
                {
                    blinkState = !blinkState;
                }
                return blinkState ? "BAU" : "   ";
            }
        }
    } // namespace

    void Initialisation(void)
    {
        display.begin(&Adafruit128x64, SCREEN_ADDRESS);
        println("SSD1306 init OK");

        display.setFont(System5x7);
        display.clear();
        ResetElements();

        Screen::Logo();
        delay(500);
        blinkTimeOut.Start(500);

        taskUpdateScreen = TaskThread(TaskUpdateScreen, "TaskUpdateScreen", 10000, 15, 0);
    }

    void ClearDisplay()
    {
        display.clear();
        ResetElements();
    }

    void Display()
    {
        // SSD1306Ascii écrit directement sur l'écran via I2C.
    }

    void Text(const String &text, uint8_t row, uint8_t col)
    {
        if (col >= kCols || row >= kRows)
        {
            return;
        }

        display.setCursor(col * kCharWidthPx, row);
        display.print(text);
    }

    void Logo(void)
    {
        Element logo{2, 0, "     PAMI READY", ""};
        write_element(logo);
    }

    void TaskUpdateScreen(void *pvParameters)
    {
        (void)pvParameters;
        println("Start Task Update Screen");
        Chrono chrono("Screen", 100);
        while (true)
        {
            chrono.Start();
            try
            {
                float voltage_V = PowerMonitor::getBusVoltage_V();
                float current_mA = PowerMonitor::getCurrent_mA();
                elementMode.text = (IHM::switchMode == 0) ? "TEST" : ((IHM::switchMode == 1) ? "MATCH" : "MODE");
                elementBau.text = BAUToText3();
                elementState.text = MatchStateToText(Match::matchState);

                elementColor.text = TeamToText(IHM::team);
                elementTime.text = FormatTimeSec();

                elementBattery.text = String("BAT ") + String(voltage_V, 2) + "V " + String(current_mA, 0) + "mA";
                elementWifi.text = WifiToText4();
                
                elementServo1.text = String(static_cast<uint8_t>(Hardware_Config::ServoID::VL53)) + ":" + String(ServoAX12::GetServoPosition(Hardware_Config::ServoID::VL53), 1);
                elementServo2.text = String(static_cast<uint8_t>(Hardware_Config::ServoID::Bras)) + ":" + String(ServoAX12::GetServoPosition(Hardware_Config::ServoID::Bras), 1);

                write_element(elementMode);
                write_element(elementBau);
                write_element(elementState);
                write_element(elementColor);
                write_element(elementTime);
                write_element(elementBlankLine2);
                write_element(elementPosX);
                write_element(elementAx12Title);
                write_element(elementPosY);
                write_element(elementServo1);
                write_element(elementPosA);
                write_element(elementServo2);
                write_element(elementBlankLine6);
                write_element(elementBattery);
                write_element(elementWifi);
            }
            catch (const std::exception &e)
            {
                printError(e.what());
            }
            if (chrono.Check())
            {
                printChrono(chrono);
            }
            vTaskDelay(200);
        }
        println("Screen Update Task STOPPED !");
    }

} // namespace Screen