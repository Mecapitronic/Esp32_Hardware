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
        constexpr uint8_t maxObstacles = 8;

        struct Element
        {
            uint8_t line;
            uint8_t startCol;
            String text;
            String oldText;
        };

        Timeout blinkTimeOut;

        Element elementMode{0, 0, "", ""};
        Element elementBau{0, 9, "", ""};
        Element elementPami{0, 14, "", ""};
        Element elementState{0, 16, "", ""};

        Element elementColor{1, 0, "", ""};
        Element elementLowBattery{1, 7, "", ""};
        Element elementTime{1, 16, "", ""};

        //Element elementBlankLine2{2, 0, "                     ", ""};
        Element elementObstacles{2, 0, "", ""};

        Element elementPosX{3, 0, "X  123", ""};
        Element elementPosTargetX{3, 6, "X  123", ""};
        Element elementAx12Title{3, 16, "AX12", ""};

        Element elementPosY{4, 0, "Y  456", ""};
        Element elementPosTargetY{4, 6, "Y  123", ""};
        Element elementServo1{4, 13, "Srv1:111", ""};

        Element elementPosA{5, 0, "A  789", ""};
        Element elementPosTargetA{5, 6, "A  123", ""};
        Element elementServo2{5, 13, "Srv2:222", ""};

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
            elementPami.oldText = "";
            elementTime.oldText = "";
            //elementBlankLine2.oldText = "";
            elementObstacles.oldText = "";
            elementLowBattery.oldText = "";
            elementPosX.oldText = "";
            elementPosTargetX.oldText = "";
            elementAx12Title.oldText = "";
            elementPosY.oldText = "";
            elementPosTargetY.oldText = "";
            elementServo1.oldText = "";
            elementPosA.oldText = "";
            elementPosTargetA.oldText = "";
            elementServo2.oldText = "";
            elementBlankLine6.oldText = "";
            elementBattery.oldText = "";
            elementWifi.oldText = "";
        }

        String MatchStateToText()
        {
            switch (Match::matchState)
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

        String TeamToText()
        {
            if (IHM::team == IHM::Team::Jaune)
            {
                return "JAUNE";
            }
            if (IHM::team == IHM::Team::Bleu)
            {
                return "BLEU";
            }
            return "COLOR?";
        }

        String ModeToText()
        {
            if (IHM::switchMode == 0)
            {
                return "TEST";
            }
            if (IHM::switchMode == 1)
            {
                return "MATCH";
            }
            return "MODE";
        }

        bool onHold = false;
        int obstacles[maxObstacles] = {};
        String ObstaclesToText()
        {
            String result = "Obs ";
            for (size_t i = 0; i < maxObstacles; i++)
            {
                if (obstacles[i] != 0)
                {
                    result += String(obstacles[i]);
                }
                else
                {
                    result += "-";
                }
            }
            
            if(onHold)
                result+=" HOLD";
            else
                result+="     ";
            return result;
        }

        String NumPamiToText()
        {
            if(Match::GetNumPami() == 10)
                return "R";
            return String(Match::GetNumPami());
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
            if (timeSec > 9999)
            {
                timeSec = 9999;
            }

            String out = String(timeSec);
            while (out.length() < 4)
            {
                out = " " + out;
            }
            return out + "s";
        }

        String WifiToText()
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

        String ServoToText(int servoNum)
        {
            ServoAX12::ServoMotion servo = ServoAX12::GetServoByNumber(servoNum);
            if (servo.config.ax12Id != (uint8_t)Hardware_Config::ServoID::BroadCast)
            {
                if(!servo.initialized)
                {
                    return String(servo.name) + ": ?";
                }
                return String(servo.name) + ":" + String(servo.position, 0);
            }
            else
            {
                return String(servoNum) + ": X";
            }
        }

        String BatteryToText()
        {
            return (Power::isPowerON() ? "ON  " : "OFF ") + String(Power::getBusVoltage_V(), 2) + "V " + String(Power::getCurrent_mA(), 0) + "mA";
        }

        String LowBatteryToText()
        {
            return Power::isLowVoltage() ? "! Low !" : "       ";
        }

        bool blinkState = false;
        String BAUToText()
        {
            if (IHM::bauReady == 1)
            {
                return "   ";
            }
            else
            {
                if (blinkTimeOut.IsTimeOut())
                {
                    blinkState = !blinkState;
                }
                return blinkState ? "BAU" : "   ";
            }
        }

        Pose pose = {0, 0, 0};
        String PoseXToText()
        {
            return "X " + String(pose.x);
        }

        String PoseYToText()
        {
            return "Y " + String(pose.y);
        }

        String PoseAToText()
        {
            return "A " + String(degrees(pose.h),0);
        }

        Pose target = {0, 0, 0};
        String TargetXToText()
        {
            return "X " + String(target.x);
        }

        String TargetYToText()
        {
            return "Y " + String(target.y);
        }

        String TargetAToText()
        {
            return "A " + String(degrees(target.h),0);
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
        blinkTimeOut.Start(500);
    }

    void ClearDisplay()
    {
        display.clear();
        ResetElements();
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
        int numPami = Match::GetNumPami();
        String logoTexte = "";
        if(numPami == 10 )
            logoTexte = "    ROBOT - START   ";
        else
            logoTexte = "    PAMI " + String(numPami) + " START   ";
        Element logo{2, 0, logoTexte, ""};
        write_element(logo);
    }

    void Update()
    {
        try
        {
            elementMode.text = ModeToText();
            elementBau.text = BAUToText();
            elementState.text = MatchStateToText();
            elementColor.text = TeamToText();
            elementPami.text = NumPamiToText();
            elementTime.text = FormatTimeSec();

            elementBattery.text = BatteryToText();
            elementWifi.text = WifiToText();

            elementLowBattery.text = LowBatteryToText();

            elementObstacles.text = ObstaclesToText();
            elementPosX.text = PoseXToText();
            elementPosY.text = PoseYToText();
            elementPosA.text = PoseAToText();

            elementPosTargetX.text = TargetXToText();
            elementPosTargetY.text = TargetYToText();
            elementPosTargetA.text = TargetAToText();

            elementServo1.text = ServoToText(0);
            elementServo2.text = ServoToText(1);
            write_element(elementMode);
            write_element(elementBau);
            write_element(elementPami);
            write_element(elementState);

            write_element(elementColor);
            write_element(elementLowBattery);
            write_element(elementTime);

            //write_element(elementBlankLine2);
            write_element(elementObstacles);

            write_element(elementPosX);
            write_element(elementPosTargetX);
            write_element(elementAx12Title);

            write_element(elementPosY);
            write_element(elementPosTargetY);
            write_element(elementServo1);

            write_element(elementPosA);
            write_element(elementPosTargetA);
            write_element(elementServo2);

            write_element(elementBlankLine6);

            write_element(elementBattery);
            write_element(elementWifi);
        }
        catch (const std::exception &e)
        {
            printError(e.what());
        }
    }

    void SetPose(const Pose newPose)
    {
        pose = newPose;
    }

    void SetTarget(const Pose newTarget)
    {
        target = newTarget;
    }

    void SetHold(bool hold)
    {
        onHold = hold;
    }

    void SetObstacle(int id, int num)
    {
        if(id < 0 || id >= maxObstacles)
        {
            return;
        }
        obstacles[id] = num;
    }

} // namespace Screen