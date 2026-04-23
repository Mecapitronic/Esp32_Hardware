#include "ScreenSSD1306.h"
using namespace Printer;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
static const uint8_t I2C_SDA = SDA;
static const uint8_t I2C_SCL = SCL;
static const uint32_t clk = 400000UL;

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

namespace Screen
{
    namespace
    {
        Adafruit_SSD1306 display(
            SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, clk, clk);
        // https://github.com/pkolt/bitmap_editor
        // https://forum.arduino.cc/t/tuto-conversion-dune-image-bmp-ou-jpg-pour-affichage-sur-oled/1180079
        // https://javl.github.io/image2cpp/

        constexpr uint8_t pixelWidthFontSize = 6;
        constexpr uint8_t pixelHeightFontSize = 8;

        constexpr uint8_t fontSize = 1;

        constexpr uint8_t line1 = pixelHeightFontSize * fontSize * 0;
        constexpr uint8_t line2 = pixelHeightFontSize * fontSize * 1;
        constexpr uint8_t line3 = pixelHeightFontSize * fontSize * 2;
        constexpr uint8_t line4 = pixelHeightFontSize * fontSize * 3;
        constexpr uint8_t line5 = pixelHeightFontSize * fontSize * 4;
        constexpr uint8_t line6 = pixelHeightFontSize * fontSize * 5;
        constexpr uint8_t line7 = pixelHeightFontSize * fontSize * 6;
        constexpr uint8_t line8 = pixelHeightFontSize * fontSize * 7;

        TaskThread taskUpdateScreen;
        Timeout blinkTimeOut;
    } // namespace

    // Initialize the OLED display.
    void Initialisation(void)
    {
        // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
        if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
        {
            printError("SSD1306 allocation failed");
        }
        else
        {
            println("SSD1306 init OK");
        }
        // Clear the buffer
        display.clearDisplay();

        Screen::Logo();
        delay(500);
        blinkTimeOut.Start(500);

        taskUpdateScreen = TaskThread(TaskUpdateScreen, "TaskUpdateScreen", 2000, 15, 0);
    }

    // Clear contents of display buffer (set all pixels to off).
    void ClearDisplay()
    {
        display.clearDisplay();
    }

    // Push data currently in RAM to SSD1306 display.
    void Display()
    {
        display.display();
    }

    // If true, switch to invert mode (black-on-white), else normal mode (white-on-black).
    void InvertColor(bool invert)
    {
        display.invertDisplay(invert);
    }

    void Text(const String &text, int size, int cursorX, int cursorY, int color)
    {
        // for size 1, max number of letter for the screen width is 21 (128 / 6)
        // for size 1, max line for the screen height is 8 (64 / 8)
        //
        // for size 2, max number of letter for the screen width is 10 (128 / 12)
        // for size 2, max line for the screen height is 4 (64 / 16)
        //
        // for size 3, max number of letter for the screen width is 7 (128 / 18)
        // for size 3, max line for the screen height is 3 (64 / 24)
        //
        // for size 4, max number of letter for the screen width is 5 (128 / 24)
        // for size 4, max line for the screen height is 2 (64 / 32)

        display.setTextSize(size);
        display.setTextColor(color);
        if (cursorX < 0 || cursorX > display.width())
        {
            cursorX = 0;
        }
        if (cursorY < 0 || cursorY > display.height())
        {
            cursorY = 0;
        }
        display.setCursor(cursorX, cursorY);
        display.print(text);
    }

    // Display the logo in the center of the screen.
    void Logo(void)
    {
        // display.clearDisplay();
        display.drawBitmap((display.width() - mecapi_bmp_width) / 2,
                           (display.height() - mecapi_bmp_height) / 2,
                           mecapi_bmp,
                           mecapi_bmp_width,
                           mecapi_bmp_height,
                           1);
        display.display();
    }

    bool colorBAU = 0;
    bool colorWifi = 0;

    void TaskUpdateScreen(void *pvParameters)
    {
        println("Start Task Update Screen");
        Chrono chrono("Screen", 100);
        while (true)
        {
            chrono.Start();
            try
            {
                display.clearDisplay();
                String text = "";

                // -----------------------------------
                // Top Left
                if (IHM::switchMode == 0)
                    Text("TEST", fontSize, 0, line1);
                else if (IHM::switchMode == 1)
                    Text("MATCH", fontSize, 0, line1);
                else
                    Text("MODE ?", fontSize, 0, line1);

                if (IHM::team == IHM::Team::Jaune)
                    Text("JAUNE", fontSize, 0, line2);
                else if (IHM::team == IHM::Team::Bleu)
                    Text("BLEU", fontSize, 0, line2);
                else
                    Text("COLOR ?", fontSize, 0, line2);

                // Top Center
                text = "BAU";
                Text(text,
                     2,
                     SCREEN_WIDTH / 2 - pixelWidthFontSize * 2 * text.length() / 2,
                     0,
                     (colorBAU ? 1 : 0));
                if (IHM::bauReady == 0 && blinkTimeOut.IsTimeOut())
                    colorBAU = !colorBAU;
                else if (IHM::bauReady == 1)
                    colorBAU = 0;

                // Top Right
                switch (Match::matchState)
                {
                case Match::State::MATCH_BOOT:
                    text = "BOOT";
                    break;
                case Match::State::MATCH_WAIT:
                    text = "WAIT";
                    break;
                case Match::State::MATCH_RUN:
                    text = "RUN";
                    break;
                case Match::State::MATCH_STOP:
                    text = "STOP";
                    break;
                case Match::State::MATCH_END:
                    text = "END";
                    break;
                default:
                    break;
                }
                Text(text,
                     fontSize,
                     SCREEN_WIDTH - pixelWidthFontSize * fontSize * text.length(),
                     line1);

                int time = Match::getMatchTimeSec();
                int min = time / 60;
                int sec = time % 60;
                text = String(min) + ":" + (sec < 10 ? "0" : "") + String(sec);
                Text(text,
                     fontSize,
                     SCREEN_WIDTH - pixelWidthFontSize * fontSize * text.length(),
                     line2);

                // -----------------------------------
                // Mid Left
                Text("X  200", 1, 0, line4);
                Text("Y 1500", 1, 0, line5);
                Text("A  180", 1, 0, line6);

                // -----------------------------------
                // Bottom
                float voltage_V = PowerMonitor::getBusVoltage_V();
                float current_mA = PowerMonitor::getCurrent_mA();
                float battPercent = (voltage_V - PowerMonitor::minVoltage_V) / (PowerMonitor::maxVoltage_V - PowerMonitor::minVoltage_V) * 100;

                if (battPercent > 80)
                    display.drawBitmap(0,
                                       line8,
                                       battery_4_bmp,
                                       battery_bmp_width,
                                       battery_bmp_height,
                                       1);
                else if (battPercent > 60)
                    display.drawBitmap(0,
                                       line8,
                                       battery_3_bmp,
                                       battery_bmp_width,
                                       battery_bmp_height,
                                       1);
                else if (battPercent > 40)
                    display.drawBitmap(0,
                                       line8,
                                       battery_2_bmp,
                                       battery_bmp_width,
                                       battery_bmp_height,
                                       1);
                else if (battPercent > 20)
                    display.drawBitmap(0,
                                       line8,
                                       battery_1_bmp,
                                       battery_bmp_width,
                                       battery_bmp_height,
                                       1);
                else
                    display.drawBitmap(0,
                                       line8,
                                       battery_0_bmp,
                                       battery_bmp_width,
                                       battery_bmp_height,
                                       1);

                text = " " + String(voltage_V, 2) + "V " + String(current_mA, 2) + "mA";
                Text(text,
                     fontSize,
                     battery_bmp_width,
                     SCREEN_HEIGHT - pixelHeightFontSize * fontSize);
                
                colorWifi = 1;
                if (Wifi_Helper::IsEnable())
                {
                    if (Wifi_Helper::IsClientConnected())
                    {
                        display.drawBitmap(SCREEN_WIDTH - wifi_bmp_width,
                                       line8,
                                       wifi_ok_bmp,
                                       wifi_bmp_width,
                                       wifi_bmp_height,
                                       (colorWifi ? 1 : 0));
                    }
                    else if (Wifi_Helper::IsWifiConnected())
                    {
                        display.drawBitmap(SCREEN_WIDTH - wifi_bmp_width,
                                       line8,
                                       wifi_on_bmp,
                                       wifi_bmp_width,
                                       wifi_bmp_height,
                                       (colorWifi ? 1 : 0));
                    }
                    else
                    {
                        display.drawBitmap(SCREEN_WIDTH - wifi_bmp_width,
                                        line8,
                                        wifi_off_bmp,
                                        wifi_bmp_width,
                                        wifi_bmp_height,
                                        (colorWifi ? 1 : 0));
                    }
                }

                display.display();
            }
            catch (const std::exception &e)
            {
                printError(e.what());
            }
            if (chrono.Check())
            {
                printChrono(chrono);
            }
            vTaskDelay(100);
        }
        println("Screen Update Task STOPPED !");
    }

} // namespace Screen