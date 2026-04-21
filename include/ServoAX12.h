#ifndef SERVO_AX12_H
#define SERVO_AX12_H

#include <Dynamixel2Arduino.h>
#include <unordered_map>

#include "ESP32_Hardware.h"

namespace ServoAX12
{
    // https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/tree/master

    constexpr size_t MAX_BAUD = 5;
    const BaudRate dxlBaud[MAX_BAUD] = {BaudRate::BAUD_RATE_57600,
                                        BaudRate::BAUD_RATE_115200,
                                        BaudRate::BAUD_RATE_1000000,
                                        BaudRate::BAUD_RATE_2000000,
                                        BaudRate::BAUD_RATE_3000000};

    enum class DxlProtocolVersion
    {
        PROTOCOL_1 = 1,
        PROTOCOL_2 = 2
    };

    constexpr size_t MAX_PROTOCOL = 2;
    const DxlProtocolVersion dxlProtocol[MAX_PROTOCOL] = {DxlProtocolVersion::PROTOCOL_1,
                                                          DxlProtocolVersion::PROTOCOL_2};

    // id vitesse acceleration position command_position ledState
    /**
     * @brief Structure représentant l'état d'un servo moteur.
     * @param id Identifiant du servo moteur.
     * @param vitesse Vitesse maximale du servo moteur en degrés par seconde.
     * @param positionMin Position minimale du servo moteur.
     * @param positionMax Position maximale du servo moteur.
     * @param command_position Position cible du servo moteur.
     * @param ledState État de la LED du servo moteur (allumée en mouvement, éteinte
     * sinon).
     */
    struct ServoMotion
    {
        uint8_t id;
        String name;
        float position;
        Hardware_Config::ServoPosition positionMin;
        Hardware_Config::ServoPosition positionMax;
        float command_position;
        bool IsMoving;
        bool ledState;

        ServoMotion()
        {
            id = (uint8_t)Hardware_Config::ServoID::BroadCast;
            name = "";
            position = 0;
            positionMin = Hardware_Config::ServoPosition::Min;
            positionMax = Hardware_Config::ServoPosition::Max;
            command_position = 0;
            IsMoving = false;
            ledState = false;
        }

        /**
         * @brief Construct a new Servo Motion object
         *
         * @param _id Identifiant du servo moteur
         * @param _vitesse Vitesse maximale du servo moteur en degrés par seconde
         * @param _acceleration Accélération maximale du servo moteur en degrés par
         * seconde carrée
         */
        ServoMotion(Hardware_Config::ServoID _id,
                    String _name,
                    Hardware_Config::ServoPosition _positionMin,
                    Hardware_Config::ServoPosition _positionMax)
        {
            // Initialisation des valeurs
            id = (uint8_t)_id;
            name = _name;
            position = 0;
            positionMin = _positionMin;
            positionMax = _positionMax;
            command_position = 0;
            IsMoving = false;
            ledState = false;
        }

        bool operator==(const ServoMotion &other) const
        {
            return id == other.id;
        }
    };

    void Initialisation(HardwareSerial &serial, int8_t rxPin, int8_t txPin, int8_t dirPin, BaudRate baudRate = BaudRate::BAUD_RATE_1000000, DxlProtocolVersion dxlProtocolVersion = DxlProtocolVersion::PROTOCOL_1);
    
    [[noreturn]] void TaskUpdateServo(void *pvParameters);

    void InitAllServo();
    void InitServo(ServoMotion &servo);
    
    void AddServo(Hardware_Config::ServoID id, String name, Hardware_Config::ServoPosition positionMin, Hardware_Config::ServoPosition positionMax);

    void StopAllServo();
    void StopServo(ServoMotion &servo);

    void StartAllServo();
    void StartServo(ServoMotion &servo);

    void UpdateAllServo();
    void UpdateServo(ServoMotion &servo);

    bool AreAllServoMoving();
    bool IsServoMoving(Hardware_Config::ServoID id);

    void SetServoPosition(Hardware_Config::ServoID id, Hardware_Config::ServoPosition position);
    void SetServoPosition(Hardware_Config::ServoID id, float position);
    
    void HandleCommand(Command cmd);
    const void PrintCommandHelp();
    int16_t Scan();
    int16_t Scan(DxlProtocolVersion _protocol, BaudRate _dxlBaud);
    void PrintDxlInfo(Hardware_Config::ServoID id = Hardware_Config::ServoID::BroadCast);

    void TeleplotAllPosition();
    void TeleplotPosition(Hardware_Config::ServoID id);
    void PrintAllPosition();
    void PrintPosition(Hardware_Config::ServoID id);
} // namespace ServoAX12

namespace std
{
    template <> struct hash<ServoAX12::ServoMotion>
    {
        std::size_t operator()(const ServoAX12::ServoMotion &k) const
        {
            // Hash only the name, since operator== only compares id
            return std::hash<uint8_t>()(k.id);
        }
    };
} // namespace std
#endif