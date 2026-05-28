#ifndef SERVO_AX12_H
#define SERVO_AX12_H

#include <array>
#include <Dynamixel2Arduino.h>
#include <unordered_map>

#include "ESP32_Hardware.h"

namespace ServoAX12
{
        constexpr size_t MAX_SERVO_POSITIONS = 10;
        static_assert(Hardware_Config::ServoPositionCount <= MAX_SERVO_POSITIONS,
                      "ServoPositionCount exceeds MAX_SERVO_POSITIONS");

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

    /**
     * @brief Structure représentant la configuration d'un servo moteur.
     * @param ax12Id ID physique du servo moteur sur le bus Dynamixel
     * @param positions Tableau des positions prédéfinies pour ce servo moteur.
     *                  positions[0] = min, positions[positionCount-1] = max
     * @param positionCount Nombre de positions valides dans le tableau (doit être <= MAX_SERVO_POSITIONS)
     */
    struct ServoConfig
    {
        uint8_t ax12Id = (uint8_t)Hardware_Config::ServoID::BroadCast;
        std::array<int32_t, MAX_SERVO_POSITIONS> positions{};
        uint8_t positionCount = 1;

        ServoConfig() = default;
        ServoConfig(uint8_t _ax12Id, const std::array<int32_t, MAX_SERVO_POSITIONS> &_positions, uint8_t _positionCount)
            : ax12Id(_ax12Id), positions(_positions), positionCount(_positionCount) {}
        void AddPosition(int32_t position, Hardware_Config::ServoPosition index)
        {
            size_t idx = static_cast<size_t>(index);
            if (idx < MAX_SERVO_POSITIONS)
            {
                if (positionCount == 0 || positionCount > MAX_SERVO_POSITIONS)
                {
                    positionCount = 1;
                }
                positions[idx] = position;
                if (positionCount <= idx)
                {
                    positionCount = idx + 1;
                }
            }
        }
    };

    /**
     * @brief Structure représentant l'état d'un servo moteur.
     * @param id Identifiant du servo moteur.
     * @param position Position actuelle du servo moteur.
     * @param command_position Position cible du servo moteur.
     * @param ledState État de la LED du servo moteur (allumée en mouvement, éteinte sinon).
     * sinon).
     */
    struct ServoMotion
    {
        String name = "";
        int position = 0;
        ServoConfig config = ServoConfig((uint8_t)Hardware_Config::ServoID::BroadCast, {0}, 1);
        int command_position = 0;
        bool IsMoving = false;
        bool ledState = false;
        Timeout timeOut;
        bool goalPositionAcked = false;
        // Tracking d'état d'initialisation
        bool initialized = false;       // Servo connecté et prêt
        int initState = 0;
        uint32_t lastInitAttempt = 0;   // Timestamp du dernier tentative d'init (ms)
        int failureCount = 0;           // Compteur d'erreurs consécutives

        ServoMotion() = default;

        /**
         * @brief Construct a new Servo Motion object
         *
         * @param _name Nom du servo
         * @param _config Configuration du servo
         */
        ServoMotion(String _name,
                    const ServoConfig &_config)
        {
            // Initialisation des valeurs
            name = _name;
            config = _config;
            command_position = (int)config.positions[0];
        }

        bool operator==(const ServoMotion &other) const
        {
            return config.ax12Id == other.config.ax12Id;
        }
    };

    void Initialisation(HardwareSerial &serial, int8_t rxPin, int8_t txPin, int8_t dirPin, BaudRate baudRate = BaudRate::BAUD_RATE_1000000, DxlProtocolVersion dxlProtocolVersion = DxlProtocolVersion::PROTOCOL_1);

    [[noreturn]] void TaskUpdateServo(void *pvParameters);

    void InitServo(ServoMotion &servo);
    void AddServo(Hardware_Config::ServoID logicalId, String name, const ServoConfig &defaults);
    void AddOrUpdateServo(Hardware_Config::ServoID logicalId, const String &name, const ServoConfig &config);

    ServoMotion GetServoByNumber(uint8_t number);

    void UpdateServo(ServoMotion &servo);

    bool AreAllServoMoving();
    bool IsServoMoving(Hardware_Config::ServoID logicalId);

    void SetServoPosition(Hardware_Config::ServoID logicalId, Hardware_Config::ServoPosition servoPosition, int timeOutMs = 0);
    void SetServoPosition(Hardware_Config::ServoID logicalId, int position, int timeOutMs = 0);

    int GetServoPosition(Hardware_Config::ServoID logicalId);

    bool WriteControlTableItem(ControlTableItem::ControlTableItemIndex item,
                               uint8_t id,
                               int32_t value,
                               uint32_t timeout = 10);

    bool HandleCommand(Command cmd);
    void PrintCommandHelp();

    int16_t Scan();
    int16_t Scan(DxlProtocolVersion _protocol, BaudRate _dxlBaud);
    void PrintDxlInfo(Hardware_Config::ServoID logicalId = Hardware_Config::ServoID::BroadCast);

    void PrintServoConfigs();

    void TeleplotAllPosition();
    void TeleplotPosition(Hardware_Config::ServoID logicalId);
    void PrintAllPosition();
    void PrintPosition(Hardware_Config::ServoID logicalId);
} // namespace ServoAX12

namespace std
{
    template <> struct hash<ServoAX12::ServoMotion>
    {
        std::size_t operator()(const ServoAX12::ServoMotion &k) const
        {
            // Hash on physical AX12 ID, same key as operator==
            return std::hash<uint8_t>()(k.config.ax12Id);
        }
    };
} // namespace std
#endif