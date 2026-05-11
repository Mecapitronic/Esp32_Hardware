#include "ServoAX12.h"
using namespace Printer;
using namespace Hardware_Config;

namespace ServoAX12
{
    namespace
    {
        int8_t _rxPin;
        int8_t _txPin;
        int8_t _dirPin;
        BaudRate _baudRate;
        DxlProtocolVersion _dxlProtocolVersion;
        Dynamixel2Arduino dxl;

        bool scanning = false;

        TaskThread taskUpdateServo;
        std::unordered_map<Hardware_Config::ServoID, ServoMotion, std::hash<Hardware_Config::ServoID>> Servos;

        // Charge la config d'un servo depuis les préférences NVS, avec des valeurs par défaut
        // Clés NVS : "srv.<name>.id", "srv.<name>.min", "srv.<name>.pos1", "srv.<name>.pos2", "srv.<name>.max"
        // Limite NVS : 15 chars max par clé → nom servo limité à 7 chars
        ServoConfig LoadServoConfig(const String &name, const ServoConfig &defaults)
        {
            ServoConfig cfg = defaults;
            cfg.ax12Id = (uint8_t)Preferences_Helper::LoadFromPreference("srv." + name + ".id", (int32_t)defaults.ax12Id);
            cfg.positionCount = (uint8_t)Preferences_Helper::LoadFromPreference("srv." + name + ".cnt", (int32_t)defaults.positionCount);
            if (cfg.positionCount == 0 || cfg.positionCount > MAX_SERVO_POSITIONS)
            {
                cfg.positionCount = defaults.positionCount;
            }
            for (uint8_t i = 0; i < cfg.positionCount; ++i)
            {
                String key = "srv." + name + ".p" + String(i);
                cfg.positions[i] = Preferences_Helper::LoadFromPreference(key, defaults.positions[i]);
            }
            return cfg;
        }

        void GetPositionBounds(const ServoMotion &servo, float &minPos, float &maxPos)
        {
            minPos = (float)servo.config.positions[0];
            maxPos = (float)servo.config.positions[0];
            for (uint8_t i = 1; i < servo.config.positionCount; ++i)
            {
                float value = (float)servo.config.positions[i];
                if (value < minPos)
                    minPos = value;
                if (value > maxPos)
                    maxPos = value;
            }
        }

        // Check if servo exists and log error if not
        bool ServoExists(ServoID id)
        {
            if (Servos.find(id) == Servos.end())
            {
                printError("Servo ID " + String((uint8_t)id) + " not found");
                return false;
            }
            return true;
        }
    } // namespace

    void Initialisation(HardwareSerial &serial, int8_t rxPin, int8_t txPin, int8_t dirPin, BaudRate baudRate, DxlProtocolVersion dxlProtocolVersion)
    {
        _rxPin = rxPin;
        _txPin = txPin;
        _dirPin = dirPin;
        _baudRate = baudRate;
        _dxlProtocolVersion = dxlProtocolVersion;
        serial.setPins(_rxPin, _txPin);
        dxl = Dynamixel2Arduino(serial, _dirPin);
        // Set Port baudrate. This has to match with DYNAMIXEL baudrate.
        dxl.begin((unsigned long)_baudRate);
        // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
        dxl.setPortProtocolVersion((float)_dxlProtocolVersion);

        Servos.clear();

        taskUpdateServo = TaskThread(TaskUpdateServo, "TaskUpdateServo", 10000, 15, 0);
    }

    void TaskUpdateServo(void *pvParameters)
    {
        println("Start Task Update Servo");
        Chrono chrono("Servo", 1000);
        while (true)
        {
            chrono.Start();
            try
            {
                if (!scanning)
                {
                    for (auto &[servoKey, servo] : Servos)
                    {
                        // Ne pas réessayer trop souvent
                        uint32_t now = millis();
                        if (!servo.initialized && (now - servo.lastInitAttempt) > 3000)
                        {
                            println("Retrying init for Servo %s (ID %d)...", servo.name.c_str(), servo.config.ax12Id);
                            InitServo(servo);
                            servo.lastInitAttempt = now;
                        }
                        // Mettre à jour les servos, 1ms/servo
                        UpdateServo(servo);
                    }
                }
            }
            catch (const std::exception &e)
            {
                printError(e.what());
            }
            if (chrono.Check())
            {
                printChrono(chrono);
            }
            vTaskDelay(10);
        }
        println("Servo Update Task STOPPED !");
    }

    void InitServo(ServoMotion &servo)
    {
        if (servo.initialized || !Power::isPowerON())
        {
            return;
        }

        print("Init Servo ID : %i name : %s", servo.config.ax12Id, servo.name.c_str());
        if (simulation)
        {
            servo.position = servo.command_position = (float)servo.config.positions[0];
            servo.initialized = true;
            servo.failureCount = 0;
            println("Servo %s %d position: %f [SIM]", servo.name, servo.config.ax12Id, servo.position);
            return;
        }

        if (dxl.ping(servo.config.ax12Id))
        {
            PrintDxlInfo();

            servo.ledState = true;
            dxl.ledOn(servo.config.ax12Id);

            // Turn off torque when configuring items in EEPROM area
            dxl.torqueOff(servo.config.ax12Id);
            dxl.setOperatingMode(servo.config.ax12Id, OP_POSITION);
            dxl.torqueOn(servo.config.ax12Id);

            // sometimes : timeout at 100ms
            float presentPosition = dxl.getPresentPosition(servo.config.ax12Id, UNIT_DEGREE);
            if(presentPosition == 0.0f && dxl.getLastLibErrCode() != DXLLibErrorCode::DXL_LIB_OK)
            {
                servo.failureCount++;
            }
            else
            {
                servo.position = servo.command_position = presentPosition;
                servo.initialized = true;
                servo.failureCount = 0;
            }

            println("Servo %s %d position: %f", servo.name, servo.config.ax12Id, servo.position);
        }
        else
        {
            servo.initialized = false;
            servo.failureCount++;
            println(" NOT connected! (attempt %d)", servo.failureCount);
        }
    }

    void AddServo(ServoID logicalId, String name, const ServoConfig &defaults)
    {
        ServoConfig cfg = LoadServoConfig(name, defaults);
        Servos[logicalId] = ServoMotion(name, cfg);
        // Will be initialised in the Update Task
    }
/*
    ServoMotion GetServoByName(const String &name)
    {
        for (const auto &[servoKey, servo] : Servos)
        {
            if (servo.name == name)
            {
                return servo;
            }
        }
        // printError("Servo with name " + name + " not found");
        return ServoMotion(); // Return default ServoMotion
    }
*/
    ServoMotion GetServoByNumber(uint8_t number)
    {
        int i = 0;
        for (const auto &[servoKey, servo] : Servos)
        {
            if (i == number)
            {
                return servo;
            }
            i++;
        }
        // printError("Servo with number " + String(number) + " not found");
        return ServoMotion(); // Return default ServoMotion
    }
/*
    ServoMotion GetServoByID(Hardware_Config::ServoID logicalId)
    {
        if (Servos.find(logicalId) != Servos.end())
        {
            return Servos.at(logicalId);
        }
        // printError("Servo with ID " + String((uint8_t)logicalId) + " not found");
        return ServoMotion(); // Return default ServoMotion
    }

    ServoMotion GetServoByIDNumber(uint8_t ax12Id)
    {
        for (const auto &[sid, servo] : Servos)
        {
            if (servo.config.ax12Id == ax12Id)
            {
                return servo;
            }
        }
        // printError("Servo with ID " + String(ax12Id) + " not found");
        return ServoMotion(); // Return default ServoMotion
    }*/

    void StopAllServo()
    {
        println("Stop All Servo");
        for (auto &[servoKey, servo] : Servos)
        {
            StopServo(servo);
        }
    }

    void StopServo(ServoMotion &servo)
    {
        if (!simulation)
        {
            dxl.torqueOff(servo.config.ax12Id);
            dxl.ledOff(servo.config.ax12Id);
        }
    }

    void StartAllServo()
    {
        println("Start All Servo");
        for (auto &[servoKey, servo] : Servos)
        {
            StartServo(servo);
        }
    }

    void StartServo(ServoMotion &servo)
    {
        if (!simulation)
            dxl.torqueOn(servo.config.ax12Id);
    }

    void UpdateServo(ServoMotion &servo)
    {
        // Si le servo n'est pas initialisé, ne pas le forcer
        if (!servo.initialized)
        {
            return;
        }

        // Vérifier la tension avant de faire une opération I2C
        if (!Power::isPowerON()) // seuil de fonctionnement des AX12
        {
            // servo.IsMoving = false;
            return;
        }

        // On récupère la position actuelle du servo
        if (simulation)
        {
            servo.position = servo.position + (servo.command_position - servo.position) / 2;
        }
        else
        {
            // sometimes : timeout at 100ms
            float position = dxl.getPresentPosition(servo.config.ax12Id, UNIT_DEGREE);
            if(position == 0.0f && dxl.getLastLibErrCode() != DXLLibErrorCode::DXL_LIB_OK)
            {
                servo.failureCount++;
            }
            else            
                servo.position = position;
        }

        // On considère que le servo est en mouvement s'il est à plus ou moins de 5 degrés de la position commandée
        // on allume la LED pour indiquer qu'une commande est en cours
        // Si on a indiqué un timeOut à la commande, on considère que le servo est en mouvement tant que le timeOut n'est pas expiré
        // à la fin du timeOut (si != 0), le servo sera considéré arrivé (pour éviter de bloquer le code)
        if ((servo.position >= servo.command_position + 5 || servo.position <= servo.command_position - 5) && !servo.timeOut.IsTimeOut())
        {
            if (!servo.ledState)
            {
                servo.ledState = true;
                if (!simulation)
                    dxl.ledOn(servo.config.ax12Id);
            }
            servo.IsMoving = true;
            dxl.setGoalPosition(servo.config.ax12Id, servo.command_position, UNIT_DEGREE);
        }
        else
        {
            if (servo.ledState)
            {
                servo.ledState = false;
                if (!simulation)
                    dxl.ledOff(servo.config.ax12Id);
            }
            servo.IsMoving = false;
        }
    }

    bool AreAllServoMoving()
    {
        for (const auto &[servoKey, servo] : Servos)
        {
            if (servo.IsMoving)
            {
                return true;
            }
        }
        return false;
    }

    bool IsServoMoving(ServoID logicalId)
    {
        if (!ServoExists(logicalId))
            return false;
        return Servos.at(logicalId).IsMoving;
    }

    void SetServoPosition(ServoID logicalId, Hardware_Config::ServoPosition servoPosition, int timeOutMs)
    {
        if (!ServoExists(logicalId))
            return;
        ServoMotion &servo = Servos.at(logicalId);

        size_t index = static_cast<size_t>(servoPosition);
        if (index >= servo.config.positionCount)
        {
            println("SetServoPosition: index hors limite (%d >= %d)", (int)index, (int)servo.config.positionCount);
            return;
        }

        float position = (float)servo.config.positions[index];
        SetServoPosition(logicalId, position, timeOutMs);
    }

    void SetServoPosition(ServoID logicalId, float position, int timeOutMs)
    {
        if (!ServoExists(logicalId))
            return;
        ServoMotion &servo = Servos.at(logicalId);

        float minPos = 0.0f;
        float maxPos = 0.0f;
        GetPositionBounds(servo, minPos, maxPos);
        if (position < minPos || position > maxPos)
        {
            println("Position out of range for Servo ID : %i", logicalId);
            println("Position : %f", position);
            println("Min : %f", minPos);
            println("Max : %f", maxPos);
            return;
        }
        servo.command_position = position;
        servo.IsMoving = true;
        if (timeOutMs > 0)
        {
            servo.timeOut.Start(timeOutMs);
        }
        else
        {
            servo.timeOut.Stop();
        }
        // We set the command into the task, if power is enable
        // if (!simulation)
        //    dxl.setGoalPosition((uint8_t)logicalId, servo.command_position, UNIT_DEGREE);
    }

    float GetServoPosition(ServoID logicalId)
    {
        if (!ServoExists(logicalId))
            return -1.0f;
        return Servos.at(logicalId).position;
    }

    bool HandleCommand(Command cmd)
    {
        if (cmd.cmdEquals("AX12Scan"))
        {
            // AX12Scan:1:1000000
            if (cmd.size == 2)
            {
                Scan((DxlProtocolVersion)cmd.data[0], (BaudRate)cmd.data[1]);
            }
            else
            {
                Scan();
            }
        }
        else if (cmd.cmdEquals("AX12PrintInfo"))
        {
            // AX12PrintInfo:1
            if (cmd.size == 1)
            {
                ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                PrintDxlInfo(logicalId);
            }
            else
                PrintDxlInfo();
        }
        else if (cmd.cmdEquals("AX12Config"))
        {
            // AX12Config:<name>:<field>:<value>
            // field: id | cnt | p0..p9
            // AX12Config:VL53:p3:180
            if (cmd.size == 1 && strlen(cmd.dataStr1) > 0 && strlen(cmd.dataStr2) > 0)
            {
                String name(cmd.dataStr1);
                String field(cmd.dataStr2);
                bool isPositionField = field.length() >= 2 && field[0] == 'p';
                if (field != "id" && field != "cnt" && !isPositionField)
                {
                    printError("AX12Config: field invalide. Utiliser: id, cnt, p0..p9");
                    return true;
                }
                ConfigureServo(name, field, cmd.data[0]);
            }
            else
            {
                printError("AX12Config: usage -> AX12Config:<nom>:<field>:<valeur>");
            }
        }
        else if (cmd.cmdEquals("AX12ConfigReset"))
        {
            // AX12ConfigReset          -> réinitialise tous les servos
            // AX12ConfigReset:<name>   -> réinitialise un servo nommé
            if (strlen(cmd.dataStr1) > 0)
            {
                ResetServoConfig(String(cmd.dataStr1));
            }
            else
            {
                for (const auto &[servoKey, servo] : Servos)
                    ResetServoConfig(servo.name);
            }
        }
        else if (cmd.cmdEquals("AX12ConfigPrint"))
        {
            PrintServoConfigs();
        }
        else if (cmd.cmdEquals("AX12Pos"))
        {
            if (cmd.size == 2)
            {
                // AX12Pos:1:100
                ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                print("AX12 Servo id: %i ", logicalId);
                float position = static_cast<float>(cmd.data[1]);
                if (ServoExists(logicalId))
                {
                    println("Set Position : %f", position);
                    SetServoPosition(logicalId, position);
                }
                else
                {
                    println("Servo ID %i is not initialized", logicalId);
                }
            }
            else if (cmd.size == 1)
            {
                // AX12Pos:1
                ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                PrintPosition(logicalId);
            }
            else
            {
                PrintAllPosition();
            }
        }
        else if (cmd.cmdEquals("AX12Stop"))
        {
            println("AX12Stop");
            StopAllServo();
        }
        else if (cmd.cmdEquals("AX12Start"))
        {
            println("AX12Start");
            StartAllServo();
        }
        else
        {
            Printer::println("Not a AX12 command !");
            return false;
        }
        return true;
    }

    void PrintCommandHelp()
    {
        Printer::println("AX12 Command Help :");
        Printer::println(" > AX12Scan");
        Printer::println("      Scan all Dynamixel on all protocols and baudrates");
        Printer::println(" > AX12PrintInfo:[id]");
        Printer::println("      Print info for all servos or for the given id");
        Printer::println(" > AX12Config:<name>:<field>:<value>");
        Printer::println("      Configure a servo field (id|cnt|p0..p9) and save to NVS");
        Printer::println("      Example: AX12Config:VL53:p3:180");
        Printer::println(" > AX12ConfigReset[:<name>]");
        Printer::println("      Reset NVS config to defaults for one or all servos");
        Printer::println(" > AX12ConfigPrint");
        Printer::println("      Print current config of all registered servos");
        Printer::println(" > AX12Pos:[id]:[position]");
        Printer::println("      Set servo [id] to [position] (in degrees)");
        Printer::println("      If only 1 argument, print current position of the servo with the given id");
        Printer::println("      If no argument, print all currents positions");
        Printer::println(" > AX12Stop");
        Printer::println("      Stop all servos (torque off)");
        Printer::println(" > AX12Start");
        Printer::println("      Start all servos (torque on)");
        Printer::println();
    }

    int16_t Scan()
    {
        int16_t found_dynamixel = 0;
        for (auto &&proto : dxlProtocol)
        {
            for (auto &&baud : dxlBaud)
            {
                found_dynamixel += Scan(proto, baud);
                vTaskDelay(1);
            }
        }
        println("Total : %i Dynamixel(s) found", found_dynamixel);
        return found_dynamixel;
    }

    int16_t Scan(DxlProtocolVersion _protocol, BaudRate _dxlBaud)
    {
        scanning = true;
        // Save original protocol and version
        float version = dxl.getPortProtocolVersion();
        unsigned long baud = dxl.getPortBaud();
        int16_t found_dynamixel = 0;
        // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
        dxl.setPortProtocolVersion((float)_protocol);
        print("Scan Protocol %f - ", (float)_protocol);

        // Set Port baudrate.
        dxl.begin((int)_dxlBaud);
        println("Scan Baudrate %i", (int)_dxlBaud);
        for (int id = 0; id < DXL_BROADCAST_ID; id++)
        {
            // iterate until all ID in each baudrate is scanned.
            if (dxl.ping(id))
            {
                println("ID : %i, Model Number: %i", id, dxl.getModelNumber(id));
                found_dynamixel++;
            }
            vTaskDelay(1);
        }
        println("Found %i Dynamixel(s)", found_dynamixel);

        // Put back original protocol and version
        dxl.setPortProtocolVersion(version);
        dxl.begin(baud);

        scanning = false;
        return found_dynamixel;
    }

    void PrintDxlInfo(ServoID logicalId)
    {
        if (logicalId != ServoID::BroadCast)
        {
            if (ServoExists(logicalId))
            {
                ServoMotion &servo = Servos.at(logicalId);
                uint8_t ax12Id = servo.config.ax12Id;
                if (dxl.ping(ax12Id))
                {
                    println("ID : %i, Name: %s, Model Number: %i, position: %f, command: %f, isMoving: %i",
                            ax12Id, servo.name, dxl.getModelNumber(ax12Id), servo.position, servo.command_position, servo.IsMoving);
                }
                else
                {
                    println("Dynamixel ID : %i not found", ax12Id);
                }
            }
            else
            {
                println("Dynamixel ID : %i not found", (uint8_t)logicalId);
            }
        }
        else
        {
            for (const auto &[_id, servo] : Servos)
            {
                uint8_t ax12Id = servo.config.ax12Id;
                println("ID : %i, Name: %s, Model Number: %i, position: %f, command: %f, isMoving: %i",
                        ax12Id, servo.name, dxl.getModelNumber(ax12Id), servo.position, servo.command_position, servo.IsMoving);
            }
        }
    }

    void TeleplotAllPosition()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            teleplot("Servo " + servo.name + " " + String(servo.config.ax12Id), servo.position);
        }
    }

    void TeleplotPosition(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
            teleplot("Servo " + servo.name + " " + String(servo.config.ax12Id), servo.position);
        }
    }

    void PrintAllPosition()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            println("Servo %s %d : %f", servo.name, servo.config.ax12Id, servo.position);
        }
    }

    void PrintPosition(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
            println("Servo %s %d : %f", servo.name, servo.config.ax12Id, servo.position);
        }
    }

    void ConfigureServo(const String &name, const String &field, int32_t value)
    {
        String key = "srv." + name + "." + field;
        Preferences_Helper::SaveToPreference(key, value);
        // Mise à jour en mémoire si le servo est déjà enregistré
        for (auto &[servoKey, servo] : Servos)
        {
            if (servo.name == name)
            {
                if (field == "id")
                {
                    servo.config.ax12Id = (uint8_t)value;
                    servo.initialized = false;
                }
                else if (field == "cnt")
                {
                    if (value > 0 && value <= (int32_t)MAX_SERVO_POSITIONS)
                        servo.config.positionCount = (uint8_t)value;
                }
                else if (field.length() >= 2 && field[0] == 'p')
                {
                    int32_t index = field.substring(1).toInt();
                    if (index >= 0 && index < (int32_t)MAX_SERVO_POSITIONS)
                    {
                        servo.config.positions[index] = value;
                        if (index >= servo.config.positionCount)
                            servo.config.positionCount = (uint8_t)(index + 1);
                    }
                }
                println("Servo %s updated: %s = %d", name.c_str(), field.c_str(), value);
                break;
            }
        }
    }

    void ResetServoConfig(const String &name)
    {
        Preferences_Helper::RemoveFromPreference("srv." + name + ".id");
        Preferences_Helper::RemoveFromPreference("srv." + name + ".cnt");
        for (uint8_t i = 0; i < MAX_SERVO_POSITIONS; ++i)
        {
            Preferences_Helper::RemoveFromPreference("srv." + name + ".p" + String(i));
        }
        println("Servo config reset for '%s'. Redémarrer pour appliquer les valeurs par défaut.", name.c_str());
    }

    void PrintServoConfigs()
    {
        println("Servo Configs (%d servos):", Servos.size());
        for (const auto &[servoKey, servo] : Servos)
        {
            String line = "  " + servo.name + ": ax12Id=" + String(servo.config.ax12Id) + " cnt=" + String(servo.config.positionCount) + " pos=[";
            for (uint8_t i = 0; i < servo.config.positionCount; ++i)
            {
                if (i > 0)
                    line += ",";
                line += String(servo.config.positions[i]);
            }
            line += "] initialized=" + String(servo.initialized) + " isMoving=" + String(servo.IsMoving);
            println(line);
        }
    }
} // namespace ServoAX12