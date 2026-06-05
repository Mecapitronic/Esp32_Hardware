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

        struct ServoInitStep
        {
            ControlTableItem::ControlTableItemIndex item;
            int32_t value;
        };

        // AX12 init sequence order, indexed by servo.initState
        constexpr ServoInitStep kServoInitSteps[] = {
            {ControlTableItem::ControlTableItemIndex::TORQUE_ENABLE, 0},
            {ControlTableItem::ControlTableItemIndex::CW_ANGLE_LIMIT, 0}, //EEPROM, 0 by default
            {ControlTableItem::ControlTableItemIndex::CCW_ANGLE_LIMIT, 1023}, //EEPROM, 1023 by default
            {ControlTableItem::ControlTableItemIndex::TORQUE_LIMIT, 200},
            {ControlTableItem::ControlTableItemIndex::MAX_TORQUE, 200}, //EEPROM, 1023 by default
            {ControlTableItem::ControlTableItemIndex::CW_COMPLIANCE_SLOPE, 8},
            {ControlTableItem::ControlTableItemIndex::CCW_COMPLIANCE_SLOPE, 8},
            {ControlTableItem::ControlTableItemIndex::MOVING_SPEED, 200},
            {ControlTableItem::ControlTableItemIndex::TORQUE_ENABLE, 1},
        };
        constexpr uint8_t kServoInitStepCount =
            sizeof(kServoInitSteps) / sizeof(kServoInitSteps[0]);

        void GetPositionBounds(const ServoMotion &servo, int &minPos, int &maxPos)
        {
            minPos = servo.config.positions[0];
            maxPos = servo.config.positions[0];
            for (uint8_t i = 1; i < servo.config.positionCount; ++i)
            {
                int value = servo.config.positions[i];
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
        if (servo.initialized || Power::isLowVoltage())
        {
            return;
        }

        println("Init Servo ID : %i name : %s - Init State %d",
                servo.config.ax12Id,
                servo.name.c_str(),
                servo.initState);
        if (simulation)
        {
            servo.position = servo.command_position = servo.config.positions[0];
            servo.speed = 0;
            servo.command_speed = 50;
            servo.initialized = true;
            servo.failureCount = 0;
            println("Servo %s %d position: %d [SIM]",
                    servo.name.c_str(),
                    servo.config.ax12Id,
                    servo.position);
            return;
        }

        if (dxl.ping(servo.config.ax12Id))
        {
            int id = servo.config.ax12Id;
            while (servo.initState < kServoInitStepCount)
            {
                const ServoInitStep &step = kServoInitSteps[servo.initState];
                if (!WriteControlTableItem(step.item, id, step.value))
                {
                    break;
                }
                servo.initState++;
                vTaskDelay(1);
            }

            if (servo.initState == kServoInitStepCount)
            {
                float presentPosition =
                    dxl.getPresentPosition(servo.config.ax12Id, UNIT_DEGREE);
                if (presentPosition == 0.0f
                    && dxl.getLastLibErrCode() != DXLLibErrorCode::DXL_LIB_OK)
                {
                    servo.failureCount++;
                }
                else
                {

                    println("Servo %s %d at position: %d go to %d - Init Done !",
                            servo.name,
                            servo.config.ax12Id,
                            servo.position,
                            servo.config.positions[0]);
                    // on le met à la position courante pour éviter de donner un ordre non voulu
                    servo.position = servo.command_position = (int)presentPosition;
                    servo.speed = 0;
                    servo.command_speed = 0;
                    servo.initialized = true;
                    servo.failureCount = 0;
                    servo.initState++;
                }
            }
        }
        else
        {
            servo.failureCount++;
            println(" NOT connected! (attempt %d)", servo.failureCount);
        }
    }

    void AddServo(ServoID logicalId, String name, const ServoConfig &defaults)
    {
        Servos[logicalId] = ServoMotion(name, defaults);
        // Will be initialised in the Update Task
    }

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

    void UpdateServo(ServoMotion &servo)
    {
        // Si le servo n'est pas initialisé, ne pas le forcer
        if (!servo.initialized)
        {
            return;
        }

        // Vérifier la tension avant de faire une opération I2C
        if (Power::isLowVoltage() || !IHM::bauReady)
        {
            // servo.IsMoving = false;
            return;
        }

        // On récupère la position actuelle du servo
        if (simulation)
        {
            if(servo.command_position != servo.position)
            {
                // Update every 10ms, so 100 deg in 1 sec
                if(servo.command_position - servo.position > 0)
                    servo.position++;
                else
                    servo.position--;
            }
        }
        else
        {
            // sometimes : timeout at 100ms
            float position = dxl.getPresentPosition(servo.config.ax12Id, UNIT_DEGREE);
            if (position == 0.0f && dxl.getLastLibErrCode() != DXLLibErrorCode::DXL_LIB_OK)
            {
                servo.failureCount++;
            }
            else
                servo.position = (int)position;
            
            float speed = dxl.getPresentSpeed(servo.config.ax12Id, UNIT_PERCENT);
            if (dxl.getLastLibErrCode() != DXLLibErrorCode::DXL_LIB_OK)
            {
                servo.failureCount++;
            }
            else
                servo.speed = (int)speed;
        }

        // Send Set servo speed if not already send
        if (!servo.goalSpeedAcked)
        {
            if (!simulation)
                servo.goalSpeedAcked = dxl.setMovingSpeed(
                    servo.config.ax12Id, (float)servo.command_speed, UNIT_PERCENT);
            else
                servo.goalSpeedAcked = true;
            if (!servo.goalSpeedAcked)
            {
                servo.failureCount++;
            }
        }
        
        // Send Set servo position if not already send
        if (!servo.goalPositionAcked)
        {
            if (!simulation)
                servo.goalPositionAcked = dxl.setGoalPosition(
                    servo.config.ax12Id, (float)servo.command_position, UNIT_DEGREE);
            else
                servo.goalPositionAcked = true;
            if (!servo.goalPositionAcked)
            {
                servo.failureCount++;
            }
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

        int position = servo.config.positions[index];
        SetServoPosition(logicalId, position, timeOutMs);
    }

    void SetServoPosition(ServoID logicalId, int position, int timeOutMs)
    {
        if (!ServoExists(logicalId))
            return;
        ServoMotion &servo = Servos.at(logicalId);

        int minPos = 0;
        int maxPos = 0;
        GetPositionBounds(servo, minPos, maxPos);
        if (position < minPos || position > maxPos)
        {
            println("Position out of range for Servo ID : %i", logicalId);
            println("Position : %d", position);
            println("Min : %d", minPos);
            println("Max : %d", maxPos);
            return;
        }
        
        if(servo.command_position == position)
        {
            // same command, do nothing
            return;
        }

        println("Servo ID : %i, Set position from %d to %d", logicalId, servo.position, position);
        
        servo.command_position = position;
        servo.IsMoving = true;
        servo.goalPositionAcked = false;
        if (timeOutMs > 0)
        {
            servo.timeOut.Start(timeOutMs);
        }
        else
        {
            servo.timeOut.Stop();
        }
        // We set the command into the task, if power is not too low
    }

    void SetServoSpeed(ServoID logicalId, int speed)
    {
        if (!ServoExists(logicalId))
            return;
        ServoMotion &servo = Servos.at(logicalId);

        if(speed < 0 || speed > 100)
            return;        

        if(servo.command_speed == speed)
        {
            // same command, do nothing
            return;            
        }
        
        println("Servo ID : %i, Set speed from %d to %d", logicalId, servo.speed, speed);

        servo.command_speed = speed;
        servo.goalSpeedAcked = false;
        // We set the command into the task, if power is not too low
    }

    int GetServoPosition(ServoID logicalId)
    {
        if (!ServoExists(logicalId))
            return -1;
        return Servos.at(logicalId).position;
    }

    int GetServoSpeed(ServoID logicalId)
    {
        if (!ServoExists(logicalId))
            return 0;
        return Servos.at(logicalId).speed;
    }

    bool WriteControlTableItem(ControlTableItem::ControlTableItemIndex item,
                               uint8_t id,
                               int32_t value,
                               uint32_t timeout)
    {
        int retry = 0;
        while (retry < 3)
        {
            println("WriteControlTableItem %d : %d for Servo ID %d (attempt %d)...",
                    item,
                    value,
                    id,
                    retry);
            bool ret = dxl.writeControlTableItem(item, id, value, timeout);
            if (ret)
                return true;
            retry++;
        }
        return false;
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
        else if (cmd.cmdEquals("AX12ConfigPrint"))
        {
            PrintServoConfigs();
        }
        else if (cmd.cmdEquals("AX12Pos"))
        {
            if (cmd.size == 0)
            {
                PrintAllPosition();
            }
            else if (cmd.size == 1)
            {
                // AX12Pos:1
                ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                PrintPosition(logicalId);
            }
            else if (cmd.size == 2)
            {
                // AX12Pos:1:180
                // AX12Pos:2:160
                ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                print("AX12 Servo id: %i ", logicalId);
                int position = static_cast<int>(cmd.data[1]);
                if (ServoExists(logicalId))
                {
                    println("Set Position : %d", position);
                    SetServoPosition(logicalId, position);
                }
                else
                {
                    println("Servo ID %i is not initialized", logicalId);
                }
            }
        }
        else if (cmd.cmdEquals("AX12Speed"))
        {
            if (cmd.size == 0)
            {
                PrintAllSpeed();
            }
            else if (cmd.size == 1)
            {
                // AX12Speed:1
                ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                PrintSpeed(logicalId);
            }
            else if (cmd.size == 2)
            {
                // AX12Speed:1:50
                ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                int speed = static_cast<int>(cmd.data[1]);
                if (ServoExists(logicalId))
                {
                    println("Set Speed : %d", speed);
                    SetServoSpeed(logicalId, speed);
                }
                else
                {
                    println("Servo ID %i is not initialized", logicalId);
                }
            }
        }
        else if (cmd.cmdEquals("AX12Table"))
        {
            // AX12Table:14
            if (cmd.size >= 1)
            {
                for (const auto &[servoKey, servo] : Servos)
                {
                    println("Read Data %d : %d",
                            cmd.data[0],
                            dxl.readControlTableItem((uint8_t)cmd.data[0],
                                                     (uint8_t)servo.config.ax12Id));
                }
            }
            if (cmd.size >= 2)
            {
                for (const auto &[servoKey, servo] : Servos)
                {
                    println("Write Data %d : %d -> %d",
                            cmd.data[0],
                            cmd.data[1],
                            dxl.writeControlTableItem((uint8_t)cmd.data[0],
                                                      (uint8_t)servo.config.ax12Id,
                                                      cmd.data[1]));
                }
            }
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
        Printer::println(" > AX12ConfigPrint");
        Printer::println("      Print current config of all registered servos");
        Printer::println(" > AX12Pos:[id]:[position]");
        Printer::println("      Set servo [id] to [position] (in degrees)");
        Printer::println("      If only 1 argument, print current position of the servo with the given id");
        Printer::println("      If no argument, print all currents positions");
        Printer::println(" > AX12Speed:[id]:[speed]");
        Printer::println("      Set speed of servo [id] to [speed] (in %)");
        Printer::println("      If only 1 argument, print current speed of the servo with the given id");
        Printer::println("      If no argument, print all currents speeds");
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
                        println("ID : %i, Name: %s, Model Number: %i, position: %d, command: %d, isMoving: %i",
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
                println("ID : %i, Name: %s, Model Number: %i, position: %d, command: %d, isMoving: %i",
                        ax12Id, servo.name, dxl.getModelNumber(ax12Id), servo.position, servo.command_position, servo.IsMoving);
            }
        }
    }

    void TeleplotAllPosition()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            teleplot("Servo_" + servo.name + "_" + String(servo.config.ax12Id) + "_Pos", servo.position);
        }
    }

    void TeleplotPosition(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
            teleplot("Servo_" + servo.name + "_" + String(servo.config.ax12Id) + "_Pos", servo.position);
        }
    }
    

    void TeleplotAllSpeed()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            teleplot("Servo_" + servo.name + "_" + String(servo.config.ax12Id) + "_Speed", servo.position);
        }
    }

    void TeleplotSpeed(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
            teleplot("Servo_" + servo.name + "_" + String(servo.config.ax12Id) + "_Speed", servo.position);
        }
    }

    void PrintAllPosition()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            println("Servo_%s_%d Pos : %d", servo.name, servo.config.ax12Id, servo.position);
        }
    }

    void PrintPosition(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
            println("Servo_%s_%d Pos : %d", servo.name, servo.config.ax12Id, servo.position);
        }
    }

    void PrintAllSpeed()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            println("Servo_%s_%d Speed : %d", servo.name, servo.config.ax12Id, servo.speed);
        }
    }
    
    void PrintSpeed(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
            println("Servo_%s_%d Speed : %d", servo.name, servo.config.ax12Id, servo.speed);
        }
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