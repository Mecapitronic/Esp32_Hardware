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
            {ControlTableItem::ControlTableItemIndex::DXL_LED, 1},
            {ControlTableItem::ControlTableItemIndex::TORQUE_ENABLE, 0},
            //{ControlTableItem::ControlTableItemIndex::TEMPERATURE_LIMIT, 70}, //EEPROM, 70 by default
            //{ControlTableItem::ControlTableItemIndex::CW_ANGLE_LIMIT, 0}, //EEPROM, 0 by default
            //{ControlTableItem::ControlTableItemIndex::CCW_ANGLE_LIMIT, 1023}, //EEPROM, 1023 by default
            //{ControlTableItem::ControlTableItemIndex::MAX_TORQUE, 200}, //EEPROM, 1023 by default
            //{ControlTableItem::ControlTableItemIndex::TORQUE_LIMIT, 200}, // Max Torque at power up
            //{ControlTableItem::ControlTableItemIndex::CW_COMPLIANCE_SLOPE, 8}, // 32 by default
            //{ControlTableItem::ControlTableItemIndex::CCW_COMPLIANCE_SLOPE, 8}, // 32 by default
            //{ControlTableItem::ControlTableItemIndex::MOVING_SPEED, 200}, // 0 by default (max)
            {ControlTableItem::ControlTableItemIndex::TORQUE_ENABLE, 1},
            {ControlTableItem::ControlTableItemIndex::DXL_LED, 0},
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

        void IncrementFailureCount(ServoMotion &servo)
        {
            DXLLibErrorCode_t code = dxl.getLastLibErrCode();
            if (code == DXLLibErrorCode::DXL_LIB_OK)
                return;
            uint8_t index = (uint8_t)code;
            if (index < servo.DXLLibErrorCodeMax)
            {
                servo.failureCount[index]++;
            }
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

        println("Init Servo ID %i name %s - Init State %d",
                servo.config.ax12Id,
                servo.name.c_str(),
                servo.initState);
        if (simulation)
        {
            servo.position = servo.command_position = servo.config.positions[0];
            servo.speed = 0;
            servo.command_speed = 50;
            servo.initialized = true;
            servo.torqueEnable = true;
            println("Servo %s %d position %d [SIM]",
                    servo.name.c_str(),
                    servo.config.ax12Id,
                    servo.position);
            return;
        }
    
        while (servo.initState < kServoInitStepCount)
        {
            const ServoInitStep &step = kServoInitSteps[servo.initState];
            int value = 0;
            if(!ReadControlTableItem(step.item, value, &servo))
                break;
            println("Servo %s ID %d init step %d: item %d read value %d, set value %d",
                servo.name.c_str(),
                servo.config.ax12Id,
                servo.initState,
                step.item,
                value,
                step.value);
            if(value == step.value)
            {
                servo.initState++;
                continue;
            }
            if (!WriteControlTableItem(step.item, step.value, &servo))
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
            IncrementFailureCount(servo);
            println("Servo %s %d at position %0.2f",
                servo.name.c_str(),
                servo.config.ax12Id,
                presentPosition);
            
            int minPos = 0;
            int maxPos = 0;
            GetPositionBounds(servo, minPos, maxPos);
            if (presentPosition <= minPos - 1 || presentPosition >= maxPos + 1)
            {
                println("Unexpected position for Servo %s %d: %0.2f (expected between %d and %d)",
                        servo.name.c_str(),
                        servo.config.ax12Id,
                        presentPosition,
                        minPos,
                        maxPos);
            }
            else
            {
                // on le met à la position courante pour éviter de donner un ordre non voulu
                servo.position = servo.command_position = (int)presentPosition;
                servo.speed = 0;
                servo.command_speed = 0;
                servo.initialized = true;
                servo.torqueEnable = true;
                servo.initState++;
                println("Servo %s %d at position %d - Init Done !",
                        servo.name.c_str(),
                        servo.config.ax12Id,
                        servo.position);
            }
        }
        else
        {
            println("Failed to init Servo %s %d at step %d, error code %d",
                servo.name.c_str(),
                    servo.config.ax12Id,
                    servo.initState,
                    dxl.getLastLibErrCode());
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
        if (Power::isLowVoltage())
        {
            return;
        }

        // On récupère la position actuelle du servo
        if (simulation)
        {
            servo.speed = servo.command_speed;
            if(servo.command_position != servo.position)
            {
                // Update every 10ms, so 100 deg in 1 sec for 100% speed
                if(servo.command_position - servo.position > 0)
                {
                    servo.positionSimulationIncrement += (float)(servo.speed) / 100;
                    if(servo.positionSimulationIncrement > (float)(servo.command_position - servo.position))
                    {
                        servo.positionSimulationIncrement = (float)(servo.command_position - servo.position);
                    }
                }
                else
                {
                    servo.positionSimulationIncrement -= (float)(servo.speed) / 100;
                    if(servo.positionSimulationIncrement < (float)(servo.command_position - servo.position))
                    {
                        servo.positionSimulationIncrement = (float)(servo.command_position - servo.position);
                    }
                }
                servo.position = (int)servo.positionSimulationIncrement;
            }
            else
                servo.positionSimulationIncrement = servo.position;
        }
        else
        {
            // POSITION
            // sometimes timeout at 100ms
            float position = dxl.getPresentPosition(servo.config.ax12Id, UNIT_DEGREE);
            IncrementFailureCount(servo);
            if (position != 0.0f && dxl.getLastLibErrCode() == DXLLibErrorCode::DXL_LIB_OK)
                servo.position = (int)position;
            
            // SPEED
            int speed = dxl.readControlTableItem(ControlTableItem::ControlTableItemIndex::PRESENT_SPEED, servo.config.ax12Id);
            IncrementFailureCount(servo);
            if (dxl.getLastLibErrCode() == DXLLibErrorCode::DXL_LIB_OK)
            {
                if (speed >= 0 && speed <= 1023)
                    servo.speed = map(speed, 0, 1023, 0, -100);
                else if (speed >= 1024 && speed <= 2047)
                    servo.speed = map(speed, 1024, 2047, 0, 100);
            }

            // LOAD
            int32_t load = dxl.readControlTableItem(ControlTableItem::ControlTableItemIndex::PRESENT_LOAD, servo.config.ax12Id);
            IncrementFailureCount(servo);
            if (dxl.getLastLibErrCode() == DXLLibErrorCode::DXL_LIB_OK)
            {
                if (load >= 0 && load <= 1023)
                    servo.load = map(load, 0, 1023, 0, -100);
                else if (load >= 1024 && load <= 2047)
                    servo.load = map(load, 1024, 2047, 0, 100);
            }

            // MOVING
            int32_t moving = dxl.readControlTableItem(ControlTableItem::ControlTableItemIndex::MOVING, servo.config.ax12Id);
            IncrementFailureCount(servo);
            if (dxl.getLastLibErrCode() == DXLLibErrorCode::DXL_LIB_OK)
                servo.moving = (bool)moving;

            // ERROR
            int32_t error = dxl.readControlTableItem(ControlTableItem::ControlTableItemIndex::SHUTDOWN, servo.config.ax12Id);
            IncrementFailureCount(servo);                
            if (dxl.getLastLibErrCode() == DXLLibErrorCode::DXL_LIB_OK)
                servo.error = ServoError(error);
        }

        // Stop the torque if the BAU is engaged
        // TODO It will stop the movement instantly but the movement will not resume after BAU is released
        if(!IHM::bauReady)
        {
            if(servo.torqueEnable)
            {
                println("Stopping the servo %s ID %d because of BAU", servo.name.c_str(), servo.config.ax12Id);
                if (!simulation)
                {
                    dxl.torqueOff(servo.config.ax12Id);
                    IncrementFailureCount(servo); 
                    servo.torqueEnable = (bool)dxl.getTorqueEnableStat(servo.config.ax12Id);
                    IncrementFailureCount(servo); 
                }
                else
                {
                    servo.torqueEnable = false;
                }
            }
            servo.command_position = servo.position;
            servo.goalPositionAcked = true;
            servo.command_speed = 0;
            servo.goalSpeedAcked = true;
        }
        else
        {
            if(!servo.torqueEnable)
            {
                println("Re-enabling the servo %s ID %d because of BAU", servo.name.c_str(), servo.config.ax12Id);
                if (!simulation)
                {
                    dxl.torqueOn(servo.config.ax12Id);
                    IncrementFailureCount(servo); 
                    servo.torqueEnable = (bool)dxl.getTorqueEnableStat(servo.config.ax12Id);
                    IncrementFailureCount(servo);
                }
                else
                {
                    servo.torqueEnable = true;
                }
            }
        }

        // Send Set servo speed if not already send
        if (!servo.goalSpeedAcked)
        {
            if (!simulation)
            {
                servo.goalSpeedAcked = dxl.writeControlTableItem(ControlTableItem::ControlTableItemIndex::MOVING_SPEED,
                    servo.config.ax12Id,
                    servo.command_speed * 1023 / 100);
                IncrementFailureCount(servo); 
                println("Set Speed for Servo ID %i to %d %% %s",
                        servo.config.ax12Id,
                        servo.command_speed,
                        servo.goalSpeedAcked ? "ACK" : "NACK");
            }
            else
                servo.goalSpeedAcked = true;
        }
        
        // Send Set servo position if not already send
        if (!servo.goalPositionAcked)
        {
            if (!simulation)
            {
                servo.goalPositionAcked = dxl.setGoalPosition(
                    servo.config.ax12Id, (float)servo.command_position, UNIT_DEGREE);
                IncrementFailureCount(servo); 
                println("Set Position for Servo ID %i from %d to %d %s",
                        servo.config.ax12Id,
                        servo.position,
                        servo.command_position,
                        servo.goalPositionAcked ? "ACK" : "NACK");
            }
            else
                servo.goalPositionAcked = true;
        }
        
        // Gestion du timeout : log si timeout expiré pendant le mouvement
        if(servo.moving || servo.shouldMove)
        {
            if(servo.timeOut.IsTimeOut())
            {
                println("Servo %s ID %d timeout reached for position command %d (current %d)",
                        servo.name.c_str(),
                        servo.config.ax12Id,
                        servo.command_position,
                        servo.position);
                servo.timeOut.Stop();
            }
        }
        else
        {
            if(servo.timeOut.IsTimeOut())
                servo.timeOut.Stop();
        }

        // On considère que le servo est en mouvement si la position n'est pas atteinte ±5° ET timeout actif
        servo.shouldMove = (servo.position >= servo.command_position + 5 || servo.position <= servo.command_position - 5) && !servo.timeOut.IsTimeOut();

        if (servo.shouldMove)
        {
            if (!servo.ledState)
            {
                servo.ledState = true;
                if (!simulation)
                {
                    dxl.ledOn(servo.config.ax12Id);
                    IncrementFailureCount(servo);
                }
            }
        }
        else
        {
            if (servo.ledState)
            {
                servo.ledState = false;
                if (!simulation)
                {
                    dxl.ledOff(servo.config.ax12Id);
                    IncrementFailureCount(servo);
                }
            }
        }
    }

    bool AreAllServoMoving()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            if (IsServoMoving(servoKey))
                return true;
        }
        return false;
    }

    bool IsServoMoving(ServoID logicalId)
    {
        if (!ServoExists(logicalId))
            return false;
        auto &servo = Servos.at(logicalId);
        if ((servo.moving || servo.shouldMove) && !servo.timeOut.IsTimeOut())
        {
            return true;
        }
        return false;
    }

    void SetServoPosition(ServoID logicalId, Hardware_Config::ServoPosition servoPosition, int timeOutMs)
    {
        if (!ServoExists(logicalId))
            return;
        ServoMotion &servo = Servos.at(logicalId);

        size_t index = static_cast<size_t>(servoPosition);
        if (index >= servo.config.positionCount)
        {
            println("SetServoPosition index hors limite (%d >= %d)", (int)index, (int)servo.config.positionCount);
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
            println("Position out of range for Servo ID %i", logicalId);
            println("Position %d", position);
            println("Min %d", minPos);
            println("Max %d", maxPos);
            return;
        }
        
        if(servo.command_position == position)
        {
            // same command, do nothing
            return;
        }

        println("Servo ID %i, Set position from %d to %d", logicalId, servo.command_position, position);
        
        servo.command_position = position;
        servo.goalPositionAcked = false;
        servo.timeOut.Start(timeOutMs);
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
        
        println("Servo ID %i, Set speed from %d to %d", logicalId, servo.command_speed, speed);

        servo.command_speed = speed;
        servo.goalSpeedAcked = false;
        // We set the command into the task, if power is not too low
    }

    void SetTorqueLimit(ServoID logicalId, int torqueLimit)
    {
        if (!ServoExists(logicalId))
            return;
        ServoMotion &servo = Servos.at(logicalId);

        if(torqueLimit < 0 || torqueLimit > 100)
            return;        

        println("Servo ID %i, Set torque limit to %d", logicalId, torqueLimit);

        if (!simulation)
        {
            if (!WriteControlTableItem(ControlTableItem::ControlTableItemIndex::TORQUE_LIMIT,
                                       torqueLimit * 1023 / 100,
                                       &servo)) // Convert percentage to value (0-1023)
            {
                println("Failed to set torque limit for Servo ID %i", logicalId);
            }
        }
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
                               int32_t value,
                               ServoMotion *servo,
                               uint32_t timeout)
    {
        if (servo == nullptr)
            return false;
        int retry = 0;
        while (retry < 3)
        {
            // println("WriteControlTableItem %d %d for Servo ID %d (attempt %d)...",
            //         item,
            //         value,
            //         id,
            //         retry);
            bool ret = dxl.writeControlTableItem(item, servo->config.ax12Id, value, timeout);
            IncrementFailureCount(*servo);
            if (ret)
                return true;
            retry++;
            vTaskDelay(1);
        }
        return false;
    }

    bool ReadControlTableItem(ControlTableItem::ControlTableItemIndex item,
                              int32_t &value,
                              ServoMotion *servo,
                              uint32_t timeout)
    {
        if (servo == nullptr)
            return false;
        int retry = 0;
        while (retry < 3)
        {
            // println("ReadControlTableItem %d for Servo ID %d (attempt %d)...",
            //         item,
            //         id,
            //         retry);
            value = dxl.readControlTableItem(item, servo->config.ax12Id, timeout);
            IncrementFailureCount(*servo);
            if (dxl.getLastLibErrCode() == DXLLibErrorCode::DXL_LIB_OK)
                return true;
            retry++;
            vTaskDelay(1);
        }
        return false;
    }

    bool HandleCommand(Command cmd)
    {
        // PrintTeleplot:1;AX12Torque:1:30;AX12Torque:2:30;AX12Speed:1:30;AX12Speed:2:30;AX12Pos:1:150;AX12Pos:2:160;AX12Wait;Delay:2000;AX12Pos:1:280;AX12Pos:2:210;AX12Wait;
        // PrintTeleplot:0;AX12Torque:1:100;AX12Torque:2:100;AX12Speed:1:10;AX12Speed:2:10;AX12Pos:1:150;AX12Pos:2:160;Delay:2000;AX12Pos:1:280;AX12Pos:2:210
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
            {
                for (const auto &[_id, servo] : Servos)
                {
                    PrintDxlInfo(_id);
                }
            }
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
            else if (cmd.size >= 2)
            {
                // AX12Pos:1:180
                // AX12Pos:2:160
                // AX12Pos:1:180;AX12Pos:2:160
                // AX12Pos:1:180;AX12Pos:2:160
                ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                print("AX12 Servo id %i ", logicalId);
                int position = static_cast<int>(cmd.data[1]);
                int timeout = 0;
                
                if (ServoExists(logicalId))
                {
                    if(cmd.size >= 3)
                    {
                        timeout = static_cast<int>(cmd.data[2]);
                        println("Set Servo %d Position %d with timeout %d", logicalId, position, timeout);
                    }
                    else
                    {
                        println("Set Servo %d Position %d", logicalId, position);
                    }
                    SetServoPosition(logicalId, position, timeout);
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
            else if (cmd.size == 2)
            {
                // AX12Speed:1:50
                ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                int speed = static_cast<int>(cmd.data[1]);
                if (ServoExists(logicalId))
                {
                    println("Set Servo %d Speed %d", logicalId, speed);
                    SetServoSpeed(logicalId, speed);
                }
                else
                {
                    println("Servo ID %i is not initialized", logicalId);
                }
            }
        }
        else if (cmd.cmdEquals("AX12Torque"))
        {
            if (cmd.size == 0)
            {
                PrintAllTorqueLimit();
            }
            else if (cmd.size == 2)
            {
                // AX12Torque:1:50
                ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                int torqueLimit = static_cast<int>(cmd.data[1]);
                println("Set Servo %d Torque Limit %d", logicalId, torqueLimit);
                SetTorqueLimit(logicalId, torqueLimit);
            }
        }
        else if (cmd.cmdEquals("AX12Table"))
        {
            // TORQUE_ENABLE AX12Table:34
            // TORQUE_LIMIT AX12Table:70
            // MOVING_SPEED AX12Table:61
            // PRESENT_POSITION AX12Table:66
            if (cmd.size >= 1)
            {
                for (auto &[servoKey, servo] : Servos)
                {
                    println("Servo ID %d Read Data %d %d",
                            servo.config.ax12Id,
                            cmd.data[0],
                            dxl.readControlTableItem((uint8_t)cmd.data[0],
                                                     (uint8_t)servo.config.ax12Id));
                    IncrementFailureCount(servo);
                    vTaskDelay(1);
                }
            }
            if (cmd.size == 2)
            {
                for (auto &[servoKey, servo] : Servos)
                {
                    println("Servo ID %d Write Data %d %d -> %d",
                            servo.config.ax12Id,
                            cmd.data[0],
                            cmd.data[1],
                            dxl.writeControlTableItem((uint8_t)cmd.data[0],
                                                      (uint8_t)servo.config.ax12Id,
                                                      cmd.data[1]));
                    IncrementFailureCount(servo);
                    vTaskDelay(1);
                }
            }
            else if (cmd.size == 3)
            {
                ServoID logicalId = static_cast<ServoID>(cmd.data[2]);
                if (ServoExists(logicalId))
                {
                    ServoMotion &servo = Servos.at(logicalId);
                    println("Servo ID %d Write Data %d %d -> %d",
                            servo.config.ax12Id,
                            cmd.data[0],
                            cmd.data[1],
                            dxl.writeControlTableItem((uint8_t)cmd.data[0],
                                                    (uint8_t)servo.config.ax12Id,
                                                    cmd.data[1]));
                    IncrementFailureCount(servo);
                }
            }
        }
        else if (cmd.cmdEquals("AX12Stop"))
        {
            println("Stopping all servos...");
            for (auto &[servoKey, servo] : Servos)
            {
                dxl.torqueOff(servo.config.ax12Id);
                IncrementFailureCount(servo);
            }
        }
        else if (cmd.cmdEquals("AX12Start"))
        {
            println("Starting all servos...");
            for (auto &[servoKey, servo] : Servos)
            {
                dxl.torqueOn(servo.config.ax12Id);
                IncrementFailureCount(servo);
            }
        }
        else if (cmd.cmdEquals("AX12Wait"))
        {
            if (cmd.size == 0)
            {
                println("Wait all servos...");
                while (AreAllServoMoving())
                {
                    vTaskDelay(1);
                }
                println("All servos stopped moving");
            }
            else if (cmd.size == 1)
            {
                ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                if (ServoExists(logicalId))
                {
                    println("Wait Servo %d...", logicalId);
                    while (IsServoMoving(logicalId))
                    {
                        vTaskDelay(1);
                    }
                    println("Servo %d stopped moving", logicalId);
                }
                else
                {
                    println("Servo ID %i is not initialized", logicalId);
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
        Printer::println("AX12 Command Help");
        Printer::println(" > AX12Scan");
        Printer::println("      Scan all Dynamixel on all protocols and baudrates");
        Printer::println(" > AX12PrintInfo:[id]");
        Printer::println("      Print info for all servos or for the given id");
        Printer::println(" > AX12ConfigPrint");
        Printer::println("      Print current config of all registered servos");
        Printer::println(" > AX12Pos:[id]:[position]");
        Printer::println("      Set servo [id] to [position] (in degrees)");
        Printer::println("      If no argument, print all currents positions");
        Printer::println(" > AX12Speed:[id]:[speed]");
        Printer::println("      Set speed of servo [id] to [speed] (in %)");
        Printer::println("      If no argument, print all current speeds");
        Printer::println(" > AX12Torque:[id]:[torque]");
        Printer::println("      Set torque limit of servo [id] to [torque] (in %)");
        Printer::println("      If no argument, print all torque limits");
        Printer::println(" > AX12Table:[item]:[value]:[id]");
        Printer::println("      Read/Write Dynamixel control table item for all servos");
        Printer::println("      With 1 arg: read item from all servos");
        Printer::println("      With 2 or 3 args (optional): write value to item for all servos or for the given id");
        Printer::println(" > AX12Wait:[id]");
        Printer::println("      Wait for all servos to finish moving");
        Printer::println("      If [id] is given, wait only for that servo");
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
        println("Total %i Dynamixel(s) found", found_dynamixel);
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
                println("ID %i, Model Number %i", id, dxl.getModelNumber(id));
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
        if (ServoExists(logicalId))
        {
            ServoMotion &servo = Servos.at(logicalId);
            uint8_t ax12Id = servo.config.ax12Id;
            int modelNumber = dxl.getModelNumber(ax12Id);
            IncrementFailureCount(servo);
            int failureCountSum = 0;
            for (size_t i = 0; i < servo.DXLLibErrorCodeMax; i++)
            {
                failureCountSum += servo.failureCount[i];
            }
            println("ID %i, Name %s, Model Number %i, position %d, command position %d, speed %d, command speed %d, load %d, torque enable %d, error code 0x%02X, failure count sum %d, moving %d, shouldMove %d", 
                ax12Id, servo.name.c_str(), modelNumber, servo.position, servo.command_position, servo.speed, servo.command_speed, servo.load, servo.torqueEnable, servo.error.errorCode, failureCountSum, servo.moving, servo.shouldMove);
        }
        else
        {
            println("Dynamixel ID %i not found", (uint8_t)logicalId);
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
            teleplot("Servo_" + servo.name + "_" + String(servo.config.ax12Id) + "_Speed", servo.speed);
        }
    }

    void TeleplotSpeed(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
            teleplot("Servo_" + servo.name + "_" + String(servo.config.ax12Id) + "_Speed", servo.speed);
        }
    }

    void TeleplotAllLoad()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            teleplot("Servo_" + servo.name + "_" + String(servo.config.ax12Id) + "_Load", servo.load);
        }
    }

    void TeleplotLoad(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
            teleplot("Servo_" + servo.name + "_" + String(servo.config.ax12Id) + "_Load", servo.load);
        }
    }

    void TeleplotAllFailureCount()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            for (int16_t i = 0; i < servo.DXLLibErrorCodeMax; i++)
            {
                if(servo.failureCount[i] != servo.failureCountPrev[i])
                {
                    teleplot("Servo_" + servo.name + "_" + String(servo.config.ax12Id) + "_failureCount", Point(i, servo.failureCount[i]));
                    servo.failureCountPrev[i] = servo.failureCount[i];
                }
            }
        }
    }

    void TeleplotFailureCount(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
            for (int16_t i = 0; i < servo.DXLLibErrorCodeMax; i++)
            {                
                if(servo.failureCount[i] != servo.failureCountPrev[i])
                {
                    teleplot("Servo_" + servo.name + "_" + String(servo.config.ax12Id) + "_failureCount", Point(i, servo.failureCount[i]));
                    servo.failureCountPrev[i] = servo.failureCount[i];
                }
            }
        }
    }

    void TeleplotAllTorqueEnable()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            teleplot("Servo_" + servo.name + "_" + String(servo.config.ax12Id) + "_TorqueEnable", servo.torqueEnable ? 1 : 0);
        }
    }

    void TeleplotTorqueEnable(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
            teleplot("Servo_" + servo.name + "_" + String(servo.config.ax12Id) + "_TorqueEnable", servo.torqueEnable ? 1 : 0);
        }
    }

    void PrintAllPosition()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            println("Servo_%s_%d Pos %d", servo.name.c_str(), servo.config.ax12Id, servo.position);
            println("Servo_%s_%d Pos Cmd %d", servo.name.c_str(), servo.config.ax12Id, servo.command_position);
        }
    }

    void PrintPosition(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
            println("Servo_%s_%d Pos %d", servo.name.c_str(), servo.config.ax12Id, servo.position);
            println("Servo_%s_%d Pos Cmd %d", servo.name.c_str(), servo.config.ax12Id, servo.command_position);
        }
    }

    void PrintAllSpeed()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            println("Servo_%s_%d Speed %d", servo.name.c_str(), servo.config.ax12Id, servo.speed);
            println("Servo_%s_%d Speed Cmd %d", servo.name.c_str(), servo.config.ax12Id, servo.command_speed);
        }
    }
    
    void PrintSpeed(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
            println("Servo_%s_%d Speed %d", servo.name.c_str(), servo.config.ax12Id, servo.speed);
            println("Servo_%s_%d Speed Cmd %d", servo.name.c_str(), servo.config.ax12Id, servo.command_speed);
        }
    }
    
    void PrintAllTorqueLimit()
    {
        for (auto &[servoKey, servo] : Servos)
        {
            int torqueLimit = dxl.readControlTableItem(ControlTableItem::ControlTableItemIndex::TORQUE_LIMIT, servo.config.ax12Id);
            IncrementFailureCount(servo);
            println("Servo_%s_%d Torque Limit %d", servo.name.c_str(), servo.config.ax12Id, torqueLimit);
        }
    }

    void PrintServoConfigs()
    {
        println("Servo Configs (%d servos)", Servos.size());
        for (const auto &[servoKey, servo] : Servos)
        {
            String line = "  " + servo.name + ": ax12Id=" + String(servo.config.ax12Id) + " cnt=" + String(servo.config.positionCount) + " pos=[";
            for (uint8_t i = 0; i < servo.config.positionCount; ++i)
            {
                if (i > 0)
                    line += ",";
                line += String(servo.config.positions[i]);
            }
            line += "]";
            println(line);
        }
    }
} // namespace ServoAX12