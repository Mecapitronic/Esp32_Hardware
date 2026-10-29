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

        // Simulation tuning based on AX12 observed behavior in logs.
        constexpr float kSimulationDegPerTickAt100 = 1.0f; // 10 ms tick -> 100 deg/s at simulated 100%

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
    
        RegisterCommands();
    }

    void RegisterCommands()
    {
        // PrintTeleplot:1;AX12Torque:1:30;AX12Torque:2:30;AX12Speed:1:30;AX12Speed:2:30;AX12Pos:1:150;AX12Pos:2:160;AX12Wait;Delay:2000;AX12Pos:1:280;AX12Pos:2:210;AX12Wait;
        // PrintTeleplot:0;AX12Torque:1:100;AX12Torque:2:100;AX12Speed:1:10;AX12Speed:2:10;AX12Pos:1:150;AX12Pos:2:160;Delay:2000;AX12Pos:1:280;AX12Pos:2:210

        ESP32_Helper::RegisterCommand("AX12Scan",
            "AX12Scan - Scan Dynamixel devices on all supported protocols and baud rates."
            "\n     AX12Scan:<protocol:int>:<baudRate:int@bit/s> - Scan one protocol and baud rate; protocol: 1 or 2.",
            FunctionCommand( if (cmd.hasData(2)) Scan((DxlProtocolVersion)cmd.data[0], (BaudRate)cmd.data[1]); else Scan(); ));
        ESP32_Helper::RegisterCommand("AX12PrintInfo", "AX12PrintInfo(:id:int) - Show servo information; omit logical id for all registered servos.",
            FunctionCommand( if (cmd.hasData()) { PrintDxlInfo(static_cast<ServoID>(cmd.data[0])); } else { for (const auto &[_id, servo] : Servos) { PrintDxlInfo(_id); } } ));
        ESP32_Helper::RegisterCommand("AX12ConfigPrint", "AX12ConfigPrint - Show the configuration of all registered servos.",
            FunctionCommand( PrintServoConfigs(); ));
            
        ESP32_Helper::RegisterCommand("AX12Start", "AX12Start - Enable torque on all registered servos.",
            FunctionCommand( for (auto &[servoKey, servo] : Servos) {
                dxl.torqueOn(servo.config.ax12Id);
                IncrementFailureCount(servo); } ));
            
        ESP32_Helper::RegisterCommand("AX12Stop", "AX12Stop - Disable torque on all registered servos.",
            FunctionCommand( for (auto &[servoKey, servo] : Servos) {
                dxl.torqueOff(servo.config.ax12Id);
                IncrementFailureCount(servo); } ));
            
        ESP32_Helper::RegisterCommand("AX12Wait", "AX12Wait(:id:int) - Wait for motion to finish; omit logical id to wait for all servos.",
            FunctionCommand( 
                if (cmd.hasData()) {
                    ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                    println("Wait Servo %d...", logicalId);
                    while (IsServoMoving(logicalId)) { vTaskDelay(1); }                      
                    println("Servo %d stopped moving", logicalId);
                } else {
                    println("Wait all servos...");
                    while (AreAllServoMoving()) {
                            vTaskDelay(1); }
                    println("All servos stopped moving");
                } ));
                
            // TORQUE_ENABLE AX12Table:34
            // TORQUE_LIMIT AX12Table:70
            // MOVING_SPEED AX12Table:61
            // PRESENT_POSITION AX12Table:66
        ESP32_Helper::RegisterCommand("AX12Table",
            "AX12Table:<item:int> - Read a control-table item for all registered servos."
            "\n     AX12Table:<item:int>:<value:int>(:id:int) - Read all, then write to one logical id or all if omitted."
            "\n     item: Dynamixel2Arduino control-table item index, not a register address; value: raw register units.",
            FunctionCommand(
                if (cmd.hasData()) {
                    for (auto &[servoKey, servo] : Servos) {
                        println("Servo ID %d Read Data %d %d",
                                servo.config.ax12Id,
                                cmd.data[0],
                                dxl.readControlTableItem((uint8_t)cmd.data[0], (uint8_t)servo.config.ax12Id));
                        IncrementFailureCount(servo);
                        vTaskDelay(1); } }
                if (cmd.hasData(2)) {
                    for (auto &[servoKey, servo] : Servos) {
                        if(cmd.hasData(3) && cmd.data[2] != static_cast<int32_t>(servoKey))
                            continue;
                        println("Servo ID %d Write Data %d %d -> %d",
                            servo.config.ax12Id,
                            cmd.data[0],
                            cmd.data[1],
                            dxl.writeControlTableItem((uint8_t)cmd.data[0],
                                                      (uint8_t)servo.config.ax12Id,
                                                      cmd.data[1]));
                        IncrementFailureCount(servo);
                        vTaskDelay(1); }
                } ));
                
                // AX12Pos:1:180
                // AX12Pos:2:160
                // AX12Pos:1:180;AX12Pos:2:160
                // AX12Pos:1:180;AX12Pos:2:160
        ESP32_Helper::RegisterCommand("AX12Pos",
            "AX12Pos - Show positions of all servos in degrees."
            "\n     AX12Pos:<id:int> - Show position of one logical servo id in degrees."
            "\n     AX12Pos:<id:int>:<position:int@deg>(:timeout:int@ms=10000) - Show position, then set the target."
            "\n     position: within the configured servo limits; timeout > 0.",
            FunctionCommand(
                if (!cmd.hasData()) {
                    PrintAllPosition();
                } else if (cmd.hasData(1)) {
                    ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                    PrintPosition(logicalId);
                    if (cmd.hasData(2)) {
                        int position = static_cast<int>(cmd.data[1]);
                        if (cmd.hasData(3)) {
                            int timeout = static_cast<int>(cmd.data[2]);
                            println("Set Servo %d Position %d with timeout %d", logicalId, position, timeout);
                            SetServoPosition(logicalId, position, timeout);
                        } else {
                        println("Set Servo %d Position %d", logicalId, position);
                        SetServoPosition(logicalId, position); }}
                } ));
        ESP32_Helper::RegisterCommand("AX12Speed",
            "AX12Speed - Show speeds of all servos in percent."
            "\n     AX12Speed:<id:int> - Show speed of one logical servo id in percent."
            "\n     AX12Speed:<id:int>:<speed:int@%> - Show speed, then set it; speed: 0..100.",
            FunctionCommand(
                if (!cmd.hasData()) {
                    PrintAllSpeed();
                } else if (cmd.hasData(1)) {
                    ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                    PrintSpeed(logicalId);
                    if (cmd.hasData(2)) {
                        int speed = static_cast<int>(cmd.data[1]);
                        println("Set Servo %d Speed %d", logicalId, speed);
                        SetServoSpeed(logicalId, speed); }
                } ));
        ESP32_Helper::RegisterCommand("AX12Torque",
            "AX12Torque - Show raw torque limits of all servos (0..1023)."
            "\n     AX12Torque:<id:int> - Show raw torque limit of one logical servo id (0..1023)."
            "\n     AX12Torque:<id:int>:<torque:int@%> - Show raw torque limit, then set it in percent; torque: 0..100.",
            FunctionCommand(
                if (!cmd.hasData()) {
                    PrintAllTorqueLimit();
                } else if (cmd.hasData(1)) {
                    ServoID logicalId = static_cast<ServoID>(cmd.data[0]);
                    PrintTorqueLimit(logicalId);
                    if (cmd.hasData(2)) {
                        int torque = static_cast<int>(cmd.data[1]);
                        println("Set Servo %d Torque Limit %d", logicalId, torque);
                        SetTorqueLimit(logicalId, torque); }
                } ));
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
            const int error = servo.command_position - servo.position;
            const int speedSign = (error < 0) ? 1 : ((error > 0) ? -1 : 0);
            const int positionDirection = (error > 0) ? 1 : ((error < 0) ? -1 : 0);
            if (positionDirection != 0 && servo.torqueEnable && servo.command_speed > 0)
            {
                int absSpeedPercent = 0;
                if (servo.command_speed <= 10)
                {
                    absSpeedPercent = servo.command_speed;
                }
                else
                {
                    // Keep low speeds near 1:1, compress high speeds (100 -> about 60).
                    absSpeedPercent = 10 + ((servo.command_speed - 10) * 50) / 90;
                }

                if (absSpeedPercent < 0)
                    absSpeedPercent = 0;
                if (absSpeedPercent > 100)
                    absSpeedPercent = 100;

                servo.speed = speedSign * absSpeedPercent;

                float step = ((float)absSpeedPercent / 100.0f) * kSimulationDegPerTickAt100;
                float nextPosition = servo.positionSimulationIncrement + ((float)positionDirection * step);

                if ((positionDirection > 0 && nextPosition > (float)servo.command_position) ||
                    (positionDirection < 0 && nextPosition < (float)servo.command_position))
                {
                    nextPosition = (float)servo.command_position;
                }

                servo.positionSimulationIncrement = nextPosition;
                servo.position = (int)roundf(nextPosition);

                if (servo.position == servo.command_position)
                {
                    servo.speed = 0;
                    servo.positionSimulationIncrement = (float)servo.position;
                    servo.moving = false;
                }
                else
                    servo.moving = true;
            }
            else
            {
                servo.speed = 0;
                servo.positionSimulationIncrement = (float)servo.position;
                servo.moving = false;
            }
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

#pragma region Teleplot Functions
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
#pragma endregion

#pragma region Print Functions
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

    void PrintTorqueLimit(ServoID logicalId)
    {
        if (ServoExists(logicalId))
        {
            auto &servo = Servos.at(logicalId);
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
#pragma endregion
} // namespace ServoAX12