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
    } // namespace

    void Initialisation(HardwareSerial &serial, int8_t rxPin, int8_t txPin,int8_t dirPin, BaudRate baudRate, DxlProtocolVersion dxlProtocolVersion)
    {
        _rxPin =rxPin;
        _txPin=txPin;
        _dirPin=dirPin;
        _baudRate = baudRate;
        _dxlProtocolVersion = dxlProtocolVersion;
        serial.setPins(_rxPin, _txPin);
        dxl = Dynamixel2Arduino(serial, _dirPin);
        // Set Port baudrate. This has to match with DYNAMIXEL baudrate.
        dxl.begin((unsigned long)_baudRate);
        // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
        dxl.setPortProtocolVersion((float)_dxlProtocolVersion);

        Servos.clear();
        
        taskUpdateServo = TaskThread(TaskUpdateServo, "TaskUpdateServo", 2000, 15, 0);
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
                if(!scanning)
                {
                    // take some time to update the servo
                    ServoAX12::UpdateAllServo();
                }
            }
            catch (const std::exception &e)
            {
                printError(e.what());
            }
            if (chrono.Check() && Chrono::print)
            {
                printChrono(chrono);
            }
            vTaskDelay(10);
        }
        println("Servo Update Task STOPPED !");
    }

    void AddServo(ServoID id, String name, ServoPosition positionMin, ServoPosition positionMax)
    {
        Servos[id] = ServoMotion(id, name, positionMin, positionMax);
        InitServo(Servos.at(id));
    }
    
    void InitAllServo()
    {
        for (auto &[id, servo] : Servos)
        {
            InitServo(servo);
        }   
    }

    void InitServo(ServoMotion &servo)
    {
        print("Init Servo ID : %i name : %s", servo.id, servo.name.c_str());
        if (simulation)
        {
            servo.position = servo.command_position = (float)servo.positionMin;
            println("Servo %s %d position: %f", servo.name, servo.id, servo.position);
            return;
        }
        int retry = 10;
        bool pingOK = false;
        while (!pingOK && retry>=0)
        {
            if(dxl.ping(servo.id))
            {
                pingOK = true;
                break;
            }
            retry--;
            print(".");
            vTaskDelay(10);
        }
        if(pingOK)
        {
            ServoID id = static_cast<ServoID>(servo.id);
            PrintDxlInfo(id);

            servo.ledState = true;
            dxl.ledOn(servo.id);

            // Turn off torque when configuring items in EEPROM area
            dxl.torqueOff(servo.id);
            dxl.setOperatingMode(servo.id, OP_POSITION);
            dxl.torqueOn(servo.id);

            servo.position = servo.command_position = dxl.getPresentPosition(servo.id, UNIT_DEGREE);
            
            println(" position: %f", servo.position);
        }
        else
        {
            println(" NOT connected !");
        }
    }

    void StopAllServo()
    {
        println("Stop All Servo");
        for (auto &[id, servo] : Servos)
        {
            StopServo(servo);
        }
    }

    void StopServo(ServoMotion &servo)
    {
        if (!simulation)
        {
            dxl.torqueOff(servo.id);
            dxl.ledOff(servo.id);
        }
    }

    void StartAllServo()
    {
        println("Start All Servo");
        for (auto &[id, servo] : Servos)
        {
            StartServo(servo);
        }
    }

    void StartServo(ServoMotion &servo)
    {
        if (!simulation)
            dxl.torqueOn(servo.id);
    }

    void UpdateAllServo()
    {
        // 1 ms / servo
        for (auto &[id, servo] : Servos)
        {
            UpdateServo(servo);
        }
    }

    void UpdateServo(ServoMotion &servo)
    {
        if (simulation)
        {
            servo.position = servo.position + (servo.command_position - servo.position) / 2;
        }
        else
            servo.position = dxl.getPresentPosition(servo.id, UNIT_DEGREE);

        if (servo.position >= servo.command_position + 5
            || servo.position <= servo.command_position - 5)
        {
            if (!servo.ledState)
            {
                servo.ledState = true;
                if (!simulation)
                    dxl.ledOn(servo.id);
            }
            servo.IsMoving = true;
        }
        else
        {
            if (servo.ledState)
            {
                servo.ledState = false;
                if (!simulation)
                    dxl.ledOff(servo.id);
            }
            servo.IsMoving = false;
        }
    }

    bool AreAllServoMoving()
    {
        for (const auto &[id, servo] : Servos)
        {
            if (servo.IsMoving)
            {
                return true;
            }
        }
        return false;
    }

    bool IsServoMoving(ServoID id)
    {
        return Servos.at(id).IsMoving;
    }
    
    void SetServoPosition(ServoID id, ServoPosition position)
    {
        SetServoPosition(id, (float)position);
    }

    void SetServoPosition(ServoID id, float position)
    {
        if (position < (float)Servos.at(id).positionMin || position > (float)Servos.at(id).positionMax)
        {
            println("Position out of range for Servo ID : %i", id);
            println("Position : %f", position);
            println("Min : %f", (float)Servos.at(id).positionMin);
            println("Max : %f", (float)Servos.at(id).positionMax);
            return;
        }
        Servos.at(id).command_position = position;
        Servos.at(id).IsMoving = true;
        if (!simulation)
            dxl.setGoalPosition((uint8_t)id, Servos.at(id).command_position, UNIT_DEGREE);
    }

    void HandleCommand(Command cmd)
    {
        if (cmd.cmd == "AX12Scan")
        {
            //AX12Scan:1:1000000
            if (cmd.size == 2)
            {
                Scan((DxlProtocolVersion)cmd.data[0],(BaudRate)cmd.data[1]);
            }
            else
            {
                Scan();
            }
        }
        else if (cmd.cmd == "AX12PrintInfo")
        {
            //AX12PrintInfo:1
            if (cmd.size == 1)
            {
                ServoID id = static_cast<ServoID>(cmd.data[0]);
                PrintDxlInfo(id);
            }
            else
                PrintDxlInfo();
        }
        else if (cmd.cmd == "AX12AddServo")
        {
            //AX12AddServo:1:Left:0:300
            if (cmd.size == 3 && cmd.dataStr1 != "")
            {
                ServoID id = static_cast<ServoID>(cmd.data[0]);
                ServoPosition positionMin = static_cast<ServoPosition>(cmd.data[1]);
                ServoPosition positionMax = static_cast<ServoPosition>(cmd.data[2]);
                AddServo(id, cmd.dataStr1, positionMin, positionMax);
            }
        }
        else if (cmd.cmd == "AX12Pos")
        {            
            if (cmd.size == 2)
            {
                // AX12Pos:1:100
                ServoID id = static_cast<ServoID>(cmd.data[0]);
                print("AX12 Servo id: %i ", id);
                float position = static_cast<float>(cmd.data[1]);
                if (Servos.find(id) != Servos.end())
                {
                    println("Set Position : %f", position);
                    SetServoPosition(id, position);
                }
                else
                {
                    println("Servo ID %i is not initialized", id);
                }
            }
            else if (cmd.size == 1)
            {
                //AX12Pos:1
                ServoID id = static_cast<ServoID>(cmd.data[0]);
                PrintPosition(id);
            }
            else
            {
                PrintAllPosition();
            }
        }
        else if (cmd.cmd == "AX12Stop")
        {
            println("AX12Stop");
            StopAllServo();
        }
        else if (cmd.cmd == "AX12Start")
        {
            println("AX12Start");
            StartAllServo();
        }
        else if (cmd.cmd == "AX12Help")
        {
            PrintCommandHelp();
        }
        else
        {
            println("Unknown command : ", cmd.cmd);
        }
    }

    const void PrintCommandHelp()
    {
        Printer::println("AX12 Command Help :");
        Printer::println(" > AX12Scan");
        Printer::println("      Scan all Dynamixel on all protocols and baudrates");
        Printer::println(" > AX12PrintInfo:[id]");
        Printer::println("      Print info for all servos or for the given id");
        Printer::println(" > AX12AddServo:[id]:[name]:[min]:[max]");
        Printer::println("      Add a servo with the given id, name, min and max position");
        Printer::println(" > AX12Pos:[id]:[position]");
        Printer::println("      Set servo [id] to [position] (in degrees)");
        Printer::println("      If only 1 argument, print current position of the servo with the given id");
        Printer::println("      If no argument, print all currents positions");
        Printer::println(" > AX12Stop");
        Printer::println("      Stop all servos (torque off)");
        Printer::println(" > AX12Help");
        Printer::println("      Print this help");
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
        //Save original protocol and version
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
                println("ID : %i, Model Number: %i",id, dxl.getModelNumber(id));
                found_dynamixel++;                
            }
            vTaskDelay(1);
        }
        println("Found %i Dynamixel(s)", found_dynamixel);

        //Put back original protocol and version
        dxl.setPortProtocolVersion(version);
        dxl.begin(baud);

        scanning = false;
        return found_dynamixel;
    }

    void PrintDxlInfo(ServoID id)
    {
        if(id != ServoID::BroadCast)
        {
            if (dxl.ping((uint8_t)id))
            {
                println("ID : %i, Name: %s, Model Number: %i, position: %f, command: %f, isMoving: %i",
                     (uint8_t)id, Servos.at(id).name, dxl.getModelNumber((uint8_t)id), Servos.at(id).position, Servos.at(id).command_position, Servos.at(id).IsMoving);
            }
            else
            {
                println("Dynamixel ID : %i not found", (uint8_t)id);
            }
        }
        else
        {
            for (const auto &[_id, servo] : Servos)
            {
                println("ID : %i, Name: %s, Model Number: %i, position: %f, command: %f, isMoving: %i",
                     _id, servo.name, dxl.getModelNumber((uint8_t)_id), servo.position, servo.command_position, servo.IsMoving);
            }
        }
    }

    void TeleplotAllPosition()
    {
        for (auto &[id, servo] : Servos)
        {
            teleplot("Servo " + servo.name + " " + String(servo.id), servo.position);
        }
    }
    
    void TeleplotPosition(ServoID id)
    {
        if (Servos.find(id) != Servos.end())
        {
            auto &servo = Servos.at(id);
            teleplot("Servo " + servo.name + " " + String(servo.id), servo.position);
        }
    }

    void PrintAllPosition()
    {
        for (auto &[id, servo] : Servos)
        {
            println("Servo %s %d : %f", servo.name, servo.id, servo.position);
        }
    }

    void PrintPosition(ServoID id)
    {
        if (Servos.find(id) != Servos.end())
        {
            auto &servo = Servos.at(id);
            println("Servo %s %d : %f", servo.name, servo.id, servo.position);
        }
    }
} // namespace ServoAX12