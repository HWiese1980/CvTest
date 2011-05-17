/* 
 * File:   MotorHelper.cpp
 * Author: ros
 * 
 * Created on 12. Mai 2011, 15:44
 */

#include "MotorHelper.h"
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <string.h>
#include <errno.h>


#include "global.h"

using namespace LibSerial;


MovementState MotorHelper::State = Stopped;
SerialStream MotorHelper::arduinoConnection;

void MotorHelper::Initialize()
{
    
    arduinoConnection.Open("/dev/ttyARD_relay");
    // arduinoConnection.Open("/dev/ttyUSB1");
    arduinoConnection.SetBaudRate(SerialStreamBuf::BAUD_115200);
    arduinoConnection.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    arduinoConnection.SetParity(SerialStreamBuf::PARITY_NONE);
    arduinoConnection.SetNumOfStopBits(1);

    std::string answer;
    
    std::cout << "Initialize I2C" << std::endl;
    
    std::cout << "Arduino Motor Relay opened: " << (arduinoConnection.good() ? "YES" : "NO") << std::endl;
}

void MotorHelper::Reset()
{
    assert(arduinoConnection.IsOpen() && arduinoConnection.good());
    arduinoConnection << 0x01 << 0x02 << 0x03 << std::flush;
    assert(arduinoConnection.good());
}

void MotorHelper::RawSend(unsigned char directionA, unsigned char directionB, unsigned char A, unsigned char B)
{
    unsigned char direction = (((directionA & 0x0F) << 4) | (directionB & 0x0F));
    assert(arduinoConnection.IsOpen() && arduinoConnection.good());
    arduinoConnection << direction << A << B << std::flush;
    assert(arduinoConnection.good());
}

void MotorHelper::GoForward(unsigned char speed)
{
    std::cout << "Forward " << std::dec << (unsigned int)speed << std::endl;
    RawSend(8, 8, speed, speed);
    State = GoingForward;
}

void MotorHelper::GoBackward(unsigned char speed)
{
    std::cout << "Backward " << std::dec << (unsigned int)speed << std::endl;
    RawSend(4, 4, speed, speed);
    State = GoingBackward;
}

void MotorHelper::TurnLeft(unsigned char speed)
{
    std::cout << "TurnLeft " << std::dec << (unsigned int)speed << std::endl;
    RawSend(4, 8, speed, speed);
    State = TurningLeft;
}

void MotorHelper::TurnRight(unsigned char speed)
{
    std::cout << "TurnRight " << std::dec << (unsigned int)speed << std::endl;
    RawSend(8, 4, speed, speed);
    State = TurningRight;
}

void MotorHelper::Stop()
{
    std::cout << "Stop" << std::endl;
    RawSend(0, 0, 0, 0);
    State = Stopped;
}

MotorHelper::MotorHelper()
{
}

MotorHelper::MotorHelper(const MotorHelper& orig)
{
}

MotorHelper::~MotorHelper()
{
}

