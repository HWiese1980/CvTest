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

std::ofstream MotorHelper::arduinoStream;
MovementState MotorHelper::State = Stopped;

void MotorHelper::Initialize()
{
    arduinoStream.open("/dev/ard_relay", std::ios::out | std::ios::binary);
    std::cout << "Arduino Motor Relay opened: " << (arduinoStream.good() ? "YES" : "NO") << std::endl;
}

void MotorHelper::RawSend(unsigned char directionA, unsigned char directionB, unsigned char A, unsigned char B)
{
    char direction = (((directionA & 0x0F) << 4) | (directionB & 0x0F));
    assert(arduinoStream.is_open() && arduinoStream.good());
    
    arduinoStream << direction << A << B << std::flush;
    assert(arduinoStream.good());
}

void MotorHelper::GoForward(unsigned char speed)
{
    std::cout << "Forward " << std::dec << (unsigned int)speed << std::endl;
    RawSend(2, 2, speed, speed);
    State = GoingForward;
}

void MotorHelper::GoBackward(unsigned char speed)
{
    std::cout << "Backward " << std::dec << (unsigned int)speed << std::endl;
    RawSend(1, 1, speed, speed);
    State = GoingBackward;
}

void MotorHelper::TurnLeft(unsigned char speed)
{
    std::cout << "TurnLeft " << std::dec << (unsigned int)speed << std::endl;
    RawSend(1, 2, speed, speed);
    State = TurningLeft;
}

void MotorHelper::TurnRight(unsigned char speed)
{
    std::cout << "TurnRight " << std::dec << (unsigned int)speed << std::endl;
    RawSend(2, 1, speed, speed);
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

