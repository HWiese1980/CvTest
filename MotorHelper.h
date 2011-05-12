/* 
 * File:   MotorHelper.h
 * Author: ros
 *
 * Created on 12. Mai 2011, 15:44
 */

#ifndef MOTORHELPER_H
#define	MOTORHELPER_H

#include <iostream>
#include <fstream>

typedef enum { 
    GoingForward,
    GoingBackward,
    TurningLeft,
    TurningRight,
    Stopped
} MovementState;

class MotorHelper {
public:
    static MovementState State;
    static void TurnLeft(unsigned char speed);
    static void TurnRight(unsigned char speed);
    static void Stop();
    static void GoForward(unsigned char speed);
    static void GoBackward(unsigned char speed);
    
    static void Initialize();

    MotorHelper();
    MotorHelper(const MotorHelper& orig);
    virtual ~MotorHelper();
private:
    static std::ofstream arduinoStream;
    static void RawSend(unsigned char directionA, unsigned char directionB, unsigned char A, unsigned char B);
};

#endif	/* MOTORHELPER_H */

