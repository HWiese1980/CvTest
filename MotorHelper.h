/* 
 * File:   MotorHelper.h
 * Author: ros
 *
 * Created on 12. Mai 2011, 15:44
 */

#ifndef MOTORHELPER_H
#define	MOTORHELPER_H

class MotorHelper {
public:
    
    static void TurnLeft(char speed);
    static void TurnRight(char speed);
    static void Stop();
    static void Go(char speed);

    MotorHelper();
    MotorHelper(const MotorHelper& orig);
    virtual ~MotorHelper();
private:
    static void RawSend(char address, char A, char B);
};

#endif	/* MOTORHELPER_H */

