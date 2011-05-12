/* 
 * File:   PhidgetHelper.cpp
 * Author: ros
 * 
 * Created on 12. Mai 2011, 12:56
 */

#include "PhidgetHelper.h"
#include <cmath>
#include <signal.h>
#include <time.h>

CPhidgetSpatialHandle PhidgetHelper::handle = 0;
bool PhidgetHelper::bZeroSet = false;
double PhidgetHelper::angle = 0.0;
long PhidgetHelper::lastAngleTime = 0;


int spat_cb(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
    return PhidgetHelper::SpatialDataHandler(spatial, userptr, data, count);
}

std::ostream& operator<<(std::ostream& s, timespec t)
{
    s << "s: " << t.tv_sec << "; ns: " << t.tv_nsec;
    return s;
}

int PhidgetHelper::SpatialDataHandler(CPhidgetSpatialHandle spatial, void* userptr, CPhidgetSpatial_SpatialEventDataHandle* data, int count)
{
    timespec t;
    if(clock_gettime(CLOCK_MONOTONIC, &t))
    {
        std::cerr << "Error fetching HPT time!" << std::endl;
        raise(SIGINT);
    }

    unsigned long curAngleTime = t.tv_sec * pow(10, 9) + t.tv_nsec;
    unsigned long delta = curAngleTime - lastAngleTime;
    
    double secs = delta / pow(10, 9);
    // std::cout << "Time" << t << "; Current " << curAngleTime << "; Time Delta: " << delta << "ns" << " Seconds: " << secs << std::endl;
    
    CPhidgetSpatial_SpatialEventDataHandle last = data[count-1];
    
    double length = sqrt(pow(last->angularRate[0], 2) + pow(last->angularRate[1], 2) + pow(last->angularRate[2], 2));
    if(length > 1 && lastAngleTime != 0)
    {
        double add = (last->angularRate[2] * secs);
        if(angle + add > 360) angle -= 360;
        if(angle - add < 0) angle += 360;
        
        angle += add;
        
        std::cout << "Angle changed to: " << angle << std::endl;
    }

    lastAngleTime = curAngleTime;
    
}

void PhidgetHelper::Initialize()
{
    CPhidgetSpatial_create(&handle);
    std::cout << "Handle: " << handle << std::endl;
    CPhidgetSpatial_set_OnSpatialData_Handler(handle, PhidgetHelper::SpatialDataHandler, NULL);
    std::cout << "Initialized spatial callback" << std::endl;
    CPhidget_open((CPhidgetHandle)handle, -1);
    std::cout << "Handle opened" << std::endl;
    
    int result = 0;
    if(result = CPhidget_waitForAttachment((CPhidgetHandle)handle, 1500))
    {
        const char* err;
        CPhidget_getErrorDescription(result, &err);
        std::cerr << "PHIDGET ERROR: " << err << std::endl;
        raise(SIGTERM);
        return;
    }
    
    std::cout << "Attached" << std::endl;

    // double mag[3];
    
    // CPhidgetSpatial_getMagneticField(handle, 0, &mag[0]);
    // CPhidgetSpatial_getMagneticField(handle, 1, &mag[1]);
    // CPhidgetSpatial_getMagneticField(handle, 2, &mag[2]);
    
    // std::cout << "Magnetic Field " << "x: " << mag[0] << "; y: " << mag[1] << "; z: " << mag[2] << std::endl;
    // double mag_angle = atan(mag[0] / mag[1]);

    CPhidgetSpatial_setDataRate(handle, 16);
    
    
    std::cout << "Datarate set" << std::endl;
}

PhidgetHelper::PhidgetHelper()
{
}

PhidgetHelper::PhidgetHelper(const PhidgetHelper& orig)
{
}

PhidgetHelper::~PhidgetHelper()
{
}

