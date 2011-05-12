/* 
 * File:   PhidgetHelper.h
 * Author: ros
 *
 * Created on 12. Mai 2011, 12:56
 */

#ifndef PHIDGETHELPER_H
#define	PHIDGETHELPER_H

#include <stdlib.h>
#include <phidget21.h>
#include <iostream>

class PhidgetHelper {
public:
    
    static double angle;
    
    
    static long lastAngleTime;
    static double zeroangle;
    static bool bZeroSet;
    
    static CPhidgetSpatialHandle handle;
    static void Initialize();
    static int SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count);
    
    PhidgetHelper();
    PhidgetHelper(const PhidgetHelper& orig);
    virtual ~PhidgetHelper();
private:

};

#endif	/* PHIDGETHELPER_H */

