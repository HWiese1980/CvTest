/* 
 * File:   KinectHelper.h
 * Author: ros
 *
 * Created on 28. April 2011, 15:07
 */

#ifndef KINECTHELPER_H
#define	KINECTHELPER_H

#include <opencv/cv.h>
#include <libfreenect/libfreenect.h>
#include <iostream>

#include <list>

class KinectHelper {
public:
    static double GetDirectDistanceInCM(double distanceValue);
    static double GetDistanceOverGround(double distanceValue);
    static double GetKinectHeight();
    
    static std::list<double> avg_values;
    
    static double view_angle;
    static int absolute_x, absolute_y;
    
    static freenect_device* dev;
    static CvArr* depthData;
};

#endif	/* KINECTHELPER_H */

