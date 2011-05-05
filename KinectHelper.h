/* 
 * File:   KinectHelper.h
 * Author: ros
 *
 * Created on 28. April 2011, 15:07
 */

#ifndef KINECTHELPER_H
#define	KINECTHELPER_H

#define DEG2RAD(x) (x * (3.14159 / 180))

#include <opencv/cv.h>
#include <libfreenect/libfreenect.h>
#include <cvblob.h>
#include <iostream>

#include <list>
#include <cmath>

class KinectHelper {
public:
    static const double fov = DEG2RAD(62.7);
    static double view_plane_distance_cm, v_px_per_cm, h_px_per_cm; 
    
    static double GetDirectDistance(CvPoint pt);
    static double GetDirectDistanceInCM(double distanceValue);
    static double GetDistanceOverGround(double distanceValue);
    static double GetKinectHeight();
    static double GetTilt();
    
    static void CalibrateVanishingPoint(std::vector<CvPoint> four_points);
    static void CalibrateAnglesAndViewport();
    static void DrawCalibrationData(CvArr* img);
    static CvPoint GetAbsoluteX(CvPoint point);
    
    static CvPoint GetAbsoluteCoordinates(double distanceValue, double xOnImage);
    
    static std::list<double> avg_values;
    
    static double view_angle;
    static int absolute_x, absolute_y;
    
    static double add_depth_cm, add_depth_px;
    
    static freenect_device* dev;
    static CvArr* depthData;
    
    static CvPoint VanishingPoint;
private:
    static bool bCalibrated;
};

CvPoint operator+(CvPoint a, CvPoint b);
std::ostream& operator<<(std::ostream& s, const cv::Mat& mat);
std::ostream& operator<<(std::ostream& s, const CvPoint& point);
std::istream& operator>>(std::istream& s, CvPoint& point);

bool leftOf(const std::pair<int, cvb::CvBlob*>& point, const std::pair<int, cvb::CvBlob*>& of);
bool rightOf(const std::pair<int, cvb::CvBlob*>& point, const std::pair<int, cvb::CvBlob*>& of);

bool smallestIndex(const std::pair<int, cvb::CvBlob*>& a, const std::pair<int, cvb::CvBlob*>& b);

#endif	/* KINECTHELPER_H */

