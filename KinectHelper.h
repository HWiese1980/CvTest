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

#include <functional>
#include <iostream>
#include <queue>

#include <list>
#include <cmath>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

#include "CvShapes.h"
#include "PlayingField.h"

#define BNU boost::numeric::ublas

/*
bool fartherRight(CvPoint a, CvPoint b);
bool fartherLeft(CvPoint a, CvPoint b);
bool fartherUp(CvPoint a, CvPoint b);
bool fartherDown(CvPoint a, CvPoint b);
void Raster(CvPoint& point, double x, double y);
std::queue<CvPoint> Border(std::vector<CvPoint> points, std::function<bool()> func);
*/

typedef enum {
    Vertical,
    Horizontal,
} Orientation;

class KinectHelper {
public:
    static const double fov;
    static double view_plane_distance_cm, v_px_per_cm, h_px_per_cm; 
    static double frame_offset;
    static double distance_coefficient;
    
    static double scale;
    
    static CvRect straight_rect;
    
    static BNU::vector<double> projectiveTransformationVector;
    
    static double GetDirectDistance(CvPoint pt);
    static double GetDirectDistanceInCM(double distanceValue);
    static double GetDistanceOverGround(double distanceValue);
    static double GetKinectHeight();
    static double GetTilt();
    
    static void CalibrateVanishingPoint(); static bool bVPCalibrated;
    static void CalibrateAnglesAndViewport(); static bool bAandVCalibrated;
    static void SetupProjectionVector();
    static void SetupPlayingField(int sX, int sY);
    
    static void DrawCalibrationData(CvArr* img);

    static CvPoint GetAbsoluteX(CvPoint point);
    static CvPoint GetAbsoluteCoordinates(double distanceValue, double xOnImage);
    
    static CvPoint GetLeftFrameEdgeVector();
    static CvPoint GetOnImageVector(double XOnImage);
    static CvPoint GetToPosVector(double Distance);
    
    static double In_px(double cm, Orientation o);
    static double In_cm(double px, Orientation o);
    
    static void Raster(CvPoint& point, double x, double y);
    
    static void DrawProjectedPoint(CvArr* img, CvPoint point);
    static CvPoint ProjectPoint(CvPoint point);
    static void ProjectImage(CvArr* src, CvArr* dst);
    
    static std::list<double> avg_values;
    
    static double view_angle;
    static int absolute_x, absolute_y;
    
    static double add_depth_cm, add_depth_px;
    
    static freenect_device* dev;
    static CvArr* depthData;
    
    static CvPoint VanishingPoint;
    static std::vector<CvPoint> pointsUsedForCalibration;
    
};

bool operator>(CvScalar a, double max);

bool leftOf(const std::pair<int, cvb::CvBlob*>& point, const std::pair<int, cvb::CvBlob*>& of);
bool rightOf(const std::pair<int, cvb::CvBlob*>& point, const std::pair<int, cvb::CvBlob*>& of);

bool smallestIndex(const std::pair<int, cvb::CvBlob*>& a, const std::pair<int, cvb::CvBlob*>& b);

#endif	/* KINECTHELPER_H */



