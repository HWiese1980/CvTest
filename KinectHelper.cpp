#include "KinectHelper.h"
#include "Corners.h"
#include <fstream>

#include <boost/regex.hpp>

// bool KinectHelper::bCalibrated = false;
CvPoint KinectHelper::VanishingPoint = cv::Point(-1, -1);
freenect_device* KinectHelper::dev = NULL;
CvArr* KinectHelper::depthData = NULL;
std::list<double> KinectHelper::avg_values;
std::vector<CvPoint> KinectHelper::pointsUsedForCalibration;


double KinectHelper::frame_offset = -320;
double KinectHelper::view_angle = DEG2RAD(0.0);
double KinectHelper::add_depth_cm = 0.0, KinectHelper::add_depth_px = 0.0;
double KinectHelper::view_plane_distance_cm = 0.0;
double KinectHelper::v_px_per_cm = 0.0, KinectHelper::h_px_per_cm = 0.0;
double KinectHelper::distance_coefficient = 8.0;

int KinectHelper::absolute_x = 0, KinectHelper::absolute_y = 0;
bool KinectHelper::bAandVCalibrated = false;
bool KinectHelper::bVPCalibrated = false;


CvPoint SubPoints(const CvPoint& a, const CvPoint& b);

double KinectHelper::GetTilt()
{
    freenect_update_tilt_state(dev);
    freenect_raw_tilt_state* state = freenect_get_tilt_state(dev);
    return freenect_get_tilt_degs(state);
}

double KinectHelper::GetKinectHeight()
{
    return 38.0; 
}

double KinectHelper::GetDirectDistanceInCM(double distanceValue)
{
    assert(dev != NULL);
    assert(depthData != NULL);
    double ret = ((tan(distanceValue / 1024 + 0.5) * 33.825 + 5.7)); // Distanz direkt von Kinect zum Objekt
    return ret;
}

double KinectHelper::GetDistanceOverGround(double distanceValue)
{
    assert(dev != NULL);
    assert(depthData != NULL);
    double dist_cm = GetDirectDistanceInCM(distanceValue);
    double height_cm = GetKinectHeight();
    double ret = sqrt(pow(dist_cm, 2) - pow(height_cm, 2)); // Pythagoras
    
    return ret;
}

void KinectHelper::CalibrateAnglesAndViewport()
{
    if(bAandVCalibrated) return;
    double pitch_rad = DEG2RAD(GetTilt());
    view_plane_distance_cm = sin(pitch_rad) * KinectHelper::GetKinectHeight();
    add_depth_cm = view_plane_distance_cm * (1/tan(pitch_rad) - tan(fov/2));
    
    view_plane_distance_cm = KinectHelper::GetDirectDistanceInCM(cvGet2D(depthData, 320, 240).val[0]);
    v_px_per_cm = abs(240 / (view_plane_distance_cm * tan(fov/2)));
    h_px_per_cm = abs(320 / (view_plane_distance_cm * tan(fov/2)));
    
    std::cout << "|--> Vertical px/cm: " << v_px_per_cm << "; Horizontal px/cm: " << h_px_per_cm << std::endl;
    
    add_depth_px = add_depth_cm * v_px_per_cm;
    std::cout << "|--> Viewplane distance: " << view_plane_distance_cm << std::endl;
    std::cout << "|--> Additional vertical pixels: " << add_depth_px << std::endl;
    bAandVCalibrated = true;
}

void KinectHelper::CalibrateVanishingPoint()
{
    if(bVPCalibrated) return;
    std::map<CornerPosition, CvPoint> calibration_corners;
    if(pointsUsedForCalibration.size() == 4 /*&& VanishingPoint.x != 320*/) // Vierpunktkalibrierung
    {
        calibration_corners.clear();
        for(std::vector<CvPoint>::iterator it = pointsUsedForCalibration.begin(); it != pointsUsedForCalibration.end(); it++)
        {
            CvPoint& pnt = (*it);
            if(pnt.x > 320) // rechts von der Mitte;
            {
                if(calibration_corners.count(TopRight) == 0) calibration_corners[TopRight] = pnt;
                else if(calibration_corners.count(BottomRight) == 0) calibration_corners[BottomRight] = pnt;
            }
            else if(pnt.x < 320) // links von der Mitte;
            {
                if(calibration_corners.count(TopLeft) == 0) calibration_corners[TopLeft] = pnt;
                else if(calibration_corners.count(BottomLeft) == 0) calibration_corners[BottomLeft] = pnt;
            }
        }

        double delta_x = calibration_corners[BottomRight].x - calibration_corners[TopRight].x;
        double delta_y = calibration_corners[BottomRight].y - calibration_corners[TopRight].y;
        double delta_v_x = calibration_corners[TopRight].x - 320;

        double fact_x = delta_v_x / delta_x;
        double delta_v_y = delta_y * fact_x;

        double vanishing_x = 320, vanishing_y = calibration_corners[TopRight].y - delta_v_y;
        
        KinectHelper::VanishingPoint.x  = vanishing_x; KinectHelper::VanishingPoint.y = vanishing_y;

        // std::cout << "H-Dist 1: " << h_dist1 << "; H-Dist 2: " << h_dist2 << std::endl;
        std::cout << "|--> VANISHING POINT: " << KinectHelper::VanishingPoint << std::endl;
            bVPCalibrated = true;    
    }
}

void KinectHelper::DrawCalibrationData(CvArr* img)
{
    // if(!bCalibrated) return;
    
    cvCircle(img, VanishingPoint, 100, CV_RGB(255, 255, 0));
    
    
    for(std::vector<CvPoint>::iterator it = pointsUsedForCalibration.begin(); it != pointsUsedForCalibration.end(); it++)
    {
        cvCircle(img, *it, 5, CV_RGB(0,255,255), 2);
    }
    
    if(pointsUsedForCalibration.size() != 4) return;
    cvLine(img, pointsUsedForCalibration[0], pointsUsedForCalibration[1], CV_RGB(0, 255, 255));
    cvLine(img, pointsUsedForCalibration[1], pointsUsedForCalibration[3], CV_RGB(0, 255, 255));
    cvLine(img, pointsUsedForCalibration[3], pointsUsedForCalibration[2], CV_RGB(0, 255, 255));
    cvLine(img, pointsUsedForCalibration[2], pointsUsedForCalibration[0], CV_RGB(0, 255, 255));
}



CvPoint KinectHelper::GetAbsoluteX(CvPoint point)
{
    CvPoint delta = point - VanishingPoint;
    double m = (double)delta.x / (double)delta.y;
    
    double frame_x = ((480 + add_depth_px) - VanishingPoint.y) * m;
    
    CvPoint ret = cv::Point(320 + frame_x, 480 + add_depth_px); 
    return ret;
}

CvPoint KinectHelper::GetAbsoluteCoordinates(double distanceValue, double xOnImage)
{
    CvPoint absKinect = cv::Point(absolute_x, absolute_y);

    CvPoint ToLeftBorder = GetLeftFrameEdgeVector();
    CvPoint ToFrame = GetOnImageVector(xOnImage);
    CvPoint ToPos = GetToPosVector(distanceValue);
    
    CvPoint ret = (absKinect + ToLeftBorder + ToFrame + ToPos) * 0.1;
    
    // std::cout << "GetAbsoluteCoordinates: ToFrame: " << ToFrame << std::endl;
    // std::cout << "GetAbsoluteCoordinates: xOnImage: " << xOnImage << std::endl;
    return ret;
}

CvPoint KinectHelper::GetToPosVector(double Distance)
{
    return cv::Point(cos(view_angle) * Distance, -sin(view_angle) * Distance);
}

CvPoint KinectHelper::GetOnImageVector(double XOnImage)
{
    return cv::Point(sin(view_angle) * XOnImage, cos(view_angle) * XOnImage);
}

CvPoint KinectHelper::GetLeftFrameEdgeVector()
{
    return cv::Point((sin(view_angle)*320), cos(view_angle) * 320);
}


CvPoint operator+(CvPoint a, CvPoint b)
{
    CvPoint ret = cv::Point(a.x + b.x, a.y + b.y);
    return ret;
}

CvPoint operator-(CvPoint a, CvPoint b)
{
    CvPoint ret = cv::Point(a.x - b.x, a.y - b.y);
    return ret;
}

CvPoint operator*(CvPoint a, double scalar)
{
    CvPoint ret = cv::Point(a.x * scalar, a.y *scalar);
    return ret;
}

bool operator>(CvScalar a, double max)
{
    double sum = 0.0;
    for(int elm = 0; elm < sizeof(a.val); elm++)
    {
        sum += pow(a.val[elm], 2);
    }
    double length = sqrt(sum);
    return (length > max);
}


std::ostream& operator<<(std::ostream& s, const cv::Mat& mat)
{
    s << "[Channels: " << mat.channels() << "][Size: W:" << mat.size().width << ";H: " << mat.size().height << "][Rows: " << mat.rows << "][Cols: " << mat.cols << "]";
    return s;
}

std::ostream& operator<<(std::ostream& s, const CvPoint& point)
{
    s << "[" << point.x << ":" << point.y << "]"; 
    return s;
}

std::istream& operator>>(std::istream& s, CvPoint& point)
{
    string data;
    s >> data;
    
    boost::regex rgx("\\[(.*?):(.*?)\\]");
    boost::match_results<std::string::const_iterator> what;
    if(boost::regex_match(data, what, rgx, boost::match_default | boost::match_partial) != 0)
    {
        std::cout << "OK" << std::endl;
    }
    std::cout << "What: " << what[1] << " - " << what[2] << std::endl;
    string x = what[1].str();
    string y = what[2].str();
    point.x = atoi(x.c_str());
    point.y = atoi(y.c_str());
    
    return s;
}

bool leftOf(const std::pair<int, cvb::CvBlob*>& point, const std::pair<int, cvb::CvBlob*>& of)
{
    return point.second->centroid.x < of.second->centroid.x;
}

bool rightOf(const std::pair<int, cvb::CvBlob*>& point, const std::pair<int, cvb::CvBlob*>& of)
{
    return point.second->centroid.x > of.second->centroid.x;
}

bool smallestIndex(const std::pair<int, cvb::CvBlob*>& a, const std::pair<int, cvb::CvBlob*>& b)
{
    return a.first < b.first;
}

/*
CvPoint SubPoints(const CvPoint& a, const CvPoint& b)
{
    return cv::Point(a.x - b.x, a.y - b.y);
}
 * 
 * **/