#include "KinectHelper.h"
#include "Corners.h"
#include <fstream>

#include <boost/regex.hpp>

bool KinectHelper::bCalibrated = false;
CvPoint KinectHelper::VanishingPoint = cv::Point(-1, -1);
freenect_device* KinectHelper::dev = NULL;
CvArr* KinectHelper::depthData = NULL;
std::list<double> KinectHelper::avg_values;

double KinectHelper::view_angle = DEG2RAD(180.0 + 45.0);
double KinectHelper::add_depth_cm = 0.0, KinectHelper::add_depth_px = 0.0;
double KinectHelper::view_plane_distance_cm = 0.0;
double KinectHelper::v_px_per_cm = 0.0, KinectHelper::h_px_per_cm = 0.0;

int KinectHelper::absolute_x = 0, KinectHelper::absolute_y = 0;


CvPoint SubPoints(const CvPoint& a, const CvPoint& b);

double KinectHelper::GetTilt()
{
    freenect_update_tilt_state(dev);
    freenect_raw_tilt_state* state = freenect_get_tilt_state(dev);
    return freenect_get_tilt_degs(state);
}

double KinectHelper::GetKinectHeight()
{
    return 38.0; // hardcoded in cm; alles andere hat keinen Sinn
    
/*    
    assert(dev != NULL);
    assert(depthData != NULL);
    cv::Mat depthMat = cv::Mat((IplImage*)depthData);
    
    int rows = depthMat.rows;
    int cols = depthMat.cols;
    
    int posX = cols / 2.0;
    int posY = (rows * 0.75);
    
    
    uchar value = depthMat.ptr(posY)[posX];
    freenect_update_tilt_state(dev);
    
    freenect_raw_tilt_state* state = freenect_get_tilt_state(dev);
    double angle = freenect_get_tilt_degs(state);
    double dist = GetDirectDistanceInCM(value) / 2;

    double height_inCM = cos(angle) * dist;
    
    avg_values.push_front(height_inCM);
    if(avg_values.size() > 1000) avg_values.pop_back();
    
    double sum = 0.0;
    for(std::list<double>::iterator it = avg_values.begin(); it != avg_values.end(); it++)
    {
        sum += (*it);
    }
    
    /*
    std::cout << "KinectHelper" << std::endl;
    std::cout << "-------------------------" << std::endl;
    std::cout << "| Pos: x:" << posX << "; y:" << posY << std::endl;
    std::cout << "| Distance: " << dist << std::endl;
    std::cout << "| Angle: " << angle << std::endl;
    std::cout << "| AvgValues: " << avg_values.size() << std::endl;
    std::cout << "-------------------------" << std::endl;
    
    return sum / avg_values.size();
 * */
}

double KinectHelper::GetDirectDistanceInCM(double distanceValue)
{
    assert(dev != NULL);
    assert(depthData != NULL);
    double ret = ((tan(distanceValue / 255 + 0.5) * 33.825 + 5.7)); // Distanz direkt von Kinect zum Objekt
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

CvPoint KinectHelper::GetAbsoluteCoordinates(double distanceValue, double xOnImage)
{
    // double xOnFrame = xOnImage - 320; // Relative horizontale Position der Figur zum Kinect IM BILD
    
    CvPoint absKinect = cv::Point(absolute_x, absolute_y);

    CvPoint ToLeftBorder = cv::Point(-sin(view_angle) * 320, cos(view_angle) * 320);
    CvPoint ToFrame = cv::Point(sin(view_angle) * xOnImage, cos(view_angle) * xOnImage);
    CvPoint ToPos = cv::Point(cos(view_angle) * distanceValue, sin(view_angle) * distanceValue);
    
    CvPoint ret = absKinect + ToLeftBorder + ToFrame + ToPos;
    
    return ret;
}

void KinectHelper::CalibrateAnglesAndViewport()
{
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
}

void KinectHelper::CalibrateVanishingPoint(std::vector<CvPoint> four_points)
{
    std::ifstream f_calibration_data_in;
    f_calibration_data_in.open("/home/ros/.otter1/calibration");
    if(f_calibration_data_in.is_open())
    {
        f_calibration_data_in >> VanishingPoint;
        std::cout << "Restored Vanishing Point: " << VanishingPoint << std::endl;
        f_calibration_data_in.close();
    }
    
    std::map<CornerPosition, CvPoint> calibration_corners;
    
    std::ofstream f_calibration_data_out;
    f_calibration_data_out.open("/home/ros/.otter1/calibration", ios::trunc);
    assert(f_calibration_data_out.is_open());
    
    if(four_points.size() == 4 && VanishingPoint.x != 320) // Vierpunktkalibrierung
    {
        calibration_corners.clear();
        for(std::vector<CvPoint>::iterator it = four_points.begin(); it != four_points.end(); it++)
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

        // double h_dist1 = calibration_corners[TopRight].x - calibration_corners[TopLeft].x;
        // double h_dist2 = calibration_corners[BottomRight].x - calibration_corners[BottomLeft].x;

        double delta_x = calibration_corners[BottomRight].x - calibration_corners[TopRight].x;
        double delta_y = calibration_corners[BottomRight].y - calibration_corners[TopRight].y;
        double delta_v_x = calibration_corners[TopRight].x - 320;

        double fact_x = delta_v_x / delta_x;
        double delta_v_y = delta_y * fact_x;

        double vanishing_x = 320, vanishing_y = calibration_corners[TopRight].y - delta_v_y;
        
        KinectHelper::VanishingPoint.x  = vanishing_x; KinectHelper::VanishingPoint.y = vanishing_y;

        // std::cout << "H-Dist 1: " << h_dist1 << "; H-Dist 2: " << h_dist2 << std::endl;
        std::cout << "|--> VANISHING POINT: " << KinectHelper::VanishingPoint << std::endl;
    }

    f_calibration_data_out << VanishingPoint << std::endl;
    f_calibration_data_out.close();
}

void KinectHelper::DrawCalibrationData(CvArr* img)
{
    cvCircle(img, VanishingPoint, 100, CV_RGB(255, 255, 0));
}

CvPoint KinectHelper::GetAbsoluteX(CvPoint point)
{
    CvPoint delta = SubPoints(point, VanishingPoint);
    double m = (double)delta.x / (double)delta.y;
    
    double frame_x = ((480 + add_depth_px) - VanishingPoint.y) * m;
    
    CvPoint ret = cv::Point(320 + frame_x, 480 + add_depth_px); 
    return ret;
}

CvPoint operator+(CvPoint a, CvPoint b)
{
    CvPoint ret = cv::Point(a.x + b.x, a.y + b.y);
    return ret;
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

CvPoint SubPoints(const CvPoint& a, const CvPoint& b)
{
    return cv::Point(a.x - b.x, a.y - b.y);
}