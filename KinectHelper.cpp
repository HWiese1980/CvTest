#include "KinectHelper.h"


freenect_device* KinectHelper::dev = NULL;
CvArr* KinectHelper::depthData = NULL;
std::list<double> KinectHelper::avg_values;

int KinectHelper::absolute_x = 0, KinectHelper::absolute_y = 0;
double KinectHelper::view_angle = 0.0;

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
