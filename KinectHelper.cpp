#include "KinectHelper.h"


freenect_device* KinectHelper::dev = NULL;
CvArr* KinectHelper::depthData = NULL;
std::list<double> KinectHelper::avg_values;

int KinectHelper::absolute_x = 0, KinectHelper::absolute_y = 0;
double KinectHelper::view_angle = 0.0;

double KinectHelper::GetKinectHeight()
{
    return 62.0; // hardcoded in cm; alles andere hat keinen Sinn
    
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