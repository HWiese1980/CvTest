#include "KinectHelper.h"
#include "Corners.h"
#include <fstream>

#include <boost/numeric/bindings/lapack/lapack.hpp>
#include <boost/regex.hpp>

// bool KinectHelper::bCalibrated = false;
const double KinectHelper::fov = DEG2RAD(62.7);
CvPoint KinectHelper::VanishingPoint = cv::Point(-1, -1);
freenect_device* KinectHelper::dev = NULL;
CvArr* KinectHelper::depthData = NULL;
std::list<double> KinectHelper::avg_values;
std::vector<CvPoint> KinectHelper::pointsUsedForCalibration;
CvRect KinectHelper::straight_rect;

BNU::vector<double> KinectHelper::projectiveTransformationVector;

double KinectHelper::frame_offset = -320;
double KinectHelper::view_angle = DEG2RAD(0.0);
double KinectHelper::add_depth_cm = 0.0, KinectHelper::add_depth_px = 0.0;
double KinectHelper::view_plane_distance_cm = 0.0;
double KinectHelper::v_px_per_cm = 0.0, KinectHelper::h_px_per_cm = 0.0;
double KinectHelper::distance_coefficient = 8.0;
double KinectHelper::scale = 0.15;


int KinectHelper::absolute_x = 0, KinectHelper::absolute_y = 0;
bool KinectHelper::bAandVCalibrated = false;
bool KinectHelper::bVPCalibrated = false;

using boost::numeric::ublas::matrix;

CvPoint SubPoints(const CvPoint& a, const CvPoint& b);

double KinectHelper::GetTilt()
{
    freenect_update_tilt_state(dev);
    freenect_raw_tilt_state* state = freenect_get_tilt_state(dev);
    return freenect_get_tilt_degs(state);
}

double KinectHelper::GetKinectHeight()
{
    return 37.0; 
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

/*
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
 * 
 * **/

void KinectHelper::CalibrateAnglesAndViewport()
{
    if(bAandVCalibrated) return;
    double pitch_rad = DEG2RAD(-GetTilt());
    double to_lower_border_rad = DEG2RAD(90) - pitch_rad - (fov/2); 
    std::cout << "FOV: " << fov <<  "; Pitch: " << pitch_rad << std::endl;
    
    double dist_at_lower_border = KinectHelper::GetKinectHeight() / cos(to_lower_border_rad); // in cm
    
    double v_cm = (dist_at_lower_border * sin(fov/2));
    
    v_px_per_cm = 240.0 / v_cm; 
    h_px_per_cm = v_px_per_cm * (640.0 / 480.0);
    
    view_plane_distance_cm = sin(pitch_rad) * KinectHelper::GetKinectHeight();
    
    std::cout << "Vertical Pixels per cm: " << v_px_per_cm << std::endl;
    std::cout << "Horizontal Pixels per cm: " << h_px_per_cm << std::endl;
    std::cout << "Viewplane is: " << view_plane_distance_cm << std::endl;
    
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

bool pointSort(CvPoint a, CvPoint b)
{
    return(a.x + (a.y * 1000) < b.x + (b.y * 1000));
}

double KinectHelper::In_cm(double px, Orientation o)
{
    switch(o)
    {
        case Horizontal: return (px / h_px_per_cm);
        case Vertical: return (px / v_px_per_cm);
        default: throw "Blubb";
    }
}

double KinectHelper::In_px(double cm, Orientation o)
{
    switch(o)
    {
        case Horizontal: return (cm * h_px_per_cm);
        case Vertical: return (cm * h_px_per_cm);
        default: throw "Blubb";
    }
}



void KinectHelper::SetupProjectionVector()
{
    std::vector<CvPoint> straightRect;
    
    if(pointsUsedForCalibration.size() == 4)
    {
        std::vector<CvPoint> points = std::vector<CvPoint>(pointsUsedForCalibration);
        sort(points.begin(), points.end(), pointSort);

        double height = In_px(70, Vertical);
        double width = In_px(70, Horizontal);
        CvPoint center_of_calibration = cv::Point( (points[0].x + points[1].x + points[2].x + points[3].x) / 4, (points[0].y + points[1].y + points[2].y + points[3].y) / 4);
        
        CvPoint p1 = cv::Point(-(width/2), -(height/2));
        CvPoint p2 = cv::Point((width/2), -(height/2));
        CvPoint p3 = cv::Point(-(width/2), (height/2));
        CvPoint p4 = cv::Point((width/2), (height/2));
        
        straightRect.push_back(center_of_calibration + p1);
        straightRect.push_back(center_of_calibration + p2);
        straightRect.push_back(center_of_calibration + p3);
        straightRect.push_back(center_of_calibration + p4);
        
        BNU::vector<double> outVect = BNU::vector<double>(8);
        BNU::vector<double> inVect = BNU::vector<double>(8);
        BNU::matrix<double, BNU::column_major> inMatrix = BNU::matrix<double>(8,8);

        for(int m = 0; m < 4; m++)
        {
            int n = m * 2;
            
            inMatrix(n, 0) = points[m].x;
            inMatrix(n, 1) = points[m].y;       

            inMatrix(n, 2) = 1;
            inMatrix(n, 3) = 0;

            inMatrix(n, 4) = 0;
            inMatrix(n, 5) = 0;

            inMatrix(n, 6) = -straightRect[m].x * points[m].x;
            inMatrix(n, 7) = -straightRect[m].x * points[m].y;       

            inMatrix(n+1, 0) = 0;
            inMatrix(n+1, 1) = 0;       

            inMatrix(n+1, 2) = 0;
            inMatrix(n+1, 3) = points[m].x;

            inMatrix(n+1, 4) = points[m].y;
            inMatrix(n+1, 5) = 1;

            inMatrix(n+1, 6) = -straightRect[m].y * points[m].x;
            inMatrix(n+1, 7) = -straightRect[m].y * points[m].y;       
            
            inVect(n) = straightRect[m].x;
            inVect(n+1) = straightRect[m].y;
        }
        
        boost::numeric::bindings::lapack::gesv(inMatrix, inVect);
        projectiveTransformationVector = inVect;
    }
}


void KinectHelper::DrawCalibrationData(CvArr* img)
{
    // if(!bCalibrated) return;
    
    cvCircle(img, VanishingPoint, 100, CV_RGB(255, 255, 255));
    
    for(std::vector<CvPoint>::iterator it = pointsUsedForCalibration.begin(); it != pointsUsedForCalibration.end(); it++)
    {
        if(pointsUsedForCalibration.size() >= 4) DrawProjectedPoint(img, *it);
        cvCircle(img, *it, 5, CV_RGB(0,255,255), 2);
    }
    
    if(pointsUsedForCalibration.size() != 4) return;
    cvLine(img, pointsUsedForCalibration[0], pointsUsedForCalibration[1], CV_RGB(0, 255, 255));
    cvLine(img, pointsUsedForCalibration[1], pointsUsedForCalibration[3], CV_RGB(0, 255, 255));
    cvLine(img, pointsUsedForCalibration[3], pointsUsedForCalibration[2], CV_RGB(0, 255, 255));
    cvLine(img, pointsUsedForCalibration[2], pointsUsedForCalibration[0], CV_RGB(0, 255, 255));
}

void KinectHelper::DrawProjectedPoint(CvArr* img, CvPoint point)
{
    CvPoint prj = ProjectPoint(point);
    
    cvCross(img, point, 3, CV_RGB(128,128,255), 2);
    cvCross(img, prj, 3, CV_RGB(128,255,128), 2);
    cvLine(img, point, prj, CV_RGB(128,128,0));
    
}

void KinectHelper::ProjectImage(CvArr* src, CvArr* dst)
{
    if(pointsUsedForCalibration.size() != 4) return;
    
    cv::Mat m_src = cv::Mat((IplImage*)src);
    cv::Mat m_dst = cv::Mat((IplImage*)dst);
    
    for(int row = 0; row < m_src.rows; row++)
    {
        uchar* s_row = m_src.ptr(row);
        for(int col=0; col < m_src.cols * 3; col++)
        {
            CvPoint pnt = cv::Point(col, row);
            CvPoint prj_pnt = ProjectPoint(pnt);
            
            if(prj_pnt.x < 0 | prj_pnt.x >= m_dst.cols) continue;
            if(prj_pnt.y < 0 | prj_pnt.y >= m_dst.rows) continue;
            
            uchar* d_row = m_dst.ptr(prj_pnt.y);
            
            (&d_row[prj_pnt.x])[2] = (&s_row[col])[2];
            (&d_row[prj_pnt.x])[1] = (&s_row[col])[1];
            (&d_row[prj_pnt.x])[0] = (&s_row[col])[0];
        }
    }
}

CvPoint KinectHelper::ProjectPoint(CvPoint point)
{
    CvPoint ret;
    ret.x = (projectiveTransformationVector[0] * point.x + projectiveTransformationVector[1] * point.y + projectiveTransformationVector[2]) / (projectiveTransformationVector[6] * point.x + projectiveTransformationVector[7] * point.y + 1);
    ret.y = (projectiveTransformationVector[3] * point.x + projectiveTransformationVector[4] * point.y + projectiveTransformationVector[5]) / (projectiveTransformationVector[6] * point.x + projectiveTransformationVector[7] * point.y + 1);

    /*
    
    double middle_x = 320;
    double middle_y = 240;
    
   
    CvPoint upper_y = cv::Point(middle_x, (pointsUsedForCalibration[0].y + pointsUsedForCalibration[1].y) / 2);
    CvPoint lower_y = cv::Point(middle_x, (pointsUsedForCalibration[2].y + pointsUsedForCalibration[3].y) / 2);

    CvPoint left_x = cv::Point((pointsUsedForCalibration[0].x + pointsUsedForCalibration[2].x) / 2, middle_y);
    CvPoint right_x = cv::Point((pointsUsedForCalibration[1].x + pointsUsedForCalibration[3].x) / 2, middle_y);
    
    double r_x = abs((right_x.x - left_x.x) / 4);
    double r_y = abs((lower_y.y - upper_y.y) / 4);
    
    Raster(ret, r_x, r_y); */
    
    return ret;
}

CvPoint KinectHelper::GetAbsoluteX(CvPoint point)
{
    CvPoint delta = point - VanishingPoint;
    double m = (double)delta.x / (double)delta.y;
    
    double frame_x = ((480 + add_depth_px) - VanishingPoint.y) * m;
    
    // CvPoint ret = cv::Point(320 + frame_x, 480 + add_depth_px); 
    
    
    CvPoint ret = ProjectPoint(point);
    return ret;
}

CvPoint KinectHelper::GetAbsoluteCoordinates(double yOnImage_in_cm, double xOnImage_in_cm)
{
    CvPoint absKinect = cv::Point(absolute_x, absolute_y);

    CvPoint ToLeftBorder = GetLeftFrameEdgeVector();
    CvPoint ToFrame = GetOnImageVector(xOnImage_in_cm);
    CvPoint ToPos = GetToPosVector(yOnImage_in_cm);
    
    CvPoint ret = (absKinect + ToLeftBorder + ToFrame + ToPos);
    
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
    return cv::Point(-sin(view_angle) * In_cm(320, Horizontal), -cos(view_angle) * In_cm(320, Horizontal));
}

void KinectHelper::Raster(CvPoint& point, double GridX, double GridY)
{
    // std::cout << "Raster; Before: " << point;

    double _x = (point.x / GridX);
    double _y = (point.y / GridY);
    point.x = round(_x) * GridX;
    point.y = round(_y) * GridY;
    
    // std::cout << "; After: " << point << std::endl;
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

