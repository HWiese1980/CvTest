#include "CvShapes.h"
#include <iostream>

void cvCross(CvArr* img, CvPoint pt, int size, CvScalar color, int thickness)
{
    CvPoint left = pt + cv::Point(-size, 0);
    CvPoint right = pt + cv::Point(size, 0);
    CvPoint up = pt + cv::Point(0, -size);
    CvPoint down = pt + cv::Point(0, size);
    
    cvLine(img, left, right, color, thickness);
    cvLine(img, up, down, color, thickness);
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

CvPoint operator*(CvPoint a, CvPoint b)
{
    CvPoint ret = cv::Point(a.x * b.x, a.y * b.y);
    return ret;
}

CvPoint operator/(CvPoint a, CvPoint b)
{
    return cv::Point(a.x / b.x, a.y / b.y);
}

CvPoint operator/(CvPoint a, double scalar)
{
    return cv::Point(a.x / scalar, a.y / scalar);
}

CvPoint FlipHorizontal(CvPoint a, int ImageHeight)
{
    double center_line = ImageHeight / 2.0;
    
    return cv::Point(a.x, -(center_line - a.y));
}

CvPoint operator+=(CvPoint a, CvPoint b)
{
    return cv::Point(a.x + b.x, a.y + b.y);
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


bool EllipseOK(CvBox2D box)
{
    bool bRet = false;
    double aspect = box.size.width / box.size.height;
    bRet = (0.1 < aspect & aspect < 0.8 );
    return bRet;
}

