#include "CvShapes.h"

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

