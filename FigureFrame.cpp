/* 
 * File:   FigureFrame.cpp
 * Author: ros
 * 
 * Created on 21. April 2011, 16:15
 */

#include <opencv/cxcore.h>
#include <stdio.h>
#include "FigureFrame.h"

FigureFrame::FigureFrame()
{
}

FigureFrame::FigureFrame(int _x, int _y, int _w, int _h, int _i)
{
    this->x = _x;
    this->y = _y;
    this->width = _w;
    this->height = _h;
    this->index = _i;
}

FigureFrame::FigureFrame(const FigureFrame& orig)
{
    this->x = orig.x;
    this->y = orig.y;
    this->width = orig.width;
    this->height = orig.height;
    this->index = orig.index;
}

FigureFrame::~FigureFrame()
{
}

int FigureFrame::GetDistanceFromKinect(CvArr* img)
{
    cv::Mat _mat = cv::Mat((IplImage*)img);
    int dist = _mat.ptr(this->y)[this->x];
    return -1;
}

void FigureFrame::Draw(CvArr* img, CvScalar Color)
{
    if(this->height < 20 | this->width < 30 | this->height > 60 | this->width > 100) return;
    
    CvPoint center = this->GetCenterPoint();
    cvCircle(img, center, 5, Color);
    cvRectangle(img, cvPoint(this->x, this->y), cvPoint(this->x + this->width, this->y + this->height), Color);
}

CvPoint FigureFrame::GetCenterPoint()
{
    CvPoint _p = cvPoint(this->x + (this->width / 2), this->y + (this->height / 2));
    printf("Centepoint of %i: %i %i\n", this->index, _p.x, _p.y);
    return _p;
}
