/* 
 * File:   FigureFrame.cpp
 * Author: ros
 * 
 * Created on 21. April 2011, 16:15
 */

#include <opencv/cxcore.h>
#include <stdio.h>
#include "FigureFrame.h"

SMinMax FigureFrame::mmHeight = {50, 150};
SMinMax FigureFrame::mmWidth = {10, 120};

FigureFrame::FigureFrame()
{
    this->depthImage = NULL;
}

FigureFrame::FigureFrame(int _x, int _y, int _w, int _h, int _i)
{
    this->depthImage = NULL;
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

int FigureFrame::GetDistanceFromKinect()
{
    if(this->depthImage == NULL) return -1;
    
    cv::Mat _mat = cv::Mat((IplImage*)this->depthImage);
    int dist = _mat.ptr(this->y)[this->x];
    return dist;
}

bool FigureFrame::IsInside(int _x, int _y)
{
    return (_x > this->x & _x < (this->x + this->width) & _y > this->y & _y < (this->y + this->height));
}

void FigureFrame::Draw(CvArr* img, CvScalar Color)
{
    if(this->height < mmHeight.Min | this->width < mmWidth.Min | this->height > mmHeight.Max | this->width > mmWidth.Max) return;
    
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
