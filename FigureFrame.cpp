/* 
 * File:   FigureFrame.cpp
 * Author: ros
 * 
 * Created on 21. April 2011, 16:15
 */

#include <opencv/cxcore.h>
#include <stdio.h>
#include "FigureFrame.h"
#include <cmath>

IplImage* FigureFrame::depthImage = NULL;
SMinMax FigureFrame::mmWidth = {4, 2};
SMinMax FigureFrame::mmHeight = {10, 120};

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
    this->Matrix = orig.Matrix;
}

FigureFrame::~FigureFrame()
{
}

bool FigureFrame::IsValid()
{
    //if (this->height < mmHeight.Min | this->width < mmWidth.Min | this->height > mmHeight.Max | this->width > mmWidth.Max) return;
    //if(((double)this->width / (double)this->height) < 1.0) return;
    
    CvPoint grav = this->GetCenterOfGravity();
    CvPoint cent = this->GetCenterPoint();
    int deltaX = abs(grav.x - cent.x);
    int deltaY = abs(grav.y - cent.y);
    
    return(deltaX < 3 & deltaY < 3);
}

int FigureFrame::GetDistanceFromKinect()
{
    if (this->depthImage == NULL) return -1;

    cv::Mat _mat = cv::Mat((IplImage*) FigureFrame::depthImage);
    int dist = _mat.ptr(this->y)[this->x];
    return dist;
}

bool FigureFrame::IsInside(int _x, int _y)
{
    return (_x > this->x & _x < (this->x + this->width) & _y > this->y & _y < (this->y + this->height));
}

CvPoint FigureFrame::GetCenterOfGravity()
{
    if(this->Matrix.rows <= 0 | this->Matrix.cols <= 0)
        return cvPoint(0,0);
    
    
    IplImage* pImg = new IplImage(this->Matrix);
    
    cvMoments(pImg, &this->moments, 1);

    CvPoint pt;
    pt.x = (int)(this->moments.m10 / this->moments.m00) + this->x;
    pt.y = (int)(this->moments.m01 / this->moments.m00) + this->y;
    
    return pt;
}

void FigureFrame::Draw(CvArr* img, int Thickness)
{

    
    bool isValid = this->IsValid();
    
    cv::Scalar col = (isValid) ? cvScalar(0, 255, 0, 0) : cvScalar(0, 0, 255, 0);
    
    CvPoint center = this->GetCenterPoint();
    CvPoint gravity = this->GetCenterOfGravity();
    cvCircle(img, center, 2, col, CV_FILLED);
    if(isValid) cvCircle(img, gravity, 2, col, CV_FILLED);
    cvRectangle(img, cvPoint(this->x, this->y), cvPoint(this->x + this->width, this->y + this->height), col, Thickness);
    
}

void FigureFrame::DrawAsMask(CvArr* img)
{
    this->Draw(img, CV_FILLED);
}


CvPoint FigureFrame::GetCenterPoint()
{
    CvPoint _p = cvPoint(this->x + (this->width / 2), this->y + (this->height / 2));
    // printf("Centepoint of %i: %i %i\n", this->index, _p.x, _p.y);
    return _p;
}
