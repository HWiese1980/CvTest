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

    cvInitFont(&this->font, CV_FONT_VECTOR0, 1, 1, 0, 1);
}

FigureFrame::FigureFrame(const FigureFrame& orig)
{
}

FigureFrame::~FigureFrame()
{
}

void FigureFrame::Draw(CvArr* img, CvScalar Color)
{
    char buff[255];
    sprintf(buff, "%i", this->index);
    cvRectangle(img, cvPoint(this->x, this->y), cvPoint(this->x + this->width, this->y + this->height), Color);
    cvPutText(img, buff, cvPoint(this->x, this->y - 10), &this->font, Color);
}

CvPoint& FigureFrame::GetCenterPoint()
{
    CvPoint _p = cv::Point(this->x + (this->width / 2), this->y + (this->height / 2));
    return _p;
}
