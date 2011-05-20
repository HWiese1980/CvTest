/* 
 * File:   CvShapes.h
 * Author: ros
 *
 * Created on 17. Mai 2011, 18:14
 */

#ifndef CVSHAPES_H
#define	CVSHAPES_H

#include <opencv/cv.h>

#include "PlayingField.h"

void cvCross(CvArr* img, CvPoint pt, int size, CvScalar color, int thickness = 1);

CvPoint operator+(CvPoint a, CvPoint b);
CvPoint operator-(CvPoint a, CvPoint b);
CvPoint operator*(CvPoint a, CvPoint b);
CvPoint operator*(CvPoint a, double scalar);
CvPoint operator/(CvPoint a, double scalar);
CvPoint operator/(CvPoint a, CvPoint b);
CvPoint operator+=(CvPoint a, CvPoint b);

std::ostream& operator<<(std::ostream& s, const cv::Mat& mat);
std::ostream& operator<<(std::ostream& s, const CvPoint& point);


CvPoint FlipHorizontal(CvPoint a, int ImageHeight);


bool EllipseOK(CvBox2D box);
#endif	/* CVSHAPES_H */

