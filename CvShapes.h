/* 
 * File:   CvShapes.h
 * Author: ros
 *
 * Created on 17. Mai 2011, 18:14
 */

#ifndef CVSHAPES_H
#define	CVSHAPES_H

#include <opencv/cv.h>

void cvCross(CvArr* img, CvPoint pt, CvScalar color, int size, int thickness = 1);
CvPoint operator+(CvPoint a, CvPoint b);
CvPoint operator-(CvPoint a, CvPoint b);
CvPoint operator*(CvPoint a, double scalar);



#endif	/* CVSHAPES_H */

