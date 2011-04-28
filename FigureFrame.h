/* 
 * File:   FigureFrame.h
 * Author: ros
 *
 * Created on 21. April 2011, 16:15
 */

#ifndef FIGUREFRAME_H
#define	FIGUREFRAME_H

#include <opencv/cv.h>

#include "MinMax.h"

class FigureFrame {
public:
    FigureFrame();
    FigureFrame(int _x, int _y, int _w, int _h, int _i);
    FigureFrame(const FigureFrame& orig);
    virtual ~FigureFrame();
    
public:
    CvPoint GetCenterPoint();
    int GetDistanceFromKinect();
    void Draw(CvArr* img, int Thickness = 1);
    void DrawAsMask(CvArr* img);
    
    static IplImage* depthImage; 
    
    static SMinMax mmWidth, mmHeight;
    
    bool IsInside(int x, int y);
    int Index() { return index; }
    
    CvPoint GetCenterOfGravity();

    bool IsValid();

public:
    cv::Mat Matrix;
    
private:
    int x, y, width, height, index;
    CvFont font;
    CvMoments moments;
};

#endif	/* FIGUREFRAME_H */