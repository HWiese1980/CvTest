#include "highgui.h"

#define CV_NO_BACKWARD_COMPATIBILITY

#ifdef DEBUG
#define DBG_WHEREAMI std::cout << "[DEBUG] You are here: " << __FILE__ << ":" << __LINE__ << std::endl;
#else
#define DBG_WHEREAMI
#endif

#include <stdio.h>
#include <iostream>
#include <libfreenect/libfreenect.h>

#include <libusb.h>
#include <pthread.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <time.h>
#include <stdlib.h>

#include <vector>
#include <map>
#include <algorithm>

#include "FigureFrame.h"
#include "KinectHelper.h"
#include "PhidgetHelper.h"
#include "MotorHelper.h"
#include "Corners.h"
#include "CvShapes.h"

#include <cvblob.h>
#include <opencv/cv.hpp>
#include <opencv/cxcore.h>

pthread_mutex_t mutex_depth = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_rgb = PTHREAD_MUTEX_INITIALIZER;
pthread_t cv_thread;

#define FREENECTOPENCV_WINDOW_D "Depthimage"
#define FREENECTOPENCV_WINDOW_N "Normalimage"
#define FREENECTOPENCV_RGB_DEPTH 3
#define FREENECTOPENCV_DEPTH_DEPTH 1
#define FREENECTOPENCV_RGB_WIDTH 640
#define FREENECTOPENCV_RGB_HEIGHT 480
#define FREENECT_RGB_SIZE (FREENECTOPENCV_RGB_WIDTH * FREENECTOPENCV_RGB_HEIGHT * 3)
#define FREENECT_FORMAT_RGB 0
#define FREENECTOPENCV_DEPTH_WIDTH 640
#define FREENECTOPENCV_DEPTH_HEIGHT 480

IplImage* depthimg = 0;
IplImage* rgbimg = 0;
IplImage* bgrimg = 0;
IplImage* hsvimg = 0;
IplImage* hsvmask = 0;
IplImage* checker_mask = 0;
IplImage* depthmask = 0;
IplImage* rectangles = 0;
IplImage* rectmask = 0;
IplImage* depth_rgb = 0;

IplImage* clamped_hsv = 0; // hsv mask married to clamped depth data

CvScalar hsv_min = cvScalar(20, 70, 0, 0);
CvScalar hsv_max = cvScalar(50, 255, 255, 0);
CvScalar depth_clamp_min = cvScalar(0, 0, 0, 0);
CvScalar depth_clamp_max = cvScalar(128, 128, 128, 128);

// void FloodFill(cv::Mat& matrix, int x, int y, int64_t recursion_level = 0);

int currentBlobMaxX = -1, currentBlobMaxY = -1, currentBlobMinX = -1, currentBlobMinY = -1;
std::vector<FigureFrame> frames;

/*
void Merge(CvArr* a, CvArr* b, CvArr* dst, CvArr* mask)
{
    cvCopy(a, dst);
    cvCopy(b, dst, mask);
}
/**/

void StackTransparent(std::vector< std::pair<CvArr*, double> > images, CvArr* c)
{
    
    IplImage* dst = (IplImage*)c;
    
    cv::Mat matDst = cv::Mat(dst);
    
    for(std::vector< std::pair<CvArr*, double> >::iterator it = images.begin(); it != images.end(); it++)
    {
        IplImage* img = (IplImage*)it->first;
        double tp = it->second;
        cv::Mat mat = cv::Mat(img);
        
        for(int y = 0; y < matDst.rows; y++)
        {
            uchar* srcRow = mat.ptr(y);
            uchar* dstRow = matDst.ptr(y);
            for(int x = 0; x < matDst.cols * 3; x++)
            {
                uchar r = (&srcRow[x])[2];
                uchar g = (&srcRow[x])[1];
                uchar b = (&srcRow[x])[0];
                if(r + b + g <= 0) continue;

                uchar* newR = &(&dstRow[x])[2];
                uchar* newG = &(&dstRow[x])[1];
                uchar* newB = &(&dstRow[x])[0];
                
                *newR = r * tp + (*newR * (1.0 - tp));
                *newG = g * tp + (*newG * (1.0 - tp));
                *newB = b * tp + (*newB * (1.0 - tp));
            }
        }
    }
}

void CombineTransparent(CvArr* a, CvArr* b, CvArr* c, double Transparency = 1.0)
{
    IplImage* srcA = (IplImage*)a;
    IplImage* srcB = (IplImage*)b;
    IplImage* dst = (IplImage*)c;
    
    cv::Mat matA = cv::Mat(srcA);
    cv::Mat matB = cv::Mat(srcB);
    cv::Mat matDst = cv::Mat(dst);

    for(int y = 0; y < matA.rows; y++)
    {
        uchar* rowPtrA = matA.ptr(y);
        uchar* rowPtrB = matB.ptr(y);
        uchar* rowPtrDst = matDst.ptr(y);
        
        for(int x = 0; x < matA.cols * 3; x++)
        {
            uchar* ptrA = &rowPtrA[x];
            uchar* ptrB = &rowPtrB[x];
            uchar* ptrDst = &rowPtrDst[x];
            
            uchar ra = ptrA[2];
            uchar ga = ptrA[1];
            uchar ba = ptrA[0];

            uchar rb = ptrB[2];
            uchar gb = ptrB[1];
            uchar bb = ptrB[0];
            
            int sum = 0;
            
            sum = rb + gb + bb;
            
            
            
            ptrDst[2] = (sum != 0) ? rb  * Transparency + ra * (1.0 - Transparency): ra;
            ptrDst[1] = (sum != 0) ? gb  * Transparency + ga * (1.0 - Transparency): ga;
            ptrDst[0] = (sum != 0) ? bb  * Transparency + ba * (1.0 - Transparency): ba;
        }
    }
}

// callback for depthimage, called by libfreenect
void depth_cb(freenect_device *dev, void *depth, uint32_t timestamp)
{
    cv::Mat depth8;
    cv::Mat mydepth = cv::Mat(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT, CV_8UC1, depth);

    // mydepth.convertTo(depth8, CV_8UC1, 1.0 / 4.0);
    pthread_mutex_lock(&mutex_depth);
    memcpy(depthimg->imageData, mydepth.data, 640 * 480);

    // unlock mutex
    pthread_mutex_unlock(&mutex_depth);
}

// callback for rgbimage, called by libfreenect
void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
    // lock mutex for opencv rgb image
    pthread_mutex_lock(&mutex_rgb);
    memcpy(bgrimg->imageData, rgb, FREENECT_RGB_SIZE);
    cvConvertImage(bgrimg, rgbimg, CV_CVTIMG_SWAP_RB);
    // unlock mutex
    pthread_mutex_unlock(&mutex_rgb);
}

void cv_lower_cb(int value)
{
    hsv_min.val[0] = (double) value;
}

void cv_upper_cb(int value)
{
    hsv_max.val[0] = (double) value;
}

void cv_lower_sat_cb(int value)
{
    hsv_min.val[1] = (double) value;
}

void cv_upper_sat_cb(int value)
{
    hsv_max.val[1] = (double) value;
}

void cv_lower_val_cb(int value)
{
    hsv_min.val[2] = (double) value;
}

void cv_upper_val_cb(int value)
{
    hsv_max.val[2] = (double) value;
}

void cv_minWidth_cb(int value)
{
    FigureFrame::mmWidth.Min = value;
}

void cv_maxWidth_cb(int value)
{
    FigureFrame::mmWidth.Max = value;
}

void cv_minHeight_cb(int value)
{
    FigureFrame::mmHeight.Min = value;
}

void cv_maxHeight_cb(int value)
{
    FigureFrame::mmHeight.Max = value;
}

void cv_KinAngle_cb(int value)
{
    KinectHelper::view_angle = DEG2RAD(value);
}

void cv_KinX_cb(int value)
{
    KinectHelper::absolute_x = value;
}

void cv_KinY_cb(int value)
{
    KinectHelper::absolute_y = value;
}

void cv_KinScale_cb(int value)
{
    KinectHelper::scale = (value / 100.0);
    std::cout << "Setting Kinect Scale to " << KinectHelper::scale << std::endl;
}

int lower_thresh = hsv_min.val[0], upper_thresh = hsv_max.val[0];
int lower_sat_thresh = hsv_min.val[1], upper_sat_thresh = hsv_max.val[1];
int lower_val_thresh = hsv_min.val[2], upper_val_thresh = hsv_max.val[2];

int _kin_angle = 0, _kin_x = 0, _kin_y = 0;

int minWidth, maxWidth, minHeight, maxHeight;
// int lower_d_thresh = 0, upper_d_thresh = 255;

void ConvertTo3D(CvArr* src, CvArr* dst, const cv::Mat& Q, bool HandleMissingValues = false)
{
    cv::Mat matA = cv::Mat((IplImage*)src);
    cv::Mat matB = cv::Mat((IplImage*)dst);
    cv::reprojectImageTo3D(matA, matB, Q, HandleMissingValues);
}

// std::map<int, CvPoint> calibPoints;
void onCalibrationWindowMouseClick(int evt, int x, int y, int flags, void* param)
{
    static int cpt_index = 0;
    if(evt==CV_EVENT_LBUTTONUP)
    {
        if(KinectHelper::pointsUsedForCalibration.size() < 4)
        {
            CvPoint pnt = cv::Point(x, y);
            KinectHelper::pointsUsedForCalibration.push_back(pnt);
            std::cout << "Got Mouse click at " << pnt << std::endl;            
        }
    }
    else if(evt==CV_EVENT_RBUTTONUP)
    {
        std::cout << "Removing all calibration points" << std::endl;
        KinectHelper::bVPCalibrated = false;
        KinectHelper::bAandVCalibrated = false;
        KinectHelper::pointsUsedForCalibration.clear();
    }
}

void ClearBlobAt(IplImage* img, CvPoint Pt)
{
    CvScalar scal = cvGet2D(img, Pt.y, Pt.x);
    if(scal.val[0] > 0) cvFloodFill(img, Pt, CV_RGB(1,0,0));
}

void DrawCheckerBord(CvArr* img)
{
    double field_width_in_px = KinectHelper::In_px(35, Horizontal);
    double field_height_in_px = KinectHelper::In_px(35, Vertical);

    std::cout << "Field - Height in PX: " <<field_height_in_px << "; Width in PX: " << field_width_in_px << std::endl;
    
    int colidx = 1;
    cv::Scalar col[2] = { CV_RGB(255, 0, 0), CV_RGB(0, 0, 255) };
    
    for(int j=0; j < 6; j++)
    {
        for(int i = 0; i < 6; i++)
        {
            cvRectangle(img, cv::Point(i * field_width_in_px, j * field_height_in_px) * KinectHelper::scale, cv::Point((i + 1) * field_width_in_px, (j + 1) * field_height_in_px) * KinectHelper::scale, col[colidx], CV_FILLED);
            colidx = (colidx == 0) ? 1 : 0;
        }
        colidx = (colidx == 0) ? 1 : 0;
    }
}

bool bSkipClearBorder = false;
void ClearBorder(IplImage* img)
{
    if(bSkipClearBorder) return;
    // upper and lower border
    for(int idx = 0; idx < img->width; idx++)
    {
        CvPoint A_Pt = cv::Point(idx, 0);
        CvPoint B_Pt = cv::Point(idx, img->height - 1);
        ClearBlobAt(img, A_Pt);
        ClearBlobAt(img, B_Pt);
    }
    
    // left and right border
    for(int idx = 0; idx < img->height; idx++)
    {
        CvPoint A_Pt = cv::Point(0, idx);
        CvPoint B_Pt = cv::Point(img->width - 1, idx);
        ClearBlobAt(img, A_Pt);
        ClearBlobAt(img, B_Pt);
    }
}

volatile bool bThreadRunning = false;

/*
 * thread for displaying the opencv content
 */
void *cv_threadfunc(void *ptr)
{
    bThreadRunning = true;

    PhidgetHelper::Initialize();
    MotorHelper::Initialize();

    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.3, 0.3, 0, 1, CV_AA);
    
    using namespace cvb;
    // cvNamedWindow( FREENECTOPENCV_WINDOW_D, CV_WINDOW_AUTOSIZE );
    // cvNamedWindow( FREENECTOPENCV_WINDOW_N, CV_WINDOW_AUTOSIZE );
    cvNamedWindow("GUI", CV_WINDOW_AUTOSIZE);

    cvCreateTrackbar("Lower Threshold", "GUI", &lower_thresh, 255, cv_lower_cb);
    cvCreateTrackbar("Upper Threshold", "GUI", &upper_thresh, 255, cv_upper_cb);
    cvCreateTrackbar("Lower Threshold Sat", "GUI", &lower_sat_thresh, 255, cv_lower_sat_cb);
    cvCreateTrackbar("Upper Threshold Sat", "GUI", &upper_sat_thresh, 255, cv_upper_sat_cb);
    cvCreateTrackbar("Lower Threshold Val", "GUI", &lower_val_thresh, 255, cv_lower_val_cb);
    cvCreateTrackbar("Upper Threshold Val", "GUI", &upper_val_thresh, 255, cv_upper_val_cb);

    cvCreateTrackbar("Erode", "GUI", &FigureFrame::mmWidth.Min, 255, cv_minWidth_cb);
    cvCreateTrackbar("Dilate", "GUI", &FigureFrame::mmWidth.Max, 255, cv_maxWidth_cb);

    cvCreateTrackbar("Min Height", "GUI", &FigureFrame::mmHeight.Min, 255, cv_minHeight_cb);
    cvCreateTrackbar("Max Height", "GUI", &FigureFrame::mmHeight.Max, 255, cv_maxHeight_cb);

    cvCreateTrackbar("Kinect Angle", "GUI", &_kin_angle, 360, cv_KinAngle_cb);
    cvCreateTrackbar("Kinect X", "GUI", &_kin_x, 10000, cv_KinX_cb);
    cvCreateTrackbar("Kinect Y", "GUI", &_kin_y, 10000, cv_KinY_cb);
    
    int scale = (KinectHelper::scale * 100);
    cvCreateTrackbar("Kinect Scale", "GUI", &scale, 100, cv_KinScale_cb);

    
    depthimg = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_16U, FREENECTOPENCV_DEPTH_DEPTH);
    
    rgbimg = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);
    bgrimg = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);
    hsvimg = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, 3);
    hsvmask = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);

    checker_mask = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);
    
    rectangles = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);
    depth_rgb = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);

    depthmask = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);
    clamped_hsv = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);

    KinectHelper::depthData = depthimg;
    // use image polling
    printf("Running Thread...\n");
    // cvNamedWindow("blob-msk", 1);
    cvNamedWindow("hsv-msk", 1);
    // cvNamedWindow( "depth-img", 1); 
    cvNamedWindow( "combined-depth-img", 1);
    cvSetMouseCallback("combined-depth-img", onCalibrationWindowMouseClick);
    
    // cvNamedWindow("labeled-blobs", 1);
    cvNamedWindow("figures", 1);
    // cvNamedWindow("projection", 1);
    // cvNamedWindow("undistorted", 1);
    
    CvTracks tracks;
    CvBlobs blobs;

    int loopCounter = 0; 
    // std::vector<cv::Point2f> corners;
    // std::vector<cv::Point2f> corners;

    std::map<CornerPosition, CvPoint> calibration_corners;
    bool run = true;
    while (run)
    {
        loopCounter++;
        //lock mutex for rgb image
        //pthread_mutex_lock(&mutex_rgb);
        // show image to window
        cvCvtColor(rgbimg, hsvimg, CV_BGR2HSV);
        // cvNamedWindow( "hsv-img", 1); cvShowImage( "hsv-img", hsvimg);
        // cvShowImage( "depth-img", depthimg);

        cvInRangeS(hsvimg, hsv_min, hsv_max, hsvmask);
        cvInRangeS(depthimg, depth_clamp_min, depth_clamp_max, depthmask);
        cvInRangeS(hsvimg, hsv_min, hsv_max, checker_mask);

        cvErode(hsvmask, hsvmask, NULL, 4);
        cvDilate(hsvmask, hsvmask, NULL, 1);
        ClearBorder(hsvmask);
        cvShowImage("hsv-msk", hsvmask); // hsvmask->origin = 1;

        // cvShowImage(FREENECTOPENCV_WINDOW_N, rgbimg);

        // cvCopy(depthmask, clamped_hsv, hsvmask);

        // cvNamedWindow( "married", 1); cvShowImage( "married", clamped_hsv);

        // cvDilate(hsvmask, hsvmask, NULL, 1);
        
        cv::Mat depth_mat = cv::Mat(hsvmask);
        // MarkBlobs(depth_mat);
        // DrawFrames();

        IplImage mat_test = (IplImage) depth_mat;
        IplImage* rectmask = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);
        IplImage* fillmask = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);
        IplImage* combined_result = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, 3);
        IplImage* combined_depth_result = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);
        
        
        /// Images for output of location data (stacked)
        IplImage* combined_labelled_blob_calibration = cvCreateImage(cvGetSize(hsvmask), IPL_DEPTH_8U, 3);
        IplImage* combined_labelled_blob_detection = cvCreateImage(cvGetSize(hsvmask), IPL_DEPTH_8U, 3);
        IplImage* combined_labelled_blob_data = cvCreateImage(cvGetSize(hsvmask), IPL_DEPTH_8U, 3);
        IplImage* combined_labelled_blob_data_bg = cvCreateImage(cvGetSize(hsvmask), IPL_DEPTH_8U, 3);
        IplImage* combined_labelled_blob = cvCreateImage(cvGetSize(hsvmask), IPL_DEPTH_8U, 3);
        IplImage* projected_image = cvCreateImage(cvGetSize(hsvmask), IPL_DEPTH_8U, 3);
        
        IplImage* labeledImage = cvCreateImage(cvGetSize(combined_result), IPL_DEPTH_LABEL, 1);
        // IplImage* rgb_equalized = cvCreateImage(cvGetSize(rgbimg), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);
        
        IplImage* figurePositionsImage = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);
        int result = cvLabel(hsvmask, labeledImage, blobs);
        
        // cvEqualizeHist(rgbimg, rgb_equalized);

        cvFilterByArea(blobs, 500, 1000000);

        
        if(KinectHelper::pointsUsedForCalibration.size() == 4) DrawCheckerBord(figurePositionsImage);

        int blobCnt = 0;
        for(CvBlobs::iterator it = blobs.begin(); it != blobs.end(); it++)
        {
            const CvBlob& blb = *(*it).second;
            blobCnt++;
            try
            {
                double cent_x = blb.centroid.x, cent_y = blb.centroid.y;
                
                // cv::Scalar dist = cvGet2D(depthimg, cent_y, cent_x); // Row/Col Reihenfolge
                // if(*dist.val > 200) continue;

                // Formel: DIST_CM = TAN(DIST_K / MAX_DIST + 0.5) * 33.825 + 5.7

                // double dist_over_ground = KinectHelper::GetDistanceOverGround(dist[0]);
                // if(dist_over_ground > 250 || dist_over_ground < 20) continue; // Objekt zu weit weg oder zu dicht dran
                
                // Infofenster
                cvRectangle(combined_labelled_blob_data_bg, cv::Point(cent_x + 5, cent_y + 5), cv::Point(cent_x + 130, cent_y + 60), CV_RGB(5,5, 5), 2);
                cvRectangle(combined_labelled_blob_data_bg, cv::Point(cent_x + 5, cent_y + 5), cv::Point(cent_x + 130, cent_y + 60), CV_RGB(60,60,60), CV_FILLED);

                CvPoint blob_pt = cv::Point(cent_x, cent_y);
                
                if(KinectHelper::pointsUsedForCalibration.size() >= 4)
                {
                    CvPoint pt;
                    CvPoint base_pt = KinectHelper::GetAbsoluteX(blob_pt);
                    
                    // cvLine(combined_labelled_blob_detection, KinectHelper::VanishingPoint, base_pt, CV_RGB(0,255,0), 2);

                    pt = KinectHelper::GetAbsoluteCoordinates(base_pt.y, base_pt.x);
                    CvPoint absXPt = KinectHelper::GetAbsoluteX(blob_pt);
                    
                    char coord_text[255];
                    sprintf(coord_text, "Absolute: x:%i y:%i", pt.x, pt.y);
                    cvPutText(combined_labelled_blob_data, coord_text, cv::Point(cent_x + 10, cent_y + 30), &font, cv::Scalar(0, 255, 255, 0) );

                    sprintf(coord_text, "Absol. X: x:%i:y:%i", absXPt.x, absXPt.y);
                    cvPutText(combined_labelled_blob_data, coord_text, cv::Point(cent_x + 10, cent_y + 45), &font, cv::Scalar(0, 255, 255, 0) );

                    /*
                    CvPoint kin_pt = cv::Point(320, 240);
                    CvPoint kinEdge_pt = kin_pt + (KinectHelper::GetLeftFrameEdgeVector() * 0.1);
                    CvPoint kinOnFr_pt = kinEdge_pt + (KinectHelper::GetOnImageVector(base_pt.x) * 0.1);
                    CvPoint kinPos__pt = kinOnFr_pt + (KinectHelper::GetToPosVector(dist[0] * KinectHelper::distance_coefficient) * 0.1);
                    
                    
                    cvCircle(figurePositionsImage, kin_pt, 4, CV_RGB(255, 128, 0), CV_FILLED);
                    cvCircle(figurePositionsImage, kin_pt, 4, CV_RGB(255, 192, 0), 2);
                    
                    cvLine(figurePositionsImage, kin_pt, kinEdge_pt, CV_RGB(255, 128, 0));
                    cvCircle(figurePositionsImage, kinOnFr_pt, 3, CV_RGB(128, 255, 0), CV_FILLED);
                    cvCircle(figurePositionsImage, kinPos__pt, 3, CV_RGB(255, 255, 0), CV_FILLED);
                    */
                    
                    KinectHelper::Raster(pt, KinectHelper::In_px(17.5 /* KinectHelper::scale*/, Horizontal), KinectHelper::In_px(17.5 /* KinectHelper::scale*/, Vertical));
                    
                    cvCircle(figurePositionsImage, pt * KinectHelper::scale, KinectHelper::In_px(10 * KinectHelper::scale, Horizontal), CV_RGB(255, 255, 0), 3);
                    cvCross(figurePositionsImage, pt * KinectHelper::scale, KinectHelper::In_px(11 * KinectHelper::scale, Horizontal), CV_RGB(255, 255, 0), 2);
                    
                }


//                char distance_text[255];
//                sprintf(distance_text, "%i: %0.2f cm", blobCnt, dist_over_ground);
//                cvPutText(combined_labelled_blob_data, distance_text, cv::Point(cent_x + 10, cent_y + 15), &font, cv::Scalar(0, 128, 255, 0) );
                
                cvCross(combined_labelled_blob_detection, blob_pt, 5, CV_RGB(255, 0, 0));
                // cvRectangle(combined_labelled_blob_detection, cv::Point(blb.minx, blb.miny), cv::Point(blb.maxx, blb.maxy), CV_RGB(0, 255, 0), 2);
                
                if(KinectHelper::pointsUsedForCalibration.size() >= 4)
                    KinectHelper::DrawProjectedPoint(combined_labelled_blob_detection, blob_pt);

                // figurePositions.push_back(pt);
                
                // cv::Mat figImgMat = cv::Mat(figurePositionsImage);

            }
            catch(...)
            {
                std::cerr << "Error fetching distance..." << std::endl;
            }
            
//            CvContourPolygon *poly = cvConvertChainCodesToPolygon(&(*it).second->contour);
//            CvContourPolygon *sPoly = cvSimplifyPolygon(poly, 5);
//            CvContourPolygon *cPoly = cvPolygonContourConvexHull(sPoly);
            
            // cvRenderContourChainCode(&(*it).second->contour, combined_labelled_blob);
            // cvRenderContourPolygon(sPoly, combined_labelled_blob, CV_RGB(0, 255, 255));
            // cvRenderContourPolygon(cPoly, combined_labelled_blob, CV_RGB(255, 255, 0));
            
        }
        
        cvLine(combined_labelled_blob_detection, cv::Point(320, 10), cv::Point(320, 470), cv::Scalar(255, 255, 255, 0));
        cvLine(combined_labelled_blob_detection, cv::Point(10, 240), cv::Point(630, 240), cv::Scalar(255, 255, 255, 0));
        cvLine(combined_labelled_blob_detection, cv::Point(20, 120), cv::Point(620, 120), cv::Scalar(255, 255, 255, 0));

        KinectHelper::CalibrateAnglesAndViewport();
        KinectHelper::CalibrateVanishingPoint();
        KinectHelper::SetupProjectionVector();
        KinectHelper::DrawCalibrationData(combined_labelled_blob_calibration);

        //KinectHelper::ProjectImage(rgbimg, projected_image);
        //cvShowImage("projection", projected_image);
        
        std::vector< std::pair< CvArr*, double > > imageStack;
        
        imageStack.push_back({ rgbimg, 1.0 });
        imageStack.push_back({ combined_labelled_blob_data_bg, 0.2 });
        imageStack.push_back({ combined_labelled_blob_data, 1.0 });
        imageStack.push_back({ combined_labelled_blob_detection, 1.0 });
        imageStack.push_back({ combined_labelled_blob_calibration, 1.0 });
        
        StackTransparent(imageStack, combined_labelled_blob);
        
        
        // CombineTransparent(rgbimg, combined_labelled_blob, combined_labelled_blob);
        // CombineTransparent(combined_labelled_blob, combined_labelled_blob_data_bg, combined_labelled_blob, 0.2);
        // CombineTransparent(combined_labelled_blob, combined_labelled_blob_data, combined_labelled_blob);
        
        cvCvtColor(&mat_test, depth_rgb, CV_GRAY2RGB);
        cvCvtColor(rectangles, rectmask, CV_RGB2GRAY);
       
        cvShowImage("combined-depth-img", combined_labelled_blob); // hsvmask->origin = 1;
        // cvShowImage("undistorted", checker_mask);
        cvShowImage("figures", figurePositionsImage);
        cvReleaseImage(&figurePositionsImage);
        cvReleaseImage(&combined_result);
        cvReleaseImage(&labeledImage);
        cvReleaseImage(&combined_labelled_blob);
        cvReleaseImage(&combined_labelled_blob_data);
        cvReleaseImage(&combined_labelled_blob_data_bg);
        cvReleaseImage(&combined_labelled_blob_calibration);
        cvReleaseImage(&combined_labelled_blob_detection);
        cvReleaseImage(&combined_depth_result);
        cvReleaseImage(&projected_image);
        cvReleaseImage(&rectmask);
        cvReleaseImage(&fillmask);

        //unlock mutex
        //pthread_mutex_unlock(&mutex_rgb);

        // wait for quit key
        char key = cvWaitKey(15);
        
        switch(key)
        {
            default: std::cout << "Unknown key: " << key << std::endl;
            case -1: break;
            case 'R': 
            {
                if(MotorHelper::State == GoingBackward) 
                    MotorHelper::Stop();
                else
                    MotorHelper::GoForward(0xFF);
                break;
            }
            case 'T': 
            {
                if(MotorHelper::State == GoingForward) 
                    MotorHelper::Stop();
                else
                    MotorHelper::GoBackward(0xFF);
                break;
            }
            case 'Q': 
            {
                if(MotorHelper::State == TurningRight)
                    MotorHelper::Stop();
                else
                    MotorHelper::TurnLeft(0xFF);
                break;
            }
            case 'S': 
            {
                if(MotorHelper::State == TurningLeft)
                    MotorHelper::Stop();
                else
                    MotorHelper::TurnRight(0xFF);
                break;
            }
            case 27: 
            {
                run = false;
                break;
            }
        }
    }
    
    std::cout << "STOPPING!" << std::endl;
    
    pthread_exit(NULL);
    bThreadRunning = false;
}

int main(int argc, char** argv)
{
    srand(time(NULL));
    
    freenect_context* f_ctx;
    freenect_device* f_dev;

    int die = 0;
    int res = 0;

    if (freenect_init(&f_ctx, NULL) < 0)
    {
        printf("Freenect init failed\n");
        return 1;
    }

    if (freenect_open_device(f_ctx, &f_dev, 0) < 0)
    {
        printf("Opening Freenect device failed\n");
        return 2;
    }
    
    KinectHelper::dev = f_dev;

    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_video_callback(f_dev, rgb_cb);
    freenect_set_video_format(f_dev, FREENECT_VIDEO_RGB);

    res = pthread_create(&cv_thread, NULL, cv_threadfunc, (void*) depthimg);

    if (res)
    {
        printf("Creating PThread failed\n");
        return 3;
    }

    freenect_start_depth(f_dev);
    freenect_start_video(f_dev);

    while (bThreadRunning && !die && freenect_process_events(f_ctx) >= 0);

    return 0;
}
