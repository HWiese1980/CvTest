#include "highgui.h"

#define CV_NO_BACKWARD_COMPATIBILITY

#include <stdio.h>

#include <libfreenect/libfreenect.h>

#include <libusb.h>
#include <pthread.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <time.h>
#include <stdlib.h>

#include <vector>

#include "FigureFrame.h"

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
IplImage* depthmask = 0;
IplImage* rectangles = 0;
IplImage* depth_rgb = 0;

IplImage* clamped_hsv = 0; // hsv mask married to clamped depth data

CvScalar hsv_min = cvScalar(20, 0, 0, 0);
CvScalar hsv_max = cvScalar(50, 255, 255, 0);
CvScalar depth_clamp_min = cvScalar(0, 0, 0, 0);
CvScalar depth_clamp_max = cvScalar(128, 128, 128, 128);

void FloodFill(cv::Mat& matrix, int x, int y, int64_t recursion_level = 0);

int currentBlobMaxX = -1, currentBlobMaxY = -1, currentBlobMinX = -1, currentBlobMinY = -1;

std::vector<FigureFrame> frames;

void Merge(IplImage* a, IplImage* b, IplImage* dst)
{
    cv::Mat mat_a = cv::Mat(a);
    cv::Mat mat_b = cv::Mat(b);
    cv::Mat mat_dst = cv::Mat(dst);

    for (int y = 0; y < mat_dst.rows; y++)
    {
        uchar* rowPtrA = mat_a.ptr(y);
        uchar* rowPtrB = mat_b.ptr(y);
        uchar* rowPtrDst = mat_dst.ptr(y);

        for (int x = 0; x < mat_dst.cols * 3; x++)
        {
            uchar* pixPtrA = &rowPtrA[x];
            uchar* pixPtrB = &rowPtrB[x];
            uchar* pixPtrDst = &rowPtrDst[x];

            if (*pixPtrB == 0)
            {
                *pixPtrDst = *pixPtrA;
            }
            else
            {
                *pixPtrDst = *pixPtrB;
            }
        }
    }
}

void MarkBlobs(cv::Mat& matrix)
{
    frames.clear();
    int rectNo = 0;
    cvZero(rectangles);
    for (int y = 0; y < matrix.rows; y++)
    {
        if (y >= currentBlobMinY && y <= currentBlobMaxY) continue;
        for (int x = 0; x < matrix.cols; x++)
        {
            if (x >= currentBlobMinX && x <= currentBlobMaxX) continue;

            FloodFill(matrix, x, y, 0);

            int rectX1 = currentBlobMinX, rectY1 = currentBlobMinY;
            int rectX2 = currentBlobMaxX, rectY2 = currentBlobMaxY;
            int rectWidth = rectX2 - rectX1, rectHeight = rectY2 - rectY1;

            if ((rectWidth > 30 & rectHeight > 20) & (rectWidth < 100 & rectHeight < 80))
            {
                // cvRectangle(rectangles, cvPoint(rectX1, rectY1), cvPoint(rectX2, rectY2), col);
                FigureFrame frame(rectX1, rectY1, rectX2-rectX1, rectY2-rectY1, rectNo);
                frames.push_back(frame);
            }
            else
            {
                cvRectangle(rectangles, cvPoint(rectX1, rectY1), cvPoint(rectX2, rectY2), cvScalar(0, 0, 0, 255), CV_FILLED);
            }

            currentBlobMaxX = currentBlobMaxY = currentBlobMinX = currentBlobMinY = -1;
        }
    }
}

void DrawFrames()
{
    cv::Scalar col = cvScalar(0, 0, 255, 0);
    std::vector<FigureFrame>::iterator it;
    for(it = frames.begin(); it != frames.end(); it++)
    {
        (*it).Draw(rectangles,col);
    }
}

void FloodFill(cv::Mat& matrix, int x, int y, int64_t recursion_level)
{
    uchar* rowPtr = matrix.ptr(y);
    uchar* pixPtr = &rowPtr[x];
    if (*pixPtr != 255) return;


    if (recursion_level == 0)
    {
        currentBlobMaxX = currentBlobMinX = x;
        currentBlobMaxY = currentBlobMinY = y;
    }
    else
    {
        if (x < currentBlobMinX) currentBlobMinX = x;
        if (x > currentBlobMaxX) currentBlobMaxX = x;
        if (y < currentBlobMinY) currentBlobMinY = y;
        if (y > currentBlobMaxY) currentBlobMaxY = y;
    }

    *pixPtr = 254;

    FloodFill(matrix, x - 1, y, recursion_level + 1);
    FloodFill(matrix, x + 1, y, recursion_level + 1);
    FloodFill(matrix, x, y + 1, recursion_level + 1);
    FloodFill(matrix, x, y - 1, recursion_level + 1);
    return;
}

// callback for depthimage, called by libfreenect

void depth_cb(freenect_device *dev, void *depth, uint32_t timestamp)
{
    cv::Mat depth8;
    cv::Mat mydepth = cv::Mat(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT, CV_16UC1, depth);

    mydepth.convertTo(depth8, CV_8UC1, 1.0 / 4.0);
    pthread_mutex_lock(&mutex_depth);
    memcpy(depthimg->imageData, depth8.data, 640 * 480);

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

int lower_thresh = 20, upper_thresh = 60;
int lower_d_thresh = 0, upper_d_thresh = 255;

void cv_lower_cb(int value)
{
    hsv_min.val[0] = (double) value;
}

void cv_upper_cb(int value)
{
    hsv_max.val[0] = (double) value;
}

void cv_lower_d_cb(int value)
{
    depth_clamp_min.val[0] = (double) value;
}

void cv_upper_d_cb(int value)
{
    depth_clamp_max.val[0] = (double) value;
}

/*
 * thread for displaying the opencv content
 */
void *cv_threadfunc(void *ptr)
{
    // cvNamedWindow( FREENECTOPENCV_WINDOW_D, CV_WINDOW_AUTOSIZE );
    // cvNamedWindow( FREENECTOPENCV_WINDOW_N, CV_WINDOW_AUTOSIZE );
    cvNamedWindow("GUI", CV_WINDOW_AUTOSIZE);

    cvCreateTrackbar("Lower Threshold", "GUI", &lower_thresh, 255, cv_lower_cb);
    cvCreateTrackbar("Upper Threshold", "GUI", &upper_thresh, 255, cv_upper_cb);

    cvCreateTrackbar("Lower Depth Threshold", "GUI", &lower_d_thresh, 255, cv_lower_d_cb);
    cvCreateTrackbar("Upper Depth Threshold", "GUI", &upper_d_thresh, 255, cv_upper_d_cb);

    depthimg = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);
    rgbimg = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);
    bgrimg = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);
    hsvimg = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, 3);
    hsvmask = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);

    rectangles = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);
    depth_rgb = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);

    depthmask = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);
    clamped_hsv = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);

    // use image polling
    printf("Running Thread...\n");
    while (1)
    {
        //lock mutex for depth image
        pthread_mutex_lock(&mutex_depth);
        // show image to window
        cvShowImage(FREENECTOPENCV_WINDOW_D, depthimg);
        //unlock mutex for depth image
        pthread_mutex_unlock(&mutex_depth);

        //lock mutex for rgb image
        pthread_mutex_lock(&mutex_rgb);
        // show image to window
        cvCvtColor(rgbimg, hsvimg, CV_BGR2HSV);
        // cvNamedWindow( "hsv-img", 1); cvShowImage( "hsv-img", hsvimg);
        // cvNamedWindow( "depth-img", 1); cvShowImage( "depth-img", depthmask);

        cvInRangeS(hsvimg, hsv_min, hsv_max, hsvmask);
        cvInRangeS(depthimg, depth_clamp_min, depth_clamp_max, depthmask);

        cvNamedWindow("hsv-msk", 1);
        cvShowImage("hsv-msk", hsvmask); // hsvmask->origin = 1;

        // cvShowImage(FREENECTOPENCV_WINDOW_N, rgbimg);

        // cvCopy(depthmask, clamped_hsv, hsvmask);

        // cvNamedWindow( "married", 1); cvShowImage( "married", clamped_hsv);

        cv::Mat depth_mat = cv::Mat(hsvmask);
        MarkBlobs(depth_mat);
        DrawFrames();

        IplImage mat_test = (IplImage) depth_mat;
        cvCvtColor(&mat_test, depth_rgb, CV_GRAY2RGB);

        Merge(depth_rgb, rectangles, depth_rgb);

        cvNamedWindow("blob-msk", 1);
        cvShowImage("blob-msk", depth_rgb); // hsvmask->origin = 1;

        //unlock mutex
        pthread_mutex_unlock(&mutex_rgb);

        // wait for quit key
        if (cvWaitKey(15) == 27)
            break;

    }
    pthread_exit(NULL);
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

    while (!die && freenect_process_events(f_ctx) >= 0);

    return 0;
}
