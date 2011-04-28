#include "highgui.h"

#define CV_NO_BACKWARD_COMPATIBILITY

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

#include "FigureFrame.h"
#include "KinectHelper.h"

#include <cvblob.h>

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
IplImage* rectmask = 0;
IplImage* depth_rgb = 0;

IplImage* clamped_hsv = 0; // hsv mask married to clamped depth data

CvScalar hsv_min = cvScalar(20, 0, 0, 0);
CvScalar hsv_max = cvScalar(50, 255, 255, 0);
CvScalar depth_clamp_min = cvScalar(0, 0, 0, 0);
CvScalar depth_clamp_max = cvScalar(128, 128, 128, 128);

void FloodFill(cv::Mat& matrix, int x, int y, int64_t recursion_level = 0);

int currentBlobMaxX = -1, currentBlobMaxY = -1, currentBlobMinX = -1, currentBlobMinY = -1;

std::vector<FigureFrame> frames;

void Merge(CvArr* a, CvArr* b, CvArr* dst, CvArr* mask)
{
    cvCopy(a, dst);
    cvCopy(b, dst, mask);
}

void CombineTransparent(CvArr* a, CvArr* b, CvArr* c)
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
            
            ptrDst[2] = (sum != 0) ? rb : ra;
            ptrDst[1] = (sum != 0) ? gb : ga;
            ptrDst[0] = (sum != 0) ? bb : ba;
        }
    }
}

bool LiesWithinExistingFrame(int x, int y)
{
    // return false;
    
    
    std::vector<FigureFrame>::iterator it;
    for(it = frames.begin(); it != frames.end(); it++)
    {
        if(it->IsInside(x,y)) return true;
    }
    return false;
   
}

/*
std::map<int, std::vector<S_XY> > indexedPixels;
int PixelIndexed(S_XY pixel)
{
    for(std::map<int, std::vector<S_XY> >::iterator it = indexedPixels.begin(); it != indexedPixels.end(); it++)
    {
        for(std::vector<S_XY>::iterator vit = (*it).second.begin(); vit != (*it).second.end(); vit++)
        {
            if((*vit).x == pixel.x & (*vit).y == pixel.y) return (*it).first;
        }
    }
    return -1;
}

std::vector<SConflict> conflicts;
std::map<int, FigureFrame> objects;
void MarkBlobs(cv::Mat& matrix)
{
    objects.clear();
    int index = 0;
    cvZero(rectangles);
    for(int y = 0; y < matrix.rows - 1; y++)
    {
        uchar* rowPtr = matrix.ptr(y);
        for(int x = 0; x < matrix.cols; x++)
        {
            uchar* pixPtr = &rowPtr[x];
            if(*pixPtr >= 255)
            {
                int idxDown = PixelIndexed({x, y + 1});
                int idxLeft = PixelIndexed({x - 1, y});
                
                S_XY currentPixel = {x, y};
                if(idxDown == -1 & idxLeft == -1) // neues Objekt
                {
                    // printf("New Object found at %i:%i\n", x,y);
                    index++;
                    indexedPixels[index].push_back(currentPixel);
                }
                else if(idxLeft == -1 & idxDown >= 0) // aktuelles Objekt
                {
                    indexedPixels[index].push_back(currentPixel);
                }
                else if(idxDown == -1 & idxLeft >= 0) // anderes Objekt mit Index
                {
                    indexedPixels[idxLeft].push_back(currentPixel);
                }
                else if(idxDown >= 0 & idxLeft >= 0)
                {
                    if(idxDown == idxLeft) // selbes Objekt
                    {
                        indexedPixels[index].push_back(currentPixel);
                    }
                    else // unterschiedliche Objekte ==> Konflikt
                    {
                        SConflict conflict;
                        conflict.x = x; conflict.y = y;
                        conflict.index1 = idxLeft;
                        conflict.index2 = idxDown;
                        conflicts.push_back(conflict);
                        printf("Conflict found at [%i:%i]: %i vs. %i\n", x, y, idxLeft, idxDown);
                    }
                }
            }
        }
    }
}
**/

void MarkBlobs(cv::Mat& matrix)
{
    frames.clear();
    int rectNo = 0;
    cvZero(rectangles);
    for (int y = 0; y < matrix.rows; y++)
    {
        for (int x = 0; x < matrix.cols; x++)
        {
            FloodFill(matrix, x, y, 0);

            int rectX1 = currentBlobMinX, rectY1 = currentBlobMinY;
            int rectX2 = currentBlobMaxX, rectY2 = currentBlobMaxY;
            int rectWidth = rectX2 - rectX1, rectHeight = rectY2 - rectY1;
            
            if(rectX1 < 0 | rectX2 < 0 | rectY1 < 0 | rectY2 < 0) continue;
            
            FigureFrame frame(rectX1, rectY1, rectWidth, rectHeight, rectNo);
            
            try
            {
                frame.Matrix = matrix(cvRect(rectX1, rectY1, rectWidth, rectHeight));
            }
            catch(...)
            {
                printf("Fehler beim Kopieren der Matrix\n");
                printf("---------------------------------------\n");
                printf("Framedaten: [%i:%i - %i:%i] [%i:%i]\n", rectY1, rectY2, rectX1, rectX2, matrix.rows, matrix.cols);
                printf("---------------------------------------\n");
            }
            frame.depthImage = depthimg;
            frames.push_back(frame);
            
            currentBlobMaxX = currentBlobMaxY = currentBlobMinX = currentBlobMinY = -1;
            rectNo++;
        }
    }
}


void DrawFrames()
{
    std::vector<FigureFrame>::iterator it;
    for(it = frames.begin(); it != frames.end(); it++)
    {
        (*it).Draw(rectangles);
    }
}

void DrawMasks(CvArr* image)
{
    std::vector<FigureFrame>::iterator it;
    for(it = frames.begin(); it != frames.end(); it++)
    {
        (*it).DrawAsMask(image);
    }
}

// void RemoveSmallBlobs()

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

    if(x - 1 >= 0) FloodFill(matrix, x - 1, y, recursion_level + 1);
    if(x + 1 < 640) FloodFill(matrix, x + 1, y, recursion_level + 1);
    if(y + 1 < 480) FloodFill(matrix, x, y + 1, recursion_level + 1);
    if(y - 1 >= 0) FloodFill(matrix, x, y - 1, recursion_level + 1);
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

void cv_lower_cb(int value)
{
    hsv_min.val[0] = (double) value;
}

void cv_upper_cb(int value)
{
    hsv_max.val[0] = (double) value;
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

int lower_thresh = 20, upper_thresh = 60;
int minWidth, maxWidth, minHeight, maxHeight;
// int lower_d_thresh = 0, upper_d_thresh = 255;

void ConvertTo3D(CvArr* src, CvArr* dst, const cv::Mat& Q, bool HandleMissingValues = false)
{
    cv::Mat matA = cv::Mat((IplImage*)src);
    cv::Mat matB = cv::Mat((IplImage*)dst);
    cv::reprojectImageTo3D(matA, matB, Q, HandleMissingValues);
}


/*
 * thread for displaying the opencv content
 */
void *cv_threadfunc(void *ptr)
{
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.3, 0.3, 0, 1, CV_AA);
    
    using namespace cvb;
    // cvNamedWindow( FREENECTOPENCV_WINDOW_D, CV_WINDOW_AUTOSIZE );
    // cvNamedWindow( FREENECTOPENCV_WINDOW_N, CV_WINDOW_AUTOSIZE );
    cvNamedWindow("GUI", CV_WINDOW_AUTOSIZE);

    cvCreateTrackbar("Lower Threshold", "GUI", &lower_thresh, 255, cv_lower_cb);
    cvCreateTrackbar("Upper Threshold", "GUI", &upper_thresh, 255, cv_upper_cb);

    cvCreateTrackbar("Min Width", "GUI", &FigureFrame::mmWidth.Min, 255, cv_minWidth_cb);
    cvCreateTrackbar("Max Width", "GUI", &FigureFrame::mmWidth.Max, 255, cv_maxWidth_cb);

    cvCreateTrackbar("Min Height", "GUI", &FigureFrame::mmHeight.Min, 255, cv_minHeight_cb);
    cvCreateTrackbar("Max Height", "GUI", &FigureFrame::mmHeight.Max, 255, cv_maxHeight_cb);

    depthimg = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);
    rgbimg = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);
    bgrimg = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);
    hsvimg = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, 3);
    hsvmask = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);

    rectangles = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);
    depth_rgb = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);

    depthmask = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);
    clamped_hsv = cvCreateImage(cvSize(FREENECTOPENCV_DEPTH_WIDTH, FREENECTOPENCV_DEPTH_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_DEPTH_DEPTH);

    KinectHelper::depthData = depthimg;
    // use image polling
    printf("Running Thread...\n");
    // cvNamedWindow("blob-msk", 1);
    cvNamedWindow( "depth-img", 1); 
    // cvNamedWindow( "combined-depth-img", 1);
    cvNamedWindow("labeled-blobs", 1);
    cvNamedWindow("figures", 1);
    
    CvTracks tracks;
    CvBlobs blobs;

    while (1)
    {
        //lock mutex for rgb image
        //pthread_mutex_lock(&mutex_rgb);
        // show image to window
        cvCvtColor(rgbimg, hsvimg, CV_BGR2HSV);
        // cvNamedWindow( "hsv-img", 1); cvShowImage( "hsv-img", hsvimg);
        cvShowImage( "depth-img", depthimg);

        cvInRangeS(hsvimg, hsv_min, hsv_max, hsvmask);
        cvInRangeS(depthimg, depth_clamp_min, depth_clamp_max, depthmask);

//        cvNamedWindow("hsv-msk", 1);
//        cvShowImage("hsv-msk", hsvmask); // hsvmask->origin = 1;

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
        IplImage* combined_labelled_blob = cvCreateImage(cvGetSize(hsvmask), IPL_DEPTH_8U, 3);
        
        IplImage* labeledImage = cvCreateImage(cvGetSize(combined_result), IPL_DEPTH_LABEL, 1);
        
        IplImage* figurePositionsImage = cvCreateImage(cvSize(FREENECTOPENCV_RGB_WIDTH, FREENECTOPENCV_RGB_HEIGHT), IPL_DEPTH_8U, FREENECTOPENCV_RGB_DEPTH);
        
        int result = cvLabel(hsvmask, labeledImage, blobs);

        cvFilterByArea(blobs, 500, 1000000);

        cvRenderBlobs(labeledImage, blobs, hsvmask, combined_labelled_blob, CV_BLOB_RENDER_BOUNDING_BOX | CV_BLOB_RENDER_CENTROID);
        // cvUpdateTracks(blobs, tracks, 200., 5);
        // cvRenderTracks(tracks, hsvmask, combined_labelled_blob, CV_TRACK_RENDER_TO_STD | CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);
        std::vector<CvPoint> figurePositions;
        for(CvBlobs::iterator it = blobs.begin(); it != blobs.end(); it++)
        {
            try
            {
                double cent_x = (*it).second->centroid.x, cent_y = (*it).second->centroid.y;
                cv::Scalar dist = cvGet2D(depthimg, cent_y, cent_x); // Row/Col Reihenfolge
                // if(*dist.val > 200) continue;

                // Formel: DIST_CM = TAN(DIST_K / MAX_DIST + 0.5) * 33.825 + 5.7
                // double dist_cm = ((tan(dist[0] / 255 + 0.5) * 33.825 + 5.7)); // Distanz direkt von Kinect zum Objekt
                double dist_cm = KinectHelper::GetDirectDistanceInCM(dist[0]);
                
                double kinect_height = KinectHelper::GetKinectHeight(); // Kinect Höhe in cm
                // double dist_over_ground = sqrt(pow(dist_cm, 2) - pow(kinect_height, 2)); // Pythagoras
                double dist_over_ground = KinectHelper::GetDistanceOverGround(dist[0]);
                if(dist_over_ground > 250) continue; // Objekt zu weit weg.
                
                /*
                std::cout << "Label: " << (*it).second->label << std::endl;
                std::cout << "--------------------------------------" << std::endl;
                std::cout << "| Kinect Height: " << kinect_height << std::endl;
                std::cout << "| x: " << cent_x << "; y: " << cent_y << std::endl;
                std::cout << "| distance: " << dist[0] << ":" << dist[1] << ":" << dist[2] << std::endl;
                std::cout << "| distance in cm: " << dist_cm << "cm" << std::endl;
                std::cout << "| distance over ground: " << dist_over_ground << std::endl;
                std::cout << "--------------------------------------" << std::endl;
                 *  **/
                
                char distance_text[255], coord_text[255];
                sprintf(distance_text, "%0.2f cm", dist_over_ground);
                
                CvPoint pt = KinectHelper::GetAbsoluteCoordinates(dist[0], cent_x);
                sprintf(coord_text, "In Image: x:%0.2f y:%0.2f; Absolute: x:%i y:%i", cent_x, cent_y,  pt.x, pt.y);

                cvPutText(combined_labelled_blob, distance_text, cv::Point(cent_x + 5, cent_y + 5), &font, cv::Scalar(0, 128, 255, 0) );
                cvPutText(combined_labelled_blob, coord_text, cv::Point(cent_x + 5, cent_y + 15), &font, cv::Scalar(0, 255, 255, 0) );
                // figurePositions.push_back(pt);
                
                cv::Mat figImgMat = cv::Mat(figurePositionsImage);
                cv::circle(figImgMat, pt, 5, cv::Scalar(0,255,255, 0), CV_FILLED);
            }
            catch(...)
            {
                std::cerr << "Error fetching distance..." << std::endl;
            }

            CvContourPolygon *poly = cvConvertChainCodesToPolygon(&(*it).second->contour);
            CvContourPolygon *sPoly = cvSimplifyPolygon(poly, 5);
            CvContourPolygon *cPoly = cvPolygonContourConvexHull(sPoly);
            
            cvRenderContourChainCode(&(*it).second->contour, combined_labelled_blob);
            cvRenderContourPolygon(sPoly, combined_labelled_blob, CV_RGB(0, 255, 255));
            cvRenderContourPolygon(cPoly, combined_labelled_blob, CV_RGB(255, 255, 0));
        }
        
        CombineTransparent(rgbimg, combined_labelled_blob, combined_labelled_blob);
        
        cv::Mat mat = cv::Mat(4, 4, CV_8UC1);
        
        double fx = 594.21;
        double fy = 591.04;
        double a = -0.0030711;
        double b = 3.3309495;
        double cx = 339.5;
        double cy = 242.7;
        
        mat.ptr(0)[0] = 1/fx;
        mat.ptr(0)[1] = 0;
        mat.ptr(0)[2] = 0;
        mat.ptr(0)[3] = -cx/fx;
        mat.ptr(1)[0] = 0;
        mat.ptr(1)[1] = -1/fy;
        mat.ptr(1)[2] = 0;
        mat.ptr(1)[3] = cy/fy;
        mat.ptr(2)[0] = 0;
        mat.ptr(2)[1] = 0;
        mat.ptr(2)[2] = 0;
        mat.ptr(2)[3] = -1;
        mat.ptr(3)[0] = 0;
        mat.ptr(3)[1] = 0;
        mat.ptr(3)[2] = a;
        mat.ptr(3)[3] = b;
        
        //TODO: Rekalkulation des 2D RGB Bildes mit der Tiefeninformation in eine 3D Punktwolke
        //Stichwort: Distortion - Perspektivverzerrung sorgt für Abweichende Änderung der X-Koordinate
        //abhängig von der Entfernung. Dadurch so noch keine Brechnung der absoluten Koordinaten der Figuren möglich. 
        //Der Weg geht wohl über die Funktion reprojectImageTo3D. Diese wehrt sich aktuell noch mit Händen und Füßen. 
        // Fehlermeldung:
        /*
         * OpenCV Error: Assertion failed (CV_ARE_SIZES_EQ(src, dst) && (CV_MAT_TYPE(stype) == CV_8UC1 || CV_MAT_TYPE(stype) == CV_16SC1 || CV_MAT_TYPE(stype) == CV_32SC1 || CV_MAT_TYPE(stype) == CV_32FC1) && (CV_MAT_TYPE(dtype) == CV_16SC3 || CV_MAT_TYPE(dtype) == CV_32SC3 || CV_MAT_TYPE(dtype) == CV_32FC3)) in cvReprojectImageTo3D, file /build/buildd/opencv-2.1.0/src/cv/cvcalibration.cpp, line 2728
                        terminate called after throwing an instance of 'cv::Exception'
                          what():  /build/buildd/opencv-2.1.0/src/cv/cvcalibration.cpp:2728: error: (-215) CV_ARE_SIZES_EQ(src, dst) && (CV_MAT_TYPE(stype) == CV_8UC1 || CV_MAT_TYPE(stype) == CV_16SC1 || CV_MAT_TYPE(stype) == CV_32SC1 || CV_MAT_TYPE(stype) == CV_32FC1) && (CV_MAT_TYPE(dtype) == CV_16SC3 || CV_MAT_TYPE(dtype) == CV_32SC3 || CV_MAT_TYPE(dtype) == CV_32FC3) in function cvReprojectImageTo3D

                        Abgebrochen
         * */
        
        IplImage* undistorted = cvCreateImage(cvGetSize(rgbimg), IPL_DEPTH_32F, CV_32FC3);
        const cv::Mat rgbmat = cv::Mat(rgbimg);
        cv::Mat undistorted_mat = cv::Mat(undistorted);
        
        cv::reprojectImageTo3D(rgbmat, undistorted_mat, mat);
        cvShowImage("labeled-blobs", undistorted);
        
        cvCvtColor(&mat_test, depth_rgb, CV_GRAY2RGB);
        cvCvtColor(rectangles, rectmask, CV_RGB2GRAY);
        // Merge(depth_rgb, rectangles, combined_result, rectmask);
        
        
        
        // DrawMasks(fillmask);
        
        // cvCopy(depthimg, combined_depth_result, fillmask );
       
        // cvShowImage("blob-msk", combined_result); // hsvmask->origin = 1;
        // cvShowImage("combined-depth-img", combined_depth_result); // hsvmask->origin = 1;

        cvShowImage("figures", figurePositionsImage);
        cvReleaseImage(&figurePositionsImage);
        cvReleaseImage(&combined_result);
        cvReleaseImage(&labeledImage);
        cvReleaseImage(&combined_labelled_blob);
        cvReleaseImage(&combined_depth_result);
        cvReleaseImage(&rectmask);
        cvReleaseImage(&fillmask);

        //unlock mutex
        //pthread_mutex_unlock(&mutex_rgb);

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

    while (!die && freenect_process_events(f_ctx) >= 0);

    return 0;
}
