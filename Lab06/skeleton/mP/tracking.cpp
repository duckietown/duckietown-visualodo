
#include "tracking.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

// include constants here:
const int edgeparam = 200; // input of function EdgeDetect (thresh = edgeparam)
int frcounter = 1; // counter for saving only every 30th image

// functions

void SvImage(IplImage* img, char* filename)
{
    int p[3] = {CV_IMWRITE_JPEG_QUALITY,95,0};
    cvSaveImage(filename, img, p);
}

IplImage* CrossTarget (IplImage* inImg, int x, int y, int size, int line_thickness)
{
    // ******* already completely implemented for you, you don't need to change anything here *******
    IplImage* outImg = cvCloneImage(inImg);

    // horizontal line
    CvPoint pt1 = cvPoint(x-size/2,y);
    CvPoint pt2 = cvPoint(x+size/2,y);
    cvLine(outImg, pt1, pt2, cvScalar(0, 200,0), line_thickness, 8, 0);
    // verical line
    pt1.x = x; pt1.y = y-size/2;
    pt2.x = x; pt2.y = y+size/2;
    cvLine(outImg, pt1, pt2, cvScalar(0,200,0), line_thickness, 8, 0);


    return outImg;
}

int ColorTrackingSetColors(IplImage* img, int* hmax, int* hmin, int* smax, int* smin, int* vmax, int* vmin)
 {

    IplImage* imgHSV = cvCreateImage(cvGetSize(img),8,3);
	  cvCvtColor(img,imgHSV,CV_RGB2BGR);
  	cvCreateTrackbar("Hmin", "Set", hmin, 255, 0);
  	cvCreateTrackbar("Hmax", "Set", hmax, 255, 0);
  	cvCreateTrackbar("Smin", "Set", smin, 255, 0);
  	cvCreateTrackbar("Smax", "Set", smax, 255, 0);
  	cvCreateTrackbar("Vmin", "Set", vmin, 255, 0);
  	cvCreateTrackbar("Vmax", "Set", vmax, 255, 0);

  	IplImage* imgThresh = cvCreateImage(cvGetSize(img),8,1);
  	IplImage* imgShow = cvCreateImage(cvGetSize(img),8,3);

    // Threshold the image using the function cvInRangeS() and save the mask in imgThresh (already done)
    cvInRangeS(imgHSV, cvScalar(*hmin,*smin,*vmin), cvScalar(*hmax,*smax,*vmax), imgThresh);
	  cvCopy(img,imgShow,imgThresh);
	  cvShowImage("Set", imgShow);
  	cvReleaseImage(&imgHSV);
  	cvReleaseImage(&imgShow);
  	cvReleaseImage(&imgThresh);

    return 0;
}


int ColorTracking (IplImage* img, int* positionX , int* positionY, CvScalar min, CvScalar max)
{

	IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);

	cvCvtColor(img, imgHSV, CV_RGB2BGR);

	IplImage* imgThresh = cvCreateImage(cvGetSize(img), 8, 1);

	cvInRangeS(imgHSV, min, max, imgThresh);

    // Create memory space for moments
    CvMoments *moments_y = (CvMoments*)malloc(sizeof(CvMoments));

    // Calculate moments
    cvMoments(imgThresh,moments_y,1);

    // Extract spatial moments and area
    double moment10_y = moments_y->m10;
    double moment01_y = moments_y->m01;
    double area_y = moments_y->m00;

  	*positionX = moment10_y/area_y;
  	*positionY = moment01_y/area_y;


  	IplImage* imgWithCross = cvCreateImage(cvGetSize(img), 8, 3);
  	imgWithCross = cvCloneImage(img);
  	imgWithCross = CrossTarget(imgWithCross, *positionX, *positionY, 25, 3);

	   cvShowImage("Set",imgWithCross);

    if (frcounter%30 == 0)
    {
        char filename[50];
        sprintf(filename,"Crossed_frame%d.jpg",frcounter);
        SvImage(imgWithCross,filename);
    }

    // Release created images and free (free()) memory (moments_y)
    free(moments_y);
  	cvReleaseImage(&imgHSV);
  	cvReleaseImage(&imgThresh);
    cvReleaseImage(&imgWithCross);

    return 0;
}


int EdgeDetect (IplImage* img, int thresh)
{   // ********** Prelab Q2 ***********

	IplImage* gray_img = cvCreateImage(cvGetSize(img),8,1);

	cvCvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

  IplImage* smooth_gray_img = cvCreateImage(cvGetSize(img),8,1);

	cvSmooth(gray_img, smooth_gray_img, CV_GAUSSIAN, 9, 0, 0, 0);

	IplImage* edge_detect = cvCreateImage(cvGetSize(img), 8, 1);

	cvCanny(smooth_gray_img, edge_detect, thresh, thresh*2, 3);
	// Create variables to store contours (already done)
	CvMemStorage *mem;
	mem = cvCreateMemStorage(0);
	CvSeq *contours = 0;

	cvFindContours(edge_detect, mem, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

	IplImage* edge_img = cvCreateImage(cvGetSize(img),8,3);

	CvScalar black = cvScalar(255,255,255);
	CvScalar red = cvScalar(0, 0, 0);


	while (contours != 0)
	{
		cvDrawContours(edge_img, contours, red, black, 0, 1, 8, cvPoint(0,0));

		contours = contours -> h_next;
	}

	cvShowImage("Set",edge_img);

    if (frcounter%30 == 0)
    {
        char filename[50];
        sprintf(filename,"Contour_frame%d.jpg",frcounter);
        SvImage(edge_img,filename);
    }


	cvReleaseImage(&edge_img);
	cvReleaseImage(&gray_img);
	cvReleaseImage(&smooth_gray_img);
	cvReleaseImage(&edge_detect);
  cvReleaseImage(&edge_img);

	return 0;
}

int main (){
	IplImage* frame = 0;

	CvCapture* capture = cvCaptureFromFile("capture.avi");

    if (!capture)
    {
        printf("Could not initialize capturing...\n");
        return -1;
    }

  	int hmin = 0;
  	int hmax = 255;
  	int smin = 0;
  	int smax = 255;
  	int vmin = 0;
  	int vmax = 200;
  	int positionX;
  	int positionY;

    cvDestroyWindow("Set");
    cvNamedWindow("Set", CV_WINDOW_AUTOSIZE);

    while (1)
    {
        // use cvQueryFrame() to grab a single video frame
        frame = cvQueryFrame(capture);
        if(!frame)
        {
				cvSetCaptureProperty(capture, CV_CAP_PROP_POS_AVI_RATIO, 0);
				continue;
		}

		ColorTrackingSetColors(frame, &hmax, &hmin, &smax, &smin, &vmax, &vmin);
		ColorTracking(frame, &positionX , &positionY, cvScalar(hmin, smin, vmin), cvScalar( hmax, smax, vmax));

		if ((char)cvWaitKey(10) == 'q') //press q to quit
		{
			break;
		}

    }


    // close the window
	cvDestroyWindow("Set");


	/**FILE *coordinates;
    coordinates = fopen("Coordinates.txt", "w+");
    cvSetCaptureProperty(capture,CV_CAP_PROP_POS_AVI_RATIO, 0);
    cvNamedWindow("Set", CV_WINDOW_AUTOSIZE);
        while (1)
    {
        // use cvQueryFrame() to grab a single video frame
        frame = cvQueryFrame(capture);

        if(!frame) cvSetCaptureProperty(capture, CV_CAP_PROP_POS_AVI_RATIO, 0);

        // use cvSetCaptureProperty() to start again from the beginning if necessary
        //cvSetCaptureProperty(capture,,);
        // cvWaitKey(10) can be used to receive user inputs
		ColorTrackingSetColors(frame, &hmax, &hmin, &smax, &smin, &vmax, &vmin);
		ColorTracking(frame, &positionX , &positionY, cvScalar(hmin, smin, vmin), cvScalar( hmax, smax, vmax));
		fprintf(coordinates, "%d, %d \n", (int)positionX, (int)positionY);
		cvWaitKey(1);
        //break;
    }
    fclose(coordinates);
	cvDestroyWindow("Set");
	**/

  	cvNamedWindow("Set",CV_WINDOW_AUTOSIZE);
  	cvSetCaptureProperty(capture,CV_CAP_PROP_POS_AVI_RATIO, 0);
    while (1)
    {
        frame = cvQueryFrame(capture);
        if(!frame){
				cvSetCaptureProperty(capture,CV_CAP_PROP_POS_AVI_RATIO, 0);
				continue;
		}
		frcounter += 1;

		EdgeDetect(frame,edgeparam); //works with /2

		if ((char)cvWaitKey(10) == 'q')
		{
			break;
		}
    }

    cvDestroyWindow("Set");
    cvReleaseImage(&frame);
    cvReleaseCapture(&capture);
	return 0;
}
