#ifndef tracking_h
#define tracking_h

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


// Draw a cross on image 
IplImage* CrossTarget (IplImage* inImg, int x, int y, int size, int line_thickness);
 /*
  * inImg: input image
  * x,y: position of center of cross
  * size: size of cross
  * line_thickness: thickness of cross lines
 */


// Set parameters for Color Tracking
int ColorTrackingSetColors (IplImage* img, 
int* hmax, int* hmin, int* smax, int* smin, int* vmax, int* vmin);
 /*
  * img: input image
  * hmax,..., vmin: pointers to integers to store values of minimal\
  *  and maximal parameters for thresholding
 */

// Color Tracker
int ColorTracking (IplImage* img, int* positionX , int* positionY,
 CvScalar min, CvScalar max);
 /*
  * img: input image
  * positionX,positionY pointers to integer to store position
  * min: minimal (hue,saturation,value,0)
  * min: maximal (hue,saturation,value,0)
 */


// Edge Detection
int EdgeDetect (IplImage* img, int thresh);
 /*
  * img: input image
  * thresh: threshold for canny edge
 */

// Save Image
void SvImage(IplImage* img, char* filename);
/*
 * img: input image
 * filename: name of image file where image will be stored
 */

#endif
