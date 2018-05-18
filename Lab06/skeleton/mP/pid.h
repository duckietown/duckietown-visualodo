#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "iostream"
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>    // Standard input/output definitions
#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>    // File control definitions
#include <errno.h>    // Error number definitions
#include <termios.h>  // POSIX terminal control definitions
#include <string.h>   // String function definitions

/*
Function serialport_init initializes a serial port for communication.
Inputs:
const char* serialport - path to input port e.g. "/dev/ttymxc3"
int baud - baud rate, speed of communication
*/
int serialport_init(const char* serialport, int baud);

/*
Function serialport_close closes serial port communication.
Inputs:
int fd - file descriptor
*/
int serialport_close(int fd);

/*
Function serialport_write writes to a serial port.
Inputs:
int fd - file descriptor
const char* str - command to be written to port
*/
int serialport_write(int fd, const char* str);

/*
Function serialport_read_until reads a serial port.
Inputs:
int fd - file descriptor
char* buf - buffer that we read into
char until - end condition for read function
int buf_max - maximum buffer length
int timeout - timeout in seconds
*/
int serialport_read_until(int fd, char* buf, char until, int buf_max, int timeout);

/*
Function CrossTarget overlays a cross at a certain target position, on a provided image
Inputs:
IplImage* inImg - source image
int x - x coordinate of the target
int y - y coordinate of the target
int size - half of the size of the cross that is drawn on the provided image (in pixels)
int line_thickness - thickness of the cross that is drawn on the provided image
Outputs:
/
Return:
IplImage* outImg - source image with a cross at the specified target position
*/
IplImage* CrossTarget (IplImage* inImg, int x, int y, int size, int line_thickness);

/*
Function ColorTracking detects the center of a specific color in the source image
Inputs:
IplImage* inImg - source image
int color - if 1, blue color is detected and if 2, yellow color is detected
Outputs:
int* positionX - x coordinate of the center of detected color (in pixels)
int* positionY - y coordinate of the center of detected color (in pixels)
Return:
0
*/
int ColorTracking (IplImage* img, int* positionX , int* positionY);

/*
Function constructCommand creates command to be sent over serial, for the SAM3x microcontroller
Inputs:
int u - number of steps
int motor - stage (motor) number that determines the axis
Outputs:
char* command - command as a string of 5 bytes
Return:
0
*/
int constructCommand (char* command, int u, int motor);

/*
Function MoveMotor moves the motor for a specified number of steps
Inputs:
int fd - serial port index
float distance - distance to be moved (in mm). It has to be <5 mm (999 steps - 3 byte limitation)
int motor - motor to be moved
Outputs:
/
Return:
1 - if switch is pressed
-1 - if there is a problem with communication or distance is too big
0 - otherwise

Notes:
- Check the pixel - mm relation
*/
int MoveMotor (int fd, float distance, int motor);
int GetCoordinates (CvCapture* capture, int*currentX, int* currentY);
/*
Function MoveMotorRectangular moves the stages in a rectangle trajectory with specified distance and
number of steps per each rectangle side. Additionally, if camera is used, coordinates in each steps
are written in a .txt file.
Inputs:
int fd - serial port index
float distance - length of each rectangle side (in mm). Each step has to be <5 mm (999 steps - 3 byte limitation)
int steps - number of steps that each rectangle side is divided in
int useVision - if 0, vision is not used, otherwise yes
int camIndex - system camera index (if vision is not used, it is not considered so it can be any number)
Outputs:
/
Return:
-1 - if there was an error
0 - otherwise

Notes:
- Adjust what color to track
- Adjust what motor goes first (1 or 2) and in which direction (+ or -)
*/
int MoveMotorRectangular (int fd, float distance, int steps, int useVision, int camIndex);

/*
Function PID uses a color tracker to control the movement of the stages. It can only move one stage
at a time.
Inputs:
int fd - serial port index
int targetPosition - coordinate of the target (in pixels)
CvCapture* capture - camera index
int motor - motor to be moved (1 or 2), determines the axis
Outputs:
/
Return:
-1 - if there was an error
0 - otherwise

Notes:
- Target in pixels or mm?
*/

int PID (int fd, int targetPositionX, int targetPositionY, CvCapture* capture);
