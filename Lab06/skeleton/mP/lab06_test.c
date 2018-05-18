/*
Lab 05: Closed-loop control

Objectives:
- Implement a function to move the stage in certain direction
- Implement a function to move the stages in a square trajectory
- Use vision feedback to compare the obtained trajectory with theoretical one
- Implement a PID for position control

Tasks:
- Initialize the serial communication
- For moving the stage:
	- specify distance and motor to be moved
	- convert distance to steps
	- construct proper command (based on motor, direction and number of steps)
	- send it to SAM3x microcontroller
	- wait for the response
- For moving the stage in a square trajectory:
	- specify the length of each side of the square and number of steps per side
	- if vision is used, initialize the camera and create a .txt file
	- after moving each step save the coordinates
- For PID control:
	- initialize the camera
	- specify axis (motor) and target position
	- do the control:
		- get image from the camera
		- determine the initial position for specified axis
		- determine the initial time
		- determine the initial error
		- repeat until error is smaller than the defined tolerance:
			- update previous time
			- get current time
			- calculate the time interval
			- calculate the P, I and D
			- calculate the controller output u
			- if u is too small, set it to 0 (to avoid too many iterations when close to the target)
			- construct a proper command and send it to the SAM3x microcontroller
			- wait for the response
			- get new image from the camera
			- determine the new position
			- update the time
			- update the error
    - after moving each step save the coordinates in a .txt file



Reference:
- IRM 2016
*/

// add your includes here
#include <math.h>
#include <stdio.h>
#include "pid.h"

int main ()
{

    // YOUR CODE BEGINS HERE:

    // initialize your parameters
    int choice; //parameter to pass user choice
    int fd; //the serial port

  	int positionX;
  	int positionY;
    int deltaPixelX = 0;



	// Initialize the serial port
  fd = serialport_init("/dev/ttymxc3", 115200);
	// prompt user to select a certain task
  printf("Please choose a task to be performed: \n");
  printf("4: calibrate\n");
  scanf("%d", &choice);


	///////// Open Loop Motion /////////
    // --------------------------------------------------------------------------
    ////////////////////////////////////
    // Task 4: move the stage in a direction with a specified distance and track the position to calibrate the camera.
    if (choice == 4){

      CvCapture*capture = cvCaptureFromCAM(3);
      cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,840);
      cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,840);
      if (!capture)
      {
      printf("Could not initialize capturing...\n");
      return -1;
      }
      usleep(10);

      int positionXinitial;
    int positionYinitial;
      int moved;
      float pixel2mm = 0;
      int dir = 1;
      for(int i = 0; i<3;i++){
        GetCoordinates(capture, &positionX, &positionY);
        positionXinitial = positionX;
        positionYinitial = positionY;

        
          //dir *= -1;

          moved = MoveMotor(fd, dir, 1);
          usleep(20);
          printf("Moved: %i\n\n", moved);

          if (moved == 0){
            GetCoordinates(capture, &positionX , &positionY);
            pixel2mm += sqrt( (pow((positionX-positionXinitial),2)) + (pow((positionY-positionYinitial),2)));
  			break;
          }
          
          if (moved == 1) dir *=-1;
        }
      

      pixel2mm = abs(pixel2mm)/10;

      printf("The relation pixel to mm is: %f\n", pixel2mm);

    }

    //---------------------------------------------------------------------------
/*
    // Task 6: move the stage in a square (5 mm sidelength) and save the coordinates
    if (choice == 6){
      MoveMotorRectangular(fd, 5, 255, 1, 1 );
    }


	//---------------------------------------------------------------------------



	///////// Position control /////////
    //---------------------------------------------------------------------------
    ////////////////////////////////////
    // Task 8: Move the magnet 5 mm with the PID function.
  if (choice == 8) {

    frame = cvQueryFrame(capture);

    usleep(10);
    if (!frame)
    {
    printf("Could not grab frame\n");
    return -1;
    }

    ColorTracking(frame, &positionXinitial , &positionYinitial, cvScalar(hmin, smin, vmin), cvScalar( hmax, smax, vmax));

    PID(fd, positionXinitial + 5, positionYinitial, capture);





  }
    //---------------------------------------------------------------------------





    //---------------------------------------------------------------------------
    // Task 9: Tune your Kp Value to achieve a "good" step response.
  if (choice == 9){

 }


    //---------------------------------------------------------------------------


    //---------------------------------------------------------------------------
    // Task 10: Use PID for position control, move magnet 5 mm.
  if (choice == 10){

  }



    //---------------------------------------------------------------------------


    ///////// Closed Loop control /////////
    //---------------------------------------------------------------------------
    ///////////////////////////////////////
    // Task 12: Use PID function to move spherical magnet along a square trajectory (5 mm sidelength)
    if (choice == 12){

    }


    //---------------------------------------------------------------------------

    ///////// Microrobotic Application /////////
    //---------------------------------------------------------------------------
    ////////////////////////////////////////////
    // Task 15: Move the microrobot on a square trajectory (5 mm sidelength) with open loop and closed loop (PID)
    if (choice == 15){

    }


    //---------------------------------------------------------------------------
	// Release captured images
	cvReleaseImage(&frame);
    cvReleaseCapture(&capture);
*/
    
    


    // Close the serial port
    serialport_close(fd);



	return 0;
}
