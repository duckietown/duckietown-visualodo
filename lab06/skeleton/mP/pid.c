/*
 * pid.c
 *
 * Copyright 2015  <ubuntu@udoobuntu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 *
 */

// write your includes here


int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;

    fd = open(serialport, O_RDWR | O_NONBLOCK);

    if (fd == -1)
        return -1;

    if (tcgetattr(fd, &toptions) < 0)
        return -1;

    speed_t brate = baud;
    switch (baud)
    {
		case 4800:   brate = B4800;
		break;
		case 9600:   brate = B9600;
		break;
		#ifdef B14400
		case 14400:  brate = B14400;
		break;
		#endif
		case 19200:  brate = B19200;
		break;
		#ifdef B28800
		case 28800:  brate = B28800;
		break;
		#endif
		case 38400:  brate = B38400;
		break;
		case 57600:  brate = B57600;
		break;
		case 115200: brate = B115200;
		break;
    }

    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    toptions.c_cflag &= ~CRTSCTS;
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &toptions);
    if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0)
        return -1;

    return fd;
}

int serialport_close(int fd)
{
    return close(fd);
}

int serialport_write(int fd, const char* str)
{
    int len = strlen(str);
    int n = write(fd, str, len);

    if (n != len)
        return -1;

    return 0;
}

int serialport_read_until(int fd, char* buf, char until, int buf_max, int timeout)
{
    char b[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    int n=0;

    do
    {
        n = read(fd, b, 1);  // read a char at a time
        if (n == -1)
			return -1;    // couldn't read

        if (n == 0)
        {
            usleep( 1 * 1000 );  // wait 1 msec try again
            timeout--;
            continue;
        }
        buf[i] = b[0];
        i++;
    } while(b[0] != until && i < buf_max && timeout>0);

    buf[i] = 0;  // null terminate the string

    return n;
}

IplImage* CrossTarget (IplImage* inImg, int x, int y, int size, int line_thickness)
{
	IplImage* outImg = inImg;

	cvLine(outImg, cvPoint(x-size,y), cvPoint(x+size,y), cvScalar(0,0,255,0), line_thickness, 8, 0);
	cvLine(outImg, cvPoint(x,y-size), cvPoint(x,y+size), cvScalar(0,0,255,0), line_thickness, 8, 0);

	return outImg;
}

int ColorTracking (IplImage* img, int* positionX , int* positionY, int color, int* posX , int* posY, int count, int drawTraj)
{
    // add your Color tracking algorithm here

	return 0;
}

int constructCommand (char* command, int u, int motor)
{

  unsigned int digit1,digit2,digit3;
  digit1 = motor/100;
  digit2 = (motor-100*digit1)/10;
  digit3 = (motor-100*digit1-10*digit2);
	// Task 2:
    // First byte determines the motor to be moved
    command[0] = atoi(motor);

	// Second byte determines the direction of movement
    (u>=0) ? command[1]=1 : command[1]= 2;

	// Third to fifth bytes determine the number of steps to be moved
    // Remember to convert integers to char first
    command[2] = '0'+digit1;
    command[3] = '0'+digit2;
    command[4] = '0'+digit3;


	return 0;
}

int MoveMotor (int fd, float distance, int motor)
{
    // Task 3:
    // initialize variables here
    char[5] cmd;
    int steps;
    char buffer;
    int response = 0;

    // The distance must be below the maximum range of 5mm.
    if (motor != 1 || motor != 2) return -1;
    if (abs(distance) > 5 ) return -2;
	// Create appropriate 5 byte command and write it to the serial port
    steps = distance / 5 *999;
    constructCommand(cmd, steps, motor);
	// Read from Microcontroller; Create a loop where you wait (usleep(10)) then use:
    // serialport_read_until(fd, buff, '\0', 1, 10) until you get a response
    while(!response){
      usleep(10);
      serialport_read_until(fd, buffer, '\0', 1, 10);
    }

    if (buffer[0] == '1'){
      printf("Switch number %u activated.\n", buffer[0])
    }


    // check in buff[0] if one of the switches was pressed (E.g. from Arduino Code, buff[0] will be '1'
    // when switch one is pressed)


	return 0;
}

int MoveMotorRectangular (int fd, float distance, int steps, int useVision, int camIndex)
{
    // Task 5:
    // initialize your variables here
    FILE *fp;


    // Create a .txt file
    fp = fopen("RectangularCoord.txt", "w+");
    if (!fp)
    {
		printf("Could not create a .txt file...\n");
		return -1;
	}

     // Move the stage along all 4 sides

	 // If vision is used, initialize camera and  store the coordinates at each point (during movement!)
     // in the .txt file

		// Get camera


		// Grab a frame from camera

        // Detect the coordinates of the object

        // Save the coordinates o






	fclose(fp);

	return 0;
}


int PID (int fd, int targetPositionX, int targetPositionY, CvCapture* capture)
{
    // Task 7,10:
    // initialize your variables here
    struct timeval currentTime;
    struct timeval prevTime;
	IplImage* frame;
	FILE *fp;

	// Create a .txt file


	// Grab the frame from the camera

    // Find the X and Y coordinates of an object

	// Get current time and initial error

    // write your do - while loop here

		// Determine the time interval

        // Determine the P, I and D (each for X and Y axis seperately)

		// Compute the control command

        // Move the stage axis X

		// Wait until done

		// Move the stage axis Y

		// Wait until done

		// Grab the new frame from the camera

		// Determine the new position

		// Save the new position as current position

		// Get current time and update the error


	fclose(fp);

    return 0;
}
