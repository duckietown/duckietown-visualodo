#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atoi */


int main()
{
  // Initialize parameters
  char command[5] = {'2','1','2','3','4'};

  int flag = 1; //if 1 command is proper, else not
  int steps = 0;
  int motor = 0;
  int direction = 0;

  // Check if there is a command on the serial por
  // Proper command contains 5 bytes:
  // 	First byte is the stage number: 1 or 2
  //	Second byte is the direction: 1 or 2
  //	Third - Fifth bytes are number of steps: 000 - 999
  if (command[0]!=0)
  {
    // Check the first byte and if it is not '1' or '2' discard it
    // First byte determines the stage (stepper motor) that needs to be moved
    if (command[0]!='1' || command[0]!= '2') flag = 0;

    // Check the second byte and if it is not '1' or '2' discard it
    // Second byte determines the direction
    if (command[1]!='1' || command[1]!= '2') flag = 0;

    // Check that third to fifth bytes are between '0' and '9'
    // make sure to convert from chars to integers (subtract 48, the ASCII constant) and multiply accordingly
    for (int i=2; i<5; i++){
      if ( (command[i]-48) < 0 || (command[i]-48) > 9 ) flag = 0;
    }



    motor =command[0]-48;
    direction = command[1]-48;
    steps = (command[2]-48)*100+(command[3]-48)*10+(command[4]-48);
    printf("steps %u\n motor %u\n direction %u\n",steps,motor,direction );
  }

  return 0;
}
