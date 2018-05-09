#include <stdio.h>    // Standard input/output definitions
#include <stdlib.h>
#include <unistd.h>   // UNIX standard function definitions
#include <sys/time.h>
#include <fcntl.h>    // File control definitions
#include <errno.h>    // Error number definitions
#include <termios.h>  // POSIX terminal control definitions
#include <string.h>   // String function definitions
#include <math.h>
/*  NOTE:
 *  To apply the filters on a full dataset (done in main file), a loop will have
 *  to be implemented in the main file to iterate through all the datapoints. The
 *  variable counter indicates the current position of the filter.
 *  Use the PseudoCode from the lecture to help you implement the filters
 */


float smoothing_filter( float* sensRawArray, int filterSamples, int counter);
/*  Online Smoothing filter. Takes average of N ("filterSamples") previous samples at a specific position ("counter") in "sensRawArray"
 *  sensRawArray - array for raw data
 *  filterSamples - number of samples in filter (N)
 *  counter - current position of filter in the sensRawArray
 *  return filtered Datapoint at filter position
 */

float blackman_coefs(int arg_M, float arg_fc, double* arg_coefs);
/*  Calculating Blackmann coefficients. Based on formula. Be Careful when dividing two int-variables!!!
 *  argM - m
 *  arg_fc_2 - fc
 *  arg_coefs - array for coefficients
 *  save normalized Blackman coefficients in arg_coefs
 *  return 0
 */

float blackman_filter( float* arg_raw_data, int arg_M, double* arg_coefs, int counter);
/*  Blackman Filter. Takes blackman filtered Data of M ("arg_M") previous samples at a specific position ("counter") in "arg_raw_data".
 *  int *arg_raw_data - array of raw filter data
 *  int arg_M - m
 *  arg_coefs - array with normalized blackman coefficients from blackman_coefs function
 *  counter - position of filter
 *  return filtered Datapoint at filter position
 */
