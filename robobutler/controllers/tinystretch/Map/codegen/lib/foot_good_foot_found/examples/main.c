/*
 * File: main.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Dec-2021 17:35:42
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "foot_good_foot_found.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void argInit_1x3_real_T(double result[3]);
static void argInit_1x5_real_T(double result[5]);
static void argInit_40x40_real_T(double result[1600]);
static short argInit_int16_T(void);
static double argInit_real_T(void);
static void main_foot_good_foot_found(void);

/* Function Definitions */

/*
 * Arguments    : double result[3]
 * Return Type  : void
 */
static void argInit_1x3_real_T(double result[3])
{
  double result_tmp;

  /* Loop over the array to initialize each element. */
  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result_tmp = argInit_real_T();
  result[0] = result_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[1] = result_tmp;

  /* Set the value of the array element.
     Change this value to the value that the application requires. */
  result[2] = result_tmp;
}

/*
 * Arguments    : double result[5]
 * Return Type  : void
 */
static void argInit_1x5_real_T(double result[5])
{
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 5; idx1++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[1600]
 * Return Type  : void
 */
static void argInit_40x40_real_T(double result[1600])
{
  int idx0;
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 40; idx0++) {
    for (idx1 = 0; idx1 < 40; idx1++) {
      /* Set the value of the array element.
         Change this value to the value that the application requires. */
      result[idx0 + 40 * idx1] = argInit_real_T();
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : short
 */
static short argInit_int16_T(void)
{
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_foot_good_foot_found(void)
{
  double now_foot_tmp[3];
  double dv[1600];
  double dv1[5];
  double gfoot[3];
  double is_found;

  /* Initialize function 'foot_good_foot_found' input arguments. */
  /* Initialize function input argument 'now_foot'. */
  argInit_1x3_real_T(now_foot_tmp);

  /* Initialize function input argument 'slip_foot'. */
  /* Initialize function input argument 'map_local_foot'. */
  /* Initialize function input argument 'map_param'. */
  /* Initialize function input argument 'check_param'. */
  /* Initialize function input argument 'robot_param'. */
  /* Call the entry-point 'foot_good_foot_found'. */
  argInit_40x40_real_T(dv);
  argInit_1x5_real_T(dv1);
  foot_good_foot_found(argInit_int16_T(), argInit_real_T(), now_foot_tmp,
                       now_foot_tmp, dv, dv1, now_foot_tmp, now_foot_tmp, gfoot,
                       &is_found);
}

/*
 * Arguments    : int argc
 *                const char * const argv[]
 * Return Type  : int
 */
int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_foot_good_foot_found();

  /* Terminate the application.
     You do not need to do this more than one time. */
  foot_good_foot_found_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
