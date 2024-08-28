#include <math.h>
#include <iostream>
#include <stddef.h>
#include <stdlib.h>
#include "rt_defines.h"
#include "rt_nonfinite.c"
#include "rtwtypes.h"
#include "ik_7dof_fourier_types.h"
#include "rtGetInf.c"
#include "norm.c"
#include "rtGetNaN.c"
#include "ik_7dof_fourier.c"
#include "ik_7dof_fourier_terminate.c"
#include "ik_7dof_fourier_initialize.c"


/* Function Declarations */
static double argInit_real_T(void);
static void main_ik_7dof_fourier(void);

/* Function Definitions */

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
// static void main_ik_7dof_fourier(double real_[7])
static void main_ik_7dof_fourier(void)
{

  double theta[7];
//   theta[5]=0.5 ;
  double real_[7];
  real_[0]=0.0;
  real_[1]=0.0;
  real_[2]=90.0;
  real_[3]=0.0;
  real_[4]=300.0;
  real_[5]=300.0;
  real_[6]=30.0;

  /* Initialize function 'ik_7dof_fourier' input arguments. */
  /* Call the entry-point 'ik_7dof_fourier'. */
  ik_7dof_fourier(real_[0], real_[1], real_[2],
                  real_[3], real_[4], real_[5],
                  real_[6], theta);
    // for (size_t i = 0; i < 7; i++)
    // {
    //     std::cout<<theta[i]<<"  ";
    // }
    // std::cout<<std::endl;
    
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

  /* Initialize the application.
     You do not need to do this more than one time. */
  ik_7dof_fourier_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_ik_7dof_fourier();

  /* Terminate the application.
     You do not need to do this more than one time. */
  ik_7dof_fourier_terminate();
  return 0;
}
