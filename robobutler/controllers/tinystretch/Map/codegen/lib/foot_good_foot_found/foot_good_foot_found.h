/*
 * File: foot_good_foot_found.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 22-Dec-2021 17:35:42
 */

#ifndef FOOT_GOOD_FOOT_FOUND_H
#define FOOT_GOOD_FOOT_FOUND_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "foot_good_foot_found_types.h"

/* Function Declarations */
extern void foot_good_foot_found(short leg_sel, double spd_fb, const double
  now_foot[3], const double slip_foot[3], const double map_local_foot[1600],
  const double map_param[5], const double check_param[3], const double
  robot_param[3], double gfoot[3], double *is_found);
extern void foot_good_foot_found_initialize(void);
extern void foot_good_foot_found_terminate(void);

#endif

/*
 * File trailer for foot_good_foot_found.h
 *
 * [EOF]
 */
