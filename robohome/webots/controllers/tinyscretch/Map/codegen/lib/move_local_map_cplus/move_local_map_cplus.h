/*
 * File: move_local_map_cplus.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 16-Dec-2021 16:11:24
 */

#ifndef MOVE_LOCAL_MAP_CPLUS_H
#define MOVE_LOCAL_MAP_CPLUS_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "move_local_map_cplus_types.h"

/* Function Declarations */
extern void move_local_map_cplus(const double cog_n[3], const double map_n
  [1000000], const double map_param[5], double map_local[1600]);
extern void move_local_map_cplus_initialize(void);
extern void move_local_map_cplus_terminate(void);

#endif

/*
 * File trailer for move_local_map_cplus.h
 *
 * [EOF]
 */
