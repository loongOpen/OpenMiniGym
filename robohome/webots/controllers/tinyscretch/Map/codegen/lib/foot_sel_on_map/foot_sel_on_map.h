/*
 * File: foot_sel_on_map.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 01-Oct-2022 23:15:16
 */

#ifndef FOOT_SEL_ON_MAP_H
#define FOOT_SEL_ON_MAP_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "foot_sel_on_map_types.h"

/* Function Declarations */
extern void foot_sel_on_map(const double map_local[6400], const double
  map_param[5], const double check_param[3], double location_map[6400], double
  map_edge_out[6400]);
extern void foot_sel_on_map_initialize(void);
extern void foot_sel_on_map_terminate(void);

#endif

/*
 * File trailer for foot_sel_on_map.h
 *
 * [EOF]
 */
