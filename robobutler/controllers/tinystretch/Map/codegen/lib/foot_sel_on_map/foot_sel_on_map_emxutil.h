/*
 * File: foot_sel_on_map_emxutil.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 01-Oct-2022 23:15:16
 */

#ifndef FOOT_SEL_ON_MAP_EMXUTIL_H
#define FOOT_SEL_ON_MAP_EMXUTIL_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "foot_sel_on_map_types.h"

/* Function Declarations */
extern void emxEnsureCapacity_int8_T(emxArray_int8_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);
extern void emxFree_int8_T(emxArray_int8_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

/*
 * File trailer for foot_sel_on_map_emxutil.h
 *
 * [EOF]
 */
