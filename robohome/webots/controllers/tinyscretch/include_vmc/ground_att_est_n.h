/*
 * File: ground_att_est_n.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 30-Aug-2020 14:04:06
 */

#ifndef GROUND_ATT_EST_N_H
#define GROUND_ATT_EST_N_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"


/* Function Declarations */
extern void ground_att_est_n(const char G[4], const double P1n[3], const double P2n[3],
	const double P3n[3], const double P4n[3], const double
	ATT_OFF[3], double Norm[3], double Plan_Param[3], double
	ATT_G[3], double *flag);

#endif

/*
 * File trailer for ground_att_est_n.h
 *
 * [EOF]
 */
