/*
 * -----------------------------------------------------------------
 * $Revision: 1.4 $
 * $Date: 2006/11/29 00:05:06 $
 * ----------------------------------------------------------------- 
 * Programmer(s): Radu Serban @ LLNL
 * -----------------------------------------------------------------
 * Copyright (c) 2005, The Regents of the University of California.
 * Produced at the Lawrence Livermore National Laboratory.
 * All rights reserved.
 * For details, see the LICENSE file.
 * -----------------------------------------------------------------
 * This is the header file for the band linear solver CSVBAND.
 * -----------------------------------------------------------------
 */

#ifndef _CVSBAND_H
#define _CVSBAND_H

#ifdef __cplusplus  /* wrapper to enable C++ usage */
extern "C" {
#endif

#include <cvodes/cvodes_direct.h>
#include <sundials/sundials_band.h>

/*
 * -----------------------------------------------------------------
 * Function : CVBand
 * -----------------------------------------------------------------
 * A call to the CVBand function links the main CVODE integrator
 * with the CVSBAND linear solver.
 *
 * cvode_mem is the pointer to the integrator memory returned by
 *           CVodeCreate.
 *
 * N is the size of the ODE system.
 *
 * mupper is the upper bandwidth of the band Jacobian
 *        approximation.
 *
 * mlower is the lower bandwidth of the band Jacobian
 *        approximation.
 *
 * The return value of CVBand is one of:
 *    CVDIRECT_SUCCESS   if successful
 *    CVDIRECT_MEM_NULL  if the cvode memory was NULL
 *    CVDIRECT_MEM_FAIL  if there was a memory allocation failure
 *    CVDIRECT_ILL_INPUT if a required vector operation is missing or
 *                     if a bandwidth has an illegal value.
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int CVBand(void *cvode_mem, int N, int mupper, int mlower);

/*
 * -----------------------------------------------------------------
 * Function: CVBandB
 * -----------------------------------------------------------------
 * CVBandB links the main CVODE integrator with the CVSBAND
 * linear solver for the backward integration.
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int CVBandB(void *cvadj_mem, int nB, int mupperB, int mlowerB);
  
#ifdef __cplusplus
}
#endif

#endif
