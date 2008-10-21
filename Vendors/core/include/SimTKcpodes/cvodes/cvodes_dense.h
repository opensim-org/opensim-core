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
 * This is the header file for the dense linear solver CVSDENSE.
 * -----------------------------------------------------------------
 */

#ifndef _CVSDENSE_H
#define _CVSDENSE_H

#ifdef __cplusplus  /* wrapper to enable C++ usage */
extern "C" {
#endif

#include <cvodes/cvodes_direct.h>
#include <sundials/sundials_dense.h>

/*
 * -----------------------------------------------------------------
 * Function: CVDense
 * -----------------------------------------------------------------
 * A call to the CVDense function links the main integrator with
 * the CVSDENSE linear solver.
 *
 * cvode_mem is the pointer to the integrator memory returned by
 *           CVodeCreate.
 *
 * N is the size of the ODE system.
 *
 * The return value of CVDense is one of:
 *    CVDIRECT_SUCCESS   if successful
 *    CVDIRECT_MEM_NULL  if the cvode memory was NULL
 *    CVDIRECT_MEM_FAIL  if there was a memory allocation failure
 *    CVDIRECT_ILL_INPUT if a required vector operation is missing
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int CVDense(void *cvode_mem, int N);

/*
 * -----------------------------------------------------------------
 * Function: CVDenseB
 * -----------------------------------------------------------------
 * CVDenseB links the main CVODE integrator with the CVSDENSE
 * linear solver for the backward integration.
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int CVDenseB(void *cvadj_mem, int nB);

#ifdef __cplusplus
}
#endif

#endif
