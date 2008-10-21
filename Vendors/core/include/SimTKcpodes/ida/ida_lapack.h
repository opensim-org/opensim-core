/*
 * -----------------------------------------------------------------
 * $Revision: 1.3 $
 * $Date: 2006/11/29 00:05:06 $
 * ----------------------------------------------------------------- 
 * Programmer: Radu Serban @ LLNL
 * -----------------------------------------------------------------
 * Copyright (c) 2006, The Regents of the University of California.
 * Produced at the Lawrence Livermore National Laboratory.
 * All rights reserved.
 * For details, see the LICENSE file.
 * -----------------------------------------------------------------
 * Header file for the IDA dense linear solver IDALAPACK.
 * -----------------------------------------------------------------
 */

#ifndef _IDALAPACK_H
#define _IDALAPACK_H

#ifdef __cplusplus  /* wrapper to enable C++ usage */
extern "C" {
#endif

#include <ida/ida_direct.h>
#include <sundials/sundials_lapack.h>

/*
 * -----------------------------------------------------------------
 * Function : IDALapackDense
 * -----------------------------------------------------------------
 * A call to the IDALapackDense function links the main integrator
 * with the IDALAPACK linear solver using dense Jacobians.
 *
 * ida_mem is the pointer to the integrator memory returned by
 *           IDACreate.
 *
 * N is the size of the ODE system.
 *
 * The return value of IDALapackDense is one of:
 *    IDADIRECT_SUCCESS   if successful
 *    IDADIRECT_MEM_NULL  if the IDA memory was NULL
 *    IDADIRECT_MEM_FAIL  if there was a memory allocation failure
 *    IDADIRECT_ILL_INPUT if a required vector operation is missing
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int IDALapackDense(void *ida_mem, int N);

/*
 * -----------------------------------------------------------------
 * Function : IDALapackBand
 * -----------------------------------------------------------------
 * A call to the IDALapackBand function links the main integrator
 * with the IDALAPACK linear solver using banded Jacobians. 
 *
 * ida_mem is the pointer to the integrator memory returned by
 *           IDACreate.
 *
 * N is the size of the ODE system.
 *
 * mupper is the upper bandwidth of the band Jacobian approximation.
 *
 * mlower is the lower bandwidth of the band Jacobian approximation.
 *
 * The return value of IDALapackBand is one of:
 *    IDADIRECT_SUCCESS   if successful
 *    IDADIRECT_MEM_NULL  if the IDA memory was NULL
 *    IDADIRECT_MEM_FAIL  if there was a memory allocation failure
 *    IDADIRECT_ILL_INPUT if a required vector operation is missing
 *                        or if a bandwidth has an illegal value.
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int IDALapackBand(void *ida_mem, int N, int mupper, int mlower);

#ifdef __cplusplus
}
#endif

#endif
