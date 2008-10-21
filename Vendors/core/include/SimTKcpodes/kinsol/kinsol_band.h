/*
 * -----------------------------------------------------------------
 * $Revision: 1.3 $
 * $Date: 2006/11/29 00:05:07 $
 * ----------------------------------------------------------------- 
 * Programmer(s): Radu Serban @ LLNL
 * -----------------------------------------------------------------
 * Copyright (c) 2002, The Regents of the University of California.
 * Produced at the Lawrence Livermore National Laboratory.
 * All rights reserved.
 * For details, see the LICENSE file.
 * -----------------------------------------------------------------
 * This is the header file for the KINSOL band linear solver, KINBAND.
 * -----------------------------------------------------------------
 */

#ifndef _KINBAND_H
#define _KINBAND_H

#ifdef __cplusplus  /* wrapper to enable C++ usage */
extern "C" {
#endif

#include <kinsol/kinsol_direct.h>
#include <sundials/sundials_band.h>

/*
 * -----------------------------------------------------------------
 * Function : KINBand
 * -----------------------------------------------------------------
 * A call to the KINBand function links the main solver with the 
 * KINBAND linear solver. Its arguments are as follows:
 *
 * kinmem - pointer to the integrator memory returned by KINCreate.
 *
 * N      - problem size
 *
 * mupper - upper bandwidth of the band Jacobian
 *
 * mlower - lower bandwidth of the band Jacobian
 *
 * The return value of KINBand is one of:
 *    KINDIRECT_SUCCESS   if successful
 *    KINDIRECT_MEM_NULL  if the kinsol memory was NULL
 *    KINDIRECT_MEM_FAIL  if there was a memory allocation failure
 *    KINDIRECT_ILL_INPUT if a required vector operation is missing
 *                        or if a bandwidth has an illegal value.
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int KINBand(void *kinmem, int N, int mupper, int mlower);

#ifdef __cplusplus
}
#endif

#endif
