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
 * This is the header file for the KINSOL dense linear solver module, 
 * KINDENSE.
 * -----------------------------------------------------------------
 */

#ifdef __cplusplus  /* wrapper to enable C++ usage */
extern "C" {
#endif

#ifndef _KINDENSE_H
#define _KINDENSE_H

#include <kinsol/kinsol_direct.h>
#include <sundials/sundials_dense.h>

/*
 * -----------------------------------------------------------------
 * Function : KINDense
 * -----------------------------------------------------------------
 * A call to the KINDense function links the main solver with the
 * KINDENSE linear solver. Its arguments are as follows:
 *
 * kinmem - pointer to an internal memory block allocated during a
 *          prior call to KINCreate
 *
 * N      - problem size
 *
 * The return value of KINDense is one of:
 *    KINDIRECT_SUCCESS   if successful
 *    KINDIRECT_MEM_NULL  if the kinsol memory was NULL
 *    KINDIRECT_MEM_FAIL  if there was a memory allocation failure
 *    KINDIRECT_ILL_INPUT if a required vector operation is missing
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int KINDense(void *kinmem, int N);

#endif

#ifdef __cplusplus
}
#endif
