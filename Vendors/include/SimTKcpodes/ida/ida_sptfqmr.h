/*
 * -----------------------------------------------------------------
 * $Revision: 1.2 $
 * $Date: 2006/11/29 00:05:07 $
 * ----------------------------------------------------------------- 
 * Programmer(s): Aaron Collier @ LLNL
 * -----------------------------------------------------------------
 * Copyright (c) 2005, The Regents of the University of California.
 * Produced at the Lawrence Livermore National Laboratory.
 * All rights reserved.
 * For details, see the LICENSE file.
 * -----------------------------------------------------------------
 * This is the public header file for the IDA scaled preconditioned
 * TFQMR linear solver module, IDASPTFQMR.
 * -----------------------------------------------------------------
 */

#ifndef _IDASPTFQMR_H
#define _IDASPTFQMR_H

#ifdef __cplusplus  /* wrapper to enable C++ usage */
extern "C" {
#endif

#include <ida/ida_spils.h>
#include <sundials/sundials_sptfqmr.h>

/*
 * -----------------------------------------------------------------
 * Function : IDASptfqmr
 * -----------------------------------------------------------------
 * A call to the IDASptfqmr function links the main integrator with
 * the IDASPTFQMR linear solver module. Its parameters are as
 * follows:
 *
 * IDA_mem  is the pointer to memory block returned by IDACreate.
 *
 * maxl     is the maximum Krylov subspace dimension, an
 *          optional input. Pass 0 to use the default value.
 *          Otherwise pass a positive integer.
 *
 * The return values of IDASptfqmr are:
 *    IDASPILS_SUCCESS    if successful
 *    IDASPILS_MEM_NULL   if the ida memory was NULL
 *    IDASPILS_MEM_FAIL   if there was a memory allocation failure
 *    IDASPILS_ILL_INPUT  if there was illegal input.
 * The above constants are defined in ida_spils.h
 *
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int IDASptfqmr(void *ida_mem, int maxl);


#ifdef __cplusplus
}
#endif

#endif
