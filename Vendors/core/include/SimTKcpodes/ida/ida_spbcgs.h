/*
 * -----------------------------------------------------------------
 * $Revision: 1.2 $
 * $Date: 2006/11/29 00:05:06 $
 * ----------------------------------------------------------------- 
 * Programmer(s): Aaron Collier @ LLNL
 * -----------------------------------------------------------------
 * Copyright (c) 2004, The Regents of the University of California.
 * Produced at the Lawrence Livermore National Laboratory.
 * All rights reserved.
 * For details, see the LICENSE file.
 * -----------------------------------------------------------------
 * This is the public header file for the IDA scaled preconditioned
 * Bi-CGSTAB linear solver module, IDASPBCG.
 * -----------------------------------------------------------------
 */

#ifndef _IDASPBCG_H
#define _IDASPBCG_H

#ifdef __cplusplus  /* wrapper to enable C++ usage */
extern "C" {
#endif

#include <ida/ida_spils.h>
#include <sundials/sundials_spbcgs.h>

/*
 * -----------------------------------------------------------------
 * Function : IDASpbcg
 * -----------------------------------------------------------------
 * A call to the IDASpbcg function links the main integrator with
 * the IDASPBCG linear solver module. Its parameters are as
 * follows:
 *
 * IDA_mem   is the pointer to memory block returned by IDACreate.
 *
 * maxl      is the maximum Krylov subspace dimension, an
 *           optional input. Pass 0 to use the default value.
 *           Otherwise pass a positive integer.
 *
 * The return values of IDASpbcg are:
 *    IDASPILS_SUCCESS    if successful
 *    IDASPILS_MEM_NULL   if the ida memory was NULL
 *    IDASPILS_MEM_FAIL   if there was a memory allocation failure
 *    IDASPILS_ILL_INPUT  if there was illegal input.
 * The above constants are defined in ida_spils.h
 *
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int IDASpbcg(void *ida_mem, int maxl);


#ifdef __cplusplus
}
#endif

#endif
