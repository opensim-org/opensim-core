/*
 * -----------------------------------------------------------------
 * $Revision: 1.4 $
 * $Date: 2006/11/29 00:05:06 $
 * ----------------------------------------------------------------- 
 * Programmer(s): Alan C. Hindmarsh, and Radu Serban @ LLNL
 * -----------------------------------------------------------------
 * Copyright (c) 2002, The Regents of the University of California.
 * Produced at the Lawrence Livermore National Laboratory.
 * All rights reserved.
 * For details, see the LICENSE file.
 * -----------------------------------------------------------------
 * This is the header file for the IDABAND linear solver module.
 * -----------------------------------------------------------------
 */

#ifndef _IDABAND_H
#define _IDABAND_H

#ifdef __cplusplus  /* wrapper to enable C++ usage */
extern "C" {
#endif

#include <ida/ida_direct.h>
#include <sundials/sundials_band.h>

/*
 * -----------------------------------------------------------------
 * Function : IDABand
 * -----------------------------------------------------------------
 * A call to the IDABand function links the main integrator       
 * with the IDABAND linear solver module.                         
 *                                                                
 * ida_mem is the pointer to the integrator memory returned by    
 *         IDACreate.                                                   
 *                                                                
 * mupper is the upper bandwidth of the banded Jacobian matrix.   
 *                                                                
 * mlower is the lower bandwidth of the banded Jacobian matrix.   
 *                                                                
 * The return values of IDABand are:                              
 *     IDABAND_SUCCESS   = 0  if successful                            
 *     IDABAND_LMEM_FAIL = -1 if there was a memory allocation failure 
 *     IDABAND_ILL_INPUT = -2 if the input was illegal or NVECTOR bad. 
 *                                                                
 * NOTE: The band linear solver assumes a serial implementation   
 *       of the NVECTOR package. Therefore, IDABand will first
 *       test for a compatible N_Vector internal representation
 *       by checking that the N_VGetArrayPointer function exists.
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int IDABand(void *ida_mem, int Neq, int mupper, int mlower);

#ifdef __cplusplus
}
#endif

#endif
