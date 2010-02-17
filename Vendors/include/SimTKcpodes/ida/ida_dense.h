/*
 * -----------------------------------------------------------------
 * $Revision: 1.4 $
 * $Date: 2006/11/29 00:05:06 $
 * ----------------------------------------------------------------- 
 * Programmer(s): Alan C. Hindmarsh and Radu Serban @ LLNL
 * -----------------------------------------------------------------
 * Copyright (c) 2002, The Regents of the University of California.
 * Produced at the Lawrence Livermore National Laboratory.
 * All rights reserved.
 * For details, see the LICENSE file.
 * -----------------------------------------------------------------
 * This is the header file for the IDADENSE linear solver module.
 * -----------------------------------------------------------------
 */

#ifndef _IDADENSE_H
#define _IDADENSE_H

#ifdef __cplusplus  /* wrapper to enable C++ usage */
extern "C" {
#endif

#include <ida/ida_direct.h>
#include <sundials/sundials_dense.h>

/*
 * -----------------------------------------------------------------
 * Function : IDADense
 * -----------------------------------------------------------------
 * A call to the IDADense function links the main integrator      
 * with the IDADENSE linear solver module.                        
 *                                                                
 * ida_mem is the pointer to integrator memory returned by        
 *     IDACreate.                                                 
 *                                                                
 * Neq  is the problem size                                       
 *                                                                
 * IDADense returns:                                              
 *     IDADENSE_SUCCESS   = 0  if successful                              
 *     IDADENSE_LMEM_FAIL = -1 if there was a memory allocation failure   
 *     IDADENSE_ILL_INPUT = -2 if NVECTOR found incompatible           
 *                                                                
 * NOTE: The dense linear solver assumes a serial implementation  
 *       of the NVECTOR package. Therefore, IDADense will first
 *       test for a compatible N_Vector internal representation
 *       by checking that the functions N_VGetArrayPointer and
 *       N_VSetArrayPointer exist.
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int IDADense(void *ida_mem, int Neq); 

#ifdef __cplusplus
}
#endif

#endif
