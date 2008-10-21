/*
 * -----------------------------------------------------------------
 * $Revision: 1.2 $
 * $Date: 2006/11/29 00:05:06 $
 * ----------------------------------------------------------------- 
 * Programmer: Radu Serban @ LLNL
 * -----------------------------------------------------------------
 * Copyright (c) 2006, The Regents of the University of California.
 * Produced at the Lawrence Livermore National Laboratory.
 * All rights reserved.
 * For details, see the LICENSE file.
 * -----------------------------------------------------------------
 * Common header file for the direct linear solvers in CVODES.
 *
 * Part I contains type definitions and function prototypes for 
 * using a CVDIRECT linear solver on forward problems (IVP 
 * integration and/or FSA)
 *
 * Part II contains type definitions and function prototypes for 
 * using a CVDIRECT linear solver on adjoint (backward) problems
 * -----------------------------------------------------------------
 */

#ifndef _CVSDIRECT_H
#define _CVSDIRECT_H

#ifdef __cplusplus  /* wrapper to enable C++ usage */
extern "C" {
#endif

#include <sundials/sundials_direct.h>
#include <sundials/sundials_nvector.h>

/*
 * =================================================================
 * C V S D I R E C T     C O N S T A N T S
 * =================================================================
 */

/* 
 * -----------------------------------------------------------------
 * CVSDIRECT return values 
 * -----------------------------------------------------------------
 */

#define CVDIRECT_SUCCESS           0
#define CVDIRECT_MEM_NULL         -1
#define CVDIRECT_LMEM_NULL        -2
#define CVDIRECT_ILL_INPUT        -3
#define CVDIRECT_MEM_FAIL         -4

/* Additional last_flag values */

#define CVDIRECT_JACFUNC_UNRECVR  -5
#define CVDIRECT_JACFUNC_RECVR    -6

/* Return values for the adjoint module */

#define CVDIRECT_ADJMEM_NULL      -101
#define CVDIRECT_LMEMB_NULL       -102

/*
 * =================================================================
 * PART I:  F O R W A R D    P R O B L E M S
 * =================================================================
 */

/*
 * -----------------------------------------------------------------
 * FUNCTION TYPES
 * -----------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------
 * Type: CVDlsDenseJacFn
 * -----------------------------------------------------------------
 *
 * A dense Jacobian approximation function Jac must be of type 
 * CVDlsDenseJacFn. Its parameters are:
 *
 * N   is the problem size.
 *
 * Jac is the dense matrix (of type DlsMat) that will be loaded
 *     by a CVDlsDenseJacFn with an approximation to the Jacobian 
 *     matrix J = (df_i/dy_j) at the point (t,y). 
 *
 * t   is the current value of the independent variable.
 *
 * y   is the current value of the dependent variable vector,
 *     namely the predicted value of y(t).
 *
 * fy  is the vector f(t,y).
 *
 * jac_data is a pointer to user data - the same as the jac_data
 *     parameter passed to CVDlsSetJacFn.
 *
 * tmp1, tmp2, and tmp3 are pointers to memory allocated for
 * vectors of length N which can be used by a CVDlsDenseJacFn
 * as temporary storage or work space.
 *
 * A CVDlsDenseJacFn should return 0 if successful, a positive 
 * value if a recoverable error occurred, and a negative value if 
 * an unrecoverable error occurred.
 *
 * -----------------------------------------------------------------
 *
 * NOTE: The following are two efficient ways to load a dense Jac:         
 * (1) (with macros - no explicit data structure references)      
 *     for (j=0; j < Neq; j++) {                                  
 *       col_j = DENSE_COL(Jac,j);                                 
 *       for (i=0; i < Neq; i++) {                                
 *         generate J_ij = the (i,j)th Jacobian element           
 *         col_j[i] = J_ij;                                       
 *       }                                                        
 *     }                                                          
 * (2) (without macros - explicit data structure references)      
 *     for (j=0; j < Neq; j++) {                                  
 *       col_j = (Jac->data)[j];                                   
 *       for (i=0; i < Neq; i++) {                                
 *         generate J_ij = the (i,j)th Jacobian element           
 *         col_j[i] = J_ij;                                       
 *       }                                                        
 *     }                                                          
 * A third way, using the DENSE_ELEM(A,i,j) macro, is much less   
 * efficient in general.  It is only appropriate for use in small 
 * problems in which efficiency of access is NOT a major concern. 
 *                                                                
 * NOTE: If the user's Jacobian routine needs other quantities,   
 *     they are accessible as follows: hcur (the current stepsize)
 *     and ewt (the error weight vector) are accessible through   
 *     CVodeGetCurrentStep and CVodeGetErrWeights, respectively 
 *     (see cvode.h). The unit roundoff is available as 
 *     UNIT_ROUNDOFF defined in sundials_types.h.
 *
 * -----------------------------------------------------------------
 */
  
  
typedef int (*CVDlsDenseJacFn)(int N, realtype t,
			       N_Vector y, N_Vector fy, 
			       DlsMat Jac, void *jac_data,
			       N_Vector tmp1, N_Vector tmp2, N_Vector tmp3);
  
/*
 * -----------------------------------------------------------------
 * Type: CVDlsBandJacFn
 * -----------------------------------------------------------------
 *
 * A band Jacobian approximation function Jac must have the
 * prototype given below. Its parameters are:
 *
 * N is the length of all vector arguments.
 *
 * mupper is the upper half-bandwidth of the approximate banded
 * Jacobian. This parameter is the same as the mupper parameter
 * passed by the user to the linear solver initialization function.
 *
 * mlower is the lower half-bandwidth of the approximate banded
 * Jacobian. This parameter is the same as the mlower parameter
 * passed by the user to the linear solver initialization function.
 *
 * t is the current value of the independent variable.
 *
 * y is the current value of the dependent variable vector,
 *      namely the predicted value of y(t).
 *
 * fy is the vector f(t,y).
 *
 * Jac is the band matrix (of type DlsMat) that will be loaded
 * by a CVDlsBandJacFn with an approximation to the Jacobian matrix
 * Jac = (df_i/dy_j) at the point (t,y).
 * Three efficient ways to load J are:
 *
 * (1) (with macros - no explicit data structure references)
 *    for (j=0; j < n; j++) {
 *       col_j = BAND_COL(Jac,j);
 *       for (i=j-mupper; i <= j+mlower; i++) {
 *         generate J_ij = the (i,j)th Jacobian element
 *         BAND_COL_ELEM(col_j,i,j) = J_ij;
 *       }
 *     }
 *
 * (2) (with BAND_COL macro, but without BAND_COL_ELEM macro)
 *    for (j=0; j < n; j++) {
 *       col_j = BAND_COL(Jac,j);
 *       for (k=-mupper; k <= mlower; k++) {
 *         generate J_ij = the (i,j)th Jacobian element, i=j+k
 *         col_j[k] = J_ij;
 *       }
 *     }
 *
 * (3) (without macros - explicit data structure references)
 *     offset = Jac->smu;
 *     for (j=0; j < n; j++) {
 *       col_j = ((Jac->data)[j])+offset;
 *       for (k=-mupper; k <= mlower; k++) {
 *         generate J_ij = the (i,j)th Jacobian element, i=j+k
 *         col_j[k] = J_ij;
 *       }
 *     }
 * Caution: Jac->smu is generally NOT the same as mupper.
 *
 * The BAND_ELEM(A,i,j) macro is appropriate for use in small
 * problems in which efficiency of access is NOT a major concern.
 *
 * jac_data is a pointer to user data - the same as the jac_data
 *          parameter passed to CVDlsSetJacFn.
 *
 * NOTE: If the user's Jacobian routine needs other quantities,
 *     they are accessible as follows: hcur (the current stepsize)
 *     and ewt (the error weight vector) are accessible through
 *     CVodeGetCurrentStep and CVodeGetErrWeights, respectively
 *     (see cvode.h). The unit roundoff is available as
 *     UNIT_ROUNDOFF defined in sundials_types.h
 *
 * tmp1, tmp2, and tmp3 are pointers to memory allocated for
 * vectors of length N which can be used by a CVDlsBandJacFn
 * as temporary storage or work space.
 *
 * A CVDlsBandJacFn should return 0 if successful, a positive value
 * if a recoverable error occurred, and a negative value if an 
 * unrecoverable error occurred.
 * -----------------------------------------------------------------
 */

typedef int (*CVDlsBandJacFn)(int N, int mupper, int mlower,
			      realtype t, N_Vector y, N_Vector fy, 
			      DlsMat Jac, void *jac_data,
			      N_Vector tmp1, N_Vector tmp2, N_Vector tmp3);

/*
 * -----------------------------------------------------------------
 * EXPORTED FUNCTIONS 
 * -----------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------
 * Optional inputs to the CVSDIRECT linear solver
 * -----------------------------------------------------------------
 *
 * CVDlsSetJacFn specifies the Jacobian approximation routine to be
 * used. When using dense Jacobians, a user-supplied jac routine must 
 * be of type CVDlsDenseJacFn. When using banded Jacobians, a 
 * user-supplied jac routine must be of type CVDlsBandJacFn.
 * By default, a difference quotient approximation, supplied with this 
 * solver is used.
 * CVDlsSetJacFn also specifies a pointer to user data which is 
 * passed to the user's jac routine every time it is called.
 *
 * The return value of CVDlsSetJacFn is one of:
 *    CVDIRECT_SUCCESS   if successful
 *    CVDIRECT_MEM_NULL  if the CVODES memory was NULL
 *    CVDIRECT_LMEM_NULL if the linear solver memory was NULL
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int CVDlsSetJacFn(void *cvode_mem, void *jac, void *jac_data);

/*
 * -----------------------------------------------------------------
 * Optional outputs from the CVSDIRECT linear solver
 * -----------------------------------------------------------------
 *
 * CVDlsGetWorkSpace   returns the real and integer workspace used
 *                     by the direct linear solver.
 * CVDlsGetNumJacEvals returns the number of calls made to the
 *                     Jacobian evaluation routine jac.
 * CVDlsGetNumRhsEvals returns the number of calls to the user
 *                     f routine due to finite difference Jacobian
 *                     evaluation.
 * CVDlsGetLastFlag    returns the last error flag set by any of
 *                     the CVSDIRECT interface functions.
 *
 * The return value of CVDlsGet* is one of:
 *    CVDIRECT_SUCCESS   if successful
 *    CVDIRECT_MEM_NULL  if the CVODES memory was NULL
 *    CVDIRECT_LMEM_NULL if the linear solver memory was NULL
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int CVDlsGetWorkSpace(void *cvode_mem, long int *lenrwLS, long int *leniwLS);
SUNDIALS_EXPORT int CVDlsGetNumJacEvals(void *cvode_mem, long int *njevals);
SUNDIALS_EXPORT int CVDlsGetNumRhsEvals(void *cvode_mem, long int *nfevalsLS);
SUNDIALS_EXPORT int CVDlsGetLastFlag(void *cvode_mem, int *flag);

/*
 * -----------------------------------------------------------------
 * The following function returns the name of the constant 
 * associated with a CVSDIRECT return flag
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT char *CVDlsGetReturnFlagName(int flag);

/*
 * =================================================================
 * PART II:  B A C K W A R D    P R O B L E M S
 * =================================================================
 */

/*
 * -----------------------------------------------------------------
 * FUNCTION TYPES
 * -----------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------
 * Type: CVDlsDenseJacFnB
 * -----------------------------------------------------------------
 * A dense Jacobian approximation function jacB for the adjoint
 * (backward) problem must have the prototype given below. 
 * -----------------------------------------------------------------
 */

typedef int (*CVDlsDenseJacFnB)(int nB, realtype t,
				N_Vector y, 
				N_Vector yB, N_Vector fyB,
				DlsMat JB, void *jac_dataB, 
				N_Vector tmp1B, N_Vector tmp2B, N_Vector tmp3B);

/*
 * -----------------------------------------------------------------
 * Type : CVDlsBandJacFnB
 * -----------------------------------------------------------------
 * A band Jacobian approximation function jacB for the adjoint 
 * (backward) problem must have the prototype given below. 
 * -----------------------------------------------------------------
 */

typedef int (*CVDlsBandJacFnB)(int nB, int mupperB, int mlowerB,
			       realtype t, 
			       N_Vector y,
			       N_Vector yB, N_Vector fyB,
			       DlsMat JB, void *jac_dataB, 
			       N_Vector tmp1B, N_Vector tmp2B, N_Vector tmp3B);

/*
 * -----------------------------------------------------------------
 * EXPORTED FUNCTIONS 
 * -----------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------
 * Functions: CVDlsSetJacFnB
 * -----------------------------------------------------------------
 * CVDlsSetJacFn specifies the Jacobian function to be used by a
 * CVSDIRECT linear solver for the bacward integration phase.
 * -----------------------------------------------------------------
 */

SUNDIALS_EXPORT int CVDlsSetJacFnB(void *cvadj_mem, void *jacB, void *jac_dataB);

#ifdef __cplusplus
}
#endif

#endif
