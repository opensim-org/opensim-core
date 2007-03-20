
/************************lmdif*************************
 *
 * Solves or minimizes the sum of squares of m nonlinear
 * functions of n variables.
 *
 * From public domain Fortran version
 * of Argonne National Laboratories MINPACK
 *
 * C translation by Steve Moshier
 *
 * Passing C++ and thread-safe by Ned Phipps
 *
 *****************************************************/

#include "osimCommonDLL.h"

void OSIMCOMMON_API lmdif_C(
  void (*fcn)(int, int, double[], double[], int *, void *),
  int    m,
  int    n,
  double x[],
  double fvec[],
  double ftol,
  double xtol,
  double gtol,
  int    maxfev,
  double epsfcn,
  double diag[],
  int    mode,
  double factor,
  int    nprint,
  int   *info,
  int   *nfev,
  double fjac[],
  int    ldfjac,
  int    ipvt[],
  double qtf[],
  double wa1[],
  double wa2[],
  double wa3[],
  double wa4[],
  void *data);
