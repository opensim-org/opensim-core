/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Lmdif.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

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
