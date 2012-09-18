/* -------------------------------------------------------------------------- *
 *                             OpenSim:  gcvspl.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

#ifdef __cplusplus
extern "C" {
#endif

/*
	constants
*/
const double zero = 0.0 ;
const double half = 0.5 ;
const double one  = 1.0 ;
const double two  = 2.0 ;

/*
	function prototypes
*/
void   basis(int m, int n, double *x, double *b, double *bl, double *q) ;
void   prep(int m, int n, double *x, double *w, double *we, double *el) ;
void   bansol(double *e, double *y, double *c, int m, int n) ;
void   bandet(double *e, int m, int n) ;
void   search(int n, double *x, double t, int *l) ;
void   gcvspl(double *x, double  *y, double *w, int m, int n,
				  double *c, double var, double *wk, int ier) ;
double splc(int m, int n, double *y, double *w, double var, double p,
				double eps, double *c, double *stat, double *b, double *we,
				double el, double *bwe) ;
double splder(int ider, int m, int n, double t, double *x,
					double *c, int *l, double *q) ;
double trinv(double *b, double *e, int m, int n) ;

#ifdef __cplusplus
}
#endif
