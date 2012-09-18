#ifndef __SimmMacros_h__
#define __SimmMacros_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  SimmMacros.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

namespace OpenSim {

#define TINY_NUMBER 0.0000001
#define ROUNDOFF_ERROR 0.0000000000002
#define MAX(a,b) ((a)>=(b)?(a):(b))
#define MIN(a,b) ((a)<=(b)?(a):(b))
#define DABS(a) ((a)>(double)0.0?(a):(-(a)))
#define DSIGN(a) ((a)>=0.0?(1):(-1))
#define SQR(x)	((x) * (x))
#define EQUAL_WITHIN_ERROR(a,b) (DABS(((a)-(b))) <= ROUNDOFF_ERROR)
#define NOT_EQUAL_WITHIN_ERROR(a,b) (DABS(((a)-(b))) > ROUNDOFF_ERROR)
#define EQUAL_WITHIN_TOLERANCE(a,b,c) (DABS(((a)-(b))) <= (c))
#define MAKE_3DVECTOR(pt1,pt2,pt3) {pt3[0]=pt2[0]-pt1[0];pt3[1]=pt2[1]-pt1[1];pt3[2]=pt2[2]-pt1[2];}
#define MAKE_3DVECTOR21(pt1,pt2,pt3) {pt3[0]=pt1[0]-pt2[0];pt3[1]=pt1[1]-pt2[1];pt3[2]=pt1[2]-pt2[2];}
#define COPY_1X3VECTOR(from,to) {\
to[0] = from[0];\
to[1] = from[1];\
to[2] = from[2];\
}
#define CALC_DETERMINANT(m) (((m[0][0]) * ((m[1][1])*(m[2][2]) - (m[1][2])*(m[2][1]))) - \
((m[0][1]) * ((m[1][0])*(m[2][2]) - (m[1][2])*(m[2][0]))) + \
((m[0][2]) * ((m[1][0])*(m[2][1]) - (m[1][1])*(m[2][0]))))

} // end of namespace OpenSim

#endif // __SimmMacros_h__


