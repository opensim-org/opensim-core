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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#define MAX(a,b) (std::max(a, b))
#define MIN(a,b) (std::min(a, b))
#define DABS(a) (std::abs(a))
#define DSIGN(a) ((a)>=0.0?(1):(-1))
#define SQR(x)  ((x) * (x))
#define EQUAL_WITHIN_ERROR(a,b) (DABS(((a)-(b))) <= ROUNDOFF_ERROR)
#define NOT_EQUAL_WITHIN_ERROR(a,b) (DABS(((a)-(b))) > ROUNDOFF_ERROR)
#define EQUAL_WITHIN_TOLERANCE(a,b,c) (DABS(((a)-(b))) <= (c))

} // end of namespace OpenSim

#endif // __SimmMacros_h__


