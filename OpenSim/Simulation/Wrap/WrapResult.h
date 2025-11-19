#ifndef OPENSIM_WRAP_RESULT_H
#define OPENSIM_WRAP_RESULT_H
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  WrapResult.h                           *
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


// INCLUDE
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Array.h>
#include "SimTKcommon/SmallMatrix.h"

namespace OpenSim {

/** @cond **/ // hide from Doxygen

//=============================================================================
//=============================================================================
/**
 * A class for holding the results of a wrapping calculation.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API WrapResult
{

//=============================================================================
// DATA
//=============================================================================
public:
    int startPoint;            // first point in muscle line that is wrapped
    int endPoint;              // second point in muscle line that is wrapped
    Array<SimTK::Vec3> wrap_pts; // array of wrapping path points
    double wrap_path_length;   // distance along curved r1->r2 path
    SimTK::Vec3 r1;              // wrap tangent point nearest to p1
    SimTK::Vec3 r2;              // wrap tangent point nearest to p2
    SimTK::Vec3 c1;              // intermediate point used by some wrap objects
    SimTK::Vec3 sv;              // intermediate point used by some wrap objects
    // TODO(chrisdembia): This member variable is not copied by the copy
    // constructor or copy assignment operator, so I've initialized it to NaN
    // so we can more easily detect any bugs caused by not copying this
    // variable.
    double factor = SimTK::NaN;  // scale factor used to normalize parameters
    bool singleWrap = false; //Flag to indicate if a single wrap object to be considered so we can optimize
    //=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    WrapResult();
    virtual ~WrapResult() = default;
    WrapResult(const WrapResult& other);
    WrapResult& operator=(const WrapResult& aWrapResult);

//=============================================================================
};  // END of class WrapResult
//=============================================================================
//=============================================================================

/** @endcond **/

} // end of namespace OpenSim

#endif // OPENSIM_WRAP_RESULT_H


