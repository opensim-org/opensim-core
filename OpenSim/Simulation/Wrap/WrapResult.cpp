/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WrapResult.cpp                          *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "../simulation.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
WrapResult::WrapResult()
{
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
WrapResult::~WrapResult()
{
}

//_____________________________________________________________________________
/**
 * Copy data members from one WrapResult to another.
 *
 * @param aWrapResult WrapResult to be copied.
 */
void WrapResult::copyData(const WrapResult& aWrapResult)
{
    wrap_pts = aWrapResult.wrap_pts;
    wrap_path_length = aWrapResult.wrap_path_length;

    startPoint = aWrapResult.startPoint;
    endPoint = aWrapResult.endPoint;

    int i;
    for (i = 0; i < 3; i++) {
        r1[i] = aWrapResult.r1[i];
        r2[i] = aWrapResult.r2[i];
        c1[i] = aWrapResult.c1[i];
        sv[i] = aWrapResult.sv[i];
    }
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
WrapResult& WrapResult::operator=(const WrapResult& aWrapResult)
{
    copyData(aWrapResult);

    return(*this);
}
