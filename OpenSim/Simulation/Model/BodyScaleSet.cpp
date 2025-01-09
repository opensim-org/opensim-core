/* -------------------------------------------------------------------------- *
 *                         OpenSim:  BodyScaleSet.cpp                         *
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

#include "BodyScaleSet.h"

using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
BodyScaleSet::~BodyScaleSet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a BodyScaleSet.
 */
BodyScaleSet::BodyScaleSet() :
    Set<BodyScale>()
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a BodyScaleSet.
 */
BodyScaleSet::BodyScaleSet(const BodyScaleSet& aBodyScaleSet):
    Set<BodyScale>(aBodyScaleSet)
{
    setNull();
    *this = aBodyScaleSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this BodyScaleSet to their null values.
 */
void BodyScaleSet::setNull()
{
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
#ifndef SWIG
BodyScaleSet& BodyScaleSet::operator=(const BodyScaleSet &aBodyScaleSet)
{
    Set<BodyScale>::operator=(aBodyScaleSet);
    return (*this);
}
#endif
