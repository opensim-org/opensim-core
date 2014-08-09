/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ConstraintSet.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include "ConstraintSet.h"
#include <OpenSim/Simulation/SimbodyEngine/Constraint.h>

using namespace std;
using namespace OpenSim;


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ConstraintSet::~ConstraintSet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a ConstraintSet.
 */
ConstraintSet::ConstraintSet()
{
    setNull();
}

ConstraintSet::ConstraintSet(Model& model) : Super(model)
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a ConstraintSet.
 */
ConstraintSet::ConstraintSet(const ConstraintSet& aAbsConstraintSet)
    :   Super(aAbsConstraintSet)
{
    *this = aAbsConstraintSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this ConstraintSet to their null values.
 */
void ConstraintSet::setNull()
{
    setAuthors("Ajay Seth");
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Anything that will be applied to all Constraints
 */

/**
 * Scale contraint set by a set of scale factors
 */
void ConstraintSet::scale(const ScaleSet& aScaleSet)
{
    for(int i=0; i<getSize(); i++) get(i).scale(aScaleSet);
}
