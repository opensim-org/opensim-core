/* -------------------------------------------------------------------------- *
 *                          OpenSim:  BodyScale.cpp                           *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "BodyScale.h"

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
BodyScale::BodyScale() :
    _axisNames(_axisNamesProp.getValueStrArray())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
BodyScale::~BodyScale()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBodyScale BodyScale to be copied.
 */
BodyScale::BodyScale(const BodyScale &aBodyScale) :
   Object(aBodyScale),
    _axisNames(_axisNamesProp.getValueStrArray())
{
    setNull();
    setupProperties();
    copyData(aBodyScale);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one BodyScale to another.
 *
 * @param aBodyScale BodyScale to be copied.
 */
void BodyScale::copyData(const BodyScale &aBodyScale)
{
    _axisNames = aBodyScale._axisNames;
}

//_____________________________________________________________________________
/**
 * Set the data members of this BodyScale to their null values.
 */
void BodyScale::setNull()
{
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void BodyScale::setupProperties()
{
    _axisNamesProp.setComment("Axes (X Y Z) along which to scale a body. "
        "For example, 'X Y Z' scales along all three axes, and 'Y' scales "
        "just along the Y axis.");
    _axisNamesProp.setName("axes");
    _propertySet.append(&_axisNamesProp);
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
BodyScale& BodyScale::operator=(const BodyScale &aBodyScale)
{
    // BASE CLASS
    Object::operator=(aBodyScale);

    copyData(aBodyScale);

    return(*this);
}
