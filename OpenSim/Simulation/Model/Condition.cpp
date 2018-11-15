/* -------------------------------------------------------------------------- *
 *                          OpenSim:  Condition.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "Condition.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
//using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Condition::Condition() : Object(),
    _isDisabled(_isDisabledProp.getValueBool()),
    _model(NULL)
{
    setNull();
    setupProperties();
    _isDisabled = false;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Condition::~Condition()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aCondition Condition to be copied.
 */
Condition::Condition(const Condition &aCondition) :
   Object(aCondition),
    _isDisabled(_isDisabledProp.getValueBool()),
    _model(NULL)
{
    setNull();
    setupProperties();
    copyData(aCondition);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Condition to another.
 *
 * @param aCondition Condition to be copied.
 */
void Condition::copyData(const Condition &aCondition)
{
    _isDisabled = aCondition._isDisabled;
    _model = aCondition._model;
}


//_____________________________________________________________________________
/**
 * Set the data members of this Condition to their null values.
 */
void Condition::setNull(void)
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Condition::setupProperties(void)
{
    _isDisabledProp.setName("isDisabled");
    _isDisabledProp.setValue(false);
    _propertySet.append(&_isDisabledProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this Condition.
 */
void Condition::connectConditionToModel(Model& aModel)
{
    _model = &aModel;
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
Condition& Condition::operator=(const Condition &aCondition)
{
    // BASE CLASS
    Object::operator=(aCondition);

    copyData(aCondition);

    return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// DISABLE
//-----------------------------------------------------------------------------

//_____________________________________________________________________________

