/* -------------------------------------------------------------------------- *
 *                  OpenSim:  FunctionThresholdCondition.cpp                  *
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
#include "FunctionThresholdCondition.h"
#include <OpenSim/Common/Function.h>


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
FunctionThresholdCondition::FunctionThresholdCondition() :
    Condition(),
    _function(_functionProp.getValueObjPtrRef()),
    _threshold( _thresholdProp.getValueDbl())
{
    setNull();
    setupProperties();
    _isDisabled = false;
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
FunctionThresholdCondition::~FunctionThresholdCondition()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aCondition FunctionThresholdCondition to be copied.
 */
FunctionThresholdCondition::FunctionThresholdCondition(const FunctionThresholdCondition &aCondition) :
    Condition(aCondition),
    _function(_functionProp.getValueObjPtrRef()),
    _threshold( _thresholdProp.getValueDbl())
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
 * Copy data members from one FunctionThresholdCondition to another.
 *
 * @param aCondition FunctionThresholdCondition to be copied.
 */
void FunctionThresholdCondition::copyData(const FunctionThresholdCondition &aCondition)
{
    Condition::copyData(aCondition);
    _function = aCondition._function;
    _threshold = aCondition._threshold;
}


//_____________________________________________________________________________
/**
 * Set the data members of this Condition to their null values.
 */
void FunctionThresholdCondition::setNull(void)
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void FunctionThresholdCondition::setupProperties(void)
{
    // Condition Function
    _functionProp.setName("condition_function");
    _propertySet.append(&_functionProp);

    //Threshold
    _thresholdProp.setName("threshold");
    _propertySet.append(&_thresholdProp);


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
FunctionThresholdCondition& FunctionThresholdCondition::operator=(const FunctionThresholdCondition &aCondition)
{
    // BASE CLASS
    Condition::operator=(aCondition);

    copyData(aCondition);

    return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________


//=============================================================================
// EVALUATE
//=============================================================================
//-----------------------------------------------------------------------------
// Condition
//-----------------------------------------------------------------------------
bool FunctionThresholdCondition::calcCondition(const SimTK::State& s) const
{
    return (_function->calcValue(SimTK::Vector(1, s.getTime())) > _threshold);
}

//_____________________________________________________________________________

