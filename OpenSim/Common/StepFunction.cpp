/* -------------------------------------------------------------------------- *
 *                         OpenSim:  StepFunction.cpp                         *
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
#include "StepFunction.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
StepFunction::~StepFunction()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
StepFunction::StepFunction() :
    _startTime(_startTimeProp.getValueDbl()), 
    _endTime(_endTimeProp.getValueDbl()),
    _startValue(_startValueProp.getValueDbl()),
    _endValue(_endValueProp.getValueDbl())

{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 */
StepFunction::StepFunction(double startTime, double endTime, double startValue, double endValue) :
    _startTime(_startTimeProp.getValueDbl()), 
    _endTime(_endTimeProp.getValueDbl()),
    _startValue(_startValueProp.getValueDbl()),
    _endValue(_endValueProp.getValueDbl())

{
    setNull();
    setupProperties();

    setStartTime(startTime);
    setEndTime(endTime);
    setStartValue(startValue);
    setEndValue(endValue);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 * All data members of the specified StepFunction are copied.
 *
 * @param aStepFunction StepFunction object to be copied.
 */
StepFunction::StepFunction(const StepFunction &aStepFunction) :
    Function(aStepFunction),
    _startTime(_startTimeProp.getValueDbl()), 
    _endTime(_endTimeProp.getValueDbl()),
    _startValue(_startValueProp.getValueDbl()),
    _endValue(_endValueProp.getValueDbl())

{
    setNull();
    setupProperties();
    copyData(aStepFunction);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their NULL or default values.
 */
void StepFunction::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void StepFunction::setupProperties()
{
    _startTimeProp.setName("transition_start_time");
    _startTimeProp.setValue(0.99);
    _propertySet.append(&_startTimeProp);

    _endTimeProp.setName("transition_end_time");
    _endTimeProp.setValue(1.01);
    _propertySet.append(&_endTimeProp);

    _startValueProp.setName("start_value");
    _startValueProp.setValue(0.0);
    _propertySet.append(&_startValueProp);

    _endValueProp.setName("end_value");
    _endValueProp.setValue(1.0);
    _propertySet.append(&_endValueProp);

}

//_____________________________________________________________________________
/**
 * Set all member variables equal to the members of another object.
 * Note that this method is private.  It is only meant for copying the data
 * members defined in this class.  It does not, for example, make any changes
 * to data members of base classes.
 *
 * @param aStepFunction StepFunction to be copied.
 */
void StepFunction::copyData(const StepFunction &aStepFunction)
{
    setStartTime(aStepFunction._startTime);
    setEndTime(aStepFunction._endTime);
    setStartValue(aStepFunction._startValue);
    setEndValue(aStepFunction._endValue);
    resetFunction();
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 * Note that data members of the base class are also assigned.
 *
 * @param aStepFunction StepFunction to be copied.
 * @return Reference to this object.
 */
StepFunction& StepFunction::operator=(const StepFunction &aStepFunction)
{
    // BASE CLASS
    Function::operator=(aStepFunction);

    // DATA
    copyData(aStepFunction);

    return(*this);
}


//=============================================================================
// UTILITY
//=============================================================================
SimTK::Function* StepFunction::createSimTKFunction() const 
{
    return new SimTK::Function::Step(_startValue, _endValue, _startTime, _endTime);
}
