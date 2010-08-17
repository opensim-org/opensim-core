// StepFunction.cpp
// Author: Ajay Seth
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "StepFunction.h"
#include <OpenSim/Common/FunctionAdapter.h>

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
	_startTime(_startTimeProp.getValueDbl()), 
	_endTime(_endTimeProp.getValueDbl()),
	_startValue(_startValueProp.getValueDbl()),
	_endValue(_endValueProp.getValueDbl())

{
	setNull();
	setupProperties();
	copyData(aStepFunction);
}

//_____________________________________________________________________________
/**
 * Copy this object.
 *
 * @return Pointer to a copy of this object.
 */
Object* StepFunction::copy() const
{
	StepFunction *aStepFunction = new StepFunction(*this);
	return(aStepFunction);
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
	setType("StepFunction");
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
