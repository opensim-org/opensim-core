// FunctionThresholdCondition.cpp
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
#include "FunctionThresholdCondition.h"
#include <OpenSim/Simulation/Model/Model.h>
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

