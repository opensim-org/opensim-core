// Condition.cpp
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
#include "Condition.h"
#include <OpenSim/Simulation/Model/Model.h>

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

