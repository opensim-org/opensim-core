/* -------------------------------------------------------------------------- *
 *                     OpenSim:  UnilateralConstraint.cpp                     *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "UnilateralConstraint.h"
#include "SimbodyEngine.h"

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
UnilateralConstraint::UnilateralConstraint() 
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
UnilateralConstraint::~UnilateralConstraint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aUnilateralConstraint UnilateralConstraint to be copied.
 */
UnilateralConstraint::UnilateralConstraint(const UnilateralConstraint &aUnilateralConstraint) :
   Constraint(aUnilateralConstraint)
{
	setNull();
	copyData(aUnilateralConstraint);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one UnilateralConstraint to another.
 *
 * @param aUnilateralConstraint UnilateralConstraint to be copied.
 */
void UnilateralConstraint::copyData(const UnilateralConstraint &aUnilateralConstraint)
{
	Constraint::copyData(aUnilateralConstraint);
}

//_____________________________________________________________________________
/**
 * Set the data members of this UnilateralConstraint to their null values.
 */
void UnilateralConstraint::setNull()
{
	setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param Model containing this UnilateralConstraint.
 */
void UnilateralConstraint::connectToModel(Model& aModel)
{
	// Base class
	Super::connectToModel(aModel);
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
UnilateralConstraint& UnilateralConstraint::operator=(const UnilateralConstraint &aConstraint)
{
	// BASE CLASS
	Constraint::operator=(aConstraint);

	copyData(aConstraint);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
