/* -------------------------------------------------------------------------- *
 *                           OpenSim:  BodySet.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

#include "BodySet.h"
#include <OpenSim/Common/ScaleSet.h>

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
BodySet::~BodySet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a BodySet.
 */
BodySet::BodySet()
{
	setNull();
}

BodySet::BodySet(Model& model) : Super(model)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a BodySet.
 */
BodySet::BodySet(const BodySet& aAbsBodySet) : Super(aAbsBodySet)
{
	setNull();
	*this = aAbsBodySet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this BodySet to their null values.
 */
void BodySet::setNull()
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
BodySet& BodySet::operator=(const BodySet &aAbsBodySet)
{
	Set<Body>::operator=(aAbsBodySet);
	return (*this);
}
#endif
//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale body set by a set of scale factors
 */
void BodySet::scale(const ScaleSet& aScaleSet, bool aScaleMass)
{
	for(int i=0; i<getSize(); i++) {
		for(int j=0; j<aScaleSet.getSize(); j++) {
			Scale& scale = aScaleSet.get(j);
			if (get(i).getName() == scale.getSegmentName()) {
				Vec3 scaleFactors(1.0);
				scale.getScaleFactors(scaleFactors);
				get(i).scale(scaleFactors, aScaleMass);
			}
		}
	}
}
