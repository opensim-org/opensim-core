/* -------------------------------------------------------------------------- *
 *                           OpenSim:  FrameSet.cpp                           *
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

#include "FrameSet.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ComponentSet.h>

using namespace std;
using namespace OpenSim;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
FrameSet::~FrameSet(void)
{
}

//_____________________________________________________________________________
/**
 * Default constructor of a FrameSet.
 */
FrameSet::FrameSet()
{
	setNull();
}

FrameSet::FrameSet(Model& model) :
	ModelComponentSet<Frame>(model)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor of a FrameSet.
 */
FrameSet::FrameSet(const FrameSet& aFrameSet):
	ModelComponentSet<Frame>(aFrameSet)
{
	setNull();
	*this = aFrameSet;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/**
 * Set the data members of this FrameSet to their null values.
 */
void FrameSet::setNull()
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
FrameSet& FrameSet::operator=(const FrameSet &aFrameSet)
{
	Set<Frame>::operator=(aFrameSet);
	return (*this);
}
#endif

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale Frame set by a set of scale factors
 *
void FrameSet::scale(const ScaleSet& aScaleSet)
{
	for(int i=0; i<getSize(); i++) get(i).scale(aScaleSet);
}
*/