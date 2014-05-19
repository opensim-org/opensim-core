/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WeldJoint.cpp                           *
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
#include "WeldJoint.h"
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
WeldJoint::~WeldJoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
WeldJoint::WeldJoint() :
	Joint()
{
	setAuthors("Ajay Seth");
	constructCoordinates();
}

//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
WeldJoint::WeldJoint(const std::string &name, const OpenSim::Body &parent,
	const SimTK::Vec3& locationInParent, const SimTK::Vec3& orientationInParent,
	const OpenSim::Body& child,
	const SimTK::Vec3& locationInchild, const SimTK::Vec3& orientationInChild,
	bool reverse) :
		Super(name, parent, locationInParent,orientationInParent,
			          child, locationInchild, orientationInChild, reverse)
{
	setAuthors("Ajay Seth");
	constructCoordinates();
}

//=============================================================================
// CONSTRUCTION
//=============================================================================


//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void WeldJoint::addToSystem(SimTK::MultibodySystem& system) const
{
	createMobilizedBody<MobilizedBody::Weld>(system,
		                                     getParentTransform(),
		                                     getChildTransform());
    // TODO: Joints require super class to be called last.
    Super::addToSystem(system);
}

