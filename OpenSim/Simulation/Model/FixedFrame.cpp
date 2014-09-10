/* -------------------------------------------------------------------------- *
 *                             OpenSim:  FixedFrame.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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
#include "FixedFrame.h"
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
//using namespace SimTK;
using namespace OpenSim;
using SimTK::Mat33;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
FixedFrame::FixedFrame() : RigidFrame()
{
	setNull();
	constructInfrastructure();
}

//_____________________________________________________________________________
/**
 * Constructors.
 */
FixedFrame::FixedFrame(const RigidFrame& parent_frame) : RigidFrame()
{
	setNull();
	constructInfrastructure();
	setParentFrame(parent_frame);
	
}

FixedFrame::FixedFrame(const RigidFrame& parent_frame, const SimTK::Transform& xform) : RigidFrame()
{
	setNull();
	constructInfrastructure();
	setParentFrame(parent_frame);
	setTransform(xform);

}
//_____________________________________________________________________________
/**
* Set a null frame as Identity rotation, 0 translation
*/
void FixedFrame::setNull()
{
	transform.setToZero();
	setAuthors("Matt DeMers");
}
//_____________________________________________________________________________
/**
* construct properties
*/

void FixedFrame::constructProperties()
{
	SimTK::Vec3 zero(0.0, 0.0, 0.0);
	constructProperty_translation(zero);
	constructProperty_orientation(zero);
	// transform at default
}

void FixedFrame::constructStructuralConnectors()
{
	constructStructuralConnector<RigidFrame>("parent_frame");
}


//=============================================================================
// FRAME COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
const SimTK::Transform& FixedFrame::getTransform() const
{
	// If properties have been updated, then update the cached transform object to be in sync.
	transform.updP() = get_translation();
	transform.updR().setRotationToBodyFixedXYZ(get_orientation());
	return transform;
}
void FixedFrame::setTransform(const SimTK::Transform& xform)
{
	transform = xform;
// Make sure properties are updated in case we either call gettters or serialize after this call
	set_translation(xform.p());
	set_orientation(xform.R().convertRotationToBodyFixedXYZ());
}

SimTK::Transform FixedFrame::calcGroundTransform(const SimTK::State &state) const
{
    return getTransform()*getParentFrame().getGroundTransform(state);

}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
void FixedFrame::setParentFrame(const RigidFrame& parent_frame) 
{ 
	updConnector<RigidFrame>("parent_frame").connect(parent_frame); 
}
const RigidFrame& FixedFrame::getParentFrame() const { return getConnector<RigidFrame>("parent_frame").getConnectee(); }


//----------------- Get Body that the FixedFrame is affixed to.
const OpenSim::Body& FixedFrame::getBody() const {
    const RigidFrame& parent = getParentFrame();
    // The following line could've used safeDownCast but that fails for const pointers!
    const OpenSim::Body* parentBody = dynamic_cast<const OpenSim::Body *>(&parent);
    if (parentBody != 0)
        return *parentBody;
    return getParentFrame().getBody();
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________



//=============================================================================
// UTILITY
//=============================================================================



//=============================================================================
// I/O
//=============================================================================



