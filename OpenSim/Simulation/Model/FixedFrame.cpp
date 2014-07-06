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
FixedFrame::FixedFrame() : Frame()
{
	setNull();
	constructInfrastructure();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
FixedFrame::FixedFrame(Frame& parent_frame) : Frame()
{
	setNull();
	constructInfrastructure();
	setParentFrame(parent_frame);
	
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
}

void FixedFrame::constructStructuralConnectors()
{
	constructStructuralConnector<Frame>("parent_frame");
}


//=============================================================================
// FRAME COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
const SimTK::Transform& FixedFrame::getTransform() const
{
	return transform;
}
void FixedFrame::setTransform(SimTK::Transform& xform)
{
	transform = xform;
}
const SimTK::Transform FixedFrame::calcTransformToGround(const SimTK::State &state) const
{


	return transform*getParentFrame().calcTransformToGround(state);

}
const SimTK::Transform FixedFrame::calcTransformFromGround(const SimTK::State &state) const
{
	return ~calcTransformToGround(state);
}
//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
void FixedFrame::setParentFrame(Frame& parent_frame) 
{ 
	updConnector<Frame>("parent_frame").connect(parent_frame); 
}
const Frame& FixedFrame::getParentFrame() const { return getConnector<Frame>("parent_frame").getConnectee(); }




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



