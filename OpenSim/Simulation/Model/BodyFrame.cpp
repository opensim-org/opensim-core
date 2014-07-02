/* -------------------------------------------------------------------------- *
 *                             OpenSim:  BodyFrame.cpp                             *
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
#include "BodyFrame.h"
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
BodyFrame::BodyFrame() : Frame()
{
	setNull();
	constructInfrastructure();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
BodyFrame::BodyFrame(Body& body) :
   Frame()
{
	setNull();
	constructInfrastructure();
	
}
//_____________________________________________________________________________
/**
* Set a null frame as Identity rotation, 0 translation
*/
void BodyFrame::setNull()
{
	setAuthors("Matt DeMers");
}


void BodyFrame::constructStructuralConnectors()
{
	constructStructuralConnector<Body>("body");
}


//=============================================================================
// FRAME COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
const SimTK::Transform& BodyFrame::getTransform() const
{
	return identityTransform; 
}
const SimTK::Transform BodyFrame::calcTransformFromGround(const SimTK::State &state) const
{
	const Body& myBody = getConnector<Body>("body").getConnectee();
	const SimTK::MobilizedBodyIndex mbi = myBody.getIndex();

	const SimTK::MobilizedBody &G = getModel().getMatterSubsystem().getGround();
	const SimTK::MobilizedBody &B = getModel().getMatterSubsystem().getMobilizedBody(mbi);
	const SimTK::Transform& ground_X_B = B.getBodyTransform(state);
	
	return ~ground_X_B;
}
//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________





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



