/* -------------------------------------------------------------------------- *
*                             OpenSim:  RigidFrame.cpp                             *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2012 Stanford University and the Authors                *
* Author(s): Matt DeMers & Ayman Habib                                       *
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
#include "RigidFrame.h"
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
* Default constructor.
*/
RigidFrame::RigidFrame() : Frame()
{
	setNull();

}


void RigidFrame::setNull()
{
	setAuthors("Matt DeMers");
}

const SimTK::MobilizedBody& RigidFrame::getMobilizedBody() const
{
    return getModel().getMatterSubsystem().getMobilizedBody(_index);
}

SimTK::MobilizedBody& RigidFrame::updMobilizedBody() 
{
    return updModel().updMatterSubsystem().updMobilizedBody(_index);
}

/**
* Implementation of Frame interface by RigidFrame
*/
SimTK::Transform RigidFrame::calcGroundTransform(const SimTK::State& state) const {

    const SimTK::MobilizedBody &B = getModel().getMatterSubsystem().getMobilizedBody(_index);
    const SimTK::Transform& X_GB = B.getBodyTransform(state);

    return X_GB*getTransformInMobilizedBody();
}

