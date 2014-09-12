/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Frame.cpp                             *
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
#include "Frame.h"
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
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
Frame::Frame() : ModelComponent()
{
	setNull();
	
}


void Frame::setNull()
{
	setAuthors("Matt DeMers");
}


//=============================================================================
// FRAME COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
SimTK::Transform Frame::calcTransformToOtherFrame(const SimTK::State& state, const Frame& otherFrame) const
{
    SimTK::Transform ground_X_me = calcGroundTransform(state);
    SimTK::Transform ground_X_other = otherFrame.calcGroundTransform(state);
	return ~ground_X_other*ground_X_me;
}

SimTK::Vec3 Frame::expressVectorInAnotherFrame(const SimTK::State& state, const SimTK::Vec3& vec, const Frame& frame) const
{
	SimTK::Transform other_X_me = calcTransformToOtherFrame(state, frame);
	return other_X_me.R()*vec;
}

SimTK::Vec3 Frame::expressPointInAnotherFrame(const SimTK::State& state, const SimTK::Vec3& point, const Frame& otherFrame) const
{
	SimTK::Transform other_X_me = calcTransformToOtherFrame(state, otherFrame);
	return other_X_me*point;
}

