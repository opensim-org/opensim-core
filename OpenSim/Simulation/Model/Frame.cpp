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
#include "Frame.h"
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
//using namespace SimTK;
using namespace OpenSim;
using SimTK::Mat33;
using SimTK::Vec3;

const SimTK::Transform OpenSim::Frame::identityTransform;
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

//_____________________________________________________________________________
/**
 * Construct properties.
 */
void Frame::constructProperties(){

	//constructProperty_last_name("Snow");
}
//_____________________________________________________________________________
/**
* Set a null frame as Identity rotation, 0 translation
*/
void Frame::setNull()
{
	setAuthors("Matt DeMers");
}


//=============================================================================
// FRAME COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
const SimTK::Transform Frame::calcTransformToOtherFrame(const SimTK::State &state, Frame &frame) const
{
	SimTK::Transform ground_X_me = calcTransformToGround(state);
	SimTK::Transform other_X_ground = frame.calcTransformFromGround(state);
	return other_X_ground*ground_X_me;
}
const SimTK::Transform Frame::calcTransformFromOtherFrame(const SimTK::State &state, Frame &frame) const
{

	SimTK::Transform other_X_me = calcTransformToOtherFrame(state, frame);
	return ~other_X_me;
}
const SimTK::Vec3& Frame::expressVectorInAnotherFrame(const SimTK::State &state, SimTK::Vec3 &vec, Frame &frame) const
{
	SimTK::Transform other_X_me = calcTransformToOtherFrame(state, frame);
	return other_X_me.R()*vec;
}
const SimTK::Vec3& Frame::expressVectorFromAnotherFrame(const SimTK::State &state, SimTK::Vec3 &vec, Frame &frame) const
{
	SimTK::Transform me_X_other = calcTransformFromOtherFrame(state, frame);
	return me_X_other.R()*vec;
}
const SimTK::Vec3& Frame::expressPointInAnotherFrame(const SimTK::State &state, SimTK::Vec3 &point, Frame &frame) const
{
	SimTK::Transform other_X_me = calcTransformToOtherFrame(state, frame);
	return other_X_me*point;
}
const SimTK::Vec3& Frame::expressPointFromAnotherFrame(const SimTK::State &state, SimTK::Vec3 &point, Frame &frame) const
{
	SimTK::Transform me_X_other = calcTransformFromOtherFrame(state, frame);
	return me_X_other*point;
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



