/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ForceAdapter.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Eastman, Ajay Seth                                        *
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
#include "ForceAdapter.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
ForceAdapter::ForceAdapter(const Force& force) : _force(&force)
{
}

//-----------------------------------------------------------------------------
// METHODS TO CALCULATE FORCE AND ENERGY
//-----------------------------------------------------------------------------
void ForceAdapter::calcForce(const SimTK::State& state,
    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,SimTK::Vector_<SimTK::Vec3>& particleForces,
    SimTK::Vector& mobilityForces) const
{
    _force->computeForce(state, bodyForces, mobilityForces);
}

SimTK::Real ForceAdapter::calcPotentialEnergy(const SimTK::State& state) const
{
    return _force->computePotentialEnergy(state);
}

bool ForceAdapter::shouldBeParallelized() const {
    return _force->shouldBeParallelized(); 
}