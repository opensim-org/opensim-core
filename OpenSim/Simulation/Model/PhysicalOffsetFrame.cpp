/* -------------------------------------------------------------------------- *
 *                  OpenSim:  PhysicalOffsetFrame.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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
#include "PhysicalOffsetFrame.h"

using namespace OpenSim;

void PhysicalOffsetFrame::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    setMobilizedBodyIndex(getParentFrame().getMobilizedBodyIndex());
}

SimTK::Transform PhysicalOffsetFrame::
calcTransformInGround(const SimTK::State& state) const
{
    return getParentFrame().getTransformInGround(state)*getOffsetTransform();
}

SimTK::SpatialVec PhysicalOffsetFrame::
calcVelocityInGround(const SimTK::State& state) const
{
    // The rigid offset of the OffsetFrame expressed in ground
    const SimTK::Vec3& r = getParentFrame().getTransformInGround(state).R()*
        getOffsetTransform().p();
    // Velocity of the base frame in ground
    SimTK::SpatialVec V_G = getParentFrame().getVelocityInGround(state);
    // translational velocity needs additional omega x r term due to offset 
    V_G(1) += V_G(0) % r;

    return V_G;
}

SimTK::SpatialVec PhysicalOffsetFrame::
calcAccelerationInGround(const SimTK::State& state) const
{
    // The rigid offset of the OffsetFrame expressed in ground
    const SimTK::Vec3& r = getParentFrame().getTransformInGround(state).R()*
        getOffsetTransform().p();
    // Velocity of the parent frame in ground
    const SimTK::SpatialVec& V_G = getParentFrame().getVelocityInGround(state);
    // Velocity of the parent frame in ground
    SimTK::SpatialVec A_G = getParentFrame().getAccelerationInGround(state);
    A_G[1] += (A_G[0] % r + V_G[0] % (V_G[0] % r));

    return A_G;
}
