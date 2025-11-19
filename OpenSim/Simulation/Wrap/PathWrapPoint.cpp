/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PathWrapPoint.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2022 Stanford University and the Authors                *
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

#include "PathWrapPoint.h"

void OpenSim::PathWrapPoint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    _wrapPath = addCacheVariable("wrap_path",
            Array<SimTK::Vec3>(SimTK::Vec3(SimTK::NaN), 0, 1),
            SimTK::Stage::Position);
    _wrapPathLength = addCacheVariable("wrap_path_length", 0.0, SimTK::Stage::Position);
    _location = addCacheVariable("wrap_location", SimTK::Vec3(SimTK::NaN), SimTK::Stage::Position);
}

const OpenSim::WrapObject* OpenSim::PathWrapPoint::getWrapObject() const
{
    return _wrapObject.get();
}

void OpenSim::PathWrapPoint::setWrapObject(const WrapObject* wrapObject)
{
    _wrapObject.reset(wrapObject);
}

const OpenSim::Array<SimTK::Vec3>& OpenSim::PathWrapPoint::getWrapPath(const SimTK::State& s) const
{
    return getCacheVariableValue(s, _wrapPath);
}

void OpenSim::PathWrapPoint::setWrapPath(const SimTK::State& s, const Array<SimTK::Vec3>& src) const
{
    updCacheVariableValue(s, _wrapPath) = src;
    markCacheVariableValid(s, _wrapPath);
}

void OpenSim::PathWrapPoint::clearWrapPath(const SimTK::State& s) const
{
    updCacheVariableValue(s, _wrapPath).setSize(0);
    markCacheVariableValid(s, _wrapPath);
}

double OpenSim::PathWrapPoint::getWrapLength(const SimTK::State& s) const
{
    return getCacheVariableValue(s, _wrapPathLength);
}

void OpenSim::PathWrapPoint::setWrapLength(const SimTK::State& s, double aLength) const
{
    updCacheVariableValue(s, _wrapPathLength) = aLength;
    markCacheVariableValid(s, _wrapPathLength);
}

SimTK::Vec3 OpenSim::PathWrapPoint::getLocation(const SimTK::State& s) const
{
    return getCacheVariableValue(s, _location);
}

void OpenSim::PathWrapPoint::setLocation(const SimTK::State& s, const SimTK::Vec3& loc) const
{
    updCacheVariableValue(s, _location) = loc;
    markCacheVariableValid(s, _location);
}

SimTK::Vec3 OpenSim::PathWrapPoint::getdPointdQ(const SimTK::State& s) const
{
    return SimTK::Vec3(0);
}

// these methods were initially adapted from `OpenSim::Station`'s implementation

SimTK::Vec3 OpenSim::PathWrapPoint::calcLocationInGround(const SimTK::State& state) const
{
    return getParentFrame().getTransformInGround(state) * getLocation(state);
}

SimTK::Vec3 OpenSim::PathWrapPoint::calcVelocityInGround(const SimTK::State& state) const
{
    const PhysicalFrame& parent = getParentFrame();

    // compute the local position vector of the station in its reference frame
    // expressed in ground
    SimTK::Vec3 r = parent.getTransformInGround(state).R() * getLocation(state);
    const SimTK::SpatialVec& V_GF = parent.getVelocityInGround(state);

    // The velocity of the station in ground is a function of its frame's
    // linear (vF = V_GF[1]) and angular (omegaF = A_GF[0]) velocity, such that
    // velocity of the station: v = vF + omegaF x r
    return V_GF[1] + V_GF[0] % r;
}

SimTK::Vec3 OpenSim::PathWrapPoint::calcAccelerationInGround(const SimTK::State& state) const
{
    const PhysicalFrame& parent = getParentFrame();

    // The spatial velocity of the reference frame expressed in ground
    const SimTK::SpatialVec& V_GF = parent.getVelocityInGround(state);

    // The spatial acceleration of the reference frame expressed in ground
    const SimTK::SpatialVec& A_GF = parent.getAccelerationInGround(state);

    // compute the local position vector of the point in its reference frame
    // expressed in ground
    SimTK::Vec3 r = parent.getTransformInGround(state).R() * getLocation(state);

    // The acceleration of the station in ground is a function of its frame's
    // linear (aF = A_GF[1]) and angular (alpha = A_GF[0]) accelerations and
    // Coriolis acceleration due to the angular velocity (omega = V_GF[0]) of
    // its frame, such that: a = aF + alphaF x r + omegaF x (omegaF x r)
    return A_GF[1] + A_GF[0]%r +  V_GF[0] % (V_GF[0] % r);
}
