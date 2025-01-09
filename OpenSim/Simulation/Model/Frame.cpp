/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Frame.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Common/ScaleSet.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Mat33;
using SimTK::Vec3;
using SimTK::State;
using SimTK::SpatialVec;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
Frame::Frame() : ModelComponent()
{
    setAuthors("Matt DeMers, Ajay Seth");

    FrameGeometry default_frame_geometry;
    default_frame_geometry.setName("frame_geometry");
    constructProperty_frame_geometry(default_frame_geometry);

    constructProperty_attached_geometry();
}

void Frame::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);
    // All the Geometry attached to this Frame should have
    // their frame connections automatically set to this Frame.
    upd_frame_geometry().setFrame(*this);
    int nag = getProperty_attached_geometry().size();
    for (int i = 0; i < nag; ++i) {
        upd_attached_geometry(i).setFrame(*this);
    }
}

void Frame::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    // If the properties, topology or coordinate values change, 
    // Stage::Position and above will be invalid.
    this->_transformCV = addCacheVariable("transform_in_g", SimTK::Transform{}, SimTK::Stage::Position);
    // if a speed (u) changes then Stage::Velocity will also be invalid
    this->_velocityCV = addCacheVariable("velocity_in_g", SpatialVec{}, SimTK::Stage::Velocity);
    // if a force changes then Stage::Acceleration will also be invalid
    this->_accelerationCV = addCacheVariable("acceleration_in_g", SpatialVec{}, SimTK::Stage::Acceleration);
}

const SimTK::Transform& Frame::getTransformInGround(const State& s) const
{
    if (isCacheVariableValid(s, _transformCV)) {
        return getCacheVariableValue(s, _transformCV);
    }

    SimTK::Transform& t = updCacheVariableValue(s, _transformCV);
    t = calcTransformInGround(s);
    markCacheVariableValid(s, _transformCV);

    return t;
}

const SimTK::SpatialVec& Frame::getVelocityInGround(const State& s) const
{
    if (isCacheVariableValid(s, _velocityCV)) {
        return getCacheVariableValue(s, _velocityCV);
    }

    SimTK::SpatialVec& v = updCacheVariableValue(s, _velocityCV);
    v = calcVelocityInGround(s);
    markCacheVariableValid(s, _velocityCV);

    return v;
}

const SimTK::Vec3& Frame::getAngularVelocityInGround(const State& s) const
{
    return getVelocityInGround(s)[0];
}

const SimTK::Vec3& Frame::getLinearVelocityInGround(const State& s) const
{
    return getVelocityInGround(s)[1];
}

const SimTK::SpatialVec& Frame::getAccelerationInGround(const State& s) const
{
    if (isCacheVariableValid(s, _accelerationCV)) {
        return getCacheVariableValue(s, _accelerationCV);
    }

    SimTK::SpatialVec& acceleration = updCacheVariableValue(s, _accelerationCV);
    acceleration = calcAccelerationInGround(s);
    markCacheVariableValid(s, _accelerationCV);
    return acceleration;
}

const SimTK::Vec3& Frame::getAngularAccelerationInGround(const State& s) const
{
    return getAccelerationInGround(s)[0];
}

const SimTK::Vec3& Frame::getLinearAccelerationInGround(const State& s) const
{
    return getAccelerationInGround(s)[1];
}

void Frame::attachGeometry(OpenSim::Geometry* geom)
{
    // Check that name exists and is unique as it's used to form PathName
    if (geom->getName().empty()) {
        bool nameFound = false;
        int index = 1;
        while (!nameFound) {
            std::stringstream ss;
            // generate candidate name
            ss << getName() << "_geom_" << index;
            std::string candidate = ss.str();
            bool exists = false;
            for (int idx = 0; 
                idx < getProperty_attached_geometry().size() && !exists; idx++) {
                if (get_attached_geometry(idx).getName() == candidate) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                nameFound = true;
                geom->setName(candidate);
            }
            else
                index++;
        }
    }

    geom->setFrame(*this);
    updProperty_attached_geometry().adoptAndAppendValue(geom);
    finalizeFromProperties();
    prependComponentPathToConnecteePath(*geom);
}

void Frame::scaleAttachedGeometry(const SimTK::Vec3& scaleFactors)
{
    for (int i = 0; i < getProperty_attached_geometry().size(); ++i) {
        Geometry& geo = upd_attached_geometry(i);
        geo.upd_scale_factors() = geo.get_scale_factors()
                                  .elementwiseMultiply(scaleFactors);
    }
}

void Frame::extendScale(const SimTK::State& s, const ScaleSet& scaleSet)
{
    Super::extendScale(s, scaleSet);

    if (getProperty_attached_geometry().size() == 0)
        return;

    // Get scale factors (if an entry for the base Body exists).
    const Vec3& scaleFactors = getScaleFactors(scaleSet, *this);
    if (scaleFactors == ModelComponent::InvalidScaleFactors)
        return;

    scaleAttachedGeometry(scaleFactors);
}

//=============================================================================
// FRAME COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
SimTK::Transform Frame::findTransformBetween(const SimTK::State& state,
        const Frame& otherFrame) const
{
    const SimTK::Transform& X_GF = getTransformInGround(state);
    const SimTK::Transform& X_GA = otherFrame.getTransformInGround(state);
    // return the transform, X_AF that expresses quantities in F into A
    return ~X_GA*X_GF;
}

SimTK::Vec3 Frame::expressVectorInAnotherFrame(const SimTK::State& state,
                                const SimTK::Vec3& vec, const Frame& frame) const
{
    return findTransformBetween(state, frame).R()*vec;
}

SimTK::Vec3 Frame::expressVectorInGround(const SimTK::State& state,
                                const SimTK::Vec3& vec_F) const
{
    return getTransformInGround(state).R()*vec_F;
}

SimTK::Vec3 Frame::findStationLocationInAnotherFrame(const SimTK::State& state,
        const SimTK::Vec3& station_F, const Frame& otherFrame) const
{
    return findTransformBetween(state, otherFrame)*station_F;
}

SimTK::Vec3 Frame::findStationLocationInGround(const SimTK::State& state,
        const SimTK::Vec3& station_F) const
{
    return getTransformInGround(state)*station_F;
}

SimTK::Vec3 Frame::findStationVelocityInGround(const SimTK::State& state,
    const SimTK::Vec3& station_F) const
{
    const SimTK::SpatialVec& V_GF = getVelocityInGround(state);
    SimTK::Vec3 r_G = expressVectorInGround(state, station_F);
    return V_GF[1] + SimTK::cross(V_GF[0], r_G);
}

SimTK::Vec3 Frame::findStationAccelerationInGround(const SimTK::State& state,
    const SimTK::Vec3& station_F) const
{
    const SimTK::SpatialVec& V_GF = getVelocityInGround(state);
    const SimTK::SpatialVec& A_GF = getAccelerationInGround(state);
    SimTK::Vec3 r_G = expressVectorInGround(state, station_F);
    return A_GF[1] + SimTK::cross(A_GF[0], r_G) +
        SimTK::cross(V_GF[0], SimTK::cross(V_GF[0], r_G));
}

const Frame& Frame::findBaseFrame() const
{
    return extendFindBaseFrame();
}

SimTK::Transform Frame::findTransformInBaseFrame() const
{
    return extendFindTransformInBaseFrame();
}
