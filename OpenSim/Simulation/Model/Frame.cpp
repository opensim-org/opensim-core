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
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Frame::Frame() : ModelComponent()
{
    setAuthors("Matt DeMers, Ajay Seth");
}


void Frame::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    SimTK::Transform x;
    SpatialVec v;
    // If the properties, topology or coordinate values change, 
    // Stage::Position and above will be invalid.
    addCacheVariable("transform_in_g", x, SimTK::Stage::Position);
    // if a speed (u) changes then Stage::Velocity will also be invalid
    addCacheVariable("velocity_in_g", v, SimTK::Stage::Velocity);
    // if a force changes then Stage::Acceleration will also be invalid
    addCacheVariable("acceleration_in_g", v, SimTK::Stage::Acceleration);
}

const SimTK::Transform& Frame::getTransformInGround(const State& s) const
{
    if (!getSystem().getDefaultSubsystem().
            isCacheValueRealized(s, _transformIndex)){
        //cache is not valid so calculate the transform
        SimTK::Value<SimTK::Transform>::downcast(
            getSystem().getDefaultSubsystem().updCacheEntry(s, _transformIndex))
            .upd() = calcTransformInGround(s);
        // mark cache as up-to-date
        getSystem().getDefaultSubsystem().
            markCacheValueRealized(s, _transformIndex);
    }
    return SimTK::Value<SimTK::Transform>::downcast(
        getSystem().getDefaultSubsystem().
            getCacheEntry(s, _transformIndex)).get();
}

const SimTK::SpatialVec& Frame::getVelocityInGround(const State& s) const
{
    if (!getSystem().getDefaultSubsystem().
        isCacheValueRealized(s, _velocityIndex)) {
        //cache is not valid so calculate the transform
        SimTK::Value<SpatialVec>::downcast(
            getSystem().getDefaultSubsystem().
            updCacheEntry(s, _velocityIndex)).upd()
            = calcVelocityInGround(s);
        // mark cache as up-to-date
        getSystem().getDefaultSubsystem().
            markCacheValueRealized(s, _velocityIndex);
    }
    return SimTK::Value<SpatialVec>::downcast(
        getSystem().getDefaultSubsystem().
        getCacheEntry(s, _velocityIndex)).get();
}

const SimTK::SpatialVec& Frame::getAccelerationInGround(const State& s) const
{
    if (!getSystem().getDefaultSubsystem().
        isCacheValueRealized(s, _accelerationIndex)) {
        //cache is not valid so calculate the transform
        SimTK::Value<SpatialVec>::downcast(
            getSystem().getDefaultSubsystem().
            updCacheEntry(s, _accelerationIndex)).upd()
            = calcAccelerationInGround(s);
        // mark cache as up-to-date
        getSystem().getDefaultSubsystem().
            markCacheValueRealized(s, _accelerationIndex);
    }
    return SimTK::Value<SpatialVec>::downcast(
        getSystem().getDefaultSubsystem().
        getCacheEntry(s, _accelerationIndex)).get();
}

void Frame::extendAddGeometry(OpenSim::Geometry& geom)
{
    Super::extendAddGeometry(geom);
    geom.setFrame(*this);
}


void Frame::attachMeshGeometry(const std::string& aGeometryFileName, const SimTK::Vec3 scale)
{
    Mesh geom(aGeometryFileName);
    geom.set_scale_factors(scale);
    geom.setFrame(*this);
    addGeometry(geom);
}


void Frame::attachGeometry(const OpenSim::Geometry& geom, const SimTK::Vec3 scale)
{
    SimTK::ClonePtr<Geometry> clone = SimTK::ClonePtr<Geometry>(geom);
    clone->set_scale_factors(scale);
    clone->setFrameName(getName());
    addGeometry(clone.updRef());

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

SimTK::Vec3 Frame::findLocationInAnotherFrame(const SimTK::State& state, const
        SimTK::Vec3& point, const Frame& otherFrame) const
{
    return findTransformBetween(state, otherFrame)*point;
}

const Frame& Frame::findBaseFrame() const
{
    return extendFindBaseFrame();
}

SimTK::Transform Frame::findTransformInBaseFrame() const
{
    return extendFindTransformInBaseFrame();
}

void Frame::extendRealizeTopology(SimTK::State& s) const
{
    Super::extendRealizeTopology(s);

    const_cast<Self*>(this)->_transformIndex =
        getCacheVariableIndex("transform_in_g");

    const_cast<Self*>(this)->_velocityIndex =
        getCacheVariableIndex("velocity_in_g");

    const_cast<Self*>(this)->_accelerationIndex =
        getCacheVariableIndex("acceleration_in_g");
}
