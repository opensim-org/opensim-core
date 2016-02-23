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
    // If the properties, topology or coordinate values change, 
    // Stage::Position will be invalid.
    addCacheVariable("ground_transform", x, SimTK::Stage::Position);
}

const SimTK::Transform& Frame::getGroundTransform(const SimTK::State& s) const
{
    if (!getSystem().getDefaultSubsystem().
            isCacheValueRealized(s, _groundTransformIndex)){
        //cache is not valid so calculate the transform
        SimTK::Value<SimTK::Transform>::downcast(
            getSystem().getDefaultSubsystem().
            updCacheEntry(s, _groundTransformIndex)).upd()
                = calcGroundTransform(s);
        // mark cache as up-to-date
        getSystem().getDefaultSubsystem().
            markCacheValueRealized(s, _groundTransformIndex);
    }
    return SimTK::Value<SimTK::Transform>::downcast(
        getSystem().getDefaultSubsystem().
            getCacheEntry(s, _groundTransformIndex)).get();
}

void Frame::extendAddGeometry(OpenSim::Geometry& geom) {
    if (geom.getFrameName() == "")
        geom.setFrameName(getName());
}


void Frame::attachMeshGeometry(const std::string& aGeometryFileName, const SimTK::Vec3 scale)
{
    Mesh geom(aGeometryFileName);
    geom.set_scale_factors(scale);
    geom.setFrameName(getName());
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
    SimTK::Transform X_GF = calcGroundTransform(state);
    SimTK::Transform X_GA = otherFrame.calcGroundTransform(state);
    // return the transform, X_AF that expresses quantities in F into A
    return ~X_GA*X_GF;
}

SimTK::Vec3 Frame::expressVectorInAnotherFrame(const SimTK::State& state,
                                const SimTK::Vec3& vec, const Frame& frame) const
{
    SimTK::Transform X_AF = findTransformBetween(state, frame);
    return X_AF.R()*vec;
}

SimTK::Vec3 Frame::findLocationInAnotherFrame(const SimTK::State& state, const
        SimTK::Vec3& point, const Frame& otherFrame) const
{
    SimTK::Transform X_AF = findTransformBetween(state, otherFrame);
    return X_AF*point;
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
    const_cast<Self*>(this)->_groundTransformIndex =
        getCacheVariableIndex("ground_transform");
}
