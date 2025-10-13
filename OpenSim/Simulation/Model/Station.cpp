/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Station.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Ajay Seth                                          *
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

#include "Station.h"

#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/ModelDisplayHints.h>

using namespace OpenSim;

// here for perf reasons: many functions take a const reference to a
// std::string. Using a C string literal results in millions of temporary
// std::strings being constructed so, instead, pre-allocate it in static
// storage
static const std::string parentFrameKey{"parent_frame"};

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
Station::Station() : Point() {
    setNull();
    constructProperties();
}

Station::Station(const PhysicalFrame& frame, const SimTK::Vec3& location) :
        Point() {
    setNull();
    constructProperties();
    setParentFrame(frame);
    set_location(location);
}

Station::~Station() {}

void Station::setNull() {
    setAuthors("Ayman Habib, Ajay Seth");
}

void Station::constructProperties() {
    constructProperty_location(SimTK::Vec3(0.0));
}

//=============================================================================
// ACCESSORS
//=============================================================================
const PhysicalFrame& Station::getParentFrame() const {
    return getSocket<PhysicalFrame>(parentFrameKey).getConnectee();
}

void Station::setParentFrame(const OpenSim::PhysicalFrame& aFrame) {
    connectSocket_parent_frame(aFrame);
}

SimTK::Vec3 Station::findLocationInFrame(const SimTK::State& s,
        const OpenSim::Frame& aFrame) const {
    return getParentFrame().findStationLocationInAnotherFrame(s, get_location(),
            aFrame);
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void Station::extendScale(const SimTK::State& s, const ScaleSet& scaleSet) {
    Super::extendScale(s, scaleSet);

    // Get scale factors (if an entry for the parent Frame's base Body exists).
    const SimTK::Vec3& scaleFactors =
            getScaleFactors(scaleSet, getParentFrame());
    if (scaleFactors == ModelComponent::InvalidScaleFactors) {
        return;
    }

    upd_location() = get_location().elementwiseMultiply(scaleFactors);
}

//=============================================================================
// COMPONENT INTERFACE
//=============================================================================
void Station::generateDecorations(bool fixed,
        const ModelDisplayHints& hints, const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const {
    Super::generateDecorations(fixed, hints, state, geometry);
    generateDecorationsImpl(fixed, hints, state, geometry);
}

//=============================================================================
// STATION INTERFACE
//=============================================================================
void Station::generateDecorationsImpl(bool fixed,
        const ModelDisplayHints& hints, const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const {
    if (!fixed) { return; }
    if (!hints.get_show_stations()) { return; }

    const OpenSim::PhysicalFrame& frame = getParentFrame();
    geometry.push_back(
        SimTK::DecorativeSphere(.005).setBodyId(frame.getMobilizedBodyIndex())
        .setColor(SimTK::Vec3(0., 1.0, 0.)).setOpacity(1.0)
        .setRepresentation(SimTK::DecorativeGeometry::DrawWireframe)
        .setTransform(frame.findTransformInBaseFrame() * get_location())
        .setScaleFactors(SimTK::Vec3(1.0)));
}

//=============================================================================
// POINT INTERFACE
//=============================================================================
SimTK::Vec3 Station::calcLocationInGround(const SimTK::State& s) const {
    return getParentFrame().getTransformInGround(s)*get_location();
}

SimTK::Vec3 Station::calcVelocityInGround(const SimTK::State& s) const {
    // Compute the local position vector of the Station in its reference frame,
    // expressed in ground.
    SimTK::Vec3 r = getParentFrame().getTransformInGround(s).R()*get_location();
    const SimTK::SpatialVec& V_GF = getParentFrame().getVelocityInGround(s);

    // The velocity of the station in ground is a function of its frame's
    // linear (vF = V_GF[1]) and angular (omegaF = A_GF[0]) velocity, such that
    // velocity of the station: v = vF + omegaF x r
    return V_GF[1] + V_GF[0] % r;
}

SimTK::Vec3 Station::calcAccelerationInGround(const SimTK::State& s) const {
    // The spatial velocity of the reference frame expressed in ground
    const SimTK::SpatialVec& V_GF = getParentFrame().getVelocityInGround(s);
    // The spatial acceleration of the reference frame expressed in ground
    const SimTK::SpatialVec& A_GF = getParentFrame().getAccelerationInGround(s);
    // Compute the local position vector of the point in its reference frame
    // expressed in ground
    SimTK::Vec3 r = getParentFrame().getTransformInGround(s).R()*get_location();

    // The acceleration of the station in ground is a function of its frame's
    // linear (aF = A_GF[1]) and angular (alpha = A_GF[0]) accelerations and
    // Coriolis acceleration due to the angular velocity (omega = V_GF[0]) of
    // its frame, such that: a = aF + alphaF x r + omegaF x (omegaF x r)
    return A_GF[1] + A_GF[0]%r +  V_GF[0] % (V_GF[0] % r) ;
}
