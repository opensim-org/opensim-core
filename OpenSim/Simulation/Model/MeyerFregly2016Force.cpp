/* -------------------------------------------------------------------------- *
 *                    OpenSim:  MeyerFregly2016Force.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco, Christopher Dembia, Spencer Williams           *
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

#include "MeyerFregly2016Force.h"

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//=============================================================================
// CONSTANTS
//=============================================================================
constexpr static SimTK::Real klow = 1e-1;
constexpr static SimTK::Real h = 1e-3;
constexpr static SimTK::Real c = 1e-3;
constexpr static SimTK::Real ymax = 1e-2;
// Prevents dividing by zero when sliding velocity is zero.
constexpr static SimTK::Real slipOffset = 1e-4;

//=============================================================================
// CONSTRUCTION
//=============================================================================
MeyerFregly2016Force::MeyerFregly2016Force() {
    constructProperties();
}

void MeyerFregly2016Force::constructProperties() {
    constructProperty_stiffness(1.0e4);
    constructProperty_dissipation(1.0e-2);
    constructProperty_spring_resting_length(0);
    constructProperty_dynamic_friction(0);
    constructProperty_viscous_friction(5.0);
    constructProperty_latch_velocity(0.05);
    constructProperty_force_visualization_scale_factor();
}

//=============================================================================
// GET AND SET
//=============================================================================
SimTK::Vec3 MeyerFregly2016Force::getContactForceOnStation(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, _forceOnStationCV)) {
        setContactForceOnStation(s, calcContactForceOnStation(s));
    }
    return getCacheVariableValue<SimTK::Vec3>(s, _forceOnStationCV);
}

void MeyerFregly2016Force::setContactForceOnStation(const SimTK::State& s,
        const SimTK::Vec3& force) const {
    setCacheVariableValue(s, _forceOnStationCV, force);
}

//=============================================================================
// FORCE INTERFACE
//=============================================================================
OpenSim::Array<std::string> MeyerFregly2016Force::getRecordLabels() const {
    OpenSim::Array<std::string> labels;
    const auto stationName = getConnectee("station").getName();
    labels.append(getName() + "." + stationName + ".force.X");
    labels.append(getName() + "." + stationName + ".force.Y");
    labels.append(getName() + "." + stationName + ".force.Z");
    return labels;
}

OpenSim::Array<double> MeyerFregly2016Force::getRecordValues(
        const SimTK::State& s) const {
    OpenSim::Array<double> values;
    const SimTK::Vec3& force = getContactForceOnStation(s);
    values.append(force[0]);
    values.append(force[1]);
    values.append(force[2]);
    return values;
}

SimTK::Vec3 MeyerFregly2016Force::calcContactForceOnStation(
        const SimTK::State& state) const {
    SimTK::Vec3 force(0);
    const auto& station = getConnectee<Station>("station");
    const auto& pos = station.getLocationInGround(state);
    const auto& vel = station.getVelocityInGround(state);

    // Limit maximum height used in force calculation.
    SimTK::Real y = pos[1] - get_spring_resting_length();

    // When the height variable reaches a critical value of 0.70947586,
    // the calculated vertical ground reaction force goes to fininity. Thus,
    // it is essential to keep the height variable below this value.
    // Originally, this goal was achieved with the following line of code:
    //
    // y = min(y, 0.70947586);
    //
    // However, allowing the height variable to reach 0.70947586 still
    // causes a singularity in the calculated vertical ground reaction
    // force. Backing off this critical value to 0.7 and changing the line
    // of code above to the following produces faster Tracking Optimization
    // convergence:
    //
    // y = min(y, 0.7);
    //
    // However, this "fix" is still non-smooth, which negatively impacts
    // convergence. As an alternative, the min function can be replaced with the
    // following tanh function to create a smooth height variable:
    //
    // y = 0.7*tanh((1/0.7)*y);
    //
    // This implementation causes the new height to be 0 when the original
    // height is zero, the new height to be essentially linear with the
    // original height when the original height is less than zero (i.e., in
    // contact situations), and the new height to follow a tanh function
    // that approaches 0.7 as the original height increases beyond 0.7
    // (i.e., out of contact situations). This tanh formulation has been
    // proven to speed up convergence substantially.
    y = 0.7*std::tanh((1.0/0.7)*y);

    // Stiffness constants.
    const SimTK::Real k = get_stiffness();
    const SimTK::Real v = (k + klow) / (k - klow);
    const SimTK::Real s = (k - klow) / 2.0;
    const SimTK::Real constant =
            -s * (v * ymax - c * log(cosh((ymax + h) / c)));

    // Vertical spring-damper force.
    const SimTK::Real Fspring =
            -s * (v * y - c * log(cosh((y + h) / c))) - constant;
    force[1] = Fspring * (1 - get_dissipation() * vel[1]);

    // Friction force.
    const SimTK::Real slidingVelocity =
            sqrt(vel[0] * vel[0] + vel[2] * vel[2]);
    const SimTK::Real horizontalForce = force[1] * (
            get_dynamic_friction() *
            tanh(slidingVelocity / get_latch_velocity()) +
            get_viscous_friction() * slidingVelocity
    );

    force[0] = -vel[0] / (slidingVelocity + slipOffset) * horizontalForce;
    force[2] = -vel[2] / (slidingVelocity + slipOffset) * horizontalForce;

    return force;
}

//=============================================================================
// COMPONENT INTERFACE
//=============================================================================
void MeyerFregly2016Force::generateDecorations(bool fixed,
        const ModelDisplayHints& hints, const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const {
    Super::generateDecorations(fixed, hints, s, geoms);
    const auto& station = getConnectee<Station>("station");

    // Station visualization.
    SimTK::DecorativeSphere sphere;
    sphere.setColor(SimTK::Green);
    sphere.setRadius(0.01);
    sphere.setBodyId(station.getParentFrame().getMobilizedBodyIndex());
    sphere.setRepresentation(SimTK::DecorativeGeometry::DrawWireframe);
    sphere.setTransform(SimTK::Transform(station.get_location()));
    geoms.push_back(sphere);

    if (fixed) return;

    // Contact force visualization.
    getModel().realizeVelocity(s);
    const auto point1 = station.getLocationInGround(s);
    const SimTK::Vec3& force = getContactForceOnStation(s);
    const SimTK::Vec3 point2 = point1 + force * m_forceVizScaleFactor;
    SimTK::DecorativeLine line(point1, point2);
    line.setColor(SimTK::Green);
    line.setLineThickness(1.0);
    geoms.push_back(line);
}

void MeyerFregly2016Force::extendRealizeInstance(
        const SimTK::State& state) const {
    Super::extendRealizeInstance(state);
    if (!getProperty_force_visualization_scale_factor().empty()) {
        m_forceVizScaleFactor = get_force_visualization_scale_factor();
    } else {
        const Model& model = getModel();
        const double mass = model.getTotalMass(state);
        const double weight = mass * model.getGravity().norm();
        m_forceVizScaleFactor = 1 / weight;
    }
}

void MeyerFregly2016Force::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    this->_forceOnStationCV = addCacheVariable<SimTK::Vec3>(
            "force_on_station", SimTK::Vec3(0),
            SimTK::Stage::Velocity);
}

//=============================================================================
// FORCE PRODUCER INTERFACE
//=============================================================================
void MeyerFregly2016Force::implProduceForces(const SimTK::State& s,
        ForceConsumer& forceConsumer) const {

    const SimTK::Vec3& force = getContactForceOnStation(s);
    const auto& station = getConnectee<Station>("station");
    const auto& pos = station.getLocationInGround(s);
    const auto& frame = station.getParentFrame();

    forceConsumer.consumePointForce(s, frame, station.get_location(), force);
    forceConsumer.consumePointForce(s, getModel().getGround(), pos, -force);
}
