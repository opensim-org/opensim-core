/* -------------------------------------------------------------------------- *
 *                   OpenSim: SmoothSphereHalfSpaceForce.cpp                  *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse, Gil Serrancoli                                 *
 * Contributors: Peter Eastman                                                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "SmoothSphereHalfSpaceForce.h"

#include <simbody/internal/SmoothSphereHalfSpaceForce.h>

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//=============================================================================
//  SMOOTH SPHERE HALF SPACE FORCE
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy
// assignment operator.

// Default constructor.
SmoothSphereHalfSpaceForce::SmoothSphereHalfSpaceForce() {
    constructProperties();
}

// Take over ownership of supplied object.
SmoothSphereHalfSpaceForce::SmoothSphereHalfSpaceForce(const std::string& name,
        const ContactSphere& contactSphere,
        const ContactHalfSpace& contactHalfSpace) {
    setName(name);
    connectSocket_sphere(contactSphere);
    connectSocket_half_space(contactHalfSpace);

    constructProperties();
}

void SmoothSphereHalfSpaceForce::extendAddToSystem(
        SimTK::MultibodySystem& system) const {

    Super::extendAddToSystem(system);

    double stiffness = get_stiffness();
    double dissipation = get_dissipation();
    double staticFriction = get_static_friction();
    double dynamicFriction = get_dynamic_friction();
    double viscousFriction = get_viscous_friction();
    double transitionVelocity = get_transition_velocity();
    double cf = get_constant_contact_force();
    double bd = get_hertz_smoothing();
    double bv = get_hunt_crossley_smoothing();

    SimTK::SmoothSphereHalfSpaceForce force(_model->updForceSubsystem());

    const auto& sphere = getConnectee<ContactSphere>("sphere");
    const auto& halfSpace = getConnectee<ContactHalfSpace>("half_space");

    force.setStiffness(stiffness);
    force.setDissipation(dissipation);
    force.setStaticFriction(staticFriction);
    force.setDynamicFriction(dynamicFriction);
    force.setViscousFriction(viscousFriction);
    force.setTransitionVelocity(transitionVelocity);
    force.setConstantContactForce(cf);
    force.setHertzSmoothing(bd);
    force.setHuntCrossleySmoothing(bv);

    force.setContactSphereBody(sphere.getFrame().getMobilizedBody());
    force.setContactSphereLocationInBody(
            sphere.getFrame().findTransformInBaseFrame() *
            sphere.get_location());
    force.setContactSphereRadius(sphere.getRadius());

    force.setContactHalfSpaceBody(halfSpace.getFrame().getMobilizedBody());

    force.setContactHalfSpaceFrame(
            halfSpace.getFrame().findTransformInBaseFrame() *
            halfSpace.getTransform());

    auto* mutableThis = const_cast<SmoothSphereHalfSpaceForce*>(this);
    mutableThis->_index = force.getForceIndex();
}

void OpenSim::SmoothSphereHalfSpaceForce::extendRealizeInstance(
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

void SmoothSphereHalfSpaceForce::constructProperties() {
    constructProperty_stiffness(1.0);
    constructProperty_dissipation(0.0);
    constructProperty_static_friction(0.0);
    constructProperty_dynamic_friction(0.0);
    constructProperty_viscous_friction(0.0);
    constructProperty_transition_velocity(0.01);
    constructProperty_constant_contact_force(1e-5);
    constructProperty_hertz_smoothing(300.0);
    constructProperty_hunt_crossley_smoothing(50.0);
    constructProperty_force_visualization_radius(0.01);
    constructProperty_force_visualization_scale_factor();
}

//=============================================================================
//  REPORTING
//=============================================================================
// Provide names of the quantities (column labels) of the force value(s)
OpenSim::Array<std::string>
SmoothSphereHalfSpaceForce::getRecordLabels() const {
    OpenSim::Array<std::string> labels("");

    labels.append(getName() + ".Sphere" + ".force.X");
    labels.append(getName() + ".Sphere" + ".force.Y");
    labels.append(getName() + ".Sphere" + ".force.Z");
    labels.append(getName() + ".Sphere" + ".torque.X");
    labels.append(getName() + ".Sphere" + ".torque.Y");
    labels.append(getName() + ".Sphere" + ".torque.Z");

    labels.append(getName() + ".HalfSpace" + ".force.X");
    labels.append(getName() + ".HalfSpace" + ".force.Y");
    labels.append(getName() + ".HalfSpace" + ".force.Z");
    labels.append(getName() + ".HalfSpace" + ".torque.X");
    labels.append(getName() + ".HalfSpace" + ".torque.Y");
    labels.append(getName() + ".HalfSpace" + ".torque.Z");

    return labels;
}

// Provide the value(s) to be reported that correspond to the labels
OpenSim::Array<double> SmoothSphereHalfSpaceForce::getRecordValues(
        const SimTK::State& state) const {

    OpenSim::Array<double> values(1);

    const auto& sphere = getConnectee<ContactSphere>("sphere");
    const auto sphereIdx = sphere.getFrame().getMobilizedBodyIndex();

    const auto& halfSpace = getConnectee<ContactHalfSpace>("half_space");
    const auto halfSpaceIdx = halfSpace.getFrame().getMobilizedBodyIndex();

    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    calcBodyForces(state, bodyForces);

    // On sphere
    const auto& thisBodyForce1 = bodyForces(sphereIdx);
    SimTK::Vec3 forces1 = thisBodyForce1[1];
    SimTK::Vec3 torques1 = thisBodyForce1[0];
    values.append(3, &forces1[0]);
    values.append(3, &torques1[0]);

    // On plane
    const auto& thisBodyForce2 = bodyForces(halfSpaceIdx);
    SimTK::Vec3 forces2 = thisBodyForce2[1];
    SimTK::Vec3 torques2 = thisBodyForce2[0];
    values.append(3, &forces2[0]);
    values.append(3, &torques2[0]);

    return values;
}

SimTK::SpatialVec SmoothSphereHalfSpaceForce::getSphereForce(
        const SimTK::State& state) const {
    const auto& sphere = getConnectee<ContactSphere>("sphere");
    const auto sphereIdx = sphere.getFrame().getMobilizedBodyIndex();

    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    calcBodyForces(state, bodyForces);
    return bodyForces(sphereIdx);
}

SimTK::SpatialVec SmoothSphereHalfSpaceForce::getHalfSpaceForce(
        const SimTK::State& state) const {
    const auto& halfSpace = getConnectee<ContactHalfSpace>("half_space");
    const auto halfSpaceIdx = halfSpace.getFrame().getMobilizedBodyIndex();

    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    calcBodyForces(state, bodyForces);
    return bodyForces(halfSpaceIdx);
}

void SmoothSphereHalfSpaceForce::calcBodyForces(
        const SimTK::State& state,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces) const {
    const Model& model = getModel();
    const auto& forceSubsys = model.getForceSubsystem();
    const SimTK::Force& abstractForce = forceSubsys.getForce(_index);
    const auto& simtkForce =
            static_cast<const SimTK::SmoothSphereHalfSpaceForce&>(
                    abstractForce);

    SimTK::Vector_<SimTK::Vec3> particleForces(0);
    SimTK::Vector mobilityForces(0);

    simtkForce.calcForceContribution(
            state, bodyForces, particleForces, mobilityForces);
}

void SmoothSphereHalfSpaceForce::generateDecorations(bool fixed,
        const ModelDisplayHints& hints, const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const {
    Super::generateDecorations(fixed, hints, state, geometry);

    if (!fixed && (state.getSystemStage() >= SimTK::Stage::Dynamics) &&
            hints.get_show_forces()) {
        // Compute the body forces.
        SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
        calcBodyForces(state, bodyForces);

        // Get the index to the associated contact sphere.
        const auto& sphere = getConnectee<ContactSphere>("sphere");
        const auto& sphereIdx = sphere.getFrame().getMobilizedBodyIndex();

        // Get the translational force for the contact sphere associated with
        // this force element.
        const auto& sphereForce = bodyForces(sphereIdx)[1];

        // Scale the contact force vector and compute the cylinder length.
        const auto& scaledContactForce =
                m_forceVizScaleFactor * sphereForce;
        const SimTK::Real length(scaledContactForce.norm());

        // Compute the force visualization transform.
        const SimTK::Vec3 contactSpherePosition =
                sphere.getFrame().findStationLocationInGround(
                        state, sphere.get_location());
        const SimTK::Transform forceVizTransform(
                SimTK::Rotation(SimTK::UnitVec3(scaledContactForce),
                        SimTK::YAxis),
                contactSpherePosition + scaledContactForce / 2.0);

        // Construct the force decoration and add it to the list of geometries.
        SimTK::DecorativeCylinder forceViz(get_force_visualization_radius(),
                0.5 * length);
        forceViz.setTransform(forceVizTransform);
        forceViz.setColor(SimTK::Vec3(0.0, 0.6, 0.0));
        geometry.push_back(forceViz);
    }
}
