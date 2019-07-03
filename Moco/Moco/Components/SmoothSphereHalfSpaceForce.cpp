/* -------------------------------------------------------------------------- *
 * OpenSim Moco: SmoothSphereHalfSpaceForce.cpp                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse , Gil Serrancoli                                *
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

#include <OpenSim/Simulation/Model/Model.h>
#include <simbody/internal/SmoothSphereHalfSpaceForce.h>

using namespace OpenSim;

//=============================================================================
//  SMOOTH SPHERE HALF SPACE FORCE
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy
// assignment operator.

// Default constructor.
SmoothSphereHalfSpaceForce::SmoothSphereHalfSpaceForce()
{
    constructProperties();
}

// Take over ownership of supplied object.
SmoothSphereHalfSpaceForce::SmoothSphereHalfSpaceForce(
    const std::string& name,
    const Frame& contactSphereBodyFrame,
    SimTK::Vec3 contactSphereLocation, double contactSphereRadius,
    const Frame& contactHalfSpaceBodyFrame,
    SimTK::Vec3 contactHalfSpaceLocation,
    SimTK::Vec3 contactHalfSpaceOrientation)
{
    this->connectSocket_sphere_frame(contactSphereBodyFrame);
    this->connectSocket_half_space_frame(contactHalfSpaceBodyFrame);

    constructProperties();
    set_contact_sphere_location(contactSphereLocation);
    set_contact_sphere_radius(contactSphereRadius);
    set_contact_half_space_location(contactHalfSpaceLocation);
    set_contact_half_space_orientation(contactHalfSpaceOrientation);
}

void SmoothSphereHalfSpaceForce::extendAddToSystem(
    SimTK::MultibodySystem& system) const {

    Super::extendAddToSystem(system);

    const SimTK::Vec3& contactSphereLocation = get_contact_sphere_location();
    const double& contactSphereRadius = get_contact_sphere_radius();

    double stiffness = get_stiffness();
    double dissipation = get_dissipation();
    double staticFriction = get_static_friction();
    double dynamicFriction = get_dynamic_friction();
    double viscousFriction = get_viscous_friction();
    double transitionVelocity = get_transition_velocity();
    double cf = get_derivative_smoothing();
    double bd = get_hertz_smoothing();
    double bv = get_hunt_crossley_smoothing();

    SimTK::SmoothSphereHalfSpaceForce force(_model->updForceSubsystem());

    const PhysicalFrame& contactSphereFrame =
        getConnectee<PhysicalFrame>("sphere_frame");
    const PhysicalFrame& contactHalfSpaceFrame =
        getConnectee<PhysicalFrame>("half_space_frame");

    force.setStiffness(stiffness);
    force.setDissipation(dissipation);
    force.setStaticFriction(staticFriction);
    force.setDynamicFriction(dynamicFriction);
    force.setViscousFriction(viscousFriction);
    force.setTransitionVelocity(transitionVelocity);
    force.setConstantContactForce(cf);
    force.setHertzSmoothing(bd);
    force.setHuntCrossleySmoothing(bv);

    force.setContactSphereBody(contactSphereFrame.getMobilizedBody());
    force.setContactSphereLocationInBody(contactSphereLocation);
    force.setContactSphereRadius(contactSphereRadius);

    force.setContactHalfSpaceBody(
        contactHalfSpaceFrame.getMobilizedBody());

    SimTK::Transform halfSpaceFrame(
        SimTK::Rotation(SimTK::BodyRotationSequence,
            get_contact_half_space_orientation()[0], SimTK::XAxis,
            get_contact_half_space_orientation()[1], SimTK::YAxis,
            get_contact_half_space_orientation()[2], SimTK::ZAxis),
        get_contact_half_space_location());

    force.setContactHalfSpaceFrame(halfSpaceFrame);

    SmoothSphereHalfSpaceForce* mutableThis =
        const_cast<SmoothSphereHalfSpaceForce *>(this);
    mutableThis->_index = force.getForceIndex();
}

void SmoothSphereHalfSpaceForce::constructProperties()
{
    constructProperty_contact_sphere_location(SimTK::Vec3(0));
    constructProperty_contact_sphere_radius(0.0);
    constructProperty_contact_half_space_location(SimTK::Vec3(0));
    constructProperty_contact_half_space_orientation(SimTK::Vec3(0));
    constructProperty_stiffness(1.0);
    constructProperty_dissipation(0.0);
    constructProperty_static_friction(0.0);
    constructProperty_dynamic_friction(0.0);
    constructProperty_viscous_friction(0.0);
    constructProperty_transition_velocity(0.01);
    constructProperty_derivative_smoothing(1e-5);
    constructProperty_hertz_smoothing(300.0);
    constructProperty_hunt_crossley_smoothing(50.0);
}

//=============================================================================
//  REPORTING
//=============================================================================
// Provide names of the quantities (column labels) of the force value(s)
OpenSim::Array<std::string> SmoothSphereHalfSpaceForce::getRecordLabels()
    const
{
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
OpenSim::Array<double> SmoothSphereHalfSpaceForce::
    getRecordValues(const SimTK::State& state) const {

    OpenSim::Array<double> values(1);

    const PhysicalFrame& contactSphereFrame =
        getConnectee<PhysicalFrame>("sphere_frame");
    SimTK::MobilizedBodyIndex contactSphereIdx =
        contactSphereFrame.getMobilizedBody();

    const PhysicalFrame& contactHalfSpaceFrame =
        getConnectee<PhysicalFrame>("half_space_frame");
    SimTK::MobilizedBodyIndex contactHalfSpaceIdx =
        contactHalfSpaceFrame.getMobilizedBody();

    const Model& model = getModel();
    const auto& forceSubsys = model.getForceSubsystem();
    const SimTK::Force& abstractForce = forceSubsys.getForce(_index);
    const auto& simtkForce =
        (SimTK::SmoothSphereHalfSpaceForce &)(abstractForce);

    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    SimTK::Vector_<SimTK::Vec3> particleForces(0);
    SimTK::Vector mobilityForces(0);

    simtkForce.calcForceContribution(state, bodyForces, particleForces,
        mobilityForces);

    // On sphere
    const auto& thisBodyForce1 = bodyForces(contactSphereIdx);
    SimTK::Vec3 forces1 = thisBodyForce1[1];
    SimTK::Vec3 torques1 = thisBodyForce1[0];
    values.append(3, &forces1[0]);
    values.append(3, &torques1[0]);

    // On plane
    const auto& thisBodyForce2 = bodyForces(contactHalfSpaceIdx);
    SimTK::Vec3 forces2 = thisBodyForce2[1];
    SimTK::Vec3 torques2 = thisBodyForce2[0];
    values.append(3, &forces2[0]);
    values.append(3, &torques2[0]);

    return values;
}
