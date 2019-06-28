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
        Vec3 contactSphereLocation, double contactSphereRadius,
        const Frame& contactHalfSpaceBodyFrame,
        Transform contactHalfSpaceTransform, double stiffness,
        double dissipation, double staticFriction, double dynamicFriction,
        double viscousFriction, double transitionVelocity, double cf,
        double bd, double bv)
	{
        this->connectSocket_body_contact_sphere(contactSphereBodyFrame);
        this->connectSocket_body_contact_half_space(contactHalfSpaceBodyFrame);

		//this->updSocket("body_contact_sphere").setConnecteePath
  //          (contactSphereBodyFrameName);
  //      this->updSocket("body_contact_half_space").setConnecteePath(
  //          contactHalfSpaceBodyFrameName);
		constructProperties();
		set_contact_sphere_location(contactSphereLocation);
		set_contact_sphere_radius(contactSphereRadius);
        set_contact_half_space_transform(contactHalfSpaceTransform);
		set_stiffness(stiffness);
		set_dissipation(dissipation);
		set_static_friction(staticFriction);
		set_dynamic_friction(dynamicFriction);
		set_viscous_friction(viscousFriction);
		set_transition_velocity(transitionVelocity);
        set_constant_contact_force(cf);
		set_hertz_smoothing(bd);
		set_hunt_crossley_smoothing(bv);
	}

    void SmoothSphereHalfSpaceForce::extendAddToSystem(
        SimTK::MultibodySystem& system) const
	{
		Super::extendAddToSystem(system);

        // TODO const? &?
		const Vec3& contactSphereLocation = get_contact_sphere_location();
		const double& contactSphereRadius = get_contact_sphere_radius();
        Transform contactHalfSpaceTransform =
            get_contact_half_space_transform();

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

		const PhysicalFrame& contactSphereFrame =
            getConnectee<PhysicalFrame>("body_contact_sphere");
        const PhysicalFrame& contactHalfSpaceFrame =
            getConnectee<PhysicalFrame>("body_contact_half_space");

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
		force.setContactHalfSpaceFrame(contactHalfSpaceTransform);

		SmoothSphereHalfSpaceForce* mutableThis =
            const_cast<SmoothSphereHalfSpaceForce *>(this);
		mutableThis->_index = force.getForceIndex();
	}

    void SmoothSphereHalfSpaceForce::constructProperties()
	{
		constructProperty_contact_sphere_location(Vec3(0));
		constructProperty_contact_sphere_radius(double(0));
        constructProperty_contact_half_space_transform(
            Transform(SimTK::Rotation(-0.5*SimTK::Pi, SimTK::ZAxis), Vec3(0)));

		constructProperty_stiffness(double(0));
		constructProperty_dissipation(double(0));
		constructProperty_static_friction(double(0));
		constructProperty_dynamic_friction(double(0));
		constructProperty_viscous_friction(double(0));
		constructProperty_transition_velocity(double(0));
        constructProperty_constant_contact_force(double(0));
		constructProperty_hertz_smoothing(double(0));
		constructProperty_hunt_crossley_smoothing(double(0));
	}

//=============================================================================
//  REPORTING
//=============================================================================
    /**
	* Provide names of the quantities (column labels) of the force value(s)
    * reported
	*/
    OpenSim::Array<std::string> SmoothSphereHalfSpaceForce::getRecordLabels()
        const
    {
	    OpenSim::Array<std::string> labels("");

	    labels.append(getName() + ".onPlane" + ".force.X");
	    labels.append(getName() + ".onPlane" + ".force.Y");
	    labels.append(getName() + ".onPlane" + ".force.Z");
	    labels.append(getName() + ".onPlane" + ".torque.X");
	    labels.append(getName() + ".onPlane" + ".torque.Y");
	    labels.append(getName() + ".onPlane" + ".torque.Z");

	    labels.append(getName() + ".onSphere" + ".force.X");
	    labels.append(getName() + ".onSphere" + ".force.Y");
	    labels.append(getName() + ".onSphere" + ".force.Z");
	    labels.append(getName() + ".onSphere" + ".torque.X");
	    labels.append(getName() + ".onSphere" + ".torque.Y");
	    labels.append(getName() + ".onSphere" + ".torque.Z");

	    return labels;
    }

    /**
	* Provide the value(s) to be reported that correspond to the labels
	*/
	OpenSim::Array<double> SmoothSphereHalfSpaceForce::
		getRecordValues(const SimTK::State& state) const {

		OpenSim::Array<double> values(1);

		const PhysicalFrame& contactSphereFrame =
            getConnectee<PhysicalFrame>("body_contact_sphere");
        SimTK::MobilizedBodyIndex contactSphereIdx =
            contactSphereFrame.getMobilizedBody();

        const PhysicalFrame& contactHalfSpaceFrame =
            getConnectee<PhysicalFrame>("body_contact_half_space");
        SimTK::MobilizedBodyIndex contactHalfSpaceIdx =
            contactHalfSpaceFrame.getMobilizedBody();

		const auto& forceSubsys = _model->getForceSubsystem();
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

 // end of namespace OpenSim
