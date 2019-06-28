#ifndef MOCO_SMOOTH_SPHERE_HALFSPACE_FORCE_H
#define MOCO_SMOOTH_SPHERE_HALFSPACE_FORCE_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: SmoothSphereHalfSpaceForce.h                                 *
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

#include "../MocoUtilities.h" //TODO
#include "../osimMocoDLL.h" //TODO

#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Common/Set.h>

using SimTK::Vec3;
using SimTK::Transform;

namespace OpenSim {

class OSIMMOCO_API SmoothSphereHalfSpaceForce : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(SmoothSphereHalfSpaceForce, Force);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(stiffness, double,
            "The stiffness constant (i.e., plain strain modulus),"
            "default is 1 (N/m^2)");
    OpenSim_DECLARE_PROPERTY(dissipation, double,
            "The dissipation coefficient, default is 0 (s/m).");
    OpenSim_DECLARE_PROPERTY(static_friction, double,
            "The coefficient of static frictiont, default is 0.");
    OpenSim_DECLARE_PROPERTY(dynamic_friction, double,
            "The coefficient of dynamic frictiont, default is 0.");
    OpenSim_DECLARE_PROPERTY(viscous_friction, double,
            "The coefficient of viscous friction, default is 0.");
    OpenSim_DECLARE_PROPERTY(transition_velocity, double,
            "The transition velocity, default is 0.01 (m/s).");
    OpenSim_DECLARE_PROPERTY(constant_contact_force, double,
            "The constant that enforces non-null derivatives,"
            "default is 1e-5.");
    OpenSim_DECLARE_PROPERTY(hertz_smoothing, double,
            "The parameter that determines the smoothness of the transition"
            "of the tanh used to smooth the Hertz force. The larger the"
            "steeper the transition but also the more discontinuous-like,"
            "default is 300.");
    OpenSim_DECLARE_PROPERTY(hunt_crossley_smoothing, double,
            "The parameter that determines the smoothness of the transition"
            "of the tanh used to smooth the Hunt-Crossley force. The larger"
            "the steeper the transition but also the more discontinuous-like,"
            "default is 50.");
    OpenSim_DECLARE_PROPERTY(contact_sphere_radius, double,
            "The radius of the contact sphere.");
    OpenSim_DECLARE_PROPERTY(contact_sphere_location, Vec3,
            "The location of the contact sphere in the body frame.");
    OpenSim_DECLARE_PROPERTY(contact_half_space_transform, Transform,
            "The transform of the contact half space in the body frame.");

//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_SOCKET(body_contact_sphere, PhysicalFrame,
            "The body to which the contact sphere is attached.");
    OpenSim_DECLARE_SOCKET(body_contact_half_space, PhysicalFrame,
            "The body to which the contact half space is attached.");

//=============================================================================
// PUBLIC METHODS
//=============================================================================
    SmoothSphereHalfSpaceForce();
#ifndef SWIG
    /** The force takes ownership of the passed-in params. */
    // TODO
    /*explicit SmoothSphereHalfSpaceForce();*/
#endif

    // TODO
    SmoothSphereHalfSpaceForce(const std::string& name,
		const Frame& contactSphereBodyFrame,
        Vec3 contactSphereLocation, double contactSphereRadius,
        const Frame& contactHalfSpaceBodyFrame,
        Transform contactHalfSpaceTransform, double stiffness,
        double dissipation, double staticFriction, double dynamicFriction,
        double viscousFriction, double transitionVelocity, double cf,
        double bd, double bv);

    //// TODO const and override
    //Vec3 getContactSphereLocation() const;
    //void setContactSphereLocation(Vec3 contactSphereLocation);
    //double getContactSphereRadius() const;
    //void setContactSphereRadius(double contactSphereRadius);
    //Transform getContactHalfSpaceTransform() const;
    //void setContactHalfSpaceTransform(Transform contactHalfSpaceTransform);
    //double getStiffness() const;
    //void setStiffness(double stiffness);
    //double getDissipation() const;
    //void setDissipation(double dissipation);
    //double getStaticFriction() const;
    //void setStaticFriction(double staticFriction);
    //double getDynamicFriction() const;
    //void setDynamicFriction(double dynamicFriction);
    //double getViscousFriction() const;
    //void setViscousFriction(double viscousFriction);
    //double getTransitionVelocity() const;
    //void setTransitionVelocity(double transitionVelocity);
    //double getConstantContactForce() const;
    //void setConstantContactForce(double cf);
    //double getHertzSmoothing() const;
    //void setHertzSmoothing(double bd);
    //double getHuntCrossleySmoothing() const;
    //void setHuntCrossleySmoothing(double bv);

//=============================================================================
// REPORTING
//=============================================================================
    /**
    * Provide name(s) of the quantities (column labels) of the force value(s)
    * to be reported
    */
    OpenSim::Array<std::string> getRecordLabels() const override ;
    /**
    *  Provide the value(s) to be reported that correspond to the labels
    */
    OpenSim::Array<double> getRecordValues(const SimTK::State& state) const
        override ;

protected:
    /**
    * Create a SimTK::Force which implements this Force.
    */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

private:
    // INITIALIZATION
    void constructProperties();

//=============================================================================
}; // END of class SmoothSphereHalfSpaceForce
//=============================================================================
//=============================================================================

} // namespace OpenSim

#endif // MOCO_SMOOTH_SPHERE_HALFSPACE_FORCE_H
