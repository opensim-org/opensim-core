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

#include "../MocoUtilities.h"
#include "../osimMocoDLL.h"

#include <OpenSim/Common/Set.h>
#include <OpenSim/Simulation/Model/Force.h>

namespace OpenSim {

class OSIMMOCO_API SmoothSphereHalfSpaceForce : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(SmoothSphereHalfSpaceForce, Force);

public:
    //=========================================================================
    // PROPERTIES
    //=========================================================================
    OpenSim_DECLARE_PROPERTY(stiffness, double,
            "The stiffness constant (i.e., plain strain modulus), "
            "default is 1 (N/m^2)");
    OpenSim_DECLARE_PROPERTY(dissipation, double,
            "The dissipation coefficient, default is 0 (s/m).");
    OpenSim_DECLARE_PROPERTY(static_friction, double,
            "The coefficient of static friction, default is 0.");
    OpenSim_DECLARE_PROPERTY(dynamic_friction, double,
            "The coefficient of dynamic friction, default is 0.");
    OpenSim_DECLARE_PROPERTY(viscous_friction, double,
            "The coefficient of viscous friction, default is 0.");
    OpenSim_DECLARE_PROPERTY(transition_velocity, double,
            "The transition velocity, default is 0.01 (m/s).");
    OpenSim_DECLARE_PROPERTY(constant_contact_force, double,
            "The constant that enforces non-null derivatives, "
            "default is 1e-5.");
    OpenSim_DECLARE_PROPERTY(hertz_smoothing, double,
            "The parameter that determines the smoothness of the transition "
            "of the tanh used to smooth the Hertz force. The larger the "
            "steeper the transition but the worse for optimization, "
            "default is 300.");
    OpenSim_DECLARE_PROPERTY(hunt_crossley_smoothing, double,
            "The parameter that determines the smoothness of the transition "
            "of the tanh used to smooth the Hunt-Crossley force. The larger "
            "the steeper the transition but the worse for optimization, "
            "default is 50.");
    OpenSim_DECLARE_PROPERTY(
            contact_sphere_radius, double, "The radius of the contact sphere.");
    OpenSim_DECLARE_PROPERTY(contact_sphere_location, SimTK::Vec3,
            "The location of the contact sphere in the sphere frame.");
    OpenSim_DECLARE_PROPERTY(contact_half_space_location, SimTK::Vec3,
            "The location of the contact half space in the half space frame, "
            "default is Vec3(0).");
    OpenSim_DECLARE_PROPERTY(contact_half_space_orientation, SimTK::Vec3,
            "The orientation of the contact half space in the half space "
            "frame (body-fixed XYZ Euler angles), default represents ground "
            "(0,0,-0.5*SimTK::Pi).");
    OpenSim_DECLARE_UNNAMED_PROPERTY(
            Appearance, "Default appearance for this Geometry");

    //=========================================================================
    // SOCKETS
    //=========================================================================
    OpenSim_DECLARE_SOCKET(sphere_frame, PhysicalFrame,
            "The body to which the contact sphere is attached.");
    OpenSim_DECLARE_SOCKET(half_space_frame, PhysicalFrame,
            "The body to which the contact half space is attached.");

    //=========================================================================
    // PUBLIC METHODS
    //=========================================================================
    SmoothSphereHalfSpaceForce();

    SmoothSphereHalfSpaceForce(const std::string& name,
            const Frame& contactSphereBodyFrame,
            SimTK::Vec3 contactSphereLocation, double contactSphereRadius,
            const Frame& contactHalfSpaceBodyFrame,
            SimTK::Vec3 contactHalfSpaceLocation,
            SimTK::Vec3 contactHalfSpaceOrientation);

    SimTK::Transform getHalfSpaceTransformInHalfSpaceFrame() const;

    //=========================================================================
    // REPORTING
    //=========================================================================
    /// Obtain names of the quantities (column labels) of the force values to
    /// be reported. The order is the three forces and three torques applied
    /// on the sphere followed by the three forces and three torques applied
    /// on the half space. Forces and torques are expressed in the ground
    /// frame.
    OpenSim::Array<std::string> getRecordLabels() const override;
    /// Obtain the values to be reported that correspond to the labels. The
    /// values are expressed in the ground frame.
    OpenSim::Array<double> getRecordValues(
            const SimTK::State& state) const override;

protected:
    /// Create a SimTK::Force which implements this Force.
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& state,
            SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override;

private:
    // INITIALIZATION
    void constructProperties();

//=============================================================================
}; // END of class SmoothSphereHalfSpaceForce
//=============================================================================
//=============================================================================

} // namespace OpenSim

#endif // MOCO_SMOOTH_SPHERE_HALFSPACE_FORCE_H
