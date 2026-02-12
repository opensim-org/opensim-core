#ifndef OPENSIM_MEYER_FREGLY_2016_FORCE_H_
#define OPENSIM_MEYER_FREGLY_2016_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  MeyerFregly2016Force.h                        *
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

#include <OpenSim/Simulation/Model/ForceConsumer.h>
#include <OpenSim/Simulation/Model/ForceProducer.h>
#include <OpenSim/Simulation/Model/Station.h>

namespace OpenSim {

/** This contact model is from the following paper:

Meyer A. J., Eskinazi, I., Jackson, J. N., Rao, A. V., Patten, C., & Fregly,
B. J. (2016). %Muscle Synergies Facilitate Computational Prediction of
Subject-Specific Walking Motions. Frontiers in Bioengineering and
Biotechnology, 4, 1055–27. http://doi.org/10.3389/fbioe.2016.00077

@note This implementation is equivalent to the
<a href="https://github.com/rcnl-org/nmsm-core/blob/v1.5.1/src/GroundContactPersonalization/Optimizations/ModelCalculation/calcModeledVerticalGroundReactionForce.m">foot-ground
contact model from the %Neuromusculoskeletal Modeling Software (NMSM) Pipeline
(version 1.5.1)</a>.

Following %OpenSim convention, this contact element assumes that the y-direction
is vertical. Vertical contact force is calculated based on vertical position
and velocity relative to a floor at y=0 using these equations:

\f[
v = \frac{k_{val} + k_{low}}{k_{val} - k_{low}}
\f]
\f[
s = \frac{k_{val} - k_{low}}{2}
\f]
\f[
R = -s * (v * y_{max} - c * log(\frac{cosh(y_{max} + h)}{c})
\f]
\f[
F = -s * (v * y - c * log(\frac{cosh(y + h)}{c}) - R
\f]

With the following values:
- \f$ k_{val} \f$: stiffness coefficient of contact element
- \f$ k_{low} \f$: small out-of-contact stiffness to assist optimizations
- \f$ y_{max} \f$: y value were out-of-contact force becomes zero
- \f$ c \f$: transition curvature of transition between linear regions
- \f$ h \f$: horizontal offset defining slope transition location
- \f$ y \f$: current y (vertical) position of contact element

Velocity is then used to incorporate non-linear damping:

\f[
F_{damped} = F * (1 - C_{val} * y_{vel})
\f]

With the following values:
- \f$ C_{val} \f$: damping coefficient of contact element
- \f$ y_{vel} \f$: current y (vertical) velocity of contact element

The force equation produces a force-penetration curve similar to a leaky
rectified linear function with a smooth transition region near zero. This
produces a smooth curve with a slight out-of-contact slope to assist
gradient-based optimizations when an element is inappropriately out of contact.

Horizontal forces are then calculated based on this vertical force and the
horizontal velocity components of the contact element. Both dynamic (modeled
with a tanh function) and viscous (modeled with a linear function) friction
models may be used. */
class OSIMSIMULATION_API MeyerFregly2016Force : public ForceProducer {
OpenSim_DECLARE_CONCRETE_OBJECT(MeyerFregly2016Force, ForceProducer);

public:
//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_SOCKET(station, Station,
            "The body-fixed point that can contact the plane.");

//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(stiffness, double,
            "Spring stiffness in N/m (default: 1e4).");
    OpenSim_DECLARE_PROPERTY(dissipation, double,
            "Dissipation coefficient in s/m (default: 0.01).");
    OpenSim_DECLARE_PROPERTY(spring_resting_length, double,
            "Spring resting length in m (default: 0).");
    OpenSim_DECLARE_PROPERTY(dynamic_friction, double,
            "Dynamic friction coefficient (default: 0).");
    OpenSim_DECLARE_PROPERTY(viscous_friction, double,
            "Viscous friction coefficient (default: 5.0).");
    OpenSim_DECLARE_PROPERTY(latch_velocity, double,
            "Latching velocity in m/s (default: 0.05).");

//=============================================================================
// OUTPUTS
//=============================================================================
    OpenSim_DECLARE_OUTPUT(force_on_station, SimTK::Vec3,
            getContactForceOnStation, SimTK::Stage::Velocity);

//=============================================================================
// METHODS
//=============================================================================

    // CONSTRUCTION
    MeyerFregly2016Force();

    // GET AND SET
    /** Get the contact force applied to the Station. */
    SimTK::Vec3 getContactForceOnStation(const SimTK::State& s) const;

    /** Set the contact force applied to the Station. */
    void setContactForceOnStation(const SimTK::State& s,
            const SimTK::Vec3& force) const;

    // FORCE INTERFACE
    OpenSim::Array<std::string> getRecordLabels() const override;
    OpenSim::Array<double> getRecordValues(const SimTK::State& s) const override;

protected:
    // COMPONENT INTERFACE
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

private:
    /** A helper function that calculates the contact force applied to the
    Station. */
    SimTK::Vec3 calcContactForceOnStation(const SimTK::State& state) const;

    // PROPERTIES
    void constructProperties();

    // FORCE PRODUCER INTERFACE
    void implProduceForces(const SimTK::State& s,
            ForceConsumer& forceConsumer) const override;

    // MEMBER AND CACHE VARIABLES
    mutable CacheVariable<SimTK::Vec3> _forceOnStationCV;
};

} // namespace OpenSim

#endif // OPENSIM_MEYER_FREGLY_2016_FORCE_H_
