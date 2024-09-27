#ifndef OPENSIM_STATIONPLANECONTACTFORCE_H
#define OPENSIM_STATIONPLANECONTACTFORCE_H
/* -------------------------------------------------------------------------- *
 * OpenSim: StationPlaneContactForce.h                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco, Chris Dembia, Spencer Williams                 *
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

#include <OpenSim/Moco/osimMocoDLL.h>
#include <OpenSim/Simulation/Model/ForceConsumer.h>
#include <OpenSim/Simulation/Model/ForceProducer.h>
#include <OpenSim/Simulation/osimSimulation.h>

namespace OpenSim {

/** This class models compliant point contact with a ground plane y=0.

Vertical force is calculated using stiffness and dissipation coefficients, 
while horizontal force is calculated using a friction model. Multiple concrete 
implementations of this class are available, but currently MeyerFregly2016Force 
is the only complete and well-tested implementation and therefore recommended.

@underdevelopment */
class OSIMSIMULATION_API StationPlaneContactForce : public ForceProducer {
OpenSim_DECLARE_ABSTRACT_OBJECT(StationPlaneContactForce, ForceProducer);
public:
    OpenSim_DECLARE_OUTPUT(force_on_station, SimTK::Vec3,
            computeContactForceOnStation, SimTK::Stage::Velocity);

    OpenSim_DECLARE_SOCKET(station, Station,
            "The body-fixed point that can contact the plane.");

    OpenSim::Array<std::string> getRecordLabels() const override {
        OpenSim::Array<std::string> labels;
        const auto stationName = getConnectee("station").getName();
        labels.append(getName() + "." + stationName + ".force.X");
        labels.append(getName() + "." + stationName + ".force.Y");
        labels.append(getName() + "." + stationName + ".force.Z");
        return labels;
    }
    OpenSim::Array<double> getRecordValues(const SimTK::State& s)
    const override {
        OpenSim::Array<double> values;
        // TODO cache.
        const SimTK::Vec3 force = computeContactForceOnStation(s);
        values.append(force[0]);
        values.append(force[1]);
        values.append(force[2]);
        return values;
    }
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& s,
            SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const override;

    virtual SimTK::Vec3 computeContactForceOnStation(
            const SimTK::State& s) const = 0;

private:
    void implProduceForces(
        const SimTK::State& s,
        ForceConsumer& forceConsumer) const override {

        const SimTK::Vec3 force = computeContactForceOnStation(s);
        const auto& pt = getConnectee<Station>("station");
        const auto& pos = pt.getLocationInGround(s);
        const auto& frame = pt.getParentFrame();

        forceConsumer.consumePointForce(s, frame, pt.get_location(), force);
        forceConsumer.consumePointForce(s, getModel().getGround(), pos, -force);
    }
};

/** This contact model is from the following paper:
Meyer A. J., Eskinazi, I., Jackson, J. N., Rao, A. V., Patten, C., & Fregly,
B. J. (2016). Muscle Synergies Facilitate Computational Prediction of
Subject-Specific Walking Motions. Frontiers in Bioengineering and
Biotechnology, 4, 1055–27. http://doi.org/10.3389/fbioe.2016.00077 

Following OpenSim convention, this contact element assumes that the y-direction
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
F_{damped} = F * (1 + C_{val} * y_{vel})
\f]

With the following values:
- \f$ C_{val} \f$: damping coefficient of contact element
- \f$ y_{vel} \f$: current y (vertical) velocity of contact element, negated

The force equation produces a force-penetration curve similar to a leaky 
rectified linear function with a smooth transition region near zero. This 
produces a smooth curve with a slight out-of-contact slope to assist 
gradient-based optimizations when an element is inappropriately out of contact.

Horizontal forces are then calculated based on this vertical force and the 
horizontal velocity components of the contact element. Both dynamic (modeled 
with a tanh function) and viscous (modeled with a linear function) friction 
models may be used. */
class OSIMSIMULATION_API MeyerFregly2016Force
        : public StationPlaneContactForce {
OpenSim_DECLARE_CONCRETE_OBJECT(MeyerFregly2016Force,
        StationPlaneContactForce);
public:
    OpenSim_DECLARE_PROPERTY(stiffness, double,
            "Spring stiffness in N/m (default: 1e4).");
    OpenSim_DECLARE_PROPERTY(dissipation, double,
            "Dissipation coefficient in s/m (default: 0.01).");
    OpenSim_DECLARE_PROPERTY(spring_resting_length, double,
            "Spring resting length in m (default: 0).");
    OpenSim_DECLARE_PROPERTY(dynamic_friction, double,
            "Dynamic friction coefficient (default: 0).");
    OpenSim_DECLARE_PROPERTY(viscous_friction, double,
            "Viscous friction coefficient (default: 5).");
    OpenSim_DECLARE_PROPERTY(latch_velocity, double,
            "Latching velocity in m/s (default: 0.05).");

    MeyerFregly2016Force() {
        constructProperties();
    }

    /// Compute the force applied to body to which the station is attached, at
    /// the station, expressed in ground.
    SimTK::Vec3 computeContactForceOnStation(const SimTK::State& s)
    const override {
        SimTK::Vec3 force(0);
        const auto& pt = getConnectee<Station>("station");
        const auto& pos = pt.getLocationInGround(s);
        const auto& vel = pt.getVelocityInGround(s);
        const SimTK::Real resting_length = get_spring_resting_length();
        const SimTK::Real y = pos[1];
        const SimTK::Real velNormal = vel[1];
        SimTK::Real velSliding = sqrt(vel[0] * vel[0] + vel[2] * vel[2]);
        const SimTK::Real depthRate = 0 - velNormal;
        const SimTK::Real Kval = get_stiffness();
        const SimTK::Real Cval = get_dissipation();
        const SimTK::Real klow = 1e-1;
        const SimTK::Real h = 1e-3;
        const SimTK::Real c = 5e-4;
        const SimTK::Real ymax = 1e-2;
        // Prevents dividing by zero when sliding velocity is zero.
        const SimTK::Real slipOffset = 1e-4;

        /// Normal force.
        y = y - resting_length;
        const SimTK::Real vp = (Kval + klow) / (Kval - klow);
        const SimTK::Real sp = (Kval - klow) / 2;

        const SimTK::Real constant =
                -sp * (vp * ymax - c * log(cosh((ymax + h) / c)));

        SimTK::Real Fspring =
                -sp * (vp * y - c * log(cosh((y + h) / c))) - constant;

        const SimTK::Real Fy = Fspring * (1 + Cval * depthRate);

        force[1] = Fy;

        /// Friction force.
        const SimTK::Real mu_d = get_dynamic_friction();
        const SimTK::Real mu_v = get_viscous_friction();
        const SimTK::Real latchvel = get_latch_velocity();

        SimTK::Real horizontalForce = force[1] * (
                mu_d * tanh(velSliding / latchvel) + mu_v * velSliding
        );
        
        force[0] = -vel[0] / (velSliding + slipOffset) * horizontalForce;
        force[2] = -vel[2] / (velSliding + slipOffset) * horizontalForce;

        return force;
    }

private:
    void constructProperties() {
        constructProperty_stiffness(1e4);
        constructProperty_dissipation(1e-2);
        constructProperty_spring_resting_length(0);
        constructProperty_dynamic_friction(0);
        constructProperty_viscous_friction(5);
        constructProperty_latch_velocity(0.05);
    }

};

/** This class is still under development. */
class OSIMSIMULATION_API AckermannVanDenBogert2010Force
        : public StationPlaneContactForce {
OpenSim_DECLARE_CONCRETE_OBJECT(AckermannVanDenBogert2010Force,
        StationPlaneContactForce);
public:
    OpenSim_DECLARE_PROPERTY(stiffness, double,
            "Spring stiffness in N/m^3 (default: 5e7).");
    OpenSim_DECLARE_PROPERTY(dissipation, double,
            "Dissipation coefficient in s/m (default: 1.0).");
    OpenSim_DECLARE_PROPERTY(friction_coefficient, double,
            "Friction coefficient");
    // TODO rename to transition_velocity
    OpenSim_DECLARE_PROPERTY(tangent_velocity_scaling_factor, double,
            "Governs how rapidly friction develops (default: 0.05).");

    AckermannVanDenBogert2010Force() {
        constructProperties();
    }

    /// Compute the force applied to body to which the station is attached, at
    /// the station, expressed in ground.
    SimTK::Vec3 computeContactForceOnStation(const SimTK::State& s)
    const override {
        SimTK::Vec3 force(0);
        const auto& pt = getConnectee<Station>("station");
        const auto& pos = pt.getLocationInGround(s);
        const auto& vel = pt.getVelocityInGround(s);
        const SimTK::Real y = pos[1];
        const SimTK::Real velNormal = vel[1];
        // TODO should project vel into ground.
        const SimTK::Real velSliding = vel[0];
        const SimTK::Real depth = 0 - y;
        const SimTK::Real depthRate = 0 - velNormal;
        const SimTK::Real& a = get_stiffness();
        const SimTK::Real& b = get_dissipation();
        if (depth > 0) {
            force[1] = fmax(0, a * pow(depth, 3) * (1 + b * depthRate));
        }
        const SimTK::Real voidStiffness = 1.0; // N/m
        force[1] += voidStiffness * depth;

        const SimTK::Real velSlidingScaling =
                get_tangent_velocity_scaling_factor();
        // The paper used (1 - exp(-x)) / (1 + exp(-x)) = tanh(2x).
        // tanh() has a wider domain than using exp().
        const SimTK::Real transition = tanh(velSliding / velSlidingScaling / 2);

        const SimTK::Real frictionForce =
                -transition * get_friction_coefficient() * force[1];

        force[0] = frictionForce;
        return force;
    }

private:
    void constructProperties() {
        constructProperty_friction_coefficient(1.0);
        constructProperty_stiffness(5e7);
        constructProperty_dissipation(1.0);
        constructProperty_tangent_velocity_scaling_factor(0.05);
    }
};

/** This contact model uses a continuous equation to transition between in and
out of contact. The equation for the smooth transition was published in
the following two papers:

Koelewijn, A. D., & van den Bogert, A. J. (2016). Joint contact forces can
be reduced by improving joint moment symmetry in below-knee amputee gait
simulations. Gait & Posture, 49, 219–225.
http://doi.org/10.1016/j.gaitpost.2016.07.007

Esposito, E. R., & Miller, R. H. (2018). Maintenance of muscle strength
retains a normal metabolic cost in simulated walking after transtibial limb
loss. PLoS ONE, 13(1), e0191310. http://doi.org/10.1371/journal.pone.0191310

@underdevelopment */
class OSIMSIMULATION_API EspositoMiller2018Force
        : public StationPlaneContactForce {
OpenSim_DECLARE_CONCRETE_OBJECT(EspositoMiller2018Force,
        StationPlaneContactForce);
public:
    OpenSim_DECLARE_PROPERTY(stiffness, double,
            "Spring stiffness in N/m^3 (default: 2e6).");
    OpenSim_DECLARE_PROPERTY(dissipation, double,
            "Dissipation coefficient in s/m (default: 1.0).");
    OpenSim_DECLARE_PROPERTY(friction_coefficient, double,
            "Friction coefficient");
    // TODO rename to transition_velocity
    OpenSim_DECLARE_PROPERTY(tangent_velocity_scaling_factor, double,
            "Governs how rapidly friction develops (default: 0.05).");
    OpenSim_DECLARE_PROPERTY(depth_offset, double,
            "'Resting length' of spring in meters (default: 0.001).");

    EspositoMiller2018Force() {
        constructProperties();
    }

    void extendFinalizeFromProperties() override {
        m_depthOffsetSquared = SimTK::square(get_depth_offset());
    }

    /// Compute the force applied to body to which the station is attached, at
    /// the station, expressed in ground.
    SimTK::Vec3 computeContactForceOnStation(const SimTK::State& s)
    const override {
        using SimTK::square;
        SimTK::Vec3 force(0);
        const auto& pt = getConnectee<Station>("station");
        const auto& pos = pt.getLocationInGround(s);
        const auto& vel = pt.getVelocityInGround(s);
        const SimTK::Real height = pos[1];
        const SimTK::Real velNormal = vel[1];
        // TODO should project vel into ground.
        const SimTK::Real velSliding = vel[0];

        // The Appendix of Esposito and Miller 2018 uses height above ground,
        // but we use penetration depth, so some signs are reversed.

        // Normal force.
        const SimTK::Real depth = 0 - height;
        const SimTK::Real depthRate = 0 - velNormal;
        const SimTK::Real a = get_stiffness();
        const SimTK::Real b = get_dissipation();

        // dy approaches 0 as y -> inf
        //               depth as y -> -inf
        const SimTK::Real dy =
                0.5 * (sqrt(square(depth) + m_depthOffsetSquared) + depth);
        const SimTK::Real voidStiffness = 1.0; // N/m
        force[1] = a * square(dy) * (1 + b * depthRate) + voidStiffness * depth;

        // Friction (TODO handle 3D).
        const SimTK::Real velSlidingScaling =
                get_tangent_velocity_scaling_factor();
        const SimTK::Real transition = tanh(velSliding / velSlidingScaling);

        const SimTK::Real frictionForce =
                -transition * get_friction_coefficient() * force[1];
        force[0] = frictionForce;
        return force;
    }

    // TODO potential energy.
private:
    void constructProperties() {
        constructProperty_stiffness(2.0e6);
        constructProperty_dissipation(1.0);
        constructProperty_friction_coefficient(1.0);
        constructProperty_tangent_velocity_scaling_factor(0.05);
        constructProperty_depth_offset(0.001);
    }
    SimTK::Real m_depthOffsetSquared;
};

} // namespace OpenSim

#endif // OPENSIM_STATIONPLANECONTACTFORCE_H
