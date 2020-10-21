#ifndef OPENSIM_STATIONPLANECONTACTFORCE_H
#define OPENSIM_STATIONPLANECONTACTFORCE_H
/* -------------------------------------------------------------------------- *
 * OpenSim: StationPlaneContactForce.h                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco, Chris Dembia                                   *
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
#include <OpenSim/Simulation/osimSimulation.h>

namespace OpenSim {

/** This class models compliant point contact with a ground plane y=0.

@underdevelopment */
class OSIMMOCO_API StationPlaneContactForce : public Force {
OpenSim_DECLARE_ABSTRACT_OBJECT(StationPlaneContactForce, Force);
public:
    OpenSim_DECLARE_OUTPUT(force_on_station, SimTK::Vec3,
            calcContactForceOnStation, SimTK::Stage::Velocity);

    OpenSim_DECLARE_SOCKET(station, Station,
            "The body-fixed point that can contact the plane.");

    void computeForce(const SimTK::State& s,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& /*generalizedForces*/) const override {
        const SimTK::Vec3 force = calcContactForceOnStation(s);
        const auto& pt = getConnectee<Station>("station");
        const auto& pos = pt.getLocationInGround(s);
        const auto& frame = pt.getParentFrame();
        applyForceToPoint(s, frame, pt.get_location(), force, bodyForces);
        applyForceToPoint(s, getModel().getGround(), pos, -force, bodyForces);
    }

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
        const SimTK::Vec3 force = calcContactForceOnStation(s);
        values.append(force[0]);
        values.append(force[1]);
        values.append(force[2]);
        return values;
    }
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& s,
            SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const override;

    // TODO rename to computeContactForceOnStation
    virtual SimTK::Vec3 calcContactForceOnStation(
            const SimTK::State& s) const = 0;
};

/** This class is still under development. */
class OSIMMOCO_API AckermannVanDenBogert2010Force
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
    SimTK::Vec3 calcContactForceOnStation(const SimTK::State& s)
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



/** This contact model is from the following paper:
Meyer A. J., Eskinazi, I., Jackson, J. N., Rao, A. V., Patten, C., & Fregly,
B. J. (2016). Muscle Synergies Facilitate Computational Prediction of
Subject-Specific Walking Motions. Frontiers in Bioengineering and
Biotechnology, 4, 1055–27. http://doi.org/10.3389/fbioe.2016.00077 
    
@underdevelopment */
class OSIMMOCO_API MeyerFregly2016Force
        : public StationPlaneContactForce {
OpenSim_DECLARE_CONCRETE_OBJECT(MeyerFregly2016Force,
        StationPlaneContactForce);
public:
    OpenSim_DECLARE_PROPERTY(stiffness, double,
            "Spring stiffness in N/m (default: 1e4).");
    OpenSim_DECLARE_PROPERTY(dissipation, double,
            "Dissipation coefficient in s/m (default: 0.01).");
    OpenSim_DECLARE_PROPERTY(tscale, double,
            "TODO");

    MeyerFregly2016Force() {
        constructProperties();
    }

    /// Compute the force applied to body to which the station is attached, at
    /// the station, expressed in ground.
    SimTK::Vec3 calcContactForceOnStation(const SimTK::State& s)
    const override {
        SimTK::Vec3 force(0);
        const auto& pt = getConnectee<Station>("station");
        const auto& pos = pt.getLocationInGround(s);
        const auto& vel = pt.getVelocityInGround(s);
        const SimTK::Real y = pos[1];
        const SimTK::Real velNormal = vel[1];
        // TODO should project vel into ground.
        const SimTK::Real velSliding = vel[0];
        // const SimTK::Real depth = 0 - y;
        const SimTK::Real depthRate = 0 - velNormal;
        const SimTK::Real Kval = get_stiffness();
        const SimTK::Real Cval = get_dissipation();
        const SimTK::Real tscale = get_tscale();
        const SimTK::Real klow = 1e-1 / (tscale * tscale);
        const SimTK::Real h = 1e-3;
        const SimTK::Real c = 5e-4;
        const SimTK::Real ymax = 1e-2;

        /// Normal force.
        const SimTK::Real vp = (Kval + klow) / (Kval - klow);
        const SimTK::Real sp = (Kval - klow) / 2;

        const SimTK::Real constant =
                -sp * (vp * ymax - c * log(cosh((ymax + h) / c)));

        SimTK::Real Fspring =
                -sp * (vp * y - c * log(cosh((y + h) / c))) - constant;
        if (SimTK::isNaN(Fspring) || SimTK::isInf(Fspring)) {
            Fspring = 0;
        }

        const SimTK::Real Fy = Fspring * (1 + Cval * depthRate);

        force[1] = Fy;

        /// Friction force.
        const SimTK::Real mu_d = 1;
        const SimTK::Real latchvel = 0.05; // m/s

        const SimTK::Real mu = mu_d * tanh(velSliding / latchvel / 2);
        force[0] = -force[1] * mu;

        return force;
    }

private:
    void constructProperties() {
        constructProperty_stiffness(1e4);
        constructProperty_dissipation(1e-2);
        constructProperty_tscale(1.0);
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
class OSIMMOCO_API EspositoMiller2018Force
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
    SimTK::Vec3 calcContactForceOnStation(const SimTK::State& s)
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
