#ifndef MUSCOLLO_MUSCOLLOSANDBOXSHARED_H
#define MUSCOLLO_MUSCOLLOSANDBOXSHARED_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MuscolloSandboxShared.h                                  *
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

#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <Muscollo/MucoCost.h>

namespace OpenSim {

/// Similar to CoordinateActuator (simply produces a generalized force) but
/// with first-order linear activation dynamics. This actuator has one state
/// variable, `activation`, with \f$ \dot{a} = (u - a) / \tau \f$, where
/// \f$ a \f$ is activation, \f$ u $\f is excitation, and \f$ \tau \f$ is the
/// activation time constant (there is no separate deactivation time constant).
/// <b>Default %Property Values</b>
/// @verbatim
/// activation_time_constant: 0.01
/// default_activation: 0.5
/// @dverbatim
class /*OSIMMUSCOLLO_API*/
ActivationCoordinateActuator : public CoordinateActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ActivationCoordinateActuator,
            CoordinateActuator);
public:
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
    "Smaller value means activation can change more rapidly (units: seconds).");

    OpenSim_DECLARE_PROPERTY(default_activation, double,
    "Value of activation in the default state returned by initSystem().");

    ActivationCoordinateActuator() {
        constructProperties();
    }

    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("activation", SimTK::Stage::Dynamics);
    }

    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, "activation", get_default_activation());
    }

    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        set_default_activation(getStateVariableValue(s, "activation"));
    }

    // TODO no need to do clamping, etc; CoordinateActuator is bidirectional.
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        const auto& tau = get_activation_time_constant();
        const auto& u = getControl(s);
        const auto& a = getStateVariableValue(s, "activation");
        const SimTK::Real adot = (u - a) / tau;
        setStateVariableDerivativeValue(s, "activation", adot);
    }

    double computeActuation(const SimTK::State& s) const override {
        return getStateVariableValue(s, "activation") * getOptimalForce();
    }
private:
    void constructProperties() {
        constructProperty_activation_time_constant(0.010);
        constructProperty_default_activation(0.5);
    }
};

class /*TODO OSIMMUSCOLLO_API*/AckermannVanDenBogert2010Force : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(AckermannVanDenBogert2010Force, Force);
public:
    OpenSim_DECLARE_PROPERTY(stiffness, double, "TODO N/m^3");
    OpenSim_DECLARE_PROPERTY(dissipation, double, "TODO s/m");
    OpenSim_DECLARE_PROPERTY(friction_coefficient, double, "TODO");
    // TODO rename to transition_velocity
    OpenSim_DECLARE_PROPERTY(tangent_velocity_scaling_factor, double, "TODO");

    OpenSim_DECLARE_OUTPUT(force_on_station, SimTK::Vec3, calcContactForce,
            SimTK::Stage::Velocity);

    OpenSim_DECLARE_SOCKET(station, Station, "TODO");

    AckermannVanDenBogert2010Force() {
        constructProperties();
    }

    /// Compute the force applied to body to which the station is attached, at
    /// the station, expressed in ground.
    SimTK::Vec3 calcContactForce(const SimTK::State& s) const {
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
        const SimTK::Real a = get_stiffness();
        const SimTK::Real b = get_dissipation();
        if (depth > 0) {
            force[1] = fmax(0, a * pow(depth, 3) * (1 + b * depthRate));
        }
        const SimTK::Real voidStiffness = 1.0; // N/m
        force[1] += voidStiffness * depth;

        const SimTK::Real velSlidingScaling =
                get_tangent_velocity_scaling_factor();
        //// The paper used (1 - exp(-x)) / (1 + exp(-x)) = tanh(2x).
        //// tanh() has a wider domain than using exp().
        const SimTK::Real transition = tanh(velSliding / velSlidingScaling / 2);

        const SimTK::Real frictionForce =
                -transition * get_friction_coefficient() * force[1];

        force[0] = frictionForce;
        return force;
    }
    void computeForce(const SimTK::State& s,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& /*generalizedForces*/) const override {
        const SimTK::Vec3 force = calcContactForce(s);
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
        const SimTK::Vec3 force = calcContactForce(s);
        values.append(force[0]);
        values.append(force[1]);
        values.append(force[2]);
        return values;
    }

    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& s,
            SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const override {
        Super::generateDecorations(fixed, hints, s, geoms);
        if (!fixed) {
            getModel().realizeVelocity(s);
            // Normalize contact force vector by body weight so that the line
            // is 1 meter long if the contact force magnitude is equal to
            // body weight.
            const double arrowLengthPerForce = 1.0 / 1000.0; // meters / Newton
            //const double mg =
            //        getModel().getTotalMass(s) * getModel().getGravity().norm();
            // TODO avoid recalculating.
            const auto& pt = getConnectee<Station>("station");
            const auto pt1 = pt.getLocationInGround(s);
            const SimTK::Vec3 force = calcContactForce(s);
            // std::cout << "DEBUGgd force " << force << std::endl;
            const SimTK::Vec3 pt2 = pt1 + force * arrowLengthPerForce; // mg;
            SimTK::DecorativeLine line(pt1, pt2);
            line.setColor(SimTK::Green);
            line.setLineThickness(0.10);
            geoms.push_back(line);

            // TODO move to fixed.
            SimTK::DecorativeSphere sphere;
            sphere.setColor(SimTK::Green);
            sphere.setRadius(0.01);
            sphere.setBodyId(pt.getParentFrame().getMobilizedBodyIndex());
            sphere.setRepresentation(SimTK::DecorativeGeometry::DrawWireframe);
            sphere.setTransform(SimTK::Transform(pt.get_location()));
            geoms.push_back(sphere);
        }
    }

    // TODO potential energy.
private:
    void constructProperties() {
        constructProperty_friction_coefficient(1.0);
        constructProperty_stiffness(5e7);
        constructProperty_dissipation(1.0);
        constructProperty_tangent_velocity_scaling_factor(0.05);
    }
};

class /*TODO OSIMMUSCOLLO_API*/ MeyerFregly2016Force : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(MeyerFregly2016Force, Force);
public:
    OpenSim_DECLARE_PROPERTY(stiffness, double, "TODO N/m");
    OpenSim_DECLARE_PROPERTY(dissipation, double, "TODO s/m");
    OpenSim_DECLARE_PROPERTY(tscale, double, "TODO");

    OpenSim_DECLARE_OUTPUT(force_on_station, SimTK::Vec3, calcContactForce,
        SimTK::Stage::Velocity);

    OpenSim_DECLARE_SOCKET(station, Station, "TODO");

    MeyerFregly2016Force() {
        constructProperties();
    }

    /// Compute the force applied to body to which the station is attached, at
    /// the station, expressed in ground.
    SimTK::Vec3 calcContactForce(const SimTK::State& s) const {
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
        const SimTK::Real latchvel = 0.05;

        const SimTK::Real mu = mu_d * tanh(velSliding / latchvel / 2);
        force[0] = -force[1] * mu;
        
        return force;
    }

    void computeForce(const SimTK::State& s,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& /*generalizedForces*/) const override {
        const SimTK::Vec3 force = calcContactForce(s);
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
        const SimTK::Vec3 force = calcContactForce(s);
        values.append(force[0]);
        values.append(force[1]);
        values.append(force[2]);
        return values;
    }

    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const override {
        Super::generateDecorations(fixed, hints, s, geoms);
        if (!fixed) {
            getModel().realizeVelocity(s);
            // Normalize contact force vector by body weight so that the line
            // is 1 meter long if the contact force magnitude is equal to
            // body weight.
            const double mg =
                getModel().getTotalMass(s) * getModel().getGravity().norm();
            // TODO avoid recalculating.
            const auto& pt = getConnectee<Station>("station");
            const auto pt1 = pt.getLocationInGround(s);
            const SimTK::Vec3 force = calcContactForce(s);
            // std::cout << "DEBUGgd force " << force << std::endl;
            const SimTK::Vec3 pt2 = pt1 + force / mg;
            SimTK::DecorativeLine line(pt1, pt2);
            line.setColor(SimTK::Green);
            line.setLineThickness(0.10);
            geoms.push_back(line);

            // TODO move to fixed.
            SimTK::DecorativeSphere sphere;
            sphere.setColor(SimTK::Green);
            sphere.setRadius(0.01);
            sphere.setBodyId(pt.getParentFrame().getMobilizedBodyIndex());
            sphere.setRepresentation(SimTK::DecorativeGeometry::DrawWireframe);
            sphere.setTransform(SimTK::Transform(pt.get_location()));
            geoms.push_back(sphere);
        }
    }

private:
    void constructProperties() {
        constructProperty_stiffness(1e4);
        constructProperty_dissipation(1e-2);
        constructProperty_tscale(1.0);
    }

};

// TODO rename ContactForceTracking? ExternalForceTracking?
// TODO add function that saves a file comparing the simulated and tracked GRF.
class MucoForceTrackingCost : public MucoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoForceTrackingCost, MucoCost);
public:
    OpenSim_DECLARE_LIST_PROPERTY(forces, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(tracked_grf_components, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(free_force_window, double, "TODO");

    MucoForceTrackingCost() {
        constructProperties();
    }
protected:
    void initializeImpl() const override {
        m_forces.clear();
        for (int i = 0; i < getProperty_forces().size(); ++i) {
            m_forces.emplace_back(
                    dynamic_cast<const AckermannVanDenBogert2010Force&>(
                            getModel().getComponent(get_forces(i))));
        }
    }

    void calcIntegralCostImpl(const SimTK::State& state, double& integrand)
    const override {
        getModel().realizeVelocity(state);
        SimTK::Vec3 netForce(0);
        SimTK::Vec3 ref(0);
        for (const auto& force : m_forces) {
            netForce += force->calcContactForce(state);
        }
        SimTK::Vector timeVec(1, state.getTime());
        ref[0] = m_refspline_x.calcValue(timeVec);
        ref[1] = m_refspline_y.calcValue(timeVec);

        // TODO: flag to specify error power?
        double error = 0;
        if (get_tracked_grf_components() == "all") {
            error = (netForce - ref).normSqr();
        } else if (get_tracked_grf_components() == "horizontal") {
            error = abs(netForce[0] - ref[0]);
        } else if (get_tracked_grf_components() == "vertical") {
            error = abs(netForce[1] - ref[1]);
        }

        if (error < get_free_force_window()) {
            integrand += 0.0;
        } else {
            integrand += error;
        }
        
    }
private:
    void constructProperties() {
        constructProperty_forces();
        constructProperty_tracked_grf_components("all");
        constructProperty_free_force_window(0.0);
    }
    mutable 
    std::vector<SimTK::ReferencePtr<const AckermannVanDenBogert2010Force>>
        m_forces;


public: // TODO
    mutable GCVSpline m_refspline_x;
    mutable GCVSpline m_refspline_y;
};

} // namespace OpenSim

#endif // MUSCOLLO_MUSCOLLOSANDBOXSHARED_H
