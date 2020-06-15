/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxWholeBodyTracking.cpp                                 *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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
#include <Moco/osimMoco.h>

using namespace OpenSim;

class /*TODO OSIMMOCO_API*/AckermannVanDenBogert2010Force : public Force {
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
        const SimTK::Real z0 = exp(-velSliding / velSlidingScaling);
        // TODO decide direction!!!

        const SimTK::Real frictionForce =
                -(1 - z0) / (1 + z0) * get_friction_coefficient() * force[1];

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

    // TODO potential energy.
private:
    void constructProperties() {
        constructProperty_friction_coefficient(1.0);
        constructProperty_stiffness(5e7);
        constructProperty_dissipation(1.0);
        constructProperty_tangent_velocity_scaling_factor(0.05);
    }
};

enum class DOFs {
    PTX_PTY_PRZ,
    PTX_PTY_PRZ_HIPRZ,
    PTX_PTY_PRZ_HIPRZ_KNEERZ,
    PTX_PTY_PRZ_HIPRZ_KNEERZ_ANKLERZ
};

Model createModel(DOFs dofs) {
    Model model;
    model.setName("right_hip");

    using SimTK::Vec3;
    using SimTK::Inertia;

    // Borrowed from the Dynamic Walker example.
    double pelvisWidth = 0.20;
    double thighLength = 0.40;
    double shankLength = 0.435;
    double footLength = 0.20;

    // Create bodies
    OpenSim::Body* pelvis = nullptr;
    OpenSim::Body* thigh = nullptr;
    OpenSim::Body* shank = nullptr;
    OpenSim::Body* foot = nullptr;
    if (dofs >= DOFs::PTX_PTY_PRZ) {
        pelvis = new OpenSim::Body("pelvis", 40, Vec3(0), Inertia(1));
        auto* pelvis_geom = new OpenSim::Ellipsoid;
        pelvis_geom->set_radii(Vec3(0.03, 0.03, pelvisWidth / 2.0));
        pelvis->attachGeometry(pelvis_geom);
        model.addBody(pelvis);
    }

    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ) {
        thigh = new OpenSim::Body("thigh", 10, Vec3(0), Inertia(1));
        auto* thigh_geom = new OpenSim::Ellipsoid;
        thigh_geom->set_radii(Vec3(0.03, thighLength / 2.0, 0.03));
        thigh->attachGeometry(thigh_geom);
        model.addBody(thigh);
    }

    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ) {
        shank = new OpenSim::Body("shank", 10, Vec3(0), Inertia(1));
        auto* shank_geom = new OpenSim::Ellipsoid;
        shank_geom->set_radii(Vec3(0.03, shankLength / 2.0, 0.03));
        shank->attachGeometry(shank_geom);
        model.addBody(shank);
    }

    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ_ANKLERZ) {
        foot = new OpenSim::Body("foot", 5, Vec3(0), Inertia(1));
        auto* foot_geom = new OpenSim::Ellipsoid;
        foot_geom->set_radii(Vec3(footLength / 2.0, 0.03, 0.03));
        foot->attachGeometry(foot_geom);
        model.addBody(foot);
    }

    Coordinate* p_tx = nullptr;
    Coordinate* p_ty = nullptr;
    Coordinate* p_rz = nullptr;
    Coordinate* hip_rz = nullptr;
    Coordinate* knee_rz = nullptr;
    Coordinate* ankle_rz = nullptr;

    // Create free joint between ground and pelvis
    //auto* gp = new FreeJoint("gp",
    //        model.getGround(), Vec3(0, 1.0, 0), Vec3(0),
    //        *pelvis, Vec3(0), Vec3(0));
    //auto& p_tx = gp->updCoordinate(FreeJoint::Coord::TranslationX);
    //p_tx.setName("p_tx");
    //auto& p_ty = gp->updCoordinate(FreeJoint::Coord::TranslationY);
    //p_ty.setName("p_ty");
    //auto& p_tz = gp->updCoordinate(FreeJoint::Coord::TranslationZ);
    //p_tz.setName("p_tz");
    //auto& p_rx = gp->updCoordinate(FreeJoint::Coord::Rotation1X);
    //p_rx.setName("p_rx");
    //auto& p_ry = gp->updCoordinate(FreeJoint::Coord::Rotation2Y);
    //p_ry.setName("p_ry");
    //auto& p_rz = gp->updCoordinate(FreeJoint::Coord::Rotation3Z);
    //p_rz.setName("p_rz");
    //model.addJoint(gp);

    if (dofs >= DOFs::PTX_PTY_PRZ) {
        auto* gp = new PlanarJoint("gp",
                model.getGround(), Vec3(0, 1.0, 0), Vec3(0),
                *pelvis, Vec3(0), Vec3(0));
        p_tx = &gp->updCoordinate(PlanarJoint::Coord::TranslationX);
        p_tx->setName("p_tx");
        p_ty = &gp->updCoordinate(PlanarJoint::Coord::TranslationY);
        p_ty->setName("p_ty");
        p_rz = &gp->updCoordinate(PlanarJoint::Coord::RotationZ);
        p_rz->setName("p_rz");
        model.addJoint(gp);
    }

    // Create hip joint
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ) {
        auto* hip = new PinJoint("hip",
                *pelvis, Vec3(0, 0, pelvisWidth/2.0), Vec3(0),
                *thigh, Vec3(0, thighLength/2.0, 0), Vec3(0, 0, 0));
        //auto& hip_rx = hip->updCoordinate(BallJoint::Coord::Rotation1X);
        //hip_rx.setName("hip_rx");
        //auto& hip_ry = hip->updCoordinate(BallJoint::Coord::Rotation2Y);
        //hip_ry.setName("hip_ry");
        hip_rz = &hip->updCoordinate();
        hip_rz->setName("hip_rz");
        model.addJoint(hip);
    }

    // Create knee joint
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ) {
        auto* knee = new PinJoint("knee",
                *thigh, Vec3(0, -thighLength/2.0, 0), Vec3(0),
                *shank, Vec3(0, shankLength/2.0, 0), Vec3(0));
        knee_rz = &knee->updCoordinate();
        knee_rz->setName("knee_rz");
        model.addJoint(knee);
    }

    // Create ankle joint
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ_ANKLERZ) {
        auto* ankle = new PinJoint("ankle",
                *shank, Vec3(0, -shankLength / 2.0, 0), Vec3(0),
                *foot, Vec3(-footLength / 2.0, 0, 0), Vec3(0));
        ankle_rz = &ankle->updCoordinate();
        ankle_rz->setName("ankle_rz");
        model.addJoint(ankle);
    }

    // Add markers
    // TODO
    Marker* hip_joint_center = nullptr;
    Marker* knee_joint_center = nullptr;
    Marker* heel = nullptr;
    Marker* ball = nullptr;
    if (dofs >= DOFs::PTX_PTY_PRZ) {
        hip_joint_center = new Marker("hip_joint_center", *pelvis,
                Vec3(0, 0, pelvisWidth / 2.0));
        model.addComponent(hip_joint_center);
    }
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ) {
        knee_joint_center = new Marker("knee_joint_center", *thigh,
                Vec3(0, -thighLength / 2.0, 0));
        model.addComponent(knee_joint_center);
    }
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ) {
        heel = new Marker("heel", *shank, Vec3(0, -shankLength / 2.0, 0));
        model.addComponent(heel);

        ball = new Marker("ball", *foot, Vec3(+footLength / 2.0, 0, 0));
        model.addComponent(ball);
    }

    // Add actuators
    // ground-pelvis
    if (dofs >= DOFs::PTX_PTY_PRZ) {
        auto* tau_p_tx = new CoordinateActuator();
        tau_p_tx->setCoordinate(p_tx);
        tau_p_tx->setName("tau_p_tx");
        tau_p_tx->setOptimalForce(1);
        model.addComponent(tau_p_tx);

        auto* tau_p_ty = new CoordinateActuator();
        tau_p_ty->setCoordinate(p_ty);
        tau_p_ty->setName("tau_p_ty");
        tau_p_ty->setOptimalForce(1);
        model.addComponent(tau_p_ty);

        //auto* tau_p_tz = new CoordinateActuator();
        //tau_p_tz->setCoordinate(&p_tz);
        //tau_p_tz->setName("tau_p_tz");
        //tau_p_tz->setOptimalForce(1);
        //model.addComponent(tau_p_tz);

        //auto* tau_p_rx = new CoordinateActuator();
        //tau_p_rx->setCoordinate(&p_rx);
        //tau_p_rx->setName("tau_p_rx");
        //tau_p_rx->setOptimalForce(1);
        //model.addComponent(tau_p_rx);

        //auto* tau_p_ry = new CoordinateActuator();
        //tau_p_ry->setCoordinate(&p_ry);
        //tau_p_ry->setName("tau_p_ry");
        //tau_p_ry->setOptimalForce(1);
        //model.addComponent(tau_p_ry);

        auto* tau_p_rz = new CoordinateActuator();
        tau_p_rz->setCoordinate(p_rz);
        tau_p_rz->setName("tau_p_rz");
        tau_p_rz->setOptimalForce(1);
        model.addComponent(tau_p_rz);
    }

    // hip
    //auto* tau_hip_rx = new CoordinateActuator();
    //tau_hip_rx->setCoordinate(&hip_rx);
    //tau_hip_rx->setName("tau_hip_rx");
    //tau_hip_rx->setOptimalForce(1);
    //model.addComponent(tau_hip_rx);

    //auto* tau_hip_ry = new CoordinateActuator();
    //tau_hip_ry->setCoordinate(&hip_ry);
    //tau_hip_ry->setName("tau_hip_ry");
    //tau_hip_ry->setOptimalForce(1);
    //model.addComponent(tau_hip_ry);

    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ) {
        auto* tau_hip_rz = new CoordinateActuator();
        tau_hip_rz->setCoordinate(hip_rz);
        tau_hip_rz->setName("tau_hip_rz");
        tau_hip_rz->setOptimalForce(1);
        model.addComponent(tau_hip_rz);
    }

    //knee
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ) {
        auto* tau_knee_rz = new CoordinateActuator();
        tau_knee_rz->setCoordinate(knee_rz);
        tau_knee_rz->setName("tau_knee_rz");
        tau_knee_rz->setOptimalForce(1);
        model.addComponent(tau_knee_rz);
    }

    // ankle
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ_ANKLERZ) {
        auto* tau_ankle_rz = new CoordinateActuator();
        tau_ankle_rz->setCoordinate(ankle_rz);
        tau_ankle_rz->setName("tau_ankle_rz");
        tau_ankle_rz->setOptimalForce(1);
        model.addComponent(tau_ankle_rz);
    }


    // Contact
    if (true) {
        const double stiffness = 5e4; // TODO 5e2;
        const double friction_coefficient = 1;
        const double velocity_scaling = 0.05;
        if (dofs >= DOFs::PTX_PTY_PRZ) {
            auto* contact = new AckermannVanDenBogert2010Force();
            contact->setName("hip_contact");
            contact->set_stiffness(stiffness);
            contact->set_friction_coefficient(friction_coefficient);
            contact->set_tangent_velocity_scaling_factor(velocity_scaling);
            model.addComponent(contact);
            contact->connectSocket_station(*hip_joint_center);
        }
        if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ) {
            auto* contact = new AckermannVanDenBogert2010Force();
            contact->setName("knee_contact");
            contact->set_stiffness(stiffness);
            contact->set_friction_coefficient(friction_coefficient);
            contact->set_tangent_velocity_scaling_factor(velocity_scaling);
            model.addComponent(contact);
            contact->connectSocket_station(*knee_joint_center);
        }
        if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ) {
            auto* heel_contact = new AckermannVanDenBogert2010Force();
            heel_contact->setName("heel_contact");
            heel_contact->set_stiffness(stiffness);
            heel_contact->set_friction_coefficient(friction_coefficient);
            heel_contact->set_tangent_velocity_scaling_factor(velocity_scaling);
            model.addComponent(heel_contact);
            heel_contact->connectSocket_station(*heel);

            auto* ball_contact = new AckermannVanDenBogert2010Force();
            ball_contact->setName("ball_contact");
            ball_contact->set_stiffness(stiffness);
            ball_contact->set_friction_coefficient(friction_coefficient);
            ball_contact->set_tangent_velocity_scaling_factor(velocity_scaling);
            model.addComponent(ball_contact);
            ball_contact->connectSocket_station(*ball);
        }
    }

    model.print("hip_model.osim");

    return model;
}

int main() {

    MocoStudy study;
    study.setName("whole_body_tracking");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& mp = study.updProblem();

    DOFs dofs = DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ_ANKLERZ;

    // Model (dynamics).
    // -----------------
    auto model = createModel(dofs);
    mp.setModel(model);

    auto state = model.initSystem();
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    // Without the next line: Simbody takes steps that are too large and NaNs
    // are generated.
    integrator.setMaximumStepSize(0.05);
    Manager manager(model, state, integrator);
    state = manager.integrate(1.6);
    const auto& statesTimeStepping = manager.getStateStorage();
//    visualize(model, statesTimeStepping);

    // Bounds.
    // -------
    const double defaultMaxSpeed = 7;
    mp.setTimeBounds(0.4, 1.6);
    // ground-pelvis
    if (dofs >= DOFs::PTX_PTY_PRZ) {
        mp.setStateInfo("gp/p_tx/value", { -10, 10 });
        mp.setStateInfo("gp/p_tx/speed", { -defaultMaxSpeed, defaultMaxSpeed });
        mp.setStateInfo("gp/p_ty/value", { -2, 10 });
        mp.setStateInfo("gp/p_ty/speed", { -defaultMaxSpeed, defaultMaxSpeed });
        //mp.setStateInfo("gp/p_tz/value", { -10, 10 });
        //mp.setStateInfo("gp/p_tz/speed", { -50, 50 });
        //mp.setStateInfo("gp/p_rx/value", { -10, 10 });
        //mp.setStateInfo("gp/p_rx/speed", { -50, 50 });
        //mp.setStateInfo("gp/p_ry/value", { -10, 10 });
        //mp.setStateInfo("gp/p_ry/speed", { -50, 50 });
        mp.setStateInfo("gp/p_rz/value", { -10, 10 });
        mp.setStateInfo("gp/p_rz/speed", { -defaultMaxSpeed, defaultMaxSpeed });
    }
    // hip
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ) {
        //mp.setStateInfo("hip/hip_rx/value", { -10, 10 });
        //mp.setStateInfo("hip/hip_rx/speed", { -50, 50 });
        //mp.setStateInfo("hip/hip_ry/value", { -10, 10 });
        //mp.setStateInfo("hip/hip_ry/speed", { -50, 50 });
        mp.setStateInfo("hip/hip_rz/value", { -10, 10 });
        mp.setStateInfo("hip/hip_rz/speed", { -defaultMaxSpeed, defaultMaxSpeed });
    }
    // knee
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ) {
        mp.setStateInfo("knee/knee_rz/value", {-10, 10});
        mp.setStateInfo("knee/knee_rz/speed", {-10, 10});
    }
    // ankle
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ_ANKLERZ) {
        mp.setStateInfo("ankle/ankle_rz/value", {-10, 10});
        mp.setStateInfo("ankle/ankle_rz/speed", {-defaultMaxSpeed, defaultMaxSpeed});
    }
    // torques
    const double Tmax = 300;
    if (dofs >= DOFs::PTX_PTY_PRZ) {
        mp.setControlInfo("tau_p_tx", { -Tmax, Tmax });
        // Without ground contact, this residual must be able to
        // provide about 2*mass*g.
        mp.setControlInfo("tau_p_ty", { -2000, 2000 });
        //mp.setControlInfo("tau_p_tz", { -Tmax, Tmax });
        //mp.setControlInfo("tau_p_rx", { -Tmax, Tmax });
        //mp.setControlInfo("tau_p_ry", { -Tmax, Tmax });
        mp.setControlInfo("tau_p_rz", { -Tmax, Tmax });
    }
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ) {
        //mp.setControlInfo("tau_hip_rx", { -Tmax, Tmax });
        //mp.setControlInfo("tau_hip_ry", { -Tmax, Tmax });
        mp.setControlInfo("tau_hip_rz", { -Tmax, Tmax });
    }
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ) {
        mp.setControlInfo("tau_knee_rz", {-Tmax, Tmax});
    }
    if (dofs >= DOFs::PTX_PTY_PRZ_HIPRZ_KNEERZ_ANKLERZ) {
        mp.setControlInfo("tau_ankle_rz", {-Tmax, Tmax});
    }

    MocoStateTrackingCost trackingCost;
    auto ref = STOFileAdapter::read("state_reference.mot");

    // Alter data.
    ref.updDependentColumn("gp/p_ty/value") -= 1.22;
    const auto& reftime = ref.getIndependentColumn();
    auto reftx = ref.updDependentColumn("gp/p_tx/value");
    // I got this speed with "guess and check" by observing, in
    // visualization, if the foot stayed in the same spot during stance.
    const double walkingSpeed = 1.05; // m/s
    for (int i = 0; i < reftx.nrow(); ++i) {
        reftx[i] += walkingSpeed * reftime[i];
    }
    auto refFilt = filterLowpass(ref, 10.0, true);

    //auto model = mp.getPhase().getModel();
    model.initSystem();
    auto refFilt2 = refFilt;
    model.getSimbodyEngine().convertDegreesToRadians(refFilt2);
    STOFileAdapter::write(refFilt2, "state_reference_radians.sto");

    trackingCost.setReference(refFilt);
    trackingCost.setAllowUnusedReferences(true);
    //trackingCost.setWeightForState("gp/p_rz/value", 100.0);
    //trackingCost.setWeightForState("gp/p_tx/value", 25.0);
    //trackingCost.setWeightForState("gp/p_ty/value", 10.0);
    //trackingCost.setWeightForState("hip/hip_rz/value", 2.0);
    mp.addGoal(trackingCost);

    // Takes longer to solve with this cost:
    // With implicit, and 5e4 stiffness, doesn't not converge within 1000
    // iterations:
    //MocoControlGoal controlCost;
    //controlCost.setName("effort");
    //controlCost.set_weight(0.00000001);
    //mp.addGoal(controlCost);

    // Configure the solver.
    // =====================
    MocoTropterSolver& ms = study.initTropterSolver();
    ms.set_multibody_dynamics_mode("implicit");
    ms.set_num_mesh_intervals(50);
    ms.set_optim_max_iterations(1000);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_hessian_approximation("exact");

    MocoTrajectory guess = ms.createGuess();
    guess.setStatesTrajectory(refFilt2, true, true);
    guess.write("states_guess.sto");
    //MocoTrajectory guess("sandboxWholeBodyTracking_solution.sto");
    ms.setGuess(guess);
    // Using the correct initial guess reduces computational time by 4x.

    study.visualize(guess);

    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve().unseal();
    solution.write("sandboxWholeBodyTracking_solution.sto");

    study.visualize(solution);

    // Notes:
    // All dofs and no contact takes ~70 seconds to solve.
    // dofs = PTX_PTY_PRZ_HIPRZ and stiffness=5e1 and friccoeff=0.1:
    //      converges in 258 iterations (~150 seconds).
    //      "solved to acceptable level"
    // Decresaing stiffness to 1 does not seem to help much (a few more
    // iteratiosns, actually).
    // dofs = all and stiffness=1 and friccoeff=0.1:
    //      converges in 369 iterations (~720 seconds).
    //      "solved to acceptable level"
    // dofs = all and stiffness=1 and friccoeff=0:
    //      converged in 79 iterations (~155 seconds).
    //      "solved to acceptable level"
    // dofs = no ankle, and stiffness=5e2 and friccoeff=0:
    //      converged in 513 iterations (~630 seconds).
    //      "solved to acceptable level"
    // dofs = no ankle, and stiffness=5e3 and friccoeff=0:
    //      does not solve; it's as if the pty actuator doesn't work; the pty
    //      coordinate looks like parabolic arc.

    // With implicit dynamics mode:
    // dofs = all, stiffness=5e4, friccoeff=1, velocity scaling=0.05
    //      converged in 116 iterations (~215 seconds).
    //      "Optimal Solution Found."

    // TODO
    // TODO maybe the sparsity detection is wrong?
    // The hip_rz and knee_rz speed defects seem the hardest to satisfy.

    // TODO
    // loosen constraint tolerance.
    // more filtering of input kinematics.
    // successive guesses.
    // build model from ground up.
    // implicit dynamics.
    // use forward simulation for initial guess.
    // alter finite diff step size.

    // Almost all the time is spent computing the Hessian, and 2x more time
    // is in the constraints Hessian than in the objective Hessian.

    // TODO I don't think we can use treadmill data for this...

    // TODO it seems like friction is the real issue.

    return EXIT_SUCCESS;

}
