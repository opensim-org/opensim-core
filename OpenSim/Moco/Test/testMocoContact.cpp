/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoContact.cpp                                          *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2025 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 * Contributors: Antoine Falisse, Nicholas Bianco                             *
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

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/MeyerFregly2016Force.h>
#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/ContactSphere.h>
#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>

#include <catch2/catch_all.hpp>
#include "Testing.h"

const double FRICTION_COEFFICIENT = 0.7;

using namespace OpenSim;
using SimTK::Vec3;

namespace {
    // Create a simple 2D point mass model with a MeyerFregly2016Force.
    Model create2DPointMassModel() {
        Model model;
        model.setName("point_mass");
        auto* intermed = new Body("intermed", 0, Vec3(0), SimTK::Inertia(0));
        model.addComponent(intermed);
        auto* body = new Body("body", 50.0, Vec3(0), SimTK::Inertia(1));
        model.addComponent(body);

        // Allows translation along x.
        auto* jointX = new SliderJoint();
        jointX->setName("tx");
        jointX->connectSocket_parent_frame(model.getGround());
        jointX->connectSocket_child_frame(*intermed);
        auto& coordX = jointX->updCoordinate(SliderJoint::Coord::TranslationX);
        coordX.setName("tx");
        model.addComponent(jointX);

        // The joint's x axis must point in the global "+y" direction.
        auto* jointY = new SliderJoint("ty",
                *intermed, Vec3(0), Vec3(0, 0, 0.5 * SimTK::Pi),
                *body, Vec3(0), Vec3(0, 0, 0.5 * SimTK::Pi));
        auto& coordY = jointY->updCoordinate(SliderJoint::Coord::TranslationX);
        coordY.setName("ty");
        model.addComponent(jointY);

        auto* station = new Station();
        station->setName("contact_point");
        model.addComponent(station);
        station->connectSocket_parent_frame(*body);

        auto* force = new MeyerFregly2016Force();
        force->setName("contact");
        force->set_stiffness(1e4);
        force->set_dissipation(1.0);
        force->set_viscous_friction(FRICTION_COEFFICIENT);
        force->set_dynamic_friction(FRICTION_COEFFICIENT);
        force->connectSocket_station(*station);
        model.addComponent(force);

        return model;
    }

    // Test that, with dissipation, the MeyerFregly2016Force contact force 
    // settles to the weight of the system. This is the same type of test done 
    // in OpenSim's testForces for HuntCrossleyForce. The test is performed with 
    // both time stepping and direct collocation.
    SimTK::Real testMeyerFregly2016Force_NormalForce() {
        Model model = create2DPointMassModel();
        model.finalizeConnections();

        SimTK::Real weight;
        {
            SimTK::State state = model.initSystem();
            weight = model.getTotalMass(state) * (-model.getGravity()[1]);
        }

        const SimTK::Real y0 = 0.5;
        const SimTK::Real finalTime = 3.0;

        // Time stepping.
        // --------------
        SimTK::Real finalHeightTimeStepping;
        {
            // Run time-stepping integration.
            SimTK::State state = model.initSystem();
            model.setStateVariableValue(state, "ty/ty/value", y0);
            Manager manager(model);
            manager.setIntegratorAccuracy(1e-6);
            manager.initialize(state);
            state = manager.integrate(finalTime);

            // Extract the contact force.
            TimeSeriesTable statesTable = manager.getStatesTable();
            auto& contact = 
                    model.template getComponent<MeyerFregly2016Force>("contact");
            model.realizeVelocity(state);
            const Vec3 contactForce = contact.getContactForceOnStation(state);

            // Check that the contact force is the weight of the system.
            // The horizontal force is not quite zero, maybe from a buildup of
            // numerical error (tightening the accuracy reduces this force).
            // The system is planar, so there is no force in the z direction.
            CHECK_THAT(contactForce[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
            CHECK_THAT(contactForce[1], Catch::Matchers::WithinRel(weight, 1e-3));
            CHECK_THAT(contactForce[2], Catch::Matchers::WithinRel(0.0, 1e-10));

            finalHeightTimeStepping =
                    model.getStateVariableValue(state, "ty/ty/value");
        }

        // Direct collocation.
        // -------------------
        // This is a simulation (initial value problem), not a trajectory
        // optimization.
        SimTK::Real finalHeightDircol;
        {
            MocoStudy study;
            MocoProblem& problem = study.updProblem();
            problem.setModelAsCopy(model);
            problem.setTimeBounds(0, finalTime);
            problem.setStateInfo("/tx/tx/value", {-1, 1}, 0);
            problem.setStateInfo("/ty/ty/value", {-0.5, 1}, y0);
            problem.setStateInfo("/tx/tx/speed", {-10, 10}, 0);
            problem.setStateInfo("/ty/ty/speed", {-10, 10}, 0);

            auto& solver = study.initCasADiSolver();
            solver.set_num_mesh_intervals(100);
            solver.set_transcription_scheme("legendre-gauss-radau-3");

            MocoSolution solution = study.solve();
            // solution.write("testContact_solution_testNormalForce.sto");
            // study.visualize(solution);

            // Extract the contact force.
            auto statesTraj = solution.exportToStatesTrajectory(problem);
            const auto& finalState = statesTraj.back();
            model.realizeVelocity(finalState);
            auto& contact = 
                    model.template getComponent<MeyerFregly2016Force>("contact");
            const Vec3 contactForce = 
                    contact.getContactForceOnStation(finalState);

            // Check that the contact force is the weight of the system.
            CHECK_THAT(contactForce[0], Catch::Matchers::WithinAbs(0.0, 1e-6));
            CHECK_THAT(contactForce[1], 
                    Catch::Matchers::WithinRel(weight, 1e-3));
            CHECK_THAT(contactForce[2], Catch::Matchers::WithinRel(0.0, 1e-10));

            finalHeightDircol =
                    model.getStateVariableValue(finalState, "ty/ty/value");
        }

        // The two methods should produce the same result.
        CHECK_THAT(finalHeightTimeStepping, 
                Catch::Matchers::WithinAbs(finalHeightDircol, 1e-6));

        return finalHeightTimeStepping;
    }

    // Test the friction component of the MeyerFregly2016Force contact force by 
    // ensuring that the point mass comes to rest after sliding a certain 
    // distance. To make the friction force roughly constant, we want the 
    // equilibrium height of the mass (from 
    // testMeyerFregly2016Force_NormalForce()).
    void testMeyerFregly2016Force_FrictionForce(
                const SimTK::Real& equilibriumHeight) {
        auto model = create2DPointMassModel();
        model.initSystem();

        const SimTK::Real y0 = equilibriumHeight;
        const SimTK::Real finalTime = 3.0;
        const SimTK::Real vx0 = 1.0;

        // Time stepping.
        // --------------
        {
            SimTK::State state = model.initSystem();
            model.setStateVariableValue(state, "ty/ty/value", y0);
            model.setStateVariableValue(state, "tx/tx/speed", vx0);
            Manager manager(model, state);
            state = manager.integrate(finalTime);

            // The system should be at rest.
            OpenSim_CHECK_MATRIX_ABSTOL(state.getU(),
                    SimTK::Vector(state.getNU(), 0.0), 1e-3);
        }

        // Direct collocation.
        // -------------------
        // This is a simulation (initial value problem), not a trajectory
        // optimization.
        {
            MocoStudy study;
            MocoProblem& mp = study.updProblem();
            mp.setModelAsCopy(model);
            mp.setTimeBounds(0, finalTime);
            mp.setStateInfo("/tx/tx/value", {-1, 1}, 0);
            mp.setStateInfo("/ty/ty/value", {-0.5, 1}, y0);
            mp.setStateInfo("/tx/tx/speed", {-10, 10}, vx0);
            mp.setStateInfo("/ty/ty/speed", {-10, 10}, 0);

            auto& ms = study.initCasADiSolver();
            ms.set_num_mesh_intervals(50);
            ms.set_transcription_scheme("legendre-gauss-radau-3");

            MocoSolution solution = study.solve();
            // solution.write("testContact_testFrictionForce_solution.sto");
            // study.visualize(solution);

            auto statesTraj = solution.exportToStatesTrajectory(mp);
            const auto& finalState = statesTraj.back();

            // The system should be at rest.
            OpenSim_CHECK_MATRIX_ABSTOL(finalState.getU(),
                    SimTK::Vector(finalState.getNU(), 0.0), 1e-3);
        }
    }

    // Test our wrapping of SmoothSphereHalfSpaceForce in Moco
    // Create a model with SmoothSphereHalfSpaceForce
    Model createBallHalfSpaceModel(
            double frictionCoefficient = FRICTION_COEFFICIENT,
            double dissipation = 1.0) {
        // Setup model.
        Model model;
        model.setName("BouncingBall_SmoothSphereHalfSpaceForce");
        auto* ball = new Body("ball", 1, Vec3(0), SimTK::Inertia(1));
        model.addComponent(ball);
        auto* groundBall = new PlanarJoint("groundBall", model.getGround(),
            Vec3(0), Vec3(0), *ball, Vec3(0), Vec3(0));
        auto& rz = groundBall->updCoordinate(PlanarJoint::Coord::RotationZ);
        rz.setPrescribedFunction(Constant(0));
        rz.setDefaultIsPrescribed(true);
        model.addComponent(groundBall);
        // Add display geometry.
        double radius = 0.10;
        Sphere bodyGeometry(radius);
        bodyGeometry.setColor(SimTK::Gray);
        ball->attachGeometry(bodyGeometry.clone());
        // Setup contact.
        double stiffness = 10000;
        double staticFriction = frictionCoefficient;
        double dynamicFriction = frictionCoefficient;
        double viscousFriction = 0;
        double transitionVelocity = 0.05;
        double cf = 1e-5;
        double bd = 300;
        double bv = 50;
        Vec3 sphereLocation(0);
        Vec3 halfSpaceLocation(0);
        // Set the plane parallel to the ground.
        Vec3 halfSpaceOrientation(0, 0, -0.5 * SimTK::Pi);
        auto* sphere = new ContactSphere(
                radius, sphereLocation, *ball, "sphere");
        model.addContactGeometry(sphere);
        auto* halfSpace = new ContactHalfSpace(halfSpaceLocation,
                halfSpaceOrientation, model.getGround(), "floor");
        model.addContactGeometry(halfSpace);
        auto* contactBallHalfSpace = new SmoothSphereHalfSpaceForce(
            "contactBallHalfSpace", *sphere, *halfSpace);
        contactBallHalfSpace->set_stiffness(stiffness);
        contactBallHalfSpace->set_dissipation(dissipation);
        contactBallHalfSpace->set_static_friction(staticFriction);
        contactBallHalfSpace->set_dynamic_friction(dynamicFriction);
        contactBallHalfSpace->set_viscous_friction(viscousFriction);
        contactBallHalfSpace->set_transition_velocity(transitionVelocity);
        contactBallHalfSpace->set_constant_contact_force(cf);
        contactBallHalfSpace->set_hertz_smoothing(bd);
        contactBallHalfSpace->set_hunt_crossley_smoothing(bv);
        contactBallHalfSpace->setName("contactBallHalfSpace");
        model.addComponent(contactBallHalfSpace);

        return model;
    }

    // Simple simulation of bouncing ball with dissipation should generate 
    // contact forces that settle to ball weight. This is the same type of test 
    // done in OpenSim's testForces for HuntCrossleyForce. The test is performed 
    // with both time stepping and direct collocation.
    SimTK::Real testSmoothSphereHalfSpaceForce_NormalForce() {
        Model model(createBallHalfSpaceModel());

        SimTK::Real weight;
        {
            SimTK::State state = model.initSystem();
            weight = model.getTotalMass(state) * (-model.getGravity()[1]);
        }

        const SimTK::Real y0 = 0.5;
        const SimTK::Real finalTime = 2.0;

        // Time stepping.
        // --------------
        SimTK::Real finalHeightTimeStepping;
        {
            SimTK::State state = model.initSystem();
            model.setStateVariableValue(state,
                "groundBall/groundBall_coord_2/value", y0);
            Manager manager(model);
            manager.setIntegratorAccuracy(1e-6);
            manager.initialize(state);
            state = manager.integrate(finalTime);

            model.realizeVelocity(state);

            auto& contactBallHalfSpace =
                model.getComponent<SmoothSphereHalfSpaceForce>(
                "contactBallHalfSpace");

            Array<double> contactForces =
                contactBallHalfSpace.getRecordValues(state);
            // no horizontal force
            CHECK_THAT(contactForces[0], Catch::Matchers::WithinAbs(0.0, 1e-4));
            // vertical force is weight
            CHECK_THAT(contactForces[1], 
                Catch::Matchers::WithinAbs(weight, 1e-4));
            // no horizontal force
            CHECK_THAT(contactForces[2], Catch::Matchers::WithinAbs(0.0, 1e-4));
            // no torque
            CHECK_THAT(contactForces[3], Catch::Matchers::WithinAbs(0.0, 1e-4));
            CHECK_THAT(contactForces[4], Catch::Matchers::WithinAbs(0.0, 1e-4));
            CHECK_THAT(contactForces[5], Catch::Matchers::WithinAbs(0.0, 1e-4));

            finalHeightTimeStepping = model.getStateVariableValue(state,
                "groundBall/groundBall_coord_2/value");
        }

        // Direct collocation.
        // -------------------
        // This is a simulation (initial value problem), not a trajectory
        // optimization.
        SimTK::Real finalHeightDircol;
        {
            MocoStudy study;
            MocoProblem& mp = study.updProblem();
            mp.setModelAsCopy(model);
            mp.setTimeBounds(0, finalTime);
            mp.setStateInfo("/groundBall/groundBall_coord_0/value", {-1, 1}, 0);
            mp.setStateInfo("/groundBall/groundBall_coord_1/value", {-1, 1}, 0);
            mp.setStateInfo("/groundBall/groundBall_coord_2/value", {-1, 1}, y0);
            mp.setStateInfo("/groundBall/groundBall_coord_0/speed", {-10, 10}, 0);
            mp.setStateInfo("/groundBall/groundBall_coord_1/speed", {-10, 10}, 0);
            mp.setStateInfo("/groundBall/groundBall_coord_2/speed", {-10, 10}, 0);

            auto& ms = study.initCasADiSolver();
            ms.set_num_mesh_intervals(50);
            ms.set_verbosity(2);
            ms.set_optim_solver("ipopt");

            MocoSolution solution = study.solve();

            auto statesTraj = solution.exportToStatesTrajectory(mp);
            const auto& finalState = statesTraj.back();
            model.realizeVelocity(finalState);

            auto& contactBallHalfSpace =
                model.getComponent<SmoothSphereHalfSpaceForce>(
                    "contactBallHalfSpace");

            Array<double> contactForces =
            contactBallHalfSpace.getRecordValues(finalState);
            // no horizontal force
            CHECK_THAT(contactForces[0],  Catch::Matchers::WithinAbs(0.0, 1e-4));
            // vertical force is weight
            CHECK_THAT(contactForces[1], 
                    Catch::Matchers::WithinAbs(weight, 1e-4));
            // no horizontal force
            CHECK_THAT(contactForces[2], Catch::Matchers::WithinAbs(0.0, 1e-4));
            // no torque
            CHECK_THAT(contactForces[3], Catch::Matchers::WithinAbs(0.0, 1e-4)); 
            CHECK_THAT(contactForces[4], Catch::Matchers::WithinAbs(0.0, 1e-4)); 
            CHECK_THAT(contactForces[5], Catch::Matchers::WithinAbs(0.0, 1e-4));

            finalHeightDircol = model.getStateVariableValue(finalState,
                    "groundBall/groundBall_coord_2/value");
        }

        CHECK_THAT(finalHeightTimeStepping,
                Catch::Matchers::WithinAbs(finalHeightDircol, 1e-5));

        return finalHeightTimeStepping;

    }

    // Test the friction component of the contact force by ensuring that the 
    // ball travels the expected horizontal distance if it starts in the ground.
    // To make the friction force roughly constant, we want the equilibrium 
    // height of the mass (from testSmoothSphereHalfSpaceForce_NormalForce()).
    void testSmoothSphereHalfSpaceForce_FrictionForce(
            const SimTK::Real& equilibriumHeight) {

        Model model(createBallHalfSpaceModel());

        const SimTK::Real y0 = equilibriumHeight;
        const SimTK::Real finalTime = 0.5;
        const SimTK::Real vx0 = 2.5;

        const SimTK::Real g = -model.getGravity()[1];

        // Expected final x position.
        // --------------------------
        // m * vxdot = F = - mu * m * g
        // vx(t) = -mu * g * t + vx0
        // x(t) = -0.5 * mu * g * t^2 + vx0 * t
        // The time at which the point reaches rest:
        // t_rest = vx0 / (mu * g)
        // Final position: x(t_rest)
        const double& mu = FRICTION_COEFFICIENT;
        const SimTK::Real restTime = vx0 / (mu * g);
        CHECK(restTime < finalTime);
        const SimTK::Real expectedFinalX =
                -0.5 * mu * g * pow(restTime, 2) + vx0 * restTime;

        // Time stepping.
        // --------------
        {
            SimTK::State state = model.initSystem();
            model.setStateVariableValue(state,
                "groundBall/groundBall_coord_2/value", y0);
            model.setStateVariableValue(state,
                "groundBall/groundBall_coord_1/speed", vx0);
            Manager manager(model, state);
            state = manager.integrate(finalTime);

            const SimTK::Real finalX = model.getStateVariableValue(
                    state, "groundBall/groundBall_coord_1/value");

            CHECK_THAT(finalX, Catch::Matchers::WithinAbs(expectedFinalX, 0.005));

            // The system should be at rest.
            OpenSim_CHECK_MATRIX_ABSTOL(state.getU(),
                    SimTK::Vector(state.getNU(), 0.0), 1e-3);
        }

        // Direct collocation.
        // -------------------
        // This is a simulation (initial value problem), not a trajectory
        // optimization.
        {
            MocoStudy study;
            MocoProblem& mp = study.updProblem();
            mp.setModelAsCopy(model);
            mp.setTimeBounds(0, finalTime);
            mp.setStateInfo("/groundBall/groundBall_coord_0/value", {-1, 1}, 0);
            mp.setStateInfo("/groundBall/groundBall_coord_1/value", {-1, 1}, 0);
            mp.setStateInfo("/groundBall/groundBall_coord_2/value", {-1, 1}, y0);
            mp.setStateInfo(
                    "/groundBall/groundBall_coord_0/speed", {-10, 10}, 0);
            mp.setStateInfo(
                    "/groundBall/groundBall_coord_1/speed", {-10, 10},vx0);
            mp.setStateInfo(
                    "/groundBall/groundBall_coord_2/speed", {-10, 10}, 0);

            auto& ms = study.initCasADiSolver();
            ms.set_num_mesh_intervals(50);
            ms.set_verbosity(2);
            ms.set_optim_solver("ipopt");

            MocoSolution solution = study.solve();

            auto statesTraj = solution.exportToStatesTrajectory(mp);
            const auto& finalState = statesTraj.back();
            const SimTK::Real finalX = model.getStateVariableValue(
                    finalState, "groundBall/groundBall_coord_1/value");

            CHECK_THAT(finalX, Catch::Matchers::WithinAbs(expectedFinalX, 0.005));

            // The system should be at rest.
            OpenSim_CHECK_MATRIX_ABSTOL(finalState.getU(),
                    SimTK::Vector(finalState.getNU(), 0.0), 1e-3);
        }
    }
}

TEST_CASE("testMeyerFregly2016Force", "[casadi]") {
    const SimTK::Real equilibriumHeight = 
            testMeyerFregly2016Force_NormalForce();
    testMeyerFregly2016Force_FrictionForce(equilibriumHeight);
}

TEST_CASE("testSmoothSphereHalfSpaceForce", "[casadi]") {
    const SimTK::Real equilibriumHeight =
        testSmoothSphereHalfSpaceForce_NormalForce();
    testSmoothSphereHalfSpaceForce_FrictionForce(equilibriumHeight);
}

TEST_CASE("MocoContactTrackingGoal", "[casadi]") {

    // We drop a ball from a prescribed initial height, record the contact
    // force, then solve a trajectory optimization that tracks the recorded
    // contact force and ensure we recover the correct initial height.
    Model model(createBallHalfSpaceModel());

    const double initialHeight = 0.65;
    const SimTK::Real finalTime = 0.8;

    const std::string dataFileName =
            "testContact_MocoContactTrackingGoal_external_loads.sto";

    // Time stepping.
    // --------------
    TimeSeriesTable externalLoadsTimeStepping;
    {
        SimTK::State initialState = model.initSystem();
        model.setStateVariableValue(initialState,
                "groundBall/groundBall_coord_2/value", initialHeight);
        Manager manager(model);
        manager.setIntegratorAccuracy(1e-6);
        manager.initialize(initialState);
        manager.integrate(finalTime);

        auto statesTraj = StatesTrajectory::createFromStatesStorage(
                model, manager.getStateStorage());
        externalLoadsTimeStepping = createExternalLoadsTableForGait(
                model, statesTraj, {"contactBallHalfSpace"}, {});
        STOFileAdapter::write(externalLoadsTimeStepping, dataFileName);
    }

    // Trajectory optimization.
    // ------------------------
    TimeSeriesTable externalLoadsDircol;
    {
        MocoStudy study;
        MocoProblem& problem = study.updProblem();
        problem.setModelAsCopy(model);
        problem.setTimeBounds(0, finalTime);
        problem.setStateInfo(
                "/groundBall/groundBall_coord_0/value", {-1, 1}, 0);
        problem.setStateInfo(
                "/groundBall/groundBall_coord_1/value", {-1, 1}, 0);
        problem.setStateInfo("/groundBall/groundBall_coord_2/value", {-1, 1.5},
                {0.5, 0.7});
        problem.setStateInfo(
                "/groundBall/groundBall_coord_0/speed", {-10, 10}, 0);
        problem.setStateInfo(
                "/groundBall/groundBall_coord_1/speed", {-10, 10}, 0);
        problem.setStateInfo(
                "/groundBall/groundBall_coord_2/speed", {-10, 10}, 0);

        // Add a MocoContactTrackingGoal.
        auto* contactTracking = problem.addGoal<MocoContactTrackingGoal>();
        ExternalLoads extLoads;
        extLoads.setDataFileName(dataFileName);
        auto extForce = std::make_unique<ExternalForce>();
        extForce->setName("right");
        extForce->set_applied_to_body("ball");
        extForce->set_force_identifier("ground_force_r_v");
        extLoads.adoptAndAppend(extForce.release());

        contactTracking->setExternalLoads(extLoads);
        contactTracking->addContactGroup({"contactBallHalfSpace"}, "right");
        contactTracking->setProjection("vector");
        contactTracking->setProjectionVector(SimTK::Vec3(0, 1, 0));

        // Solve the problem.
        auto& solver = study.initCasADiSolver();
        solver.set_num_mesh_intervals(30);

        MocoSolution solution = study.solve();

        const double actualInitialHeight =
                solution.getState("/groundBall/groundBall_coord_2/value").
                        getElt(0, 0);
        CHECK_THAT(actualInitialHeight, 
                Catch::Matchers::WithinAbs(initialHeight, 1e-2));

        externalLoadsDircol = createExternalLoadsTableForGait(model, solution,
                {"contactBallHalfSpace"}, {});
    }

    rootMeanSquare(externalLoadsDircol, "ground_force_r_vy",
            externalLoadsTimeStepping, "ground_force_r_vy",
            0.5);
}

// This is a round-trip test. First, use createExternalLoadsTableForGait() to 
// create a table of external loads based on a simulation with foot-ground 
// contact force elements. Then, use the external loads to apply forces to the 
// model with the contact force elements remove and ensure the accelerations 
// match the accelerations of the original model.
TEST_CASE("createExternalLoadsTableForGait") {
    
    // The original model with foot-ground contact elements.
    Model model("subject_20dof18musc_running.osim");
    model.initSystem();

    // A copy of the model with the foot-ground contact elements removed.
    Model modelNoContact(model);
    modelNoContact.initSystem();
    modelNoContact.updForceSet().clearAndDestroy();
    modelNoContact.updContactGeometrySet().clearAndDestroy();

    // Load the trajectory. Remove all columns not associated with the skeletal
    // kinematics.
    TimeSeriesTable trajectory("running_solution_full_stride.sto");
    auto labels = trajectory.getColumnLabels();
    for (const auto& label : labels) {
        if (label.find("/jointset") == std::string::npos) {
            trajectory.removeColumn(label);
        }
    }
    auto statesTraj = StatesTrajectory::createFromStatesTable(model, trajectory);
    
    // Create external loads for the calcaneus bodies and apply them to the
    // model without contact forces.
    std::vector<std::string> contact_r = {"/forceset/contactHeel_r", 
            "/forceset/contactLateralMidfoot_r",
            "/forceset/contactMedialMidfoot_r"};
    std::vector<std::string> contact_l = {"/forceset/contactHeel_l", 
            "/forceset/contactLateralMidfoot_l",
            "/forceset/contactMedialMidfoot_l"};
    auto externalLoadsTableCalcn = createExternalLoadsTableForGait(model, 
                statesTraj, contact_r, contact_l);

    // TODO: avoid writing to file for this conversion.
    STOFileAdapter::write(externalLoadsTableCalcn, "external_loads_temp.sto");
    Storage externalLoadsCalcn("external_loads_temp.sto");

    ExternalForce* externalForceLeftCalcn = new ExternalForce(
            externalLoadsCalcn, "ground_force_l_v", "ground_force_l_p", 
            "ground_torque_l_", "calcn_l");
    modelNoContact.addForce(externalForceLeftCalcn);

    ExternalForce* externalForceRightCalcn = new ExternalForce(
            externalLoadsCalcn, "ground_force_r_v", "ground_force_r_p", 
            "ground_torque_r_", "calcn_r");
    modelNoContact.addForce(externalForceRightCalcn);
    modelNoContact.finalizeConnections();

    // Create external loads for the toes bodies and apply them to the model
    // without contact forces.
    auto externalLoadsTableToes = createExternalLoadsTableForGait(model, 
            statesTraj, {"/forceset/contactMedialToe_r"}, 
            {"/forceset/contactMedialToe_l"});

    // TODO: avoid writing to file for this conversion.
    STOFileAdapter::write(externalLoadsTableToes, "external_loads_temp.sto");
    Storage externalLoadsToes("external_loads_temp.sto");

    ExternalForce* externalForceLeftToes = new ExternalForce(externalLoadsToes, 
            "ground_force_l_v", "ground_force_l_p", "ground_torque_l_", 
            "toes_l");
    modelNoContact.addForce(externalForceLeftToes);

    ExternalForce* externalForceRightToes = new ExternalForce(externalLoadsToes, 
            "ground_force_r_v", "ground_force_r_p", "ground_torque_r_", 
            "toes_r");
    modelNoContact.addForce(externalForceRightToes);
    modelNoContact.finalizeConnections();
    
    // If createExternalLoadsTableForGait() is working correctly, the
    // accelerations of `modelNoContact` (i.e., with the external loads applied)
    // should match the accelerations of `model` (i.e., the model with the 
    // foot-ground contact elements).
    SimTK::State stateNoContact = modelNoContact.initSystem();
    for (int i = 0; i < static_cast<int>(statesTraj.getSize()); ++i) {
        auto state = statesTraj[i];
        model.realizeAcceleration(state);

        // Set the kinematic state of the model without contact forces to match
        // the original model's state.
        stateNoContact.setTime(state.getTime());
        stateNoContact.setQ(state.getQ());
        stateNoContact.setU(state.getU());
        modelNoContact.realizeAcceleration(stateNoContact);

        // Compare accelerations.
        SimTK::Vector error = state.getUDot() - stateNoContact.getUDot();
        CAPTURE(error);
        CHECK_THAT(error.norm(), Catch::Matchers::WithinAbs(0, 1e-8));
    }
}