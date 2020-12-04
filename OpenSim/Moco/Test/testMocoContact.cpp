/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testContact.cpp                                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 * Contributors: Antoine Falisse                                              *
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

#define CATCH_CONFIG_MAIN
#include "Testing.h"
#include <OpenSim/Moco/osimMoco.h>

#include <OpenSim/Simulation/Manager/Manager.h>

const double FRICTION_COEFFICIENT = 0.7;

using namespace OpenSim;
using SimTK::Vec3;

template<typename T>
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

    auto* force = new T();
    force->setName("contact");
    force->set_stiffness(1e5);
    force->set_dissipation(1.0);
    force->set_friction_coefficient(FRICTION_COEFFICIENT);
    force->connectSocket_station(*station);
    model.addComponent(force);

    return model;
}

// Test that, with dissipation, the contact force settles to the weight of the
// system. This is the same type of test done in OpenSim's testForces for
// HuntCrossleyForce. The test is performed with both time stepping and direct
// collocation.
template<typename T>
SimTK::Real testNormalForce() {
    // TODO this copy breaks the contact force station socket path
    Model modelTemp = create2DPointMassModel<T>();
    modelTemp.finalizeConnections();
    Model model(modelTemp);
    model.finalizeConnections();
    ModelProcessor modelProc(model);

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
        model.setStateVariableValue(state, "ty/ty/value", y0);
        Manager manager(model);
        manager.setIntegratorAccuracy(1e-6);
        manager.initialize(state);
        state = manager.integrate(finalTime);

        // visualize(model, manager.getStateStorage());

        // https://stackoverflow.com/questions/34696351/template-dependent-typename
        auto& contact = model.template getComponent<StationPlaneContactForce>("contact");
        model.realizeVelocity(state);
        const Vec3 contactForce = contact.calcContactForceOnStation(state);
        // The horizontal force is not quite zero, maybe from a buildup of
        // numerical error (tightening the accuracy reduces this force).
        CHECK(contactForce[0] == Approx(0).margin(0.01));
        CHECK(contactForce[1] == Approx(weight).epsilon(0.01));
        // The system is planar, so there is no force in the z direction.
        CHECK(contactForce[2] == 0);

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
        MocoProblem& mp = study.updProblem();
        mp.setModelAsCopy(model);
        mp.setTimeBounds(0, finalTime);
        mp.setStateInfo("/tx/tx/value", {-1, 1}, 0);
        mp.setStateInfo("/ty/ty/value", {-0.5, 1}, y0);
        mp.setStateInfo("/tx/tx/speed", {-10, 10}, 0);
        mp.setStateInfo("/ty/ty/speed", {-10, 10}, 0);

        auto& ms = study.initTropterSolver();
        ms.set_num_mesh_intervals(50);
        // TODO: Hermite-Simpson has trouble converging
        ms.set_transcription_scheme("trapezoidal");

        MocoSolution solution = study.solve();
        // solution.write("testContact_solution_testNormalForce.sto");
        // moco.visualize(solution);

        auto statesTraj = solution.exportToStatesTrajectory(mp);
        const auto& finalState = statesTraj.back();
        model.realizeVelocity(finalState);
        // https://stackoverflow.com/questions/34696351/template-dependent-typename
        auto& contact = model.template getComponent<StationPlaneContactForce>("contact");
        const Vec3 contactForce = contact.calcContactForceOnStation(finalState);
        // For some reason, direct collocation doesn't produce the same
        // numerical issues with the x component of the force as seen above.
        CHECK(contactForce[0] == Approx(0).margin(1e-15));
        CHECK(contactForce[1] == Approx(weight).epsilon(0.01));
        CHECK(contactForce[2] == 0);

        finalHeightDircol =
                model.getStateVariableValue(finalState, "ty/ty/value");
    }

    CHECK(finalHeightTimeStepping == Approx(finalHeightDircol).margin(1e-5));

    return finalHeightTimeStepping;
}

// Test the friction component of the contact force by ensuring that the point
// mass travels the expected horizontal distance if it starts in the ground.
// To make the friction force roughly constant, we want the equilibrium height
// of the mass (from testNormalForce()).
template<typename T>
void testFrictionForce(const SimTK::Real& equilibriumHeight) {
    auto model = create2DPointMassModel<T>();

    {
        SimTK::State state = model.initSystem();
    }

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
        model.setStateVariableValue(state, "ty/ty/value", y0);
        model.setStateVariableValue(state, "tx/tx/speed", vx0);
        Manager manager(model, state);
        state = manager.integrate(finalTime);

        // visualize(model, manager.getStateStorage());

        const SimTK::Real finalTX =
                model.getStateVariableValue(state, "tx/tx/value");
        CHECK(finalTX == Approx(expectedFinalX).margin(0.005));

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

        auto& ms = study.initTropterSolver();
        ms.set_num_mesh_intervals(25);
        // TODO: Hermite-Simpson has trouble converging
        ms.set_transcription_scheme("trapezoidal");

        MocoSolution solution = study.solve();
        // solution.write("testContact_testFrictionForce_solution.sto");
        // study.visualize(solution);

        auto statesTraj = solution.exportToStatesTrajectory(mp);
        const auto& finalState = statesTraj.back();
        const SimTK::Real finalTX =
                model.getStateVariableValue(finalState, "tx/tx/value");

        CHECK(finalTX == Approx(expectedFinalX).margin(0.005));

        // The system should be at rest.
        OpenSim_CHECK_MATRIX_ABSTOL(finalState.getU(),
                SimTK::Vector(finalState.getNU(), 0.0), 1e-3);
    }
}

template<typename T>
void testStationPlaneContactForce() {
    const SimTK::Real equilibriumHeight = testNormalForce<T>();
    testFrictionForce<T>(equilibriumHeight);
}

// Test our wrapping of SmoothSphereHalfSpaceForce in Moco
// Create a model with SmoothSphereHalfSpaceForce
Model createBallHalfSpaceModel(double frictionCoefficient = FRICTION_COEFFICIENT,
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
    auto* sphere = new ContactSphere(radius, sphereLocation, *ball, "sphere");
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

// Simple simulation of bouncing ball with dissipation should generate contact
// forces that settle to ball weight. This is the same type of test done in
// OpenSim's testForces for HuntCrossleyForce. The test is performed with both
// time stepping and direct collocation.
SimTK::Real testSmoothSphereHalfSpaceForce_NormalForce()
{
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
        CHECK(contactForces[0] == Approx(0.0).margin(1e-4));
        // vertical force is weight
        CHECK(contactForces[1] == Approx(weight).margin(1e-4));
        // no horizontal force
        CHECK(contactForces[2] == Approx(0.0).margin(1e-4));
        CHECK(contactForces[3] == Approx(0.0).margin(1e-4)); // no torque
        CHECK(contactForces[4] == Approx(0.0).margin(1e-4)); // no torque
        CHECK(contactForces[5] == Approx(0.0).margin(1e-4)); // no torque

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
        CHECK(contactForces[0] == Approx(0.0).margin(1e-4));
        // vertical force is weight
        CHECK(contactForces[1] == Approx(weight).margin(1e-4));
        // no horizontal force
        CHECK(contactForces[2] == Approx(0.0).margin(1e-4));
        CHECK(contactForces[3] == Approx(0.0).margin(1e-4)); // no torque
        CHECK(contactForces[4] == Approx(0.0).margin(1e-4)); // no torque
        CHECK(contactForces[5] == Approx(0.0).margin(1e-4)); // no torque

        finalHeightDircol = model.getStateVariableValue(finalState,
            "groundBall/groundBall_coord_2/value");
    }

    CHECK(finalHeightTimeStepping == Approx(finalHeightDircol).margin(1e-5));

    return finalHeightTimeStepping;

}

// Test the friction component of the contact force by ensuring that the ball
// travels the expected horizontal distance if it starts in the ground.
// To make the friction force roughly constant, we want the equilibrium height
// of the mass (from testSmoothSphereHalfSpaceForce_NormalForce()).
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

        // visualize(model, manager.getStateStorage());

        const SimTK::Real finalTX = model.getStateVariableValue(state,
            "groundBall/groundBall_coord_1/value");

        CHECK(finalTX == Approx(expectedFinalX).margin(0.005));

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
        mp.setStateInfo("/groundBall/groundBall_coord_0/speed", {-10, 10}, 0);
        mp.setStateInfo("/groundBall/groundBall_coord_1/speed", {-10, 10},vx0);
        mp.setStateInfo("/groundBall/groundBall_coord_2/speed", {-10, 10}, 0);

        auto& ms = study.initCasADiSolver();
        ms.set_num_mesh_intervals(50);
        ms.set_verbosity(2);
        ms.set_optim_solver("ipopt");

        MocoSolution solution = study.solve();
        //solution.write("testContact_testFrictionForce_solution.sto");
        //study.visualize(solution);

        auto statesTraj = solution.exportToStatesTrajectory(mp);
        const auto& finalState = statesTraj.back();
        const SimTK::Real finalTX =
            model.getStateVariableValue(finalState,
            "groundBall/groundBall_coord_1/value");

        CHECK(finalTX == Approx(expectedFinalX).margin(0.005));

        // The system should be at rest.
        OpenSim_CHECK_MATRIX_ABSTOL(finalState.getU(),
                SimTK::Vector(finalState.getNU(), 0.0), 1e-3);
    }
}

TEMPLATE_TEST_CASE("testStationPlaneContactForce", "[tropter]", 
        AckermannVanDenBogert2010Force, EspositoMiller2018Force
        /* TODO MeyerFregly2016Force */) {
    testStationPlaneContactForce<TestType>();
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
        auto extForce = make_unique<ExternalForce>();
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

        // STOFileAdapter::write(externalLoadsDircol,
        //         "testContact_MocoContactTrackingGoal_external_loads_dircol."
        //         "sto");

        const double actualInitialHeight =
                solution.getState("/groundBall/groundBall_coord_2/value").
                        getElt(0, 0);
        CHECK(actualInitialHeight == Approx(initialHeight).margin(1e-2));

        externalLoadsDircol = createExternalLoadsTableForGait(model, solution,
                {"contactBallHalfSpace"}, {});
    }

    rootMeanSquare(externalLoadsDircol, "ground_force_r_vy",
            externalLoadsTimeStepping, "ground_force_r_vy",
            0.5);
}
