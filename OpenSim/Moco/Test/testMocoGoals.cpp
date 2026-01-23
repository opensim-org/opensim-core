/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoGoals.cpp                                            *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Actuators/PointActuator.h>
#include <OpenSim/Actuators/SpringGeneralizedForce.h>
#include <OpenSim/Moco/MocoGoal/MocoExpressionBasedParameterGoal.h>
#include <OpenSim/Moco/MocoOutputConstraint.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PointConstraint.h>

#include "Testing.h"
#include <catch2/catch_all.hpp>

using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

using namespace OpenSim;

/// creates a model with one sliding mass
std::unique_ptr<Model> createSlidingMassModel() {
    auto model = std::make_unique<Model>();
    model->setName("sliding_mass");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);
    body->attachGeometry(new Sphere(0.05));

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model->addComponent(actu);

    return model;
}

/// create a model with two sliding masses
std::unique_ptr<Model> createDoubleSlidingMassModel() {
    std::unique_ptr<Model> model = createSlidingMassModel();
    auto* body = new Body("body2", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);
    body->attachGeometry(new Sphere(0.05));

    // Allows translation along x.
    auto* joint = new SliderJoint("slider2", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator2");
    actu->setOptimalForce(1);
    model->addComponent(actu);

    model->finalizeConnections();
    return model;
}

/// Test the result of a sliding mass minimum effort problem.
TEMPLATE_TEST_CASE("Test MocoControlGoal", "",
        MocoCasADiSolver) {
    const int N = 9;          // mesh intervals
    const int Nc = 2 * N + 1; // collocation points (Hermite-Simpson)
    MocoSolution sol1;
    {
        MocoStudy study;
        study.setName("sliding_mass");
        MocoProblem& mp = study.updProblem();
        mp.setModel(createSlidingMassModel());
        mp.setTimeBounds(0, {0, 5});
        mp.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        mp.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        mp.setControlInfo("/actuator", MocoBounds(-10, 10));

        mp.addGoal<MocoControlGoal>();

        auto& ms = study.initSolver<TestType>();
        ms.set_num_mesh_intervals(N);

        sol1 = study.solve();
        sol1.write("testMocoGoals_testMocoControlGoal_sol1.sto");

        // Minimum effort solution is a linear control.
        OpenSim_CHECK_MATRIX_ABSTOL(sol1.getControl("/actuator"),
                createVectorLinspace(Nc, 2.23, -2.23), 0.25);
        // Symmetry.
        CHECK(sol1.getControl("/actuator").getElt(0, 0) ==
                Approx(-sol1.getControl("/actuator").getElt(Nc - 1, 0))
                        .margin(1e-3));

        // Minimum effort solution takes as long as possible.
        CHECK(sol1.getTime().getElt(Nc - 1, 0) == Approx(5).margin(1e-7));
    }

    // TODO test that we can ignore specific actuators.
    // TODO for now, the weight can just be set to 0 (not ideal).
    //{

    //    MocoStudy study;
    //    study.setName("sliding_mass");
    //    MocoProblem& mp = study.updProblem();
    //    MocoProblem& mp = study.updProblem();
    //    auto model = createSlidingMassModel();

    //    auto* actu = new CoordinateActuator();
    //    actu->setCoordinate(&model.getCoordinateSet().get("position"));
    //    actu->setName("actuator2");
    //    actu->setOptimalForce(1);
    //    model.addComponent(actu);

    //    mp.setModel(model);
    //    mp.setTimeBounds(0, 5);
    //    mp.setStateInfo("slider/position/value", {0, 1}, 0, 1);
    //    mp.setStateInfo("slider/position/speed", {-100, 100}, 0, 0);
    //    mp.setControlInfo("actuator", MocoBounds(-10, 10));
    //    mp.setControlInfo("actuator2", MocoBounds(-10, 10));

    //    MocoControlGoal effort;
    //    effort.addActuatorToInclude("actuator");
    //    effort.set
    //    mp.addGoal(effort);

    //    MocoCasADiSolver& ms = study.initSolver();
    //    ms.set_num_mesh_intervals(N);

    //    MocoSolution solution = study.solve();
    //    SimTK_TEST_EQ(solution.getControl("actuator2"), SimTK::Vector(N, 0));
    //}

    // TODO test from XML.

    // Ensure that the weights cause one actuator to be preferred over
    // another.
    MocoSolution sol2;
    std::string omocoFile = "testMocoGoals_testMocoControlGoal.omoco";
    {
        MocoStudy study;
        study.setName("sliding_mass");
        study.set_write_solution("false");
        MocoProblem& mp = study.updProblem();
        auto model = createSlidingMassModel();

        auto* actu = new CoordinateActuator();
        actu->setCoordinate(&model->updCoordinateSet().get("position"));
        actu->setName("actuator2");
        actu->setOptimalForce(1);
        model->addComponent(actu);

        mp.setModel(std::move(model));
        mp.setTimeBounds(0, 5);
        mp.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        mp.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        mp.setControlInfo("/actuator", MocoBounds(-10, 10));
        mp.setControlInfo("/actuator2", MocoBounds(-10, 10));

        auto effort = mp.addGoal<MocoControlGoal>();
        effort->setWeightForControl("/actuator2", 2.0);

        auto& ms = study.initSolver<TestType>();
        ms.set_num_mesh_intervals(N);

        sol2 = study.solve();

        study.print(omocoFile);

        // The actuator with the lower weight is more active.
        OpenSim_CHECK_MATRIX_ABSTOL(sol2.getControl("/actuator"),
                2 * sol2.getControl("/actuator2"), 1e-5);
        // Sum of control for these two actuators is the same as the control
        // in the single-actuator case.
        OpenSim_CHECK_MATRIX_ABSTOL(sol2.getControlsTrajectory().rowSum(),
                sol1.getControl("/actuator"), 1e-3);
    }

    // Cannot set a weight for a nonexistent control.
    {
        MocoStudy study;
        study.setName("sliding_mass");
        MocoProblem& mp = study.updProblem();
        mp.setModel(createSlidingMassModel());
        mp.setTimeBounds(0, {0, 5});
        mp.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        mp.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        mp.setControlInfo("/actuator", MocoBounds(-10, 10));
        auto effort = mp.addGoal<MocoControlGoal>();
        effort->setWeightForControl("nonexistent", 1.5);
        CHECK_THROWS_AS(mp.createRep(), Exception);
    }

    // De/serialization.
    {
        MocoStudy study(omocoFile);
        MocoSolution solDeserialized = study.solve();
        sol2.write("DEBUG_sol2.sto");
        solDeserialized.write("DEBUG_solDeserialized.sto");
        CHECK(solDeserialized.isNumericallyEqual(sol2, 1e-5));
    }
}

TEST_CASE("Enabled Goals", "") {
    double x = 23920;
    MocoFinalTimeGoal cost;
    Model model;
    auto state = model.initSystem();
    state.setTime(x);
    SimTK::Vector controls;
    cost.initializeOnModel(model);
    SimTK::Vector goal;
    cost.calcGoal({0, state, controls, x, state, controls, 0}, goal);
    CHECK(goal[0] == Approx(x));
    cost.setEnabled(false);
    cost.calcGoal({0, state, controls, x, state, controls, 0}, goal);
    CHECK(goal[0] == Approx(0));
}

template <class SolverType>
MocoStudy setupMocoStudyDoublePendulumMinimizeEffort(
        double initialSpeed = 0, double finalSpeed = 0) {
    using SimTK::Pi;
    const Model doublePendulum = ModelFactory::createNLinkPendulum(2);

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(doublePendulum);
    problem.addGoal<MocoControlGoal>("effort");
    problem.setTimeBounds(0, 2);
    problem.setControlInfo("/tau0", {-100, 100});
    problem.setControlInfo("/tau1", {-100, 100});
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0, Pi / 2);
    problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0, 0);
    problem.setStateInfo("/jointset/j1/q1/value", {-10, 10}, Pi, 0);
    problem.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0, 0);
    auto& solver = study.initSolver<SolverType>();
    solver.set_num_mesh_intervals(20);
    solver.set_optim_convergence_tolerance(1e-5);
    solver.set_multibody_dynamics_mode("explicit");

    return study;
}

template <typename SolverType, typename TrackingType>
void testDoublePendulumTracking(MocoStudy study,
        const MocoSolution& solutionEffort) {
    // Re-run problem, now setting effort cost function to a low weight and
    // adding a tracking cost.
    auto& problem = study.updProblem();
    problem.updPhase(0).updGoal("effort").setWeight(0.001);
    auto* tracking = problem.addGoal<TrackingType>("tracking");
    tracking->setFramePaths({"/bodyset/b0", "/bodyset/b1"});
    tracking->setStatesReference(solutionEffort.exportToStatesTable());

    study.updSolver<SolverType>().resetProblem(problem);
    auto solutionTracking = study.solve();

    // The tracking solution should match the effort solution.
    SimTK_TEST_EQ_TOL(solutionEffort.getControlsTrajectory(),
            solutionTracking.getControlsTrajectory(), 1e-1);
    SimTK_TEST_EQ_TOL(solutionEffort.getStatesTrajectory(),
            solutionTracking.getStatesTrajectory(), 1e-1);
}

TEMPLATE_TEST_CASE("Test tracking goals", "", MocoCasADiSolver) {

    // Start with double pendulum problem to minimize control effort to create
    // a trajectory to track.
    MocoStudy study = setupMocoStudyDoublePendulumMinimizeEffort<TestType>();
    auto solutionEffort = study.solve();

    SECTION("MocoControlTrackingGoal") {
        // Re-run problem, now setting effort cost function to zero and adding a
        // control tracking cost.
        auto& problem = study.updProblem();
        problem.updPhase(0).updGoal("effort").setWeight(0);
        auto* tracking =
                problem.addGoal<MocoControlTrackingGoal>("control_tracking");
        std::vector<double> time(
                solutionEffort.getTime().getContiguousScalarData(),
                solutionEffort.getTime().getContiguousScalarData() +
                        solutionEffort.getNumTimes());
        TimeSeriesTable controlsRef(time,
                solutionEffort.getControlsTrajectory(),
                solutionEffort.getControlNames());
        tracking->setReference(controlsRef);

        auto& solver = study.updSolver<TestType>();
        solver.resetProblem(problem);
        MocoTrajectory guessTracking = solutionEffort;
        guessTracking.randomizeAdd();
        solver.setGuess(guessTracking);
        auto solutionTracking = study.solve();

        // Make sure control tracking problem matches control effort problem.
        OpenSim_CHECK_MATRIX_ABSTOL(solutionEffort.getControlsTrajectory(),
                solutionTracking.getControlsTrajectory(), 1e-2);
        OpenSim_CHECK_MATRIX_ABSTOL(solutionEffort.getStatesTrajectory(),
                solutionTracking.getStatesTrajectory(), 1e-2);
    }

    SECTION("MocoOrientationTrackingGoal") {
        MocoStudy studyOrientationTracking =
                setupMocoStudyDoublePendulumMinimizeEffort<TestType>();

        SECTION("Accurately tracks states reference") {
            testDoublePendulumTracking<TestType, MocoOrientationTrackingGoal>(
                studyOrientationTracking, solutionEffort);
        }

        SECTION("Missing coordinate value in states reference") {
            auto& problem = study.updProblem();
            auto* tracking = problem.addGoal<MocoOrientationTrackingGoal>("tracking");
            tracking->setFramePaths({"/bodyset/b0", "/bodyset/b1"});
            TimeSeriesTable table = solutionEffort.exportToStatesTable();
            table.removeColumn("/jointset/j0/q0/value");
            tracking->setStatesReference(table);
            SimTK_TEST_MUST_THROW(tracking->initializeOnModel(
                problem.getPhase(0).getModel()));
        }
    }

    SECTION("MocoTranslationTrackingGoal") {
        MocoStudy studyTranslationTracking =
                setupMocoStudyDoublePendulumMinimizeEffort<TestType>();
        testDoublePendulumTracking<TestType, MocoTranslationTrackingGoal>(
            studyTranslationTracking, solutionEffort);
    }

    SECTION("MocoAngularVelocityTrackingGoal") {
        MocoStudy studyAngularVelocityTracking =
                setupMocoStudyDoublePendulumMinimizeEffort<TestType>();
        testDoublePendulumTracking<TestType, MocoAngularVelocityTrackingGoal>(
            studyAngularVelocityTracking, solutionEffort);
    }

    SECTION("MocoAccelerationTrackingGoal") {
        MocoStudy studyAccelerationTracking =
                setupMocoStudyDoublePendulumMinimizeEffort<TestType>();
        // Re-run problem, now setting effort cost function to a low weight and
        // adding an acceleration tracking cost.
        auto& problem = studyAccelerationTracking.updProblem();
        MocoProblemRep problemRep = problem.createRep();
        const Model& model = problemRep.getModelBase();
        problem.updPhase(0).updGoal("effort").setWeight(0.001);
        auto* accelerationTracking =
                problem.addGoal<MocoAccelerationTrackingGoal>("tracking");
        std::vector<std::string> framePaths = {"/bodyset/b0", "/bodyset/b1"};
        accelerationTracking->setFramePaths(framePaths);
        // Compute the accelerations from the effort minimization solution to
        // use as a tracking reference. It's fine to use the analyze() utility
        // here, since this model has no kinematic constraints.
        TimeSeriesTableVec3 accelTableEffort =
                analyzeMocoTrajectory<SimTK::Vec3>(model, solutionEffort,
                        {"/bodyset/b0\\|linear_acceleration",
                                "/bodyset/b1\\|linear_acceleration"});
        accelTableEffort.setColumnLabels(framePaths);
        accelerationTracking->setAccelerationReference(accelTableEffort);

        studyAccelerationTracking.updSolver<TestType>().resetProblem(problem);
        auto solutionTracking = studyAccelerationTracking.solve();

        // The tracking solution should match the effort solution.
        SimTK_TEST_EQ_TOL(solutionEffort.getControlsTrajectory(),
                solutionTracking.getControlsTrajectory(), 1e-2);
        SimTK_TEST_EQ_TOL(solutionEffort.getStatesTrajectory(),
                solutionTracking.getStatesTrajectory(), 1e-2);
    }

    SECTION("MocoAccelerationTrackingGoal (IMU tracking)") {
        MocoStudy studyAccelerationTracking =
                setupMocoStudyDoublePendulumMinimizeEffort<TestType>();
        // Re-run problem, now setting effort cost function to a low weight and
        // adding an acceleration tracking cost.
        auto& problem = studyAccelerationTracking.updProblem();
        MocoProblemRep problemRep = problem.createRep();
        const Model& model = problemRep.getModelBase();
        problem.updPhase(0).updGoal("effort").setWeight(0.001);
        auto* accelerationIMUTracking =
                problem.addGoal<MocoAccelerationTrackingGoal>("tracking");
        std::vector<std::string> framePaths = {"/bodyset/b0", "/bodyset/b1"};
        accelerationIMUTracking->setFramePaths(framePaths);
        // Compute the accelerations from the effort minimization solution to
        // use as a tracking reference.
        TimeSeriesTableVec3 accelTableIMU =
                createSyntheticIMUAccelerationSignals(model,
                        solutionEffort.exportToStatesTable(),
                        solutionEffort.exportToControlsTable(),
                        framePaths);
        accelerationIMUTracking->setAccelerationReference(accelTableIMU);
        accelerationIMUTracking->setGravityOffset(true);
        accelerationIMUTracking->setExpressAccelerationsInTrackingFrames(true);

        studyAccelerationTracking.updSolver<TestType>().resetProblem(problem);
        auto solutionTracking = studyAccelerationTracking.solve();

        // The tracking solution should match the effort solution.
        SimTK_TEST_EQ_TOL(solutionEffort.getControlsTrajectory(),
                          solutionTracking.getControlsTrajectory(), 1e-2);
        SimTK_TEST_EQ_TOL(solutionEffort.getStatesTrajectory(),
                          solutionTracking.getStatesTrajectory(), 1e-2);
    }

    SECTION("MocoGeneralizedForceTrackingGoal") {
        TimeSeriesTable generalizedForces = 
                study.calcGeneralizedForces(solutionEffort, {});
        MocoStudy studyGenForceTracking =
                setupMocoStudyDoublePendulumMinimizeEffort<TestType>();
        // Re-run problem, now setting effort cost function to a low weight and
        // adding a generalized force tracking cost.
        auto& problem = studyGenForceTracking.updProblem();
        problem.updPhase(0).updGoal("effort").setWeight(0.001);
        auto* genForceTracking =
                problem.addGoal<MocoGeneralizedForceTrackingGoal>("tracking");
        genForceTracking->setReference(generalizedForces);

        SECTION("Setting weights") {
            Model model = problem.createRep().getModelBase();
            SimTK::State state = model.initSystem();
            // Set acclelerations to zero, so the only model-generated forces 
            // in the integrand are zero.
            state.updUDot() = SimTK::Vector(state.getNU(), 0.0);
            MocoGoal::IntegrandInput input{0, state, {}};

            // Apply weights.
            genForceTracking->setWeightForGeneralizedForcePattern("q.*", 5.0);
            genForceTracking->setWeightForGeneralizedForce("q1_moment", 10.0);
            genForceTracking->initializeOnModel(model);

            // The integrand should be the sum of the squared generalized forces
            // multiplied by the weights.
            const SimTK::RowVectorView initialGenForces = 
                    generalizedForces.getRowAtIndex(0);
            double integrand = 
                    5.0 * (initialGenForces[0] * initialGenForces[0]) +
                    10.0 * (initialGenForces[1] * initialGenForces[1]);
            CHECK_THAT(genForceTracking->calcIntegrand(input), 
                    Catch::Matchers::WithinAbs(integrand, 1e-6));
        }
        
        SECTION("Tracking performance") {
            studyGenForceTracking.updSolver<TestType>().resetProblem(problem);
            auto solutionTracking = studyGenForceTracking.solve();

            // The tracking solution should match the effort solution.
            SimTK_TEST_EQ_TOL(solutionEffort.getControlsTrajectory(),
                            solutionTracking.getControlsTrajectory(), 1e-3);
            SimTK_TEST_EQ_TOL(solutionEffort.getStatesTrajectory(),
                            solutionTracking.getStatesTrajectory(), 1e-3);
        }

    }
}

template <class SolverType>
std::pair<MocoStudy, MocoSolution> configTestMocoScaleFactor(bool paramInitSys) {
    // Start with double pendulum problem to minimize control effort to create
    // a trajectory to track.
    MocoStudy study =
            setupMocoStudyDoublePendulumMinimizeEffort<SolverType>();
    auto solutionEffort = study.solve();

    // Change the strength of the CoordinateActuators in the Model so that we
    // force the scale factors to be used. The actuators have a default optimal
    // force of 1, and bounds of [-100, 100].
    auto& problem = study.updProblem();
    auto& model = problem.updModel();
    model.updComponent<CoordinateActuator>("/tau0").setOptimalForce(2.0);
    model.updComponent<CoordinateActuator>("/tau1").setOptimalForce(2.0);
    // Down-weight the effort term so the tracking cost dominates.
    problem.updPhase(0).updGoal("effort").setWeight(1e-6);
    // Add the control tracking goal.
    auto* tracking =
            problem.addGoal<MocoControlTrackingGoal>("control_tracking");
    std::vector<double> time(
            solutionEffort.getTime().getContiguousScalarData(),
            solutionEffort.getTime().getContiguousScalarData() +
            solutionEffort.getNumTimes());
    TimeSeriesTable controlsRef(time,
                                solutionEffort.getControlsTrajectory(),
                                solutionEffort.getControlNames());
    tracking->setReference(controlsRef);
    // Add the scale factors.
    const auto& controlNames = solutionEffort.getControlNames();
    tracking->addScaleFactor("tau0_scale_factor",
            controlNames[0], MocoBounds(0.1, 0.5));
    tracking->addScaleFactor("tau1_scale_factor",
            controlNames[1], MocoBounds(0.1, 0.5));

    // Update the solver with the new problem and disable initSystem() calls for
    // the MocoParameters.
    auto& solver = study.updSolver<SolverType>();
    solver.resetProblem(problem);
    // Construct a guess for the problem. We double the actuator strengths so
    // here we half the controls.
    auto guessTracking = solver.createGuess("bounds");
    guessTracking.insertStatesTrajectory(
            solutionEffort.exportToStatesTable(), true);
    auto effortControls = solutionEffort.getControlsTrajectory();
    effortControls.updCol(0) *= 0.5;
    effortControls.updCol(1) *= 0.5;
    solutionEffort.setControl("/tau0", effortControls.updCol(0).getAsVector());
    solutionEffort.setControl("/tau1", effortControls.updCol(1).getAsVector());
    guessTracking.insertControlsTrajectory(
            solutionEffort.exportToControlsTable(), true);
    solver.setGuess(guessTracking);
    return {study, solutionEffort};
}

void evalTestMocoScaleFactor(const MocoStudy& study,
                             const MocoSolution& solutionEffort) {
    // Solve.
    auto solutionTracking = study.solve();

    // Make sure control tracking problem matches control effort problem. We've
    // already adjusted the effort controls while constructing the initial guess
    // above, so this comparison should pass if the problem solved correctly.
    OpenSim_CHECK_MATRIX_ABSTOL(solutionEffort.getControlsTrajectory(),
            solutionTracking.getControlsTrajectory(), 1e-3);
    OpenSim_CHECK_MATRIX_ABSTOL(solutionEffort.getStatesTrajectory(),
            solutionTracking.getStatesTrajectory(), 1e-3);
}

TEST_CASE("Test MocoScaleFactor - MocoCasADiSolver", "[casadi]") {
    auto pair = configTestMocoScaleFactor<MocoCasADiSolver>(true);
    MocoStudy study = pair.first;
    study.updSolver<MocoCasADiSolver>().set_parameters_require_initsystem(false);
    evalTestMocoScaleFactor(study, pair.second);
}

TEST_CASE("Test MocoSumSquaredStateGoal") {
    using SimTK::Inertia;
    using SimTK::Vec3;
    Model model = ModelFactory::createDoublePendulum();
    const Coordinate& q0 = model.getCoordinateSet().get("q0");
    const Coordinate& q1 = model.getCoordinateSet().get("q1");
    std::string q0_str = q0.getAbsolutePathString() + "/value";
    std::string q1_str = q1.getAbsolutePathString() + "/value";

    SimTK::State state = model.initSystem();
    MocoGoal::IntegrandInput input{0, state, {}};
    q0.setValue(state, 1.0);
    q1.setValue(state, 0.5);

    MocoProblem mp;
    mp.setModelAsCopy(model);

    // If no state weights are given, should return sum squared state values
    // with weight of 1.0 for each state
    // 1.00 + 0.25 = 1.25
    auto* goal = mp.addGoal<MocoSumSquaredStateGoal>();
    goal->initializeOnModel(model);
    CHECK(goal->calcIntegrand(input) == Approx(1.25).margin(1e-6));

    // If one state weight given, use that state weight, but then also
    // set weight to all other states as 1.0.
    // 1 * 1 + 10 * 0.25 = 3.5
    auto* goal2 = mp.addGoal<MocoSumSquaredStateGoal>();
    goal2->setWeightForState(q1_str, 10.0);
    goal2->initializeOnModel(model);
    CHECK(goal2->calcIntegrand(input) == Approx(3.5).margin(1e-6));

    MocoWeightSet moco_weight_set;
    moco_weight_set.cloneAndAppend({q0_str, 0.5});
    moco_weight_set.cloneAndAppend({q1_str, 10.0});

    // 0.5 * 1 + 10.0 * 0.25 = 3.0
    auto* goal3 = mp.addGoal<MocoSumSquaredStateGoal>();
    goal3->setWeightSet(moco_weight_set);
    goal3->initializeOnModel(model);
    CHECK(goal3->calcIntegrand(input) == Approx(3.0).margin(1e-6));

    // 0.5 * 1.00 + 10.0 * 0.25 = 3.0
    auto* goal4 = mp.addGoal<MocoSumSquaredStateGoal>();
    goal4->setWeightSet(moco_weight_set);
    goal4->setPattern(".*value$");
    goal4->initializeOnModel(model);
    CHECK(goal4->calcIntegrand(input) == Approx(3.0).margin(1e-6));

    // Throws since weight set has some names that don't match pattern.
    auto* goal5 = mp.addGoal<MocoSumSquaredStateGoal>();
    goal5->setWeightSet(moco_weight_set);
    goal5->setPattern(".*pin1.*");
    CHECK_THROWS(goal5->initializeOnModel(model));

    // Throws since this state name doesn't exist in the model.
    auto* goal6 = mp.addGoal<MocoSumSquaredStateGoal>();
    goal6->setWeightForState("/jointset/j2/q2/value", 100.0);
    CHECK_THROWS(goal6->initializeOnModel(model));
}

class MocoPeriodicish : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoPeriodicish, MocoGoal);

public:
    MocoPeriodicish() = default;
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::EndpointConstraint;
    }
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(0, 2);
    }
    void calcGoalImpl(
            const GoalInput& in, SimTK::Vector& values) const override {
        values[0] = in.initial_state.getQ()[0] - in.final_state.getQ()[0];
        values[1] = in.final_state.getU()[0];
        if (getModeIsCost()) {
            values[0] = SimTK::square(values[0]);
            values[1] = SimTK::square(values[1]);
        }
    }
};

TEMPLATE_TEST_CASE("Endpoint constraints", "[casadi]", MocoCasADiSolver) {

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(ModelFactory::createPendulum());

    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/jointset/j0/q0/value", {-0.3, 0.3}, -0.3);

    auto* periodic = problem.addGoal<MocoPeriodicish>();
    auto* effort = problem.addGoal<MocoControlGoal>("control");

    study.initSolver<TestType>();

    SECTION("Endpoint constraint is satisfied.") {
        MocoSolution solution = study.solve();
        const int N = solution.getNumTimes();
        CHECK(solution.getState("/jointset/j0/q0/value").getElt(N - 1, 0) ==
                Approx(-0.3));
        CHECK(solution.getState("/jointset/j0/q0/speed").getElt(N - 1, 0) ==
                Approx(0).margin(1e-10));
    }

    SECTION("Bounds has incorrect size.") {
        periodic->updConstraintInfo().setBounds({{0.0}});
        CHECK_THROWS_WITH(study.solve(), ContainsSubstring("Size of property"));
    }

    SECTION("Non-tight bounds.") {
        periodic->updConstraintInfo().setBounds({{0.0}, {-0.05, 0}});
        MocoSolution solution = study.solve();
        const int N = solution.getNumTimes();
        CHECK(solution.getState("/jointset/j0/q0/value").getElt(N - 1, 0) ==
                Approx(-0.3));
        // Since we are minimizing effort, we should reduce breaking and allow
        // the pendulum to end with some downward velocity.
        CHECK(solution.getState("/jointset/j0/q0/speed").getElt(N - 1, 0) ==
                Approx(-0.05).margin(1e-10));
    }

    SECTION("Set bounds with scripting-friendly method.") {
        periodic->setEndpointConstraintBounds(
                std::vector<MocoBounds>(2, {-1.0, 1.0}));
        const auto& conInfo = periodic->getConstraintInfo();
        CHECK(conInfo.getBounds()[0] == MocoBounds(-1.0, 1.0));
        CHECK(conInfo.getBounds()[1] == MocoBounds(-1.0, 1.0));
    }

    SECTION("Goal works in cost mode.") {
        periodic->setMode("cost");
        periodic->setWeight(10.0);
        effort->setEnabled(false);
        MocoSolution solution = study.solve();
        const int N = solution.getNumTimes();
        CHECK(solution.getState("/jointset/j0/q0/value").getElt(N - 1, 0) ==
                Approx(-0.3).margin(1e-4));
        CHECK(solution.getState("/jointset/j0/q0/speed").getElt(N - 1, 0) ==
                Approx(0).margin(1e-6));
    }
}

TEMPLATE_TEST_CASE("MocoPeriodicityGoal", "[casadi]", MocoCasADiSolver) {

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(ModelFactory::createPendulum());

    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/jointset/j0/q0/value", {-1.0, 1.0}, 0.1);

    auto* periodic = problem.addGoal<MocoPeriodicityGoal>("periodic");
    MocoPeriodicityGoalPair pair_q0_value;
    pair_q0_value.set_initial_variable("/jointset/j0/q0/value");
    pair_q0_value.set_final_variable("/jointset/j0/q0/value");
    periodic->addStatePair(pair_q0_value);
    periodic->addStatePair({"/jointset/j0/q0/speed", "/jointset/j0/q0/speed"});
    periodic->addControlPair({"/tau0"});
    auto* effort = problem.addGoal<MocoControlGoal>("control");

    study.initSolver<TestType>();

    SECTION("Perodic constraint is satisfied.") {
        MocoSolution solution = study.solve();
        solution.write("testMocoGoals_MocoPeriodicityGoal_solution.sto");
        const int N = solution.getNumTimes();
        CHECK(solution.getState("/jointset/j0/q0/value")[N - 1] ==
                solution.getState("/jointset/j0/q0/value")[0]);
        CHECK(solution.getState("/jointset/j0/q0/speed")[N - 1] ==
                Approx(solution.getState("/jointset/j0/q0/speed")[0])
                        .margin(1e-6));
        CHECK(solution.getControl("/tau0")[N - 1] ==
                Approx(solution.getControl("/tau0")[0]).margin(1e-6));
    }

    SECTION("Goal works in cost mode.") {
        periodic->setMode("cost");
        periodic->setWeight(10.0);
        effort->setEnabled(false);
        MocoSolution solution = study.solve();
        const int N = solution.getNumTimes();
        CHECK(solution.getState("/jointset/j0/q0/value")[N - 1] ==
                Approx(solution.getState("/jointset/j0/q0/value")[0])
                        .margin(1e-6));
        CHECK(solution.getState("/jointset/j0/q0/speed")[N - 1] ==
                Approx(solution.getState("/jointset/j0/q0/speed")[0])
                        .margin(1e-6));
        CHECK(solution.getControl("/tau0")[N - 1] ==
                Approx(solution.getControl("/tau0")[0]).margin(1e-6));
    }
}

class MocoControlGoalWithEndpointConstraint : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoControlGoalWithEndpointConstraint, MocoGoal);

public:
    MocoControlGoalWithEndpointConstraint() = default;
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::EndpointConstraint;
    }
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(1, 1, SimTK::Stage::Model);
    }
    void calcIntegrandImpl(
            const IntegrandInput& input,
            SimTK::Real& integrand) const override {
        integrand = input.controls.normSqr();
    }
    void calcGoalImpl(
            const GoalInput& in, SimTK::Vector& values) const override {
        values[0] = in.integral;
    }
};

TEMPLATE_TEST_CASE(
        "Endpoint constraint with integral", "[casadi]", MocoCasADiSolver) {

    Model model;
    const double mass = 1.3169;
    auto* body = new Body("body", mass, SimTK::Vec3(0), SimTK::Inertia(1));
    model.addBody(body);

    auto* joint = new PlanarJoint("joint", model.getGround(), *body);
    model.addJoint(joint);

    auto* constr = new PointConstraint(model.getGround(), SimTK::Vec3(0),
                                       *body, SimTK::Vec3(0));
    model.addConstraint(constr);
    model.finalizeConnections();

    ModelFactory::createReserveActuators(model, 1.0);

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);

    problem.setTimeBounds(0, 0.5);

    auto* goal = problem.addGoal<MocoControlGoalWithEndpointConstraint>();
    goal->setMode("endpoint_constraint");

    auto& solver = study.initSolver<TestType>();
    solver.set_num_mesh_intervals(5);
    auto guess = solver.createGuess();
    guess.randomizeReplace();
    solver.setGuess(guess);

    auto solution = study.solve();

    // The endpoint constraint is that the sum of squared controls integrated
    // over the motion must be 0.
    CHECK(solution.getControlsTrajectory().norm() < 1e-3);
}

class MySumSquaredControls : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(MySumSquaredControls, ModelComponent);
public:
    OpenSim_DECLARE_OUTPUT(sum_squared_controls, double,
            calcSumSquaredControls, SimTK::Stage::Velocity);
    double calcSumSquaredControls(const SimTK::State& state) const {
        getModel().realizeVelocity(state);
        return getModel().getControls(state).normSqr();
    }
};

auto createStudy = [](
        MocoInitialBounds initialSpeed = {},
        MocoFinalBounds finalSpeed = {}) {
    MocoStudy study;
    study.setName("sliding_mass");
    MocoProblem& problem = study.updProblem();
    problem.setModel(createSlidingMassModel());
    problem.setTimeBounds(0, {0, 5});
    problem.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
    problem.setStateInfo("/slider/position/speed", {-100, 100},
                         initialSpeed, finalSpeed);
    problem.setControlInfo("/actuator", MocoBounds(-10, 10));
    return study;
};

TEMPLATE_TEST_CASE("MocoOutputGoal", "", MocoCasADiSolver) {

    MocoSolution solutionControl;
    SECTION("MocoOutputGoal") {
        {
            auto study = createStudy(0, 0);
            auto& problem = study.updProblem();
            problem.template addGoal<MocoControlGoal>();
            auto &solver = study.template initSolver<TestType>();
            solver.set_num_mesh_intervals(10);
            solutionControl = study.solve();
        }
        MocoSolution solutionOutput;
        {
            auto study = createStudy(0, 0);
            auto& problem = study.updProblem();
            auto model = createSlidingMassModel();

            auto* component = new MySumSquaredControls();
            component->setName("mysumsquaredcontrols");
            model->addComponent(component);
            problem.setModel(std::move(model));

            auto* goal = problem.template addGoal<MocoOutputGoal>();
            goal->setOutputPath("/mysumsquaredcontrols|sum_squared_controls");

            auto& solver = study.template initSolver<TestType>();
            solver.set_num_mesh_intervals(10);
            solutionOutput = study.solve();
        }

        CHECK(solutionControl.isNumericallyEqual(solutionOutput, 1e-5));
    }

    SECTION("MocoInitialOutputGoal") {
        auto study = createStudy({0, 100.0}, 0);
        auto& problem = study.updProblem();
        auto* goal = problem.template addGoal<MocoInitialOutputGoal>();
        goal->setName("initial_speed");
        goal->setOutputPath("/body|linear_velocity");
        auto& solver = study.template initSolver<TestType>();
        solver.set_num_mesh_intervals(10);
        MocoSolution solution = study.solve();
        CHECK(solution.getState("/slider/position/speed")[0] ==
                Approx(0).margin(1e-6));
    }

    SECTION("MocoFinalOutputGoal") {
        auto study = createStudy(0, {0, 100.0});
        auto& problem = study.updProblem();
        auto* goal = problem.template addGoal<MocoFinalOutputGoal>();
        goal->setName("final_speed");
        goal->setOutputPath("/body|linear_velocity");
        auto &solver = study.template initSolver<TestType>();
        solver.set_num_mesh_intervals(10);
        MocoSolution solution = study.solve();
        CHECK(solution.getState(
                "/slider/position/speed")[solution.getNumTimes() - 1] ==
                Approx(0).margin(1e-6));
    }

    SECTION("MocoOutputConstraint") {
        auto study = createStudy({-100.0, 100.0}, {-100.0, 100.0});
        auto& problem = study.updProblem();
        auto* pathCon =
                problem.template addPathConstraint<MocoOutputConstraint>();
        pathCon->setName("nonnegative_velocity");
        pathCon->setOutputPath("/body|linear_velocity");
        pathCon->setOutputIndex(0);
        pathCon->updConstraintInfo().setBounds({{0, 100.0}});
        auto &solver = study.template initSolver<TestType>();
        solver.set_num_mesh_intervals(10);
        MocoSolution solution = study.solve();
        auto solutionSpeed = solution.getState("/slider/position/speed");
        for (int i = 0; i < solution.getNumTimes(); ++i) {
            CHECK(solutionSpeed[i] >= Approx(0));
        }
    }
}

TEMPLATE_TEST_CASE("MocoOutputConstraint with two outputs", "", MocoCasADiSolver) {
    double bound = 2.0;
    MocoSolution solutionControl;
    MocoStudy study;
    auto& problem = study.updProblem();
    auto model = createDoubleSlidingMassModel();
    model->initSystem();
    problem.setModelAsCopy(*model);
    problem.setTimeBounds(0, 3);

    // one slider must move, the other doesn't need to
    problem.setStateInfo("/slider/position/value", MocoBounds(-50, 50),
       MocoInitialBounds(-5,-5), MocoFinalBounds(5, 5.5));
    problem.setStateInfo("/slider2/position/value", MocoBounds(-50, 50),
       MocoInitialBounds(-50, 50), MocoFinalBounds(-50, 50));
    problem.setStateInfo("/slider/position/speed", {-10, 10}, {-10, 10}, {-10, 10});
    problem.setStateInfo("/slider2/position/speed", {-10, 10}, {-10, 10}, {-10, 10});
    problem.setControlInfo("/actuator", {-100, 100});
    problem.setControlInfo("/actuator2", {-100, 100});

    // add constraint: second mass stays near moving mass
    auto* pathCon = problem.template addPathConstraint<MocoOutputConstraint>();
    pathCon->setName("velocities_goal");
    pathCon->setOutputPath("/body|position");
    pathCon->setSecondOutputPath("/body2|position");
    pathCon->setOperation("subtraction");
    pathCon->setOutputIndex(0);
    pathCon->setExponent(2);
    pathCon->updConstraintInfo().setBounds({{0, bound}});

    auto* effort = problem.template addGoal<MocoControlGoal>();
    effort->setName("effort");
    effort->setWeight(0.001);

    auto& solver = study.template initSolver<TestType>();
    solver.set_num_mesh_intervals(30);
    MocoSolution solution = study.solve();

    auto solutionPositionMoving = solution.getState("/slider/position/value");
    auto solutionPositionFollowing = solution.getState("/slider2/position/value");
    for (int i = 0; i < solution.getNumTimes(); ++i) {
        double diff = (static_cast<SimTK::Vec3>(solutionPositionMoving[i])[0]
            - static_cast<SimTK::Vec3>(solutionPositionFollowing[i])[0]);
        CHECK(diff * diff <= bound);
    }
}

TEMPLATE_TEST_CASE("MocoOutputGoal with two outputs", "", MocoCasADiSolver) {
    MocoSolution solutionControl;

    SECTION("Subtraction of Vec3 (norm)") {
        MocoStudy study;
        auto& problem = study.updProblem();
        auto model = createDoubleSlidingMassModel();
        model->initSystem();

        problem.setModelAsCopy(*model);
        problem.setTimeBounds(0, 5);

        // set up sliders to have a distance from each other at the beginning
        problem.setStateInfo("/slider/position/value", MocoBounds(-5, 5),
            MocoInitialBounds(-2), MocoFinalBounds(-5, 5));
        problem.setStateInfo("/slider2/position/value", MocoBounds(-5, 5),
            MocoInitialBounds(2), MocoFinalBounds(-5, 5));

        problem.setStateInfo("/slider/position/speed", {-10, 10}, 0, 0);
        problem.setStateInfo("/slider2/position/speed", {-10, 10}, 0, 0);
        problem.setControlInfo("/actuator", {-100, 100});
        problem.setControlInfo("/actuator2", {-100, 100});

        auto* effort = problem.addGoal<MocoControlGoal>();
        effort->setName("effort");
        effort->setWeight(0.001);

        SECTION("MocoOutputGoal") {
            // add goal of smallest distance
            auto* goal = problem.template addGoal<MocoOutputGoal>();

            // check getting the properties before setting them
            CHECK_NOTHROW(goal->getOutputPath());
            CHECK_NOTHROW(goal->getOperation());

            goal->setName("distance2");
            goal->setOutputPath("/body|position");
            goal->setSecondOutputPath("/body2|position");
            goal->setOperation("subtraction");
            goal->setExponent(2);

            auto& solver = study.template initSolver<TestType>();
            solver.set_num_mesh_intervals(30);
            MocoSolution solution = study.solve();

            // analyze result for ending distance between spheres
            StatesTrajectory trajectory = solution.exportToStatesTrajectory(*model);
            const SimTK::State& finalState = trajectory.back();
            model->realizePosition(finalState);
            const SimTK::Vec3& endPosition1 = model->getComponent<Body>("/body")
                                            .getPositionInGround(finalState);
            const SimTK::Vec3& endPosition2 = model->getComponent<Body>("/body2")
                                            .getPositionInGround(finalState);

            CHECK((endPosition1 - endPosition2).norm() == Approx(0).margin(1e-2));
        }

        SECTION("MocoFinalOutputGoal") {
            // add goal of smallest distance
            auto* goal = problem.template addGoal<MocoFinalOutputGoal>();
            goal->setName("distance2");
            goal->setOutputPath("/body|position");
            goal->setSecondOutputPath("/body2|position");
            goal->setOperation("subtraction");
            goal->setExponent(2);

            auto& solver = study.template initSolver<TestType>();
            solver.set_num_mesh_intervals(30);
            MocoSolution solution = study.solve();

            // analyze result for ending distance between spheres
            StatesTrajectory trajectory = solution.exportToStatesTrajectory(*model);
            const SimTK::State& finalState = trajectory.back();
            model->realizePosition(finalState);
            const SimTK::Vec3& endPosition1 = model->getComponent<Body>("/body")
                                            .getPositionInGround(finalState);
            const SimTK::Vec3& endPosition2 = model->getComponent<Body>("/body2")
                                            .getPositionInGround(finalState);

            CHECK((endPosition1 - endPosition2).norm() == Approx(0).margin(5e-2));
        }
    }

    SECTION("Multiplication of SpatialVec (with index)") {
        MocoStudy study;
        auto& problem = study.updProblem();
        auto model = createDoubleSlidingMassModel();
        model->initSystem();

        problem.setModelAsCopy(*model);
        problem.setTimeBounds(0, 1);
        problem.setStateInfo("/slider/position/value", MocoBounds(-5, 5),
            MocoInitialBounds(1), MocoFinalBounds(-5, 5));
        problem.setStateInfo("/slider2/position/value", MocoBounds(-5, 5),
            MocoInitialBounds(-1), MocoFinalBounds(-5, 5));

        // sliders have a starting velocity
        problem.setStateInfo("/slider/position/speed", {-50, 50}, 3);
        problem.setStateInfo("/slider2/position/speed", {-50, 50}, -1);
        problem.setControlInfo("/actuator", {-100, 100});
        problem.setControlInfo("/actuator2", {-100, 100});

        // add goal of smallest multiplied velocities
        auto* goal = problem.template addGoal<MocoOutputGoal>();
        goal->setName("multiply_velocities");
        goal->setOutputPath("/body|velocity");
        goal->setSecondOutputPath("/body2|velocity");
        goal->setOperation("multiplication");
        goal->setOutputIndex(3);
        goal->setExponent(2);

        auto* effort = problem.template addGoal<MocoControlGoal>();
        effort->setName("effort");
        effort->setWeight(0.001);

        auto& solver = study.template initSolver<TestType>();
        solver.set_num_mesh_intervals(10);
        MocoSolution solution = study.solve();

        // analyze result for ending velocities
        StatesTrajectory trajectory = solution.exportToStatesTrajectory(*model);
        const SimTK::State& finalState = trajectory.back();
        model->realizeAcceleration(finalState);
        const SimTK::SpatialVec& endVel1 = model->getComponent<Body>("/body")
                                            .getVelocityInGround(finalState);
        const SimTK::SpatialVec& endVel2 = model->getComponent<Body>("/body2")
                                            .getVelocityInGround(finalState);

        CHECK((endVel1[1][0] * endVel2[1][0]) == Approx(0).margin(1e-1));
    }

    SECTION("MocoInitialOutputGoal") {
        MocoStudy study;
        auto& problem = study.updProblem();
        auto model = createDoubleSlidingMassModel();
        model->initSystem();

        problem.setModelAsCopy(*model);
        problem.setTimeBounds(0, 3);

        // set up sliders to have a distance from each other at the end
        problem.setStateInfo("/slider/position/value", MocoBounds(-5, 5),
            MocoInitialBounds(-5, 5), MocoFinalBounds(-1, 2));
        problem.setStateInfo("/slider2/position/value", MocoBounds(-5, 5),
            MocoInitialBounds(-5, 5), MocoFinalBounds(3, 5));

        problem.setStateInfo("/slider/position/speed", {-10, 10}, 0, 0);
        problem.setStateInfo("/slider2/position/speed", {-10, 10}, 0, 0);
        problem.setControlInfo("/actuator", {-100, 100});
        problem.setControlInfo("/actuator2", {-100, 100});

        // add goal of smallest distance at the beginning
        auto* goal = problem.template addGoal<MocoInitialOutputGoal>();
        goal->setName("distance2");
        goal->setOutputPath("/body|position");
        goal->setSecondOutputPath("/body2|position");
        goal->setOperation("subtraction");
        goal->setExponent(2);

        auto* effort = problem.template addGoal<MocoControlGoal>();
        effort->setName("effort");
        effort->setWeight(0.001);

        auto& solver = study.template initSolver<TestType>();
        solver.set_num_mesh_intervals(30);
        MocoSolution solution = study.solve();

        // analyze result for starting distance between spheres
        StatesTrajectory trajectory = solution.exportToStatesTrajectory(*model);
        const SimTK::State& initialState = trajectory.front();
        model->realizePosition(initialState);
        const SimTK::Vec3& startPosition1 = model->getComponent<Body>("/body")
                                        .getPositionInGround(initialState);
        const SimTK::Vec3& startPosition2 = model->getComponent<Body>("/body2")
                                        .getPositionInGround(initialState);

        CHECK(startPosition1 - startPosition2 == Approx(0).margin(5e-2));
    }

    SECTION("Invalid Outputs") {
        MocoStudy study;
        auto& problem = study.updProblem();
        auto model = createDoubleSlidingMassModel();
        model->initSystem();
        problem.setModelAsCopy(*model);

        SECTION("Invalid Operator") {
            auto* goal = problem.template addGoal<MocoOutputGoal>();
            goal->setName("notDistance");
            goal->setOutputPath("/body|position");
            goal->setSecondOutputPath("/body2|position");
            goal->setOperation("Subtraction");   // instead of subtraction

            REQUIRE_THROWS(study.template initSolver<TestType>());
        }

        SECTION("Mismatch Type") {
            auto* goal = problem.template addGoal<MocoOutputGoal>();
            goal->setName("badCombo");
            goal->setOutputPath("/body|velocity");
            goal->setSecondOutputPath("/body2|position");
            goal->setOperation("subtraction");

            REQUIRE_THROWS(study.template initSolver<TestType>());
        }
    }
}

TEST_CASE("MocoOutputPeriodicityGoal", "[casadi]") {

    // Sliding mass problem with periodic body linear velocity.
    // --------------------------------------------------------
    auto study = createStudy({-100, 100.0}, {-100, 100.0});
    auto &problem = study.updProblem();
    problem.template addGoal<MocoControlGoal>("effort");
    auto *goal = problem.template addGoal<MocoOutputPeriodicityGoal>();
    goal->setName("periodic_speed");
    goal->setOutputPath("/body|linear_velocity");
    goal->setMode("endpoint_constraint");
    auto &solver = study.initSolver<MocoCasADiSolver>();
    solver.set_num_mesh_intervals(10);
    MocoSolution solution = study.solve();
    double initialSpeed = solution.getState("/slider/position/speed")[0];
    double finalSpeed = solution.getState(
            "/slider/position/speed")[solution.getNumTimes() - 1];
    CHECK(initialSpeed == Approx(finalSpeed).margin(1e-6));
}

TEMPLATE_TEST_CASE("MocoOutputTrackingGoal", "", MocoCasADiSolver) {

    // Have a sliding mass track a sinusoidal function.
    // ------------------------------------------------
    Sine trackingFunction(0.5, SimTK::Pi / 2.0, 0.0, 0.0);
    auto studyTracking = createStudy({-100.0, 100.0}, {-100.0, 100.0});
    auto& problemTracking = studyTracking.updProblem();
    problemTracking.setStateInfo("/slider/position/value", {-5.0, 5.0});
    problemTracking.template addGoal<MocoControlGoal>("effort", 1e-3);
    auto* goalTracking =
            problemTracking.template addGoal<MocoOutputTrackingGoal>();
    goalTracking->setName("speed_tracking");
    goalTracking->setOutputPath("/body|position");
    goalTracking->setExponent(2);
    goalTracking->setOutputIndex(0);
    goalTracking->setTrackingFunction(trackingFunction);
    auto& solverTracking = studyTracking.initSolver<TestType>();
    solverTracking.set_num_mesh_intervals(10);
    MocoSolution solutionTracking = studyTracking.solve();
    auto solutionPosition =
            solutionTracking.getState("/slider/position/value");
    SimTK::Vector time = solutionTracking.getTime();
    SimTK::Vector trackedPosition(solutionTracking.getNumTimes(), 0.0);
    for (int itime = 0; itime < solutionTracking.getNumTimes(); ++itime) {
        SimTK::Vector timeVec(1, time[itime]);
        trackedPosition[itime] = trackingFunction.calcValue(timeVec);
    }
    auto trackingError = solutionPosition - trackedPosition;
    CHECK(trackingError.normRMS() == Approx(0).margin(1e-3));
}

/// This goal violates the rule that calcIntegrandImpl() and calcGoalImpl()
/// cannot realize the state's stage beyond the stage dependency.
class MocoStageTestingGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoStageTestingGoal, MocoGoal);
public:
    MocoStageTestingGoal() = default;
    void setRealizeInitialState(bool tf) { m_realizeInitialState = tf; }
protected:
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(1, 1, SimTK::Stage::Position);
    }
    void calcIntegrandImpl(const IntegrandInput& input,
            SimTK::Real& integrand) const override {
        getModel().realizeVelocity(input.state);
    }
    void calcGoalImpl(
            const GoalInput& in, SimTK::Vector& values) const override {
        if (m_realizeInitialState) {
            getModel().realizeVelocity(in.initial_state);
        } else {
            getModel().realizeVelocity(in.final_state);
        }
    }
private:
    bool m_realizeInitialState = true;
};

// Ensure that goals do not internally realize beyond the stage they say
// they depend on.
TEST_CASE("MocoGoal stage dependency") {
    Model model;
    SimTK::State state = model.initSystem();
    MocoStageTestingGoal goal;
    goal.initializeOnModel(model);
    state.invalidateAll(SimTK::Stage::Instance);
    CHECK_THROWS_WITH(goal.calcIntegrand({0, state, {}}),
            ContainsSubstring("calcIntegrand()"));

    goal.setRealizeInitialState(true);
    state.invalidateAll(SimTK::Stage::Instance);
    auto initialState = state;
    auto finalState = state;
    MocoGoal::GoalInput input{0, initialState, {}, 0, finalState,
            {}, 0};
    SimTK::Vector goalValue;
    CHECK_THROWS_WITH(goal.calcGoal(input, goalValue),
            ContainsSubstring("calcGoal()") &&
            ContainsSubstring("initial_state"));
    goal.setRealizeInitialState(false);
    CHECK_THROWS_WITH(goal.calcGoal(input, goalValue),
            ContainsSubstring("calcGoal()") &&
            ContainsSubstring("final_state"));
}

/// This goal calculates the displacement of the system between the initial and
/// final states, the duration of the phase, and the mass of the system.
class MocoDivideByTestingGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoDivideByTestingGoal, MocoGoal);
public:
    MocoDivideByTestingGoal() = default;
protected:
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(0, 3, SimTK::Stage::Position);
    }
    void calcGoalImpl(
            const GoalInput& in, SimTK::Vector& values) const override {
        values[0] = calcSystemDisplacement(in);
        values[1] = calcDuration(in);
        values[2] = calcSystemMass(in);
    }
};

// Ensure that the "divide by" methods return the correct values.
TEST_CASE("MocoGoal divide by displacement/duration/mass") {
    Model model = ModelFactory::createSlidingPointMass();
    SimTK::State state = model.initSystem();
    const double mass = model.getTotalMass(state);
    MocoDivideByTestingGoal goal;
    goal.initializeOnModel(model);
    state.invalidateAll(SimTK::Stage::Instance);
    auto initialState = state;
    auto finalState = state;
    
    // Initial and final time.
    const double initial_time = 0.0;
    const double final_time = 2.5;
    const double duration = final_time - initial_time;
    initialState.setTime(initial_time);
    finalState.setTime(final_time);

    // Initial and final position.
    const double initial_position = 0.12;
    const double final_position = 0.74;
    const double displacement = final_position - initial_position;
    initialState.updQ()[0] = initial_position;
    finalState.updQ()[0] = final_position;

    MocoGoal::GoalInput input{initial_time, initialState, {}, 
            final_time, finalState, {}, 0};
    SimTK::Vector goalValues;
    goal.calcGoal(input, goalValues);

    CHECK_THAT(goalValues[0],
        Catch::Matchers::WithinAbs(displacement, SimTK::Eps));
    CHECK_THAT(goalValues[1],
        Catch::Matchers::WithinAbs(duration, SimTK::Eps));
    CHECK_THAT(goalValues[2],
        Catch::Matchers::WithinAbs(mass, SimTK::Eps));
}

TEST_CASE("MocoFrameDistanceConstraint de/serialization") {

    {
        auto study = createStudy(0, {0, 100.0});
        auto& problem = study.updProblem();
        auto* con = problem.addPathConstraint<MocoFrameDistanceConstraint>();
        con->setName("distance_constraint");
        con->addFramePair("/body", "/ground", 0, 1);
        study.print("testMocoGoals_MocoFrameDistanceConstraint_study.omoco");
    }

    {
        MocoStudy study(
                "testMocoGoals_MocoFrameDistanceConstraint_study.omoco");
    }
}

// from testMocoParameters
const double STIFFNESS = 100.0; // N/m
const double MASS = 5.0; // kg
const double FINAL_TIME = SimTK::Pi * sqrt(MASS / STIFFNESS);
std::unique_ptr<Model> createOscillatorTwoSpringsModel() {
    auto model = std::make_unique<Model>();
    model->setName("oscillator_two_springs");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", MASS, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addComponent(joint);

    auto* spring1 = new SpringGeneralizedForce();
    spring1->setName("spring1");
    spring1->set_coordinate("position");
    spring1->setRestLength(0.0);
    spring1->setStiffness(0.25*STIFFNESS);
    spring1->setViscosity(0.0);
    model->addComponent(spring1);

    auto* spring2 = new SpringGeneralizedForce();
    spring2->setName("spring2");
    spring2->set_coordinate("position");
    spring2->setRestLength(0.0);
    spring2->setStiffness(0.25*STIFFNESS);
    spring2->setViscosity(0.0);
    model->addComponent(spring2);

    return model;
}

TEST_CASE("MocoExpressionBasedParameterGoal - MocoCasADiSolver") {
    MocoStudy study;
    study.setName("oscillator_spring_stiffnesses");
    MocoProblem& mp = study.updProblem();
    mp.setModel(createOscillatorTwoSpringsModel());
    mp.setTimeBounds(0, FINAL_TIME);
    mp.setStateInfo("/slider/position/value", {-5.0, 5.0}, -0.5, {0.25, 0.75});
    mp.setStateInfo("/slider/position/speed", {-20, 20}, 0, 0);

    SECTION("single parameter for two values") {
        // create a parameter goal for the stiffness of both strings
        std::vector<std::string> components = {"spring1", "spring2"};
        auto* parameter = mp.addParameter("spring_stiffness", components,
                                          "stiffness", MocoBounds(0, 100));
        auto* spring_goal = mp.addGoal<MocoExpressionBasedParameterGoal>();
        // minimum is when p = 0.5*STIFFNESS
        spring_goal->setExpression(fmt::format("(p-{})^2", 0.5*STIFFNESS));
        spring_goal->addParameter(*parameter, "p");

        auto& ms = study.initCasADiSolver();
        ms.set_num_mesh_intervals(25);
        // not requiring initsystem is faster, still works with spring stiffness
        ms.set_parameters_require_initsystem(false);
        MocoSolution sol = study.solve();

        CHECK(sol.getParameter("spring_stiffness") ==
                Catch::Approx(0.5*STIFFNESS).epsilon(1e-6));
    }

    SECTION("two parameters") {
        // create two parameters to include in the goal
        auto* parameter = mp.addParameter("spring_stiffness", "spring1",
                                          "stiffness", MocoBounds(0, 100));
        auto* parameter2 = mp.addParameter("spring2_stiffness", "spring2",
                                          "stiffness", MocoBounds(0, 100));
        auto* spring_goal = mp.addGoal<MocoExpressionBasedParameterGoal>();
        // minimum is when p + q = STIFFNESS
        spring_goal->setExpression(fmt::format("square( p+q-{} )", STIFFNESS));
        spring_goal->addParameter(*parameter, "p");
        spring_goal->addParameter(*parameter2, "q");


        auto& ms = study.initCasADiSolver();
        ms.set_num_mesh_intervals(25);
        // not requiring initsystem is faster, still works with spring stiffness
        ms.set_parameters_require_initsystem(false);
        MocoSolution sol = study.solve();

        CHECK(sol.getParameter("spring_stiffness") +
              sol.getParameter("spring2_stiffness") ==
              Catch::Approx(STIFFNESS).epsilon(1e-6));
    }

    SECTION("missing parameter") {
        // create one parameter but have two variables
        auto* parameter = mp.addParameter("spring_stiffness", "spring1",
                                          "stiffness", MocoBounds(0, 100));

        auto* spring_goal = mp.addGoal<MocoExpressionBasedParameterGoal>(
                "stiffness", 1, fmt::format("(p+q-{})^2", STIFFNESS));
        spring_goal->addParameter(*parameter, "p");

        CHECK_THROWS(study.initCasADiSolver()); // missing q
    }

    SECTION("extra parameter") {
        // create two parameters for the goal, only one is used
        auto* parameter = mp.addParameter("spring_stiffness", "spring1",
                                          "stiffness", MocoBounds(0, 100));
        auto* parameter2 = mp.addParameter("spring2_stiffness", "spring2",
                                          "stiffness", MocoBounds(0, 100));

        auto* spring_goal = mp.addGoal<MocoExpressionBasedParameterGoal>(
                "stiffness", 1, fmt::format("(p-{})^2", STIFFNESS));
        spring_goal->addParameter(*parameter, "p");
        // second parameter is ignored
        spring_goal->addParameter(*parameter2, "a");

        CHECK_NOTHROW(study.initCasADiSolver());
    }

    SECTION("endpoint goal") {
        auto* parameter = mp.addParameter("spring_stiffness", "spring1",
                                          "stiffness", MocoBounds(0, 100));
        auto* parameter2 = mp.addParameter("spring2_stiffness", "spring2",
                                          "stiffness", MocoBounds(0, 100));
        auto* spring_goal = mp.addGoal<MocoExpressionBasedParameterGoal>();
        spring_goal->setExpression(fmt::format("square( p+q-{} )", STIFFNESS));
        spring_goal->addParameter(*parameter, "p");
        spring_goal->addParameter(*parameter2, "q");
        // set as endpoint constraint
        spring_goal->setMode("endpoint_constraint");

        auto& ms = study.initCasADiSolver();
        ms.set_num_mesh_intervals(25);
        // not requiring initsystem is faster, still works with spring stiffness
        ms.set_parameters_require_initsystem(false);
        MocoSolution sol = study.solve();

        CHECK(sol.getParameter("spring_stiffness") +
              sol.getParameter("spring2_stiffness") ==
              Catch::Approx(STIFFNESS).epsilon(1e-6));
    }
}
