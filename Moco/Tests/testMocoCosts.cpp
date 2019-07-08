/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoCosts.cpp                                            *
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

#define CATCH_CONFIG_MAIN
#include "Testing.h"
#include <Moco/osimMoco.h>

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/PointActuator.h>
#include <OpenSim/Common/LogManager.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

using namespace OpenSim;

std::unique_ptr<Model> createSlidingMassModel() {
    auto model = make_unique<Model>();
    model->setName("sliding_mass");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);

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

/// Test the result of a sliding mass minimum effort problem.
TEMPLATE_TEST_CASE(
        "Test MocoControlCost", "", MocoTropterSolver, MocoCasADiSolver) {
    const int N = 10;         // mesh points
    const int Nc = 2 * N - 1; // collocation points (Hermite-Simpson)
    MocoSolution sol1;
    {
        MocoStudy moco;
        moco.setName("sliding_mass");
        MocoProblem& mp = moco.updProblem();
        mp.setModel(createSlidingMassModel());
        mp.setTimeBounds(0, {0, 5});
        mp.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        mp.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        mp.setControlInfo("/actuator", MocoBounds(-10, 10));

        mp.addCost<MocoControlCost>();

        auto& ms = moco.initSolver<TestType>();
        ms.set_num_mesh_points(N);

        sol1 = moco.solve();
        sol1.write("testMocoCosts_testMocoControlCost_sol1.sto");

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

    //    MocoStudy moco;
    //    moco.setName("sliding_mass");
    //    MocoProblem& mp = moco.updProblem();
    //    MocoProblem& mp = moco.updProblem();
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

    //    MocoControlCost effort;
    //    effort.addActuatorToInclude("actuator");
    //    effort.set
    //    mp.addCost(effort);

    //    MocoTropterSolver& ms = moco.initSolver();
    //    ms.set_num_mesh_points(N);

    //    MocoSolution solution = moco.solve();
    //    SimTK_TEST_EQ(solution.getControl("actuator2"), SimTK::Vector(N, 0));
    //}

    // TODO test from XML.

    // Ensure that the weights cause one actuator to be preferred over
    // another.
    MocoSolution sol2;
    std::string omocoFile = "testMocoCosts_testMocoControlCost.omoco";
    {
        MocoStudy moco;
        moco.setName("sliding_mass");
        moco.set_write_solution("false");
        MocoProblem& mp = moco.updProblem();
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

        auto effort = mp.addCost<MocoControlCost>();
        effort->setWeight("/actuator2", 2.0);

        auto& ms = moco.initSolver<TestType>();
        ms.set_num_mesh_points(N);

        sol2 = moco.solve();

        moco.print(omocoFile);

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
        MocoStudy moco;
        moco.setName("sliding_mass");
        MocoProblem& mp = moco.updProblem();
        mp.setModel(createSlidingMassModel());
        mp.setTimeBounds(0, {0, 5});
        mp.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        mp.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        mp.setControlInfo("/actuator", MocoBounds(-10, 10));
        auto effort = mp.addCost<MocoControlCost>();
        effort->setWeight("nonexistent", 1.5);
        CHECK_THROWS_AS(mp.createRep(), Exception);
    }

    // De/serialization.
    {
        MocoStudy moco(omocoFile);
        MocoSolution solDeserialized = moco.solve();
        sol2.write("DEBUG_sol2.sto");
        solDeserialized.write("DEBUG_solDeserialized.sto");
        CHECK(solDeserialized.isNumericallyEqual(sol2, 1e-5));
    }
}

/// Make sure that multiple costs are added together properly.
// TODO can we bring this back somehow?
// TEST_CASE("Test multiple costs.") {
//     MocoStudy moco;
//     MocoProblem& problem = moco.updProblem();
//
//     auto* ft0 = problem.addCost<MocoFinalTimeCost>("ft0", 0.1);
//
//     auto* ft1 = problem.addCost<MocoFinalTimeCost>("ft1", 0.2);
//
//     MocoProblemRep rep = problem.createRep();
//     SimTK::State state = rep.getModelBase().getWorkingState();
//     const double ft = 0.35;
//     state.setTime(ft);
//
//     const double cost = rep.calcCost({state, state, 0});
//     CHECK(cost == Approx((ft0->get_weight() + ft1->get_weight()) * ft));
// }

TEST_CASE("Enabled Costs", "") {
    double x = 23920;
    MocoFinalTimeCost cost;
    Model model;
    auto state = model.initSystem();
    state.setTime(x);
    CHECK(cost.calcCost({state, state, 0}) == Approx(x));
    cost.set_enabled(false);
    CHECK(cost.calcCost({state, state, 0})  == 0);
}

template <class SolverType>
MocoStudy setupMocoStudyDoublePendulumMinimizeEffort() {
    using SimTK::Pi;
    const Model doublePendulum = ModelFactory::createNLinkPendulum(2);

    MocoStudy moco;
    auto& problem = moco.updProblem();
    problem.setModelCopy(doublePendulum);
    problem.addCost<MocoControlCost>("effort");
    problem.setTimeBounds(0, 2);
    problem.setControlInfo("/tau0", {-100, 100});
    problem.setControlInfo("/tau1", {-100, 100});
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0, Pi / 2);
    problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0, 0);
    problem.setStateInfo("/jointset/j1/q1/value", {-10, 10}, Pi, 0);
    problem.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0, 0);

    auto& solver = moco.initSolver<SolverType>();
    solver.set_num_mesh_points(20);
    solver.set_optim_convergence_tolerance(1e-6);

    return moco;
}

TEMPLATE_TEST_CASE("Test MocoControlTrackingCost", "", MocoTropterSolver,
        MocoCasADiSolver) {
    std::cout.rdbuf(LogManager::cout.rdbuf());
    std::cout.rdbuf(LogManager::cout.rdbuf());

    // Start with double pendulum problem to minimize control effort to create
    // a controls trajectory to track.
    MocoStudy moco = setupMocoStudyDoublePendulumMinimizeEffort<TestType>();
    auto solutionEffort = moco.solve();
    solutionEffort.write("testMocoCosts_MocoControlTrackingCost_effort_solution.sto");

    // Re-run problem, now setting effort cost function to zero and adding a
    // control tracking cost.
    auto& problem = moco.updProblem();
    problem.updPhase(0).updCost("effort").set_weight(0);
    auto* tracking =
            problem.addCost<MocoControlTrackingCost>("control_tracking");
    std::vector<double> time(solutionEffort.getTime().getContiguousScalarData(),
            solutionEffort.getTime().getContiguousScalarData() +
                    solutionEffort.getNumTimes());
    TimeSeriesTable controlsRef(time, solutionEffort.getControlsTrajectory(),
            solutionEffort.getControlNames());
    tracking->setReference(controlsRef);

    // Finding a solution with Hermite-Simpson and Tropter requires a better
    // initial guess.
    auto& solver = moco.updSolver<TestType>();
    solver.resetProblem(problem);
    MocoTrajectory guessTracking = solutionEffort;
    guessTracking.randomizeAdd();
    solver.setGuess(guessTracking);
    auto solutionTracking = moco.solve();
    solutionEffort.write("testMocoCosts_MocoControlTrackingCost_tracking_solution.sto");

    // Make sure control tracking problem matches control effort problem.
    OpenSim_CHECK_MATRIX_ABSTOL(solutionEffort.getControlsTrajectory(),
            solutionTracking.getControlsTrajectory(), 1e-4);
    OpenSim_CHECK_MATRIX_ABSTOL(solutionEffort.getStatesTrajectory(),
            solutionTracking.getStatesTrajectory(), 1e-4);
}

template <typename SolverType, typename TrackingType>
void testDoublePendulumTracking() {
    // Start with double pendulum problem to minimize control effort to create
    // a controls trajectory to track.
    MocoStudy moco = setupMocoStudyDoublePendulumMinimizeEffort<SolverType>();
    auto solutionEffort = moco.solve();
    const std::string typeString = TrackingType::getClassName();
    solutionEffort.write("testMocoCosts_" + typeString + "_effort_solution.sto");

    // Re-run problem, now setting effort cost function to zero and adding a
    // tracking cost.
    auto& problem = moco.updProblem();
    problem.updPhase(0).updCost("effort").set_weight(0);
    auto* tracking = problem.addCost<TrackingType>("tracking");
    tracking->setStatesReference(solutionEffort.exportToStatesTable());
    tracking->setFramePaths({"/bodyset/b0", "/bodyset/b1"});

    moco.updSolver<SolverType>().resetProblem(problem);
    auto solutionTracking = moco.solve();
    solutionTracking.write("testMocoCosts_" + typeString + "_tracking_solution.sto");

    // Check that position-level states match the effort minimization solution.
    CHECK(solutionTracking.compareContinuousVariablesRMS(solutionEffort,
                  {{"states", {"/jointset/j0/q0/value",
                                      "/jointset/j1/q1/value"}}}) ==
            Approx(0).margin(1e-2));

    // Re-run problem again, now setting effort cost function weight to a low
    // non-zero value as a regularization to smooth controls and velocity
    // states.
    problem.updPhase(0).updCost("effort").set_weight(0.001);
    moco.updSolver<SolverType>().resetProblem(problem);
    auto solutionTrackingWithRegularization = moco.solve();
    solutionTrackingWithRegularization.write("testMocoCosts_" + typeString + "_trackingWithReg_solution.sto");

    // Now the full states and controls trajectories should match the effort
    // minimization solution better.
    SimTK_TEST_EQ_TOL(solutionEffort.getControlsTrajectory(),
            solutionTrackingWithRegularization.getControlsTrajectory(), 1e-2);
    SimTK_TEST_EQ_TOL(solutionEffort.getStatesTrajectory(),
            solutionTrackingWithRegularization.getStatesTrajectory(), 1e-2);
}

TEMPLATE_TEST_CASE("Test MocoOrientationTrackingCost", "", MocoTropterSolver,
        MocoCasADiSolver) {
    testDoublePendulumTracking<TestType, MocoOrientationTrackingCost>();
}

TEMPLATE_TEST_CASE("Test MocoTranslationTrackingCost", "", MocoTropterSolver,
        MocoCasADiSolver) {
    testDoublePendulumTracking<TestType, MocoTranslationTrackingCost>();
}

TEMPLATE_TEST_CASE(
        "Test MocoJointReactionCost", "", MocoTropterSolver, MocoCasADiSolver) {

    using SimTK::Inertia;
    using SimTK::Vec3;

    // Create a model of a point mass welded to the ground.
    Model model;
    model.setName("welded_point_mass");
    model.setGravity(Vec3(10, 0, 0));

    auto* body = new Body("body", 1, Vec3(0), Inertia(0));
    model.addBody(body);
    auto* weldJoint = new WeldJoint("weld", model.getGround(), *body);
    model.addJoint(weldJoint);
    auto* actuator = new PointActuator("body");
    actuator->setName("actu");
    actuator->set_point(Vec3(0));
    actuator->set_direction(Vec3(1, 0, 0));
    model.addComponent(actuator);
    model.finalizeConnections();

    MocoStudy moco;
    moco.setName("counteract_gravity");
    MocoProblem& mp = moco.updProblem();
    mp.setModelCopy(model);

    mp.setTimeBounds(0, 1);
    mp.setControlInfo("/actu", {-20, 20});

    auto* reaction = mp.addCost<MocoJointReactionCost>();
    reaction->setJointPath("/jointset/weld");
    reaction->setReactionMeasures({"force-x"});

    auto& ms = moco.initSolver<TestType>();
    int N = 5;
    ms.set_num_mesh_points(N);
    ms.set_optim_convergence_tolerance(1e-6);

    MocoSolution solution = moco.solve().unseal();
    solution.write("testMocoCosts_testMocoJointReactionCost.sto");

    // Check that the actuator "actu" is equal to gravity (i.e. supporting all
    // of the weight).
    CHECK(solution.getControl("/actu")[0] == Approx(-10).epsilon(1e-6));
    // Check that the reaction force is zero.
    CHECK(solution.getObjective() == Approx(0.0).margin(1e-6));
}
