/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoInterface.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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
#include <OpenSim/Common/LogManager.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

using namespace OpenSim;

// TODO
// - add setGuess
// - add documentation. pre/post conditions.
// - write test cases for exceptions, for calling methods out of order.
// - model_file vs model.
// - test problems without controls (including with setting guesses).
// - test that names for setStateInfo() are actual existing states in the model.

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
    actu->setMinControl(-10);
    actu->setMaxControl(10);
    model->addComponent(actu);

    return model;
}

template <typename SolverType = MocoTropterSolver>
MocoTool createSlidingMassMocoTool() {
    MocoTool moco;
    moco.setName("sliding_mass");
    moco.set_write_solution("false");
    MocoProblem& mp = moco.updProblem();
    mp.setModel(createSlidingMassModel());
    mp.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(0, 10));
    mp.setStateInfo("/slider/position/value", MocoBounds(0, 1),
            MocoInitialBounds(0), MocoFinalBounds(1));
    mp.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
    mp.addCost<MocoFinalTimeCost>();

    auto& ms = moco.initSolver<SolverType>();
    ms.set_num_mesh_points(20);
    return moco;
}

/// This model is torque-actuated.
std::unique_ptr<Model> createPendulumModel() {
    auto model = make_unique<Model>();
    model->setName("pendulum");

    using SimTK::Inertia;
    using SimTK::Vec3;

    auto* b0 = new Body("b0", 1, Vec3(0), Inertia(1));
    model->addBody(b0);

    // Default pose: COM of pendulum is 1 meter down from the pin.
    auto* j0 = new PinJoint("j0", model->getGround(), Vec3(0), Vec3(0), *b0,
            Vec3(0, 1.0, 0), Vec3(0));
    auto& q0 = j0->updCoordinate();
    q0.setName("q0");
    model->addJoint(j0);

    auto* tau0 = new CoordinateActuator();
    tau0->setCoordinate(&j0->updCoordinate());
    tau0->setName("tau0");
    tau0->setOptimalForce(1);
    model->addForce(tau0);

    // Add display geometry.
    Ellipsoid bodyGeometry(0.1, 0.5, 0.1);
    SimTK::Transform transform(SimTK::Vec3(0, 0.5, 0));
    auto* b0Center = new PhysicalOffsetFrame("b0_center", *b0, transform);
    b0->addComponent(b0Center);
    b0Center->attachGeometry(bodyGeometry.clone());

    return model;
}

TEMPLATE_TEST_CASE("Solver options", "", MocoTropterSolver, MocoCasADiSolver) {
    MocoTool moco = createSlidingMassMocoTool<TestType>();
    auto& ms = moco.initSolver<TestType>();
    MocoSolution solDefault = moco.solve();
    ms.set_verbosity(3); // Invalid value.
    SimTK_TEST_MUST_THROW_EXC(moco.solve(), Exception);
    ms.set_verbosity(2);

    ms.set_optim_solver("nonexistent");
    SimTK_TEST_MUST_THROW_EXC(moco.solve(), Exception);
    ms.set_optim_solver("ipopt");

    ms.set_optim_hessian_approximation("nonexistent");
    SimTK_TEST_MUST_THROW(moco.solve());
    ms.set_optim_hessian_approximation("limited-memory");

    {
        ms.set_optim_max_iterations(1);
        MocoSolution solution = moco.solve();
        SimTK_TEST(solution.isSealed());
        solution.unseal();
        SimTK_TEST(solution.getNumIterations() == 1);
        ms.set_optim_max_iterations(-1);
    }

    {
        ms.set_optim_convergence_tolerance(1e-2);
        MocoSolution solLooseConvergence = moco.solve();
        // Ensure that we unset max iterations from being 1.
        SimTK_TEST(solLooseConvergence.getNumIterations() > 1);
        SimTK_TEST(solLooseConvergence.getNumIterations() <
                   solDefault.getNumIterations());
        ms.set_optim_convergence_tolerance(-1);
    }
    {
        // Must loosen convergence tolerance for constraint tolerance to have
        // an effect.
        ms.set_optim_convergence_tolerance(1e-2);
        // Tightening the constraint tolerance means more iterations.
        ms.set_optim_constraint_tolerance(1e-12);
        MocoSolution solutionTight = moco.solve();
        ms.set_optim_constraint_tolerance(1e-2);
        MocoSolution solutionLoose = moco.solve();
        SimTK_TEST(solutionTight.getNumIterations() >
                   solutionLoose.getNumIterations());
        ms.set_optim_constraint_tolerance(-1);
        ms.set_optim_convergence_tolerance(-1);
    }
}

/*

TEST_CASE("Ordering of calls") {

    // Solve a problem, edit the problem, re-solve.
    {
        // It's fine to
        MocoTool moco = createSlidingMassMocoTool();
        auto& solver = moco.initTropterSolver();
        moco.solve();
        // This flips the "m_solverInitialized" flag:
        moco.updProblem();
        // This will call initSolver() internally:
        moco.solve();
    }

    // Solve a problem, edit the problem, ask the solver to do something.
    {
        MocoTool moco = createSlidingMassMocoTool();
        auto& solver = moco.initTropterSolver();
        moco.solve();
        // This resets the problem to null on the solver.
        moco.updProblem();
        // The solver can't do anything if you've edited the model.
        SimTK_TEST_MUST_THROW_EXC(solver.getProblem(), Exception);
        SimTK_TEST_MUST_THROW_EXC(solver.solve(), Exception);
    }

    // Solve a problem, edit the solver, re-solve.
    {
        MocoTool moco = createSlidingMassMocoTool();
        auto& solver = moco.initTropterSolver();
        const int initNumMeshPoints = solver.get_num_mesh_points();
        MocoSolution sol0 = moco.solve();
        solver.set_num_mesh_points(2 * initNumMeshPoints);
        MocoSolution sol1 = moco.solve();
        solver.set_num_mesh_points(initNumMeshPoints);
        MocoSolution sol2 = moco.solve();
        // Ensure that changing the mesh has an effect.
        SimTK_TEST(!sol0.isNumericallyEqual(sol1));
        // Ensure we get repeatable results with the initial settings.
        SimTK_TEST(sol0.isNumericallyEqual(sol2));

    }
}

/// Test that we can read in a Moco setup file, solve, edit the setup,
/// re-solve.
void testOMOCOSerialization() {
    std::string fname = "testMocoInterface_testOMOCOSerialization.omoco";
    MocoSolution sol0;
    MocoSolution sol1;
    {
        MocoTool moco = createSlidingMassMocoTool();
        sol0 = moco.solve();
        moco.print(fname);
    }
    {
        MocoTool mocoDeserialized(fname);
        MocoSolution sol1 = mocoDeserialized.solve();
    }
    SimTK_TEST(sol0.isNumericallyEqual(sol1));
}

void testCopy() {
    MocoTool moco = createSlidingMassMocoTool();
    MocoSolution solution = moco.solve();
    std::unique_ptr<MocoTool> copy(moco.clone());
    MocoSolution solutionFromCopy = copy->solve();
    SimTK_TEST(solution.isNumericallyEqual(solutionFromCopy));


    // TODO what happens if just the MocoProblem is copied, or if just the
    // MocoSolver is copied?
}
 */

TEST_CASE("Bounds", "") {
    {
        SimTK_TEST(!MocoBounds().isSet());
        SimTK_TEST(MocoBounds(5.3).isSet());
        SimTK_TEST(MocoBounds(5.3).isEquality());
        SimTK_TEST(MocoBounds(5.3, 5.3).isSet());
        SimTK_TEST(MocoBounds(5.3, 5.3).isEquality());
        SimTK_TEST(!MocoBounds(5.3, 5.3 + SimTK::SignificantReal).isEquality());

        SimTK_TEST(MocoBounds(5.3).isWithinBounds(5.3));
        SimTK_TEST(
                !MocoBounds(5.3).isWithinBounds(5.3 + SimTK::SignificantReal));
        SimTK_TEST(MocoBounds(5.2, 5.4).isWithinBounds(5.3));
    }
    // TODO what to do about clamped coordinates? Use the range in the
    // coordinate, or ignore that? I think that if the coordinate is clamped,
    // then

    // By default, the bounds for coordinates, if clamped, are the
    // coordinate's range.

    // TODO what to do if the user does not specify info for some variables?

    // Get error if state/control name does not exist.
    {
        auto model = createSlidingMassModel();
        model->initSystem();
        {
            MocoTool moco;
            MocoProblem& mp = moco.updProblem();
            mp.setModel(std::unique_ptr<Model>(model->clone()));
            mp.setStateInfo("nonexistent", {0, 1});
            SimTK_TEST_MUST_THROW_EXC(mp.createRep(), Exception);
        }
        {
            MocoTool moco;
            MocoProblem& mp = moco.updProblem();
            mp.setModel(std::unique_ptr<Model>(model->clone()));
            mp.setControlInfo("nonexistent", {0, 1});
            SimTK_TEST_MUST_THROW_EXC(mp.createRep(), Exception);
        }
    }
    // TODO what if bounds are missing for some states?
}

TEST_CASE("Building a problem", "") {
    {
        MocoTool moco;
        MocoProblem& mp = moco.updProblem();
        mp.setModel(createSlidingMassModel());

        // Costs have the name "cost" by default.
        {
            auto c0 = make_unique<MocoFinalTimeCost>();
            SimTK_TEST(c0->getName() == "cost");
            mp.addCost(std::move(c0));
        }
        // Names of costs must be unique.
        {
            auto* c1 = mp.addCost<MocoFinalTimeCost>();
            SimTK_TEST_MUST_THROW_EXC(mp.createRep(), Exception);
            c1->setName("c1");
        }
        // Costs must have a name.
        {
            auto* cEmptyName = mp.addCost<MocoFinalTimeCost>("");
            SimTK_TEST_MUST_THROW_EXC(mp.createRep(), Exception);
            cEmptyName->setName("cost1");
        }
        // Parameters have the name "parameter" by default.
        {
            auto p0 = make_unique<MocoParameter>();
            SimTK_TEST(p0->getName() == "parameter");
            p0->appendComponentPath("/body");
            p0->setPropertyName("mass");
            mp.addParameter(std::move(p0));
            // Can successfully create a rep.
            mp.createRep();
        }
        // Names of parameters must be unique.
        {
            auto* param = mp.addParameter(
                    "parameter", "/body", "mass", MocoBounds(0, 0));
            SimTK_TEST_MUST_THROW_EXC(mp.createRep(), Exception);
            param->setName("parameter1");
            // Can now create rep.
            mp.createRep();
        }
        // Parameters must have a name.
        {
            auto pEmptyName = make_unique<MocoParameter>();
            pEmptyName->setName("");
            mp.addParameter(std::move(pEmptyName));
            SimTK_TEST_MUST_THROW_EXC(mp.createRep(), Exception);
        }
    }
}

TEMPLATE_TEST_CASE("Workflow", "", MocoTropterSolver, MocoCasADiSolver) {

    // Default bounds.
    {
        MocoTool moco;
        MocoProblem& problem = moco.updProblem();
        auto model = createSlidingMassModel();
        model->finalizeFromProperties();
        auto& coord = model->updComponent<Coordinate>("slider/position");
        coord.setRangeMin(-10);
        coord.setRangeMax(15);
        auto& actu = model->updComponent<ScalarActuator>("actuator");
        actu.setMinControl(35);
        actu.setMaxControl(56);
        problem.setModel(std::move(model));
        const auto& phase0 = problem.getPhase(0);
        // User did not specify state info explicitly.
        SimTK_TEST_MUST_THROW_EXC(
                phase0.getStateInfo("/slider/position/value"), Exception);
        {
            MocoProblemRep rep = problem.createRep();
            {
                const auto& info = rep.getStateInfo("/slider/position/value");
                SimTK_TEST_EQ(info.getBounds().getLower(), -10);
                SimTK_TEST_EQ(info.getBounds().getUpper(), 15);
            }
            {
                // Default speed bounds.
                const auto& info = rep.getStateInfo("/slider/position/speed");
                SimTK_TEST_EQ(info.getBounds().getLower(), -50);
                SimTK_TEST_EQ(info.getBounds().getUpper(), 50);
            }
            // No control info stored in the Problem.
            SimTK_TEST_MUST_THROW_EXC(
                    phase0.getControlInfo("/actuator"), Exception);
            {
                // Obtained from controls.
                const auto& info = rep.getControlInfo("/actuator");
                SimTK_TEST_EQ(info.getBounds().getLower(), 35);
                SimTK_TEST_EQ(info.getBounds().getUpper(), 56);
            }
        }

        problem.setControlInfo("/actuator", {12, 15});
        {
            {
                const auto& probinfo = phase0.getControlInfo("/actuator");
                SimTK_TEST_EQ(probinfo.getBounds().getLower(), 12);
                SimTK_TEST_EQ(probinfo.getBounds().getUpper(), 15);
            }
            MocoProblemRep rep = problem.createRep();
            {
                const auto& info = rep.getControlInfo("/actuator");
                SimTK_TEST_EQ(info.getBounds().getLower(), 12);
                SimTK_TEST_EQ(info.getBounds().getUpper(), 15);
            }
        }
    }

    // Ensure that changes to time bounds are obeyed.
    {
        MocoTool moco;
        MocoProblem& problem = moco.updProblem();
        problem.setModel(createSlidingMassModel());

        problem.setTimeBounds(0, {0, 10});
        problem.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        problem.setControlInfo("/actuator", {-10, 10});
        problem.addCost<MocoFinalTimeCost>();

        auto& solver = moco.initSolver<TestType>();
        solver.set_num_mesh_points(20);
        MocoIterate guess = solver.createGuess("random");
        guess.setTime(createVectorLinspace(20, 0.0, 3.0));
        solver.setGuess(guess);
        MocoSolution solution0 = moco.solve();

        problem.setTimeBounds(0, {5.8, 10});
        // Editing the problem does not affect information in the Solver; the
        // guess still exists.
        SimTK_TEST(!solver.getGuess().empty());

        guess.setTime(createVectorLinspace(20, 0.0, 7.0));
        MocoSolution solution = moco.solve();
        CHECK(solution.getFinalTime() == Approx(5.8));
    }

    {
        double finalTime0;
        {
            // Ensure that changes to the model are obeyed.
            MocoTool moco;
            MocoProblem& problem = moco.updProblem();
            auto model = problem.setModel(createSlidingMassModel());
            problem.setTimeBounds(0, {0, 10});
            problem.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
            problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
            problem.addCost<MocoFinalTimeCost>();
            auto& solver = moco.initSolver<TestType>();
            solver.set_num_mesh_points(20);
            finalTime0 = moco.solve().getFinalTime();

            auto& body = model->updComponent<Body>("body");
            body.setMass(2 * body.getMass());
            const double finalTime1 = moco.solve().getFinalTime();
            SimTK_TEST(finalTime1 > 1.1 * finalTime0);
        }

        // Can set the cost and model in any order.
        {
            MocoTool moco;
            MocoProblem& problem = moco.updProblem();
            problem.setTimeBounds(0, {0, 10});
            problem.addCost<MocoFinalTimeCost>();
            problem.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
            problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
            problem.setModel(createSlidingMassModel());
            auto& solver = moco.initSolver<TestType>();
            solver.set_num_mesh_points(20);
            const double finalTime = moco.solve().getFinalTime();
            SimTK_TEST_EQ_TOL(finalTime, finalTime0, 1e-6);
        }
    }

    // Changes to the costs are obeyed.
    {
        MocoTool moco;
        MocoProblem& problem = moco.updProblem();
        problem.setModel(createSlidingMassModel());
        problem.setTimeBounds(0, {0, 10});
        problem.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        problem.updPhase().addCost<MocoFinalTimeCost>();
        auto effort = problem.updPhase().addCost<MocoControlCost>("effort");
        const double finalTime0 = moco.solve().getFinalTime();

        // Change the weights of the costs.
        effort->set_weight(0.1);
        const double finalTime1 = moco.solve().getFinalTime();
        SimTK_TEST(finalTime1 < 0.8 * finalTime0);
    }

    // Invoking functions without initializing.
    {
        // TODO
    }

    // TODO MocoCost and MocoParameter cache pointers into some model.
    // TODO {
    // TODO     MocoFinalTimeCost cost;
    // TODO     // TODO must be initialized first.
    // TODO     // TODO MocoPhase shouldn't even have a public calcEndpointCost
    // function.
    // TODO     SimTK_TEST_MUST_THROW_EXC(cost.calcEndpointCost(state),
    // Exception);
    // TODO }

    // Allow removing costs.
    // TODO
    // {
    //     MocoTool moco;
    //     MocoProblem& problem = moco.updProblem();
    //     {
    //         // Remove by name.
    //         auto& cost = problem.addCost<MocoFinalTimeCost>();
    //         cost.setName("cost0");
    //         problem.removeCost(cost);
    //         SimTK_TEST_MUST_THROW_EXC(problem.getCost("cost0"), Exception);
    //     }
    // }
}

TEMPLATE_TEST_CASE("State tracking", "", MocoTropterSolver, MocoCasADiSolver) {
    // TODO move to another test file?
    auto makeTool = []() {
        MocoTool moco;
        moco.setName("state_tracking");
        moco.set_write_solution("false");
        MocoProblem& mp = moco.updProblem();
        mp.setModel(createSlidingMassModel());
        mp.setTimeBounds(0, 1);
        mp.setStateInfo("/slider/position/value", {-1, 1});
        mp.setStateInfo("/slider/position/speed", {-100, 100});
        mp.setControlInfo("/actuator", {-50, 50});
        return moco;
    };

    // Reference trajectory.
    std::string fname = "testMocoInterface_testStateTracking_ref.sto";
    {
        TimeSeriesTable ref;
        ref.setColumnLabels({"/slider/position/value"});
        using SimTK::Pi;
        for (double time = -0.01; time < 1.02; time += 0.01) {
            // Move at constant speed from x=0 to x=1. Really basic stuff.
            ref.appendRow(time, {1.0 * time});
        }
        STOFileAdapter::write(ref, fname);
    }

    // Setting the TimeSeriesTable directly.
    MocoSolution solDirect;
    {
        auto moco = makeTool();
        MocoProblem& mp = moco.updProblem();
        auto tracking = mp.addCost<MocoStateTrackingCost>();
        tracking->setReference(STOFileAdapter::read(fname));
        auto& ms = moco.template initSolver<TestType>();
        ms.set_num_mesh_points(5);
        ms.set_optim_hessian_approximation("exact");
        solDirect = moco.solve();
    }

    // Setting the reference to be a file.
    std::string setup_fname = "testMocoInterface_testStateTracking.omoco";
    if (std::ifstream(setup_fname).good()) std::remove(setup_fname.c_str());
    MocoSolution solFile;
    {

        auto moco = makeTool();
        MocoProblem& mp = moco.updProblem();
        auto tracking = mp.addCost<MocoStateTrackingCost>();
        tracking->setReferenceFile(fname);
        auto& ms = moco.template initSolver<TestType>();
        ms.set_num_mesh_points(5);
        ms.set_optim_hessian_approximation("exact");
        solFile = moco.solve();
        moco.print(setup_fname);
    }

    // Run the tool from a setup file.
    MocoSolution solDeserialized;
    {
        MocoTool moco(setup_fname);
        solDeserialized = moco.solve();
    }

    SimTK_TEST(solDirect.isNumericallyEqual(solFile));
    SimTK_TEST(solFile.isNumericallyEqual(solDeserialized));

    // Error if neither file nor table were provided.
    {
        auto moco = makeTool();
        MocoProblem& mp = moco.updProblem();
        MocoStateTrackingCost tracking;
        mp.addCost<MocoStateTrackingCost>();
        SimTK_TEST_MUST_THROW_EXC(moco.solve(), Exception);
    }

    // TODO error if data does not cover time window.
}

TEMPLATE_TEST_CASE("Guess", "", MocoTropterSolver, MocoCasADiSolver) {
    std::cout.rdbuf(LogManager::cout.rdbuf());
    std::cout.rdbuf(LogManager::cout.rdbuf());

    MocoTool moco = createSlidingMassMocoTool<TestType>();
    auto& ms = moco.initSolver<TestType>();
    const int N = 6;
    ms.set_num_mesh_points(N);

    std::vector<std::string> expectedStateNames{
            "/slider/position/value", "/slider/position/speed"};
    std::vector<std::string> expectedControlNames{"/actuator"};

    SimTK::Matrix expectedStatesTraj(N, 2);
    expectedStatesTraj.col(0) = 0.5;  // bounds are [0, 1].
    expectedStatesTraj(0, 0) = 0;     // initial value fixed to 0.
    expectedStatesTraj(N - 1, 0) = 1; // final value fixed to 1.
    expectedStatesTraj.col(1) = 0.0;  // bounds are [-100, 100]
    expectedStatesTraj(0, 1) = 0;     // initial speed fixed to 0.
    expectedStatesTraj(N - 1, 1) = 0; // final speed fixed to 1.

    SimTK::Matrix expectedControlsTraj(N, 1);
    expectedControlsTraj.col(0) = 0;

    // createGuess().
    // --------------

    // Initial guess based on bounds.
    {
        MocoIterate guess = ms.createGuess("bounds");
        SimTK_TEST(guess.getTime().size() == N);
        SimTK_TEST(guess.getStateNames() == expectedStateNames);
        SimTK_TEST(guess.getControlNames() == expectedControlNames);
        SimTK_TEST(guess.getTime()[0] == 0);
        SimTK_TEST_EQ(
                guess.getTime()[N - 1], 5.0); // midpoint of bounds [0, 10]

        SimTK_TEST_EQ(guess.getStatesTrajectory(), expectedStatesTraj);
        SimTK_TEST_EQ(guess.getControlsTrajectory(), expectedControlsTraj);
    }

    // Random initial guess.
    {
        MocoIterate guess = ms.createGuess("random");
        SimTK_TEST(guess.getTime().size() == N);
        SimTK_TEST(guess.getStateNames() == expectedStateNames);
        SimTK_TEST(guess.getControlNames() == expectedControlNames);

        // The numbers are random, so we don't what they are; only that they
        // are different from the guess from bounds.
        SimTK_TEST_NOTEQ(guess.getStatesTrajectory(), expectedStatesTraj);
        SimTK_TEST_NOTEQ(guess.getControlsTrajectory(), expectedControlsTraj);
    }

    // Setting a guess programmatically.
    // ---------------------------------

    // Don't need a converged solution; so ensure the following tests are fast.
    ms.set_optim_max_iterations(2);

    ms.clearGuess();
    MocoIterate solNoGuess = moco.solve().unseal();
    {
        // Using the guess from bounds is the same as not providing a guess.

        ms.setGuess(ms.createGuess());
        MocoIterate solDefaultGuess = moco.solve().unseal();

        SimTK_TEST(solDefaultGuess.isNumericallyEqual(solNoGuess));

        // Can also use convenience version of setGuess().
        ms.setGuess("bounds");
        SimTK_TEST(moco.solve().unseal().isNumericallyEqual(solNoGuess));

        // Using a random guess should give us a different "solution."
        ms.setGuess(ms.createGuess("random"));
        MocoIterate solRandomGuess = moco.solve().unseal();
        SimTK_TEST(!solRandomGuess.isNumericallyEqual(solNoGuess));

        // Convenience.
        ms.setGuess("random");
        SimTK_TEST(!moco.solve().unseal().isNumericallyEqual(solNoGuess));

        // Clearing the guess works (this check must come after using a
        // random guess).
        ms.clearGuess();
        SimTK_TEST(moco.solve().unseal().isNumericallyEqual(solNoGuess));

        // Can call clearGuess() multiple times with no weird issues.
        ms.clearGuess();
        ms.clearGuess();
        SimTK_TEST(moco.solve().unseal().isNumericallyEqual(solNoGuess));
    }

    // Guess is incompatible with problem.
    {
        MocoIterate guess = ms.createGuess();
        // Delete the second state variable name.
        const_cast<std::vector<std::string>&>(guess.getStateNames()).resize(1);
        SimTK_TEST_MUST_THROW_EXC(ms.setGuess(std::move(guess)), Exception);
    }

    // Unrecognized guess type.
    SimTK_TEST_MUST_THROW_EXC(ms.createGuess("unrecognized"), Exception);
    SimTK_TEST_MUST_THROW_EXC(ms.setGuess("unrecognized"), Exception);

    // Setting a guess from a file.
    // ----------------------------
    {
        MocoIterate guess = ms.createGuess("bounds");
        // Use weird number to ensure the solver actually loads the file:
        guess.setControl("/actuator", SimTK::Vector(N, 13.28));
        const std::string fname = "testMocoInterface_testGuess_file.sto";
        guess.write(fname);
        ms.setGuessFile(fname);

        SimTK_TEST(ms.getGuess().isNumericallyEqual(guess));
        SimTK_TEST(!moco.solve().unseal().isNumericallyEqual(solNoGuess));

        // Using setGuess(MocoIterate) overrides the file setting.
        ms.setGuess(ms.createGuess("bounds"));
        SimTK_TEST(moco.solve().unseal().isNumericallyEqual(solNoGuess));

        ms.setGuessFile(fname);
        SimTK_TEST(ms.getGuess().isNumericallyEqual(guess));
        SimTK_TEST(!moco.solve().unseal().isNumericallyEqual(solNoGuess));

        // Clearing the file causes the default guess type to be used.
        ms.setGuessFile("");
        SimTK_TEST(moco.solve().unseal().isNumericallyEqual(solNoGuess));

        // TODO mismatched state/control names.

        // Solve from deserialization.
        // TODO
    }

    // Customize a guess.
    // ------------------
    // This is really just a test of the MocoIterate class.
    {
        MocoIterate guess = ms.createGuess();
        guess.setNumTimes(2);
        SimTK_TEST(SimTK::isNaN(guess.getTime()[0]));
        SimTK_TEST(SimTK::isNaN(guess.getStatesTrajectory()(0, 0)));
        SimTK_TEST(SimTK::isNaN(guess.getControlsTrajectory()(0, 0)));

        // TODO look at how TimeSeriesTable handles this.
        // Make sure this uses the initializer list variant.
        guess.setState("/slider/position/value", {2, 0.3});
        SimTK::Vector expectedv(2);
        expectedv[0] = 2;
        expectedv[1] = 0.3;
        SimTK_TEST_EQ(guess.getState("/slider/position/value"), expectedv);

        // Can use SimTK::Vector.
        expectedv[1] = 9.4;
        guess.setState("/slider/position/value", expectedv);
        SimTK_TEST_EQ(guess.getState("/slider/position/value"), expectedv);

        // Controls
        guess.setControl("/actuator", {1, 0.6});
        SimTK::Vector expecteda(2);
        expecteda[0] = 1.0;
        expecteda[1] = 0.6;
        SimTK_TEST_EQ(guess.getControl("/actuator"), expecteda);

        expecteda[0] = 0.7;
        guess.setControl("/actuator", expecteda);
        SimTK_TEST_EQ(guess.getControl("/actuator"), expecteda);

        // Errors.

        // Nonexistent state/control.
        SimTK_TEST_MUST_THROW_EXC(
                guess.setState("none", SimTK::Vector(2)), Exception);
        SimTK_TEST_MUST_THROW_EXC(
                guess.setControl("none", SimTK::Vector(2)), Exception);
        SimTK_TEST_MUST_THROW_EXC(guess.getState("none"), Exception);
        SimTK_TEST_MUST_THROW_EXC(guess.getControl("none"), Exception);

        // Incorrect length.
        SimTK_TEST_MUST_THROW_EXC(
                guess.setState("/slider/position/value", SimTK::Vector(1)),
                Exception);
        SimTK_TEST_MUST_THROW_EXC(
                guess.setControl("/actuator", SimTK::Vector(3)), Exception);
    }

    // Resampling.
    {
        ms.set_num_mesh_points(5);
        MocoIterate guess0 = ms.createGuess();
        guess0.setControl("/actuator", createVectorLinspace(5, 2.8, 7.3));
        SimTK_TEST(guess0.getTime().size() == 5); // midpoint of [0, 10]
        SimTK_TEST_EQ(guess0.getTime()[4], 5);

        // resampleWithNumTimes
        {
            MocoIterate guess = guess0;
            guess.resampleWithNumTimes(10);
            SimTK_TEST(guess.getTime().size() == 10);
            SimTK_TEST_EQ(guess.getTime()[9], 5);
            SimTK_TEST(guess.getStatesTrajectory().nrow() == 10);
            SimTK_TEST(guess.getControlsTrajectory().nrow() == 10);
            SimTK_TEST_EQ(guess.getControl("/actuator"),
                    createVectorLinspace(10, 2.8, 7.3));
        }

        // resampleWithInterval
        {
            MocoIterate guess = guess0;
            // We can't achieve exactly the interval the user provides.
            // time_interval = duration/(num_times - 1)
            // actual_num_times = ceil(duration/desired_interval) + 1
            // actual_interval = duration/(actual_num_times - 1)
            auto actualInterval = guess.resampleWithInterval(0.9);
            int expectedNumTimes = (int)ceil(5 / 0.9) + 1;
            SimTK_TEST_EQ(actualInterval, 5 / ((double)expectedNumTimes - 1));
            SimTK_TEST(guess.getTime().size() == expectedNumTimes);
            SimTK_TEST_EQ(guess.getTime()[expectedNumTimes - 1], 5);
            SimTK_TEST(guess.getStatesTrajectory().nrow() == expectedNumTimes);
            SimTK_TEST(
                    guess.getControlsTrajectory().nrow() == expectedNumTimes);
            SimTK_TEST_EQ(guess.getControl("/actuator"),
                    createVectorLinspace(expectedNumTimes, 2.8, 7.3));
        }

        // resampleWithFrequency
        {
            // We can't achieve exactly the interval the user provides.
            // frequency = num_times/duration
            MocoIterate guess = guess0;
            // Here, we also ensure that we can downsample.
            auto actualFrequency = guess.resampleWithFrequency(0.7);
            int expectedNumTimes = (int)ceil(5 * 0.7); // 4
            SimTK_TEST_EQ(actualFrequency, (double)expectedNumTimes / 5);
            SimTK_TEST(guess.getTime().size() == expectedNumTimes);
            SimTK_TEST_EQ(guess.getTime()[expectedNumTimes - 1], 5);
            SimTK_TEST(guess.getStatesTrajectory().nrow() == expectedNumTimes);
            SimTK_TEST(
                    guess.getControlsTrajectory().nrow() == expectedNumTimes);
            SimTK_TEST_EQ(guess.getControl("/actuator"),
                    createVectorLinspace(expectedNumTimes, 2.8, 7.3));
        }

        // resample
        {
            MocoIterate guess = guess0;
            SimTK_TEST_MUST_THROW_EXC(
                    guess.resample(createVector(
                            {guess.getInitialTime() - 1e-15, 0, 1})),
                    Exception);
            SimTK_TEST_MUST_THROW_EXC(guess.resample(createVector({0.5, 0.6,
                                              guess.getFinalTime() + 1e15})),
                    Exception);
            SimTK_TEST_MUST_THROW_EXC(
                    guess.resample(createVector({0.5, 0.6, 0.59999999, 0.8})),
                    Exception);

            {
                SimTK::Vector newTime = createVector({0.1, 0.3, 0.8});
                MocoIterate guess2 = guess;
                guess2.resample(newTime);
                SimTK_TEST_EQ(newTime, guess2.getTime());
            }
        }
    }

    // Number of points required for splining.
    {
        // 3 and 2 points are okay.
        ms.set_num_mesh_points(3);
        MocoIterate guess3 = ms.createGuess();
        guess3.resampleWithNumTimes(10);

        ms.set_num_mesh_points(2);
        MocoIterate guess2 = ms.createGuess();
        guess2.resampleWithNumTimes(10);

        // 1 point is too few.
        MocoIterate guess1(guess2);
        guess1.setNumTimes(1);
        SimTK_TEST_MUST_THROW_EXC(guess1.resampleWithNumTimes(10), Exception);
    }

    // TODO ordering of states and controls in MocoIterate should not matter!

    // TODO getting a guess, editing the problem, asking for another guess,
    // requires calling initSolver<>(). TODO can check the Problem's
    // isObjectUptoDateWithProperties(); simply flipping a flag with
    // updProblem() is not sufficient, b/c a user could make changes way
    // after they get the mutable reference.
}

TEMPLATE_TEST_CASE(
        "Guess time-stepping", "", MocoTropterSolver /*, MocoCasADiSolver*/) {
    // This problem is just a simulation (there are no costs), and so the
    // forward simulation guess should reduce the number of iterations to
    // converge, and the guess and solution should also match our own forward
    // simulation.
    MocoTool moco;
    moco.setName("pendulum");
    moco.set_write_solution("false");
    auto& problem = moco.updProblem();
    problem.setModel(createPendulumModel());
    const SimTK::Real initialAngle = 0.25 * SimTK::Pi;
    const SimTK::Real initialSpeed = .5;
    // Make the simulation interesting.
    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, initialAngle);
    problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, initialSpeed);
    problem.setControlInfo("/forceset/tau0", 0);
    auto& solver = moco.initSolver<TestType>();
    solver.set_num_mesh_points(20);
    solver.setGuess("random");
    // With MUMPS: 4 iterations.
    const MocoSolution solutionRandom = moco.solve();

    solver.setGuess("time-stepping");
    // With MUMPS: 2 iterations.
    MocoSolution solutionSim = moco.solve();

    CHECK(solutionSim.getNumIterations() < solutionRandom.getNumIterations());

    {
        MocoIterate guess = solver.createGuess("time-stepping");
        REQUIRE(solutionSim.compareContinuousVariablesRMS(guess) < 1e-2);

        Model modelCopy(moco.updProblem().getPhase().getModel());
        SimTK::State state = modelCopy.initSystem();
        modelCopy.setStateVariableValue(
                state, "/jointset/j0/q0/value", initialAngle);
        modelCopy.setStateVariableValue(
                state, "/jointset/j0/q0/speed", initialSpeed);
        Manager manager(modelCopy, state);
        manager.integrate(1.0);

        auto controlsTable = modelCopy.getControlsTable();
        auto labels = controlsTable.getColumnLabels();
        for (auto& label : labels) { label = "/forceset/" + label; }
        controlsTable.setColumnLabels(labels);
        const auto iterateFromManager =
                MocoIterate::createFromStatesControlsTables(
                        moco.getProblem().createRep(), manager.getStatesTable(),
                        controlsTable);
        SimTK_TEST(solutionSim.compareContinuousVariablesRMS(
                           iterateFromManager) < 1e-2);
    }

    // Ensure the forward simulation guess uses the correct time bounds.
    {
        moco.updProblem().setTimeBounds({-10, -5}, {6, 15});
        auto& solver = moco.initSolver<TestType>();
        MocoIterate guess = solver.createGuess("time-stepping");
        SimTK_TEST(guess.getTime()[0] == -5);
        SimTK_TEST(guess.getTime()[guess.getNumTimes() - 1] == 6);
    }
}

TEST_CASE("MocoIterate") {
    // Reading and writing.
    {
        const std::string fname = "testMocoInterface_testMocoIterate.sto";
        SimTK::Vector time(3);
        time[0] = 0;
        time[1] = 0.1;
        time[2] = 0.25;
        MocoIterate orig(time, {"a", "b"}, {"g", "h", "i", "j"}, {"m"},
                {"o", "p"}, SimTK::Test::randMatrix(3, 2),
                SimTK::Test::randMatrix(3, 4), SimTK::Test::randMatrix(3, 1),
                SimTK::Test::randVector(2).transpose());
        orig.write(fname);

        MocoIterate deserialized(fname);
        SimTK_TEST(deserialized.isNumericallyEqual(orig));
    }

    // Test sealing/unsealing.
    {
        // Create a class that gives access to the sealed functions, which are
        // otherwise protected.
        class MocoIterateDerived : public MocoIterate {
        public:
            using MocoIterate::MocoIterate;
            MocoIterateDerived* clone() const override {
                return new MocoIterateDerived(*this);
            }
            void setSealedD(bool sealed) { MocoIterate::setSealed(sealed); }
            bool isSealedD() const { return MocoIterate::isSealed(); }
        };
        MocoIterateDerived iterate;
        SimTK_TEST(!iterate.isSealedD());
        iterate.setSealedD(true);
        SimTK_TEST(iterate.isSealedD());
        SimTK_TEST_MUST_THROW_EXC(iterate.getNumTimes(), MocoIterateIsSealed);
        SimTK_TEST_MUST_THROW_EXC(iterate.getTime(), MocoIterateIsSealed);
        SimTK_TEST_MUST_THROW_EXC(iterate.getStateNames(), MocoIterateIsSealed);
        SimTK_TEST_MUST_THROW_EXC(
                iterate.getControlNames(), MocoIterateIsSealed);
        SimTK_TEST_MUST_THROW_EXC(
                iterate.getControlNames(), MocoIterateIsSealed);

        // The clone() function doesn't call ensureSealed(), but the clone
        // should preserve the value of m_sealed.
        std::unique_ptr<MocoIterateDerived> ptr(iterate.clone());
        SimTK_TEST(ptr->isSealedD());
        SimTK_TEST_MUST_THROW_EXC(iterate.getNumTimes(), MocoIterateIsSealed);
    }

    // getInitialTime(), getFinalTime()
    {
        {
            // With 0 times, these functions throw an exception.
            MocoIterate it;
            SimTK_TEST_MUST_THROW_EXC(it.getInitialTime(), Exception);
            SimTK_TEST_MUST_THROW_EXC(it.getFinalTime(), Exception);
        }

        {
            SimTK::Vector time = createVectorLinspace(5, -3.1, 8.9);
            std::vector<std::string> snames{"s0", "s1"};
            std::vector<std::string> cnames{"c0"};
            SimTK::Matrix states = SimTK::Test::randMatrix(5, 2);
            SimTK::Matrix controls = SimTK::Test::randMatrix(5, 1);
            MocoIterate it(time, snames, cnames, {}, {}, states, controls,
                    SimTK::Matrix(), SimTK::RowVector());

            SimTK_TEST_EQ(it.getInitialTime(), -3.1);
            SimTK_TEST_EQ(it.getFinalTime(), 8.9);
        }

        {
            SimTK::Vector time(1, 7.2);
            std::vector<std::string> snames{"s0", "s1"};
            std::vector<std::string> cnames{"c0"};
            SimTK::Matrix states = SimTK::Test::randMatrix(1, 2);
            SimTK::Matrix controls = SimTK::Test::randMatrix(1, 1);
            MocoIterate it(time, snames, cnames, {}, {}, states, controls,
                    SimTK::Matrix(), SimTK::RowVector());

            SimTK_TEST_EQ(it.getInitialTime(), 7.2);
            SimTK_TEST_EQ(it.getFinalTime(), 7.2);
        }
    }

    // compareContinuousVariablesRMS
    auto testCompareContinuousVariablesRMS =
            [](int NT, int NS, int NC, int NM, double duration, double error,
                    std::vector<std::string> statesToCompare = {},
                    std::vector<std::string> controlsToCompare = {},
                    std::vector<std::string> multipliersToCompare = {}) {
                const double t0 = 0.2;
                std::vector<std::string> snames;
                for (int i = 0; i < NS; ++i)
                    snames.push_back("s" + std::to_string(i));
                std::vector<std::string> cnames;
                for (int i = 0; i < NC; ++i)
                    cnames.push_back("c" + std::to_string(i));
                std::vector<std::string> mnames;
                for (int i = 0; i < NM; ++i)
                    mnames.push_back("m" + std::to_string(i));
                SimTK::Matrix states(NT, NS);
                for (int i = 0; i < NS; ++i) {
                    states.updCol(i) =
                            createVectorLinspace(NT, SimTK::Test::randDouble(),
                                    SimTK::Test::randDouble());
                }
                SimTK::Matrix controls(NT, NC);
                for (int i = 0; i < NC; ++i) {
                    controls.updCol(i) =
                            createVectorLinspace(NT, SimTK::Test::randDouble(),
                                    SimTK::Test::randDouble());
                }
                SimTK::Matrix multipliers(NT, NM);
                for (int i = 0; i < NM; ++i) {
                    multipliers.updCol(i) =
                            createVectorLinspace(NT, SimTK::Test::randDouble(),
                                    SimTK::Test::randDouble());
                }
                SimTK::Vector time =
                        createVectorLinspace(NT, t0, t0 + duration);
                MocoIterate a(time, snames, cnames, mnames, {}, states,
                        controls, multipliers, SimTK::RowVector());
                MocoIterate b(time, snames, cnames, mnames, {},
                        states.elementwiseAddScalar(error),
                        controls.elementwiseAddScalar(error),
                        multipliers.elementwiseAddScalar(error),
                        SimTK::RowVector());
                // If error is constant:
                // sqrt(1/(T*N) * integral_t (sum_i^N (err_{i,t}^2))) = err
                auto rmsBA = b.compareContinuousVariablesRMS(a,
                        {{"states", statesToCompare},
                         {"controls", controlsToCompare},
                         {"multipliers", multipliersToCompare}});
                int N = 0;
                if (statesToCompare.empty())
                    N += NS;
                else if (statesToCompare[0] == "none")
                    N += 0;
                else
                    N += (int)statesToCompare.size();
                if (controlsToCompare.empty())
                    N += NC;
                else if (controlsToCompare[0] == "none")
                    N += 0;
                else
                    N += (int)controlsToCompare.size();
                if (multipliersToCompare.empty())
                    N += NM;
                else if (multipliersToCompare[0] == "none")
                    N += 0;
                else
                    N += (int)multipliersToCompare.size();
                auto rmsExpected = N == 0 ? 0 : error;
                SimTK_TEST_EQ(rmsBA, rmsExpected);
                auto rmsAB = a.compareContinuousVariablesRMS(b,
                        {{"states", statesToCompare},
                         {"controls", controlsToCompare},
                         {"multipliers", multipliersToCompare}});
                SimTK_TEST_EQ(rmsAB, rmsExpected);
            };

    testCompareContinuousVariablesRMS(10, 2, 1, 1, 0.6, 0.05);
    testCompareContinuousVariablesRMS(21, 2, 0, 2, 15.0, 0.01);
    // 6 is the minimum required number of times; ensure that it works.
    testCompareContinuousVariablesRMS(6, 0, 3, 0, 0.1, 0.9);

    // Providing a subset of states/columns to compare.
    testCompareContinuousVariablesRMS(10, 2, 3, 1, 0.6, 0.05, {"s1"});
    testCompareContinuousVariablesRMS(10, 2, 3, 1, 0.6, 0.05, {}, {"c1"});
    testCompareContinuousVariablesRMS(
            10, 2, 3, 1, 0.6, 0.05, {"none"}, {"none"}, {"none"});
    // Can't provide "none" along with other state names.
    SimTK_TEST_MUST_THROW_EXC(testCompareContinuousVariablesRMS(
                                      10, 2, 3, 1, 0.6, 0.05, {"none", "s1"}),
            Exception);
    SimTK_TEST_MUST_THROW_EXC(testCompareContinuousVariablesRMS(
                                      10, 2, 3, 1, 0.6, 0.05, {}, {"none, c0"}),
            Exception);

    // compareParametersRMS
    auto testCompareParametersRMS = [](int NP, double error,
                                            std::vector<std::string>
                                                    parametersToCompare = {}) {
        std::vector<std::string> pnames;
        for (int i = 0; i < NP; ++i) pnames.push_back("p" + std::to_string(i));
        SimTK::RowVector parameters = SimTK::Test::randVector(NP).transpose();
        MocoIterate a(SimTK::Vector(), {}, {}, {}, pnames, SimTK::Matrix(),
                SimTK::Matrix(), SimTK::Matrix(), parameters);
        MocoIterate b(SimTK::Vector(), {}, {}, {}, pnames, SimTK::Matrix(),
                SimTK::Matrix(), SimTK::Matrix(),
                parameters.elementwiseAddScalar(error).getAsRowVector());
        // If error is constant:
        // sqrt(sum_i^N (err_{i}^2) / N) = err
        auto rmsBA = b.compareParametersRMS(a, parametersToCompare);
        auto rmsExpected = error;
        SimTK_TEST_EQ(rmsBA, rmsExpected);
        auto rmsAB = a.compareParametersRMS(b, parametersToCompare);
        SimTK_TEST_EQ(rmsAB, rmsExpected);
    };
    // Compare one parameter.
    testCompareParametersRMS(1, 0.01);
    // Compare subsets of available parameters.
    testCompareParametersRMS(5, 0.5);
    testCompareParametersRMS(5, 0.5, {"p0"});
    testCompareParametersRMS(5, 0.5, {"p1", "p2"});
    // Compare a lot of parameters.
    testCompareParametersRMS(100, 0.5);
}

TEST_CASE("Interpolate", "") {
    SimTK::Vector x(2);
    x[0] = 0;
    x[1] = 1;

    SimTK::Vector y(2);
    y[0] = 1;
    y[1] = 0;

    SimTK::Vector newX(4);
    newX[0] = -1;
    newX[1] = 0.25;
    newX[2] = 0.75;
    newX[3] = 1.5;

    SimTK::Vector newY = interpolate(x, y, newX);

    SimTK_TEST(SimTK::isNaN(newY[0]));
    SimTK_TEST_EQ(newY[1], 0.75);
    SimTK_TEST_EQ(newY[2], 0.25);
    SimTK_TEST(SimTK::isNaN(newY[3]));
}

TEMPLATE_TEST_CASE("Sliding mass", "", MocoTropterSolver, MocoCasADiSolver) {

    MocoTool moco = createSlidingMassMocoTool<TestType>();
    MocoSolution solution = moco.solve();
    int numTimes = 20;
    int numStates = 2;
    int numControls = 1;

    // Check dimensions and metadata of the solution.
    SimTK_TEST((solution.getStateNames() ==
                std::vector<std::string>{
                        "/slider/position/value", "/slider/position/speed"}));
    SimTK_TEST((solution.getControlNames() ==
                std::vector<std::string>{"/actuator"}));
    SimTK_TEST(solution.getTime().size() == numTimes);
    const auto& states = solution.getStatesTrajectory();
    SimTK_TEST(states.nrow() == numTimes);
    SimTK_TEST(states.ncol() == numStates);
    const auto& controls = solution.getControlsTrajectory();
    SimTK_TEST(controls.nrow() == numTimes);
    SimTK_TEST(controls.ncol() == numControls);

    // Check the actual solution.
    const double expectedFinalTime = 2.0;
    SimTK_TEST_EQ_TOL(
            solution.getTime().get(numTimes - 1), expectedFinalTime, 1e-2);
    const double half = 0.5 * expectedFinalTime;

    for (int itime = 0; itime < numTimes; ++itime) {
        const double& t = solution.getTime().get(itime);
        // Position is a quadratic.
        double expectedPos =
                t < half ? 0.5 * pow(t, 2)
                         : -0.5 * pow(t - half, 2) + 1.0 * (t - half) + 0.5;
        SimTK_TEST_EQ_TOL(states(itime, 0), expectedPos, 1e-2);

        double expectedSpeed = t < half ? t : 2.0 - t;
        SimTK_TEST_EQ_TOL(states(itime, 1), expectedSpeed, 1e-2);

        double expectedForce = t < half ? 10 : -10;
        SimTK_TEST_EQ_TOL(controls(itime, 0), expectedForce, 1e-2);
    }
}

TEMPLATE_TEST_CASE("Solving an empty MocoProblem", "", MocoTropterSolver,
        MocoCasADiSolver) {
    MocoTool moco;
    auto& solver = moco.initSolver<TestType>();
    THEN("problem solves without error, solution trajectories are empty.") {
        MocoSolution solution = moco.solve();
        // 100 is the default num_mesh_points.
        CHECK(solution.getTime().size() == solver.get_num_mesh_points());
        CHECK(solution.getStatesTrajectory().ncol() == 0);
        CHECK(solution.getStatesTrajectory().nrow() == 0);
        CHECK(solution.getControlsTrajectory().ncol() == 0);
        CHECK(solution.getControlsTrajectory().nrow() == 0);
        CHECK(solution.getMultipliersTrajectory().ncol() == 0);
        CHECK(solution.getMultipliersTrajectory().nrow() == 0);
        CHECK(solution.getDerivativesTrajectory().ncol() == 0);
        CHECK(solution.getDerivativesTrajectory().nrow() == 0);
    }
}

/// Ensure that using a joint that has an empty quaternion slot does not
/// cause us to misalign states between OpenSim and Tropter/CasADi.
/// Even when not using quaternions, Simbody has a slot in the state vector
/// for the 4th quaternion coordinate.
template <typename SolverType>
void testSkippingOverQuaternionSlots(bool constrained,
        bool constraintDerivs,
        std::string dynamicsMode) {
    Model model;
    using SimTK::Vec3;
    auto* b1 = new Body("b1", 1, Vec3(0), SimTK::Inertia(1));
    model.addBody(b1);
    auto* j1 = new BallJoint("j1", model.getGround(), Vec3(0, -1, 0),
            Vec3(0), *b1, Vec3(0), Vec3(0));
    j1->updCoordinate(BallJoint::Coord::Rotation1X).setRangeMin(-0.2);
    j1->updCoordinate(BallJoint::Coord::Rotation1X).setRangeMin(+0.2);
    j1->updCoordinate(BallJoint::Coord::Rotation2Y).setRangeMin(-0.2);
    j1->updCoordinate(BallJoint::Coord::Rotation2Y).setRangeMin(+0.2);
    j1->updCoordinate(BallJoint::Coord::Rotation3Z).setRangeMin(-0.2);
    j1->updCoordinate(BallJoint::Coord::Rotation3Z).setRangeMin(+0.2);
    model.addJoint(j1);
    if (constrained) {
        auto* constraint = new CoordinateCouplerConstraint();
        Array<std::string> names;
        names.append("j1_coord_0");
        constraint->setIndependentCoordinateNames(names);
        constraint->setDependentCoordinateName("j1_coord_2");
        LinearFunction func(1.0, 0.0);
        constraint->setFunction(func);
        model.addConstraint(constraint);
    }

    auto* b2 = new Body("b2", 1, Vec3(0), SimTK::Inertia(2));
    model.addBody(b2);
    auto* j2 = new PinJoint(
            "j2", *b1, Vec3(0, -1, 0), Vec3(0), *b2, Vec3(0), Vec3(0));
    model.addJoint(j2);
    model.finalizeConnections();
    for (int i = 0; i < model.getCoordinateSet().getSize(); ++i) {
        const auto& coord = model.getCoordinateSet().get(i);
        auto* actu = new CoordinateActuator(coord.getName());
        actu->setName(coord.getName() + "_actuator");
        model.addForce(actu);
    }

    MocoTool moco;
    auto& problem = moco.updProblem();
    problem.setModelCopy(model);
    const double duration = 1.0;
    problem.setTimeBounds(0, duration);
    problem.setStateInfo(
            "/jointset/j1/j1_coord_1/value", {-0.2, 0.2}, -0.2, 0.2);
    const double speed = 0.36;
    problem.setStateInfo("/jointset/j2/j2_coord_0/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j2/j2_coord_0/speed", speed);

    problem.addCost<MocoControlCost>();

    auto& solver = moco.initSolver<SolverType>();
    const int N = 5;
    solver.set_num_mesh_points(N);
    solver.set_dynamics_mode(dynamicsMode);
    solver.set_transcription_scheme("hermite-simpson");
    solver.set_optim_constraint_tolerance(1e-2);
    solver.set_optim_convergence_tolerance(1e-2);
    if (constrained) {
        solver.set_enforce_constraint_derivatives(constraintDerivs);
    }

    MocoSolution solution = moco.solve();

    const auto& valueTraj = solution.getState("/jointset/j2/j2_coord_0/value");
    const double lastValue = valueTraj[valueTraj.size() - 1];
    CHECK(lastValue == Approx(speed * duration));
    for (int i = 0; i < N; ++i) {
        CHECK(solution.getState("/jointset/j2/j2_coord_0/speed").getElt(i, 0)
                == Approx(speed));
    }
}

TEST_CASE("Skip over empty quaternion slots", "") {
    std::cout.rdbuf(LogManager::cout.rdbuf());
    std::cout.rdbuf(LogManager::cout.rdbuf());

    testSkippingOverQuaternionSlots<MocoTropterSolver>(false, false, "explicit");
    testSkippingOverQuaternionSlots<MocoTropterSolver>(true, false, "explicit");
    testSkippingOverQuaternionSlots<MocoTropterSolver>(true, true, "explicit");
    testSkippingOverQuaternionSlots<MocoTropterSolver>(false, false, "implicit");

    testSkippingOverQuaternionSlots<MocoCasADiSolver>(false, false, "explicit");
    testSkippingOverQuaternionSlots<MocoCasADiSolver>(true, false, "explicit");
    testSkippingOverQuaternionSlots<MocoCasADiSolver>(true, true, "explicit");
    testSkippingOverQuaternionSlots<MocoCasADiSolver>(false, false, "implicit");
    testSkippingOverQuaternionSlots<MocoCasADiSolver>(true, false, "implicit");
    testSkippingOverQuaternionSlots<MocoCasADiSolver>(true, true, "implicit");
}

// testCopy();
// testSolveRepeatedly();
// testOMUCOSerialization();

// TODO specifying optimizer options.
