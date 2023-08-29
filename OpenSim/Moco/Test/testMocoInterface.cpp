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
#include <fstream>

#include <OpenSim/Actuators/BodyActuator.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/ScapulothoracicJoint.h>

using namespace OpenSim;

// TODO
// - add documentation. pre/post conditions.
// - write test cases for exceptions, for calling methods out of order.
// - model_file vs model.
// - test problems without controls (including with setting guesses).

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
MocoStudy createSlidingMassMocoStudy() {
    MocoStudy study;
    study.setName("sliding_mass");
    study.set_write_solution("false");
    MocoProblem& mp = study.updProblem();
    mp.setModel(createSlidingMassModel());
    mp.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(0, 10));
    mp.setStateInfo("/slider/position/value", MocoBounds(0, 1),
            MocoInitialBounds(0), MocoFinalBounds(1));
    mp.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
    mp.addGoal<MocoFinalTimeGoal>();

    auto& ms = study.initSolver<SolverType>();
    ms.set_num_mesh_intervals(19);
    ms.set_transcription_scheme("trapezoidal");
    ms.set_enforce_constraint_derivatives(false);
    return study;
}

TEMPLATE_TEST_CASE("Non-uniform mesh", "", MocoCasADiSolver, MocoTropterSolver) {
    auto transcriptionScheme =
            GENERATE(as<std::string>{}, "trapezoidal", "hermite-simpson");
    MocoStudy study;
    double finalTime = 5.0;
    study.setName("sliding_mass");
    study.set_write_solution("false");
    MocoProblem& mp = study.updProblem();
    mp.setModel(createSlidingMassModel());
    mp.setTimeBounds(0, finalTime);
    mp.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
    mp.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
    mp.addGoal<MocoControlGoal>();
    SECTION("Ensure integral handles non-uniform mesh") {
        auto& ms = study.initSolver<TestType>();
        ms.set_transcription_scheme(transcriptionScheme);
        ms.set_optim_max_iterations(1);
        std::vector<double> mesh = {
                0.0, .05, .075, .1, .4, .41, .42, .58, .8, 1};
        ms.setMesh(mesh);
        auto solution = study.solve().unseal();
        auto u = solution.getControl("/actuator");

        using SimTK::square;
        if (transcriptionScheme == "trapezoidal") {
            for (int i = 0; i < (int)mesh.size(); ++i) {
                CHECK(solution.getTime()[i] == Approx(mesh[i] * finalTime));
            }

            double manualIntegral = 0;
            for (int i = 0; i < (int)(mesh.size() - 1); ++i) {
                manualIntegral +=
                        0.5 * (mesh[i + 1] - mesh[i]) *
                        (square(u.getElt(i, 0)) + square(u.getElt(i + 1, 0)));
            }
            manualIntegral *= finalTime;

            CHECK(manualIntegral == Approx(solution.getObjective()));
        } else if (transcriptionScheme == "hermite-simpson") {
            for (int i = 0; i < (int)mesh.size() - 1; ++i) {
                CHECK(solution.getTime()[2 * i] == Approx(mesh[i] * finalTime));
                // Midpoints.
                CHECK(solution.getTime()[2 * i + 1] ==
                        Approx(0.5 * (mesh[i] + mesh[i + 1]) * finalTime));
            }

            // Simpson quadrature.
            double manualIntegral = 0;
            for (int i = 0; i < (int)mesh.size() - 1; ++i) {
                const auto a = mesh[i];
                const auto b = mesh[i + 1];
                const auto fa = square(u.getElt(2 * i, 0));
                const auto fmid = square(u.getElt(2 * i + 1, 0));
                const auto fb = square(u.getElt(2 * i + 2, 0));
                manualIntegral += (b - a) / 6.0 * (fa + 4 * fmid + fb);
            }
            manualIntegral *= finalTime;
            CHECK(manualIntegral == Approx(solution.getObjective()));
        }
    }

    SECTION("First mesh point must be zero.") {
        auto& ms = study.initSolver<TestType>();
        ms.set_transcription_scheme(transcriptionScheme);
        std::vector<double> mesh = {.5, 1};
        ms.setMesh(mesh);
        REQUIRE_THROWS_WITH(study.solve(),
                Catch::Contains("Invalid custom mesh; first mesh "
                                              "point must be zero."));
    }
    SECTION("Mesh points must be strictly increasing") {
        auto& ms = study.initSolver<TestType>();
        ms.set_transcription_scheme(transcriptionScheme);
        std::vector<double> mesh = {0, .5, .5, 1};
        ms.setMesh(mesh);
        REQUIRE_THROWS_WITH(study.solve(),
                Catch::Contains("Invalid custom mesh; mesh "
                                "points must be strictly increasing."));
    }
    SECTION("Last mesh point must be 1.") {
        auto& ms = study.initSolver<TestType>();
        ms.set_transcription_scheme(transcriptionScheme);
        std::vector<double> mesh = {0, .4, .8};
        ms.setMesh(mesh);
        REQUIRE_THROWS_WITH(
                study.solve(), Catch::Contains("Invalid custom mesh; last mesh "
                                              "point must be one."));
    }
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

TEMPLATE_TEST_CASE("Solver options", "", MocoCasADiSolver, MocoTropterSolver) {
    MocoStudy study = createSlidingMassMocoStudy<TestType>();
    auto& ms = study.initSolver<TestType>();
    MocoSolution solDefault = study.solve();
    ms.set_verbosity(3); // Invalid value.
    SimTK_TEST_MUST_THROW_EXC(study.solve(), Exception);
    ms.set_verbosity(2);

    ms.set_optim_solver("nonexistent");
    SimTK_TEST_MUST_THROW_EXC(study.solve(), Exception);
    ms.set_optim_solver("ipopt");

    ms.set_optim_hessian_approximation("nonexistent");
    SimTK_TEST_MUST_THROW(study.solve());
    ms.set_optim_hessian_approximation("limited-memory");

    {
        ms.set_optim_max_iterations(1);
        MocoSolution solution = study.solve();
        SimTK_TEST(solution.isSealed());
        solution.unseal();
        SimTK_TEST(solution.getNumIterations() == 1);
        ms.set_optim_max_iterations(-1);
    }

    {
        ms.set_optim_convergence_tolerance(1e-2);
        MocoSolution solLooseConvergence = study.solve();
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
        MocoSolution solutionTight = study.solve();
        ms.set_optim_constraint_tolerance(1e-2);
        MocoSolution solutionLoose = study.solve();
        SimTK_TEST(solutionTight.getNumIterations() >
                   solutionLoose.getNumIterations());
        ms.set_optim_constraint_tolerance(-1);
        ms.set_optim_convergence_tolerance(-1);
    }
}

TEMPLATE_TEST_CASE("Ordering of calls", "", MocoCasADiSolver,
        MocoTropterSolver) {

    // Solve a problem, edit the problem, re-solve.
    {
        MocoStudy study = createSlidingMassMocoStudy();
        auto& solver = study.initSolver<TestType>();
        study.solve();
        // This resets the problem to null on the solver.
        study.updProblem();
        // This will call initSolver() internally:
        study.solve();
    }

    // Solve a problem, edit the problem, ask the solver to do something.
    {
        MocoStudy study = createSlidingMassMocoStudy();
        auto& solver = study.initSolver<TestType>();
        study.solve();
        // This resets the problem to null on the solver.
        study.updProblem();
        // The solver can't do anything if you've edited the model.
        SimTK_TEST_MUST_THROW_EXC(solver.createGuessTimeStepping(), Exception);
    }

    // Solve a problem, edit the solver, re-solve.
    {
        MocoStudy study = createSlidingMassMocoStudy();
        auto& solver = study.initSolver<TestType>();
        const int initNumMeshPoints = solver.get_num_mesh_intervals();
        MocoSolution sol0 = study.solve();
        solver.set_num_mesh_intervals(2 * initNumMeshPoints);
        MocoSolution sol1 = study.solve();
        solver.set_num_mesh_intervals(initNumMeshPoints);
        MocoSolution sol2 = study.solve();
        // Ensure that changing the mesh has an effect.
        SimTK_TEST(!sol0.isNumericallyEqual(sol1));
        // Ensure we get repeatable results with the initial settings.
        SimTK_TEST(sol0.isNumericallyEqual(sol2));

    }
}

/// Test that we can read in a Moco setup file, solve, edit the setup,
/// re-solve.
// TODO tropter solutions are very slightly different between successive solves.
TEMPLATE_TEST_CASE("Serializing a MocoStudy", "", MocoCasADiSolver) {
    std::string fname = "testMocoInterface_testOMOCOSerialization.omoco";
    
    MocoStudy study = createSlidingMassMocoStudy<TestType>();
    MocoSolution sol0 = study.solve();
    study.print(fname);
    
    MocoStudy mocoDeserialized(fname);
    MocoSolution sol1 = mocoDeserialized.solve();

    SimTK_TEST(sol0.isNumericallyEqual(sol1));
}

// TODO does not pass consistently on Mac
//TEST_CASE("Copying a MocoStudy", "") {
//    MocoStudy study = createSlidingMassMocoStudy();
//    MocoSolution solution = study.solve();
//    std::unique_ptr<MocoStudy> copy(study.clone());
//    MocoSolution solutionFromCopy = copy->solve();
//    SimTK_TEST(solution.isNumericallyEqual(solutionFromCopy));
//}

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

    // TODO what to do if the user does not specify info for some variables?

    // Get error if state/control name does not exist.
    {
        auto model = createSlidingMassModel();
        model->initSystem();
        {
            MocoStudy study;
            MocoProblem& mp = study.updProblem();
            mp.setModel(std::unique_ptr<Model>(model->clone()));
            mp.setStateInfo("nonexistent", {0, 1});
            SimTK_TEST_MUST_THROW_EXC(mp.createRep(), Exception);
        }
        {
            MocoStudy study;
            MocoProblem& mp = study.updProblem();
            mp.setModel(std::unique_ptr<Model>(model->clone()));
            mp.setControlInfo("nonexistent", {0, 1});
            SimTK_TEST_MUST_THROW_EXC(mp.createRep(), Exception);
        }
    }
    // TODO what if bounds are missing for some states?
}

TEST_CASE("Building a problem", "") {
    {
        MocoStudy study;
        MocoProblem& mp = study.updProblem();
        mp.setModel(createSlidingMassModel());

        // Goals have the name "goal" by default.
        {
            auto c0 = make_unique<MocoFinalTimeGoal>();
            SimTK_TEST(c0->getName() == "goal");
            mp.addGoal(std::move(c0));
        }
        // Names of costs must be unique.
        {
            auto* c1 = mp.addGoal<MocoFinalTimeGoal>();
            SimTK_TEST_MUST_THROW_EXC(mp.createRep(), Exception);
            c1->setName("c1");
        }
        // Goals must have a name.
        {
            auto* cEmptyName = mp.addGoal<MocoFinalTimeGoal>("");
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

TEMPLATE_TEST_CASE("Workflow", "", MocoCasADiSolver, MocoTropterSolver) {

    // Default bounds.
    SECTION("Default bounds") {
        MocoStudy study;
        MocoProblem& problem = study.updProblem();
        auto model = createSlidingMassModel();
        model->finalizeFromProperties();
        auto* bodyAct = new BodyActuator();
        bodyAct->setName("residuals");
        bodyAct->setBody(model->getComponent<Body>("body"));
        model->addComponent(bodyAct);
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
        SECTION("User did not specify state info explicitly.") {
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
        SECTION("Setting control info explicitly") {
            {
                const auto& probinfo = phase0.getControlInfo("/actuator");
                SimTK_TEST_EQ(probinfo.getBounds().getLower(), 12);
                SimTK_TEST_EQ(probinfo.getBounds().getUpper(), 15);
            }
            {
                MocoProblemRep rep = problem.createRep();
                const auto& info = rep.getControlInfo("/actuator");
                SimTK_TEST_EQ(info.getBounds().getLower(), 12);
                SimTK_TEST_EQ(info.getBounds().getUpper(), 15);
            }
        }

        problem.setControlInfo("/residuals_0", {-5, 5});
        problem.setControlInfo("/residuals_3", {-7.5, 10});
        {
            {
                const auto& probinfo0 = phase0.getControlInfo("/residuals_0");
                SimTK_TEST_EQ(probinfo0.getBounds().getLower(), -5);
                SimTK_TEST_EQ(probinfo0.getBounds().getUpper(), 5);
                const auto& probinfo3 = phase0.getControlInfo("/residuals_3");
                SimTK_TEST_EQ(probinfo3.getBounds().getLower(), -7.5);
                SimTK_TEST_EQ(probinfo3.getBounds().getUpper(), 10);
            }
            MocoProblemRep rep = problem.createRep();
            {
                const auto& info0 = rep.getControlInfo("/residuals_0");
                SimTK_TEST_EQ(info0.getBounds().getLower(), -5);
                SimTK_TEST_EQ(info0.getBounds().getUpper(), 5);
                const auto& info3 = rep.getControlInfo("/residuals_3");
                SimTK_TEST_EQ(info3.getBounds().getLower(), -7.5);
                SimTK_TEST_EQ(info3.getBounds().getUpper(), 10);
            }
        }

        SECTION("Setting only initial/final bounds explicitly.") {
            SECTION("Initial coordinate value") {
                problem.setStateInfo("/slider/position/value", {}, {-5.0, 3.6});
                {
                    const auto& info =
                            phase0.getStateInfo("/slider/position/value");
                    SimTK_TEST(!info.getBounds().isSet());
                    SimTK_TEST_EQ(info.getInitialBounds().getLower(), -5.0);
                    SimTK_TEST_EQ(info.getInitialBounds().getUpper(), 3.6);
                    SimTK_TEST(!info.getFinalBounds().isSet());
                }
                MocoProblemRep rep = problem.createRep();
                const auto& info = rep.getStateInfo("/slider/position/value");
                SimTK_TEST_EQ(info.getBounds().getLower(), -10);
                SimTK_TEST_EQ(info.getBounds().getUpper(), 15);
                SimTK_TEST_EQ(info.getInitialBounds().getLower(), -5.0);
                SimTK_TEST_EQ(info.getInitialBounds().getUpper(), 3.6);
                SimTK_TEST(!info.getFinalBounds().isSet());
            }
            SECTION("Final coordinate value") {
                problem.setStateInfo(
                        "/slider/position/value", {}, {}, {1.3, 2.5});
                {
                    const auto& info =
                            phase0.getStateInfo("/slider/position/value");
                    SimTK_TEST(!info.getBounds().isSet());
                    SimTK_TEST(!info.getInitialBounds().isSet());
                    SimTK_TEST_EQ(info.getFinalBounds().getLower(), 1.3);
                    SimTK_TEST_EQ(info.getFinalBounds().getUpper(), 2.5);
                }
                MocoProblemRep rep = problem.createRep();
                const auto& info = rep.getStateInfo("/slider/position/value");
                SimTK_TEST_EQ(info.getBounds().getLower(), -10);
                SimTK_TEST_EQ(info.getBounds().getUpper(), 15);
                SimTK_TEST(!info.getInitialBounds().isSet());
                SimTK_TEST_EQ(info.getFinalBounds().getLower(), 1.3);
                SimTK_TEST_EQ(info.getFinalBounds().getUpper(), 2.5);
            }
            SECTION("Initial coordinate speed") {
                problem.setStateInfo("/slider/position/speed", {}, {-4.1, 3.9});
                {
                    const auto& info =
                            phase0.getStateInfo("/slider/position/speed");
                    SimTK_TEST(!info.getBounds().isSet());
                    SimTK_TEST_EQ(info.getInitialBounds().getLower(), -4.1);
                    SimTK_TEST_EQ(info.getInitialBounds().getUpper(), 3.9);
                    SimTK_TEST(!info.getFinalBounds().isSet());
                }
                MocoProblemRep rep = problem.createRep();
                const auto& info = rep.getStateInfo("/slider/position/speed");
                SimTK_TEST_EQ(info.getBounds().getLower(), -50);
                SimTK_TEST_EQ(info.getBounds().getUpper(), 50);
                SimTK_TEST_EQ(info.getInitialBounds().getLower(), -4.1);
                SimTK_TEST_EQ(info.getInitialBounds().getUpper(), 3.9);
                SimTK_TEST(!info.getFinalBounds().isSet());
            }
            SECTION("Final coordinate speed") {
                problem.setStateInfo(
                        "/slider/position/speed", {}, {}, {0.1, 0.8});
                {
                    const auto& info =
                            phase0.getStateInfo("/slider/position/speed");
                    SimTK_TEST(!info.getBounds().isSet());
                    SimTK_TEST(!info.getInitialBounds().isSet());
                    SimTK_TEST_EQ(info.getFinalBounds().getLower(), 0.1);
                    SimTK_TEST_EQ(info.getFinalBounds().getUpper(), 0.8);
                }
                MocoProblemRep rep = problem.createRep();
                const auto& info = rep.getStateInfo("/slider/position/speed");
                SimTK_TEST_EQ(info.getBounds().getLower(), -50);
                SimTK_TEST_EQ(info.getBounds().getUpper(), 50);
                SimTK_TEST(!info.getInitialBounds().isSet());
                SimTK_TEST_EQ(info.getFinalBounds().getLower(), 0.1);
                SimTK_TEST_EQ(info.getFinalBounds().getUpper(), 0.8);
            }
            SECTION("Initial control") {
                problem.setControlInfo("/actuator", {}, {-4.1, 3.9});
                {
                    const auto& info = phase0.getControlInfo("/actuator");
                    SimTK_TEST(!info.getBounds().isSet());
                    SimTK_TEST_EQ(info.getInitialBounds().getLower(), -4.1);
                    SimTK_TEST_EQ(info.getInitialBounds().getUpper(), 3.9);
                    SimTK_TEST(!info.getFinalBounds().isSet());
                }
                MocoProblemRep rep = problem.createRep();
                const auto& info = rep.getControlInfo("/actuator");
                SimTK_TEST_EQ(info.getBounds().getLower(), 35);
                SimTK_TEST_EQ(info.getBounds().getUpper(), 56);
                SimTK_TEST_EQ(info.getInitialBounds().getLower(), -4.1);
                SimTK_TEST_EQ(info.getInitialBounds().getUpper(), 3.9);
                SimTK_TEST(!info.getFinalBounds().isSet());
            }
            SECTION("Final control") {
                problem.setControlInfo("/actuator", {}, {}, {0.1, 0.8});
                {
                    const auto& info = phase0.getControlInfo("/actuator");
                    SimTK_TEST(!info.getBounds().isSet());
                    SimTK_TEST(!info.getInitialBounds().isSet());
                    SimTK_TEST_EQ(info.getFinalBounds().getLower(), 0.1);
                    SimTK_TEST_EQ(info.getFinalBounds().getUpper(), 0.8);
                }
                MocoProblemRep rep = problem.createRep();
                const auto& info = rep.getControlInfo("/actuator");
                SimTK_TEST_EQ(info.getBounds().getLower(), 35);
                SimTK_TEST_EQ(info.getBounds().getUpper(), 56);
                SimTK_TEST(!info.getInitialBounds().isSet());
                SimTK_TEST_EQ(info.getFinalBounds().getLower(), 0.1);
                SimTK_TEST_EQ(info.getFinalBounds().getUpper(), 0.8);
            }
        }
    }

    SECTION("Changes to time bounds are obeyed") {
        MocoStudy study;
        MocoProblem& problem = study.updProblem();
        problem.setModel(createSlidingMassModel());

        problem.setTimeBounds(0, {0, 10});
        problem.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        problem.setControlInfo("/actuator", {-10, 10});
        problem.addGoal<MocoFinalTimeGoal>();

        auto& solver = study.initSolver<TestType>();
        const int N = 19;         // mesh intervals
        const int Nc = 2 * N + 1; // collocation points
        solver.set_num_mesh_intervals(N);
        MocoTrajectory guess = solver.createGuess("random");
        guess.setTime(createVectorLinspace(Nc, 0.0, 3.0));
        solver.setGuess(guess);
        MocoSolution solution0 = study.solve();

        problem.setTimeBounds(0, {5.8, 10});
        // Editing the problem does not affect information in the Solver;
        // the guess still exists.
        SimTK_TEST(!solver.getGuess().empty());

        guess.setTime(createVectorLinspace(Nc, 0.0, 7.0));
        MocoSolution solution = study.solve();
        CAPTURE(solution.getObjective());
        CHECK(solution.getFinalTime() == Approx(5.8));
    }

    SECTION("Changes to model are obeyed; set costs and model in any order.") {
        double finalTime0;
        {
            // Ensure that changes to the model are obeyed.
            MocoStudy study;
            MocoProblem& problem = study.updProblem();
            auto model = problem.setModel(createSlidingMassModel());
            problem.setTimeBounds(0, {0, 10});
            problem.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
            problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
            problem.addGoal<MocoFinalTimeGoal>();
            auto& solver = study.initSolver<TestType>();
            solver.set_num_mesh_intervals(20);
            finalTime0 = study.solve().getFinalTime();

            auto& body = model->updComponent<Body>("body");
            body.setMass(2 * body.getMass());
            const double finalTime1 = study.solve().getFinalTime();
            SimTK_TEST(finalTime1 > 1.1 * finalTime0);
        }

        // Can set the cost and model in any order.
        {
            MocoStudy study;
            MocoProblem& problem = study.updProblem();
            problem.setTimeBounds(0, {0, 10});
            problem.addGoal<MocoFinalTimeGoal>();
            problem.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
            problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
            problem.setModel(createSlidingMassModel());
            auto& solver = study.initSolver<TestType>();
            solver.set_num_mesh_intervals(20);
            const double finalTime = study.solve().getFinalTime();
            SimTK_TEST_EQ_TOL(finalTime, finalTime0, 1e-6);
        }
    }

    SECTION("Changes to costs are obeyed") {
        MocoStudy study;
        MocoProblem& problem = study.updProblem();
        problem.setModel(createSlidingMassModel());
        problem.setTimeBounds(0, {0, 10});
        problem.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        problem.updPhase().addGoal<MocoFinalTimeGoal>();
        auto effort = problem.updPhase().addGoal<MocoControlGoal>("effort");
        study.initSolver<TestType>();
        const double finalTime0 = study.solve().getFinalTime();

        // Change the weights of the costs.
        effort->setWeight(0.1);
        const double finalTime1 = study.solve().getFinalTime();
        SimTK_TEST(finalTime1 < 0.8 * finalTime0);
    }

    // Invoking functions without initializing.
    // TODO

    // TODO MocoGoal and MocoParameter cache pointers into some model.
    // TODO {
    // TODO     MocoFinalTimeGoal cost;
    // TODO     // TODO must be initialized first.
    // TODO     // TODO MocoPhase shouldn't even have a public
    // calcGoal function.
    // TODO     SimTK_TEST_MUST_THROW_EXC(cost.calcGoal(state),
    // Exception);
    // TODO }

    // Allow removing costs.
    // TODO
    // {
    //     MocoStudy study;
    //     MocoProblem& problem = study.updProblem();
    //     {
    //         // Remove by name.
    //         auto& cost = problem.addGoal<MocoFinalTimeGoal>();
    //         cost.setName("cost0");
    //         problem.removeCost(cost);
    //         SimTK_TEST_MUST_THROW_EXC(problem.getCost("cost0"),
    //         Exception);
    //     }
    // }
}
TEMPLATE_TEST_CASE("Set infos with regular expression", "",
        MocoCasADiSolver, MocoTropterSolver) {
    MocoStudy study;
    MocoProblem& problem = study.updProblem();
    problem.setModelAsCopy(ModelFactory::createDoublePendulum());
    problem.setTimeBounds(0, 10);
    problem.setStateInfoPattern(".*/value", {2, 10});
    problem.setStateInfoPattern(".*/speed", {3, 10});
    MocoProblemRep problemRep = problem.createRep();
    SimTK_TEST_EQ(problemRep.getStateInfo("/jointset/j0/q0/value")
                          .getBounds()
                          .getLower(),
            2);
    SimTK_TEST_EQ(problemRep.getStateInfo("/jointset/j1/q1/value")
                          .getBounds()
                          .getLower(),
            2);
    SimTK_TEST_EQ(problemRep.getStateInfo("/jointset/j0/q0/speed")
                          .getBounds()
                          .getLower(),
            3);
    SimTK_TEST_EQ(problemRep.getStateInfo("/jointset/j1/q1/speed")
                          .getBounds()
                          .getLower(),
            3);
    problem.setStateInfo("/jointset/j0/q0/value", {3, 10});
    problem.setStateInfo("/jointset/j1/q1/speed", {4, 10});
    problemRep = problem.createRep();
    SimTK_TEST_EQ(problemRep.getStateInfo("/jointset/j0/q0/value")
                          .getBounds()
                          .getLower(),
            3);
    SimTK_TEST_EQ(problemRep.getStateInfo("/jointset/j1/q1/value")
                          .getBounds()
                          .getLower(),
            2);
    SimTK_TEST_EQ(problemRep.getStateInfo("/jointset/j0/q0/speed")
                          .getBounds()
                          .getLower(),
            3);
    SimTK_TEST_EQ(problemRep.getStateInfo("/jointset/j1/q1/speed")
                          .getBounds()
                          .getLower(),
            4);
}
TEMPLATE_TEST_CASE("Disable Actuators", "", MocoCasADiSolver,
        MocoTropterSolver) {

    MocoSolution solution;
    MocoSolution solution2;
    {
        MocoStudy study;
        study.setName("double_pendulum");

        MocoProblem& mp = study.updProblem();
        auto model = ModelFactory::createDoublePendulum();

        auto* tau2 = new CoordinateActuator("q1");
        tau2->setName("tau2");
        tau2->setOptimalForce(1);
        model.addComponent(tau2);

        mp.setModelAsCopy(model);

        mp.setTimeBounds(0, {0, 5});
        mp.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0, SimTK::Pi);
        mp.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0);
        mp.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0, 0);
        mp.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0);
        mp.setControlInfo("/tau0", {-100, 100});
        mp.setControlInfo("/tau1", {-100, 100});
        mp.setControlInfo("/tau2", {-100, 100});

        mp.addGoal<MocoFinalTimeGoal>();
        auto& ms = study.initSolver<TestType>();
        ms.set_num_mesh_intervals(15);
        solution = study.solve();
    }
    {
        MocoStudy study2;
        study2.setName("double_pendulum");

        MocoProblem& mp2 = study2.updProblem();
        OpenSim::Model model2 = ModelFactory::createDoublePendulum();

        auto* tau2 = new CoordinateActuator("q1");
        tau2->setName("tau2");
        tau2->setOptimalForce(1);
        model2.addComponent(tau2);

        SimTK::State state = model2.initSystem();
        model2.updComponent<Force>("tau1").set_appliesForce(false);
        mp2.setModelAsCopy(model2);
        mp2.setTimeBounds(0, {0, 5});
        mp2.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0, SimTK::Pi);
        mp2.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0);
        mp2.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0, 0);
        mp2.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0);
        mp2.setControlInfo("/tau0", {-100, 100});
        mp2.setControlInfo("/tau2", {-100, 100});

        mp2.addGoal<MocoFinalTimeGoal>();
        auto& ms2 = study2.initSolver<TestType>();
        ms2.set_num_mesh_intervals(15);
        solution2 = study2.solve();
    }
    CHECK(solution2.getObjective() != Approx(solution.getObjective()));
}

TEMPLATE_TEST_CASE("State tracking", "", MocoCasADiSolver,
        MocoTropterSolver) {
    // TODO move to another test file?
    auto makeTool = []() {
        MocoStudy study;
        study.setName("state_tracking");
        study.set_write_solution("false");
        MocoProblem& mp = study.updProblem();
        mp.setModel(createSlidingMassModel());
        mp.setTimeBounds(0, 1);
        mp.setStateInfo("/slider/position/value", {-1, 1});
        mp.setStateInfo("/slider/position/speed", {-100, 100});
        mp.setControlInfo("/actuator", {-50, 50});
        return study;
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
        ref.addTableMetaData("inDegrees", std::string("no"));
        STOFileAdapter::write(ref, fname);
    }

    // Setting the TimeSeriesTable directly.
    MocoSolution solDirect;
    {
        auto moco = makeTool();
        MocoProblem& mp = moco.updProblem();
        auto tracking = mp.addGoal<MocoStateTrackingGoal>();
        tracking->setReference(TimeSeriesTable(fname));
        auto& ms = moco.template initSolver<TestType>();
        ms.set_num_mesh_intervals(5);
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
        auto tracking = mp.addGoal<MocoStateTrackingGoal>();
        tracking->setReference(fname);
        auto& ms = moco.template initSolver<TestType>();
        ms.set_num_mesh_intervals(5);
        ms.set_optim_hessian_approximation("exact");
        solFile = moco.solve();
        moco.print(setup_fname);
    }

    // Run the tool from a setup file.
    MocoSolution solDeserialized;
    {
        MocoStudy study(setup_fname);
        solDeserialized = study.solve();
    }

    SimTK_TEST(solDirect.isNumericallyEqual(solFile));
    SimTK_TEST(solFile.isNumericallyEqual(solDeserialized));

    // Error if neither file nor table were provided.
    {
        auto moco = makeTool();
        MocoProblem& mp = moco.updProblem();
        mp.addGoal<MocoStateTrackingGoal>();
        SimTK_TEST_MUST_THROW_EXC(moco.solve(), Exception);
    }

    // TODO error if data does not cover time window.
}

TEMPLATE_TEST_CASE("Guess", "", MocoCasADiSolver, MocoTropterSolver) {

    MocoStudy study = createSlidingMassMocoStudy<TestType>();
    auto& ms = study.initSolver<TestType>();
    const int N = 5;
    const int Nc = 2 * N + 1;
    ms.set_num_mesh_intervals(N);

    std::vector<std::string> expectedStateNames{
            "/slider/position/value", "/slider/position/speed"};
    std::vector<std::string> expectedControlNames{"/actuator"};

    SimTK::Matrix expectedStatesTraj(Nc, 2);
    expectedStatesTraj.col(0) = 0.5;   // bounds are [0, 1].
    expectedStatesTraj(0, 0) = 0;      // initial value fixed to 0.
    expectedStatesTraj(Nc - 1, 0) = 1; // final value fixed to 1.
    expectedStatesTraj.col(1) = 0.0;   // bounds are [-100, 100]
    expectedStatesTraj(0, 1) = 0;      // initial speed fixed to 0.
    expectedStatesTraj(Nc - 1, 1) = 0; // final speed fixed to 1.

    SimTK::Matrix expectedControlsTraj(Nc, 1);
    expectedControlsTraj.col(0) = 0;

    // createGuess().
    // --------------

    // Initial guess based on bounds.
    {
        MocoTrajectory guess = ms.createGuess("bounds");
        CHECK(guess.getTime().size() == Nc);
        CHECK(guess.getStateNames() == expectedStateNames);
        CHECK(guess.getValueNames()[0] == expectedStateNames[0]);
        CHECK(guess.getSpeedNames()[0] == expectedStateNames[1]);
        CHECK(guess.getControlNames() == expectedControlNames);
        CHECK(guess.getTime()[0] == 0);
        // midpoint of bounds [0, 10]
        CHECK(guess.getTime()[Nc - 1] == Approx(5.0));

        SimTK_TEST_EQ(guess.getStatesTrajectory(), expectedStatesTraj);
        SimTK_TEST_EQ(guess.getValuesTrajectory(),
                      expectedStatesTraj.block(0, 0, Nc, 1));
        SimTK_TEST_EQ(guess.getSpeedsTrajectory(),
                      expectedStatesTraj.block(0, 1, Nc, 1));
        SimTK_TEST_EQ(guess.getControlsTrajectory(), expectedControlsTraj);
    }

    // Random initial guess.
    {
        MocoTrajectory guess = ms.createGuess("random");
        CHECK(guess.getTime().size() == Nc);
        CHECK(guess.getStateNames() == expectedStateNames);
        CHECK(guess.getValueNames()[0] == expectedStateNames[0]);
        CHECK(guess.getSpeedNames()[0] == expectedStateNames[1]);
        CHECK(guess.getControlNames() == expectedControlNames);

        // The numbers are random, so we don't what they are; only that they
        // are different from the guess from bounds.
        SimTK_TEST_NOTEQ(guess.getStatesTrajectory(), expectedStatesTraj);
        SimTK_TEST_NOTEQ(guess.getValuesTrajectory(),
                      expectedStatesTraj.block(0, 0, Nc, 1));
        SimTK_TEST_NOTEQ(guess.getSpeedsTrajectory(),
                      expectedStatesTraj.block(0, 1, Nc, 1));
        SimTK_TEST_NOTEQ(guess.getControlsTrajectory(), expectedControlsTraj);
    }

    // Setting a guess programmatically.
    // ---------------------------------

    // Don't need a converged solution; so ensure the following tests are
    // fast.
    ms.set_optim_max_iterations(2);

    ms.clearGuess();
    MocoTrajectory solNoGuess = study.solve().unseal();
    {
        // Using the guess from bounds is the same as not providing a guess.

        ms.setGuess(ms.createGuess());
        MocoTrajectory solDefaultGuess = study.solve().unseal();

        CHECK(solDefaultGuess.isNumericallyEqual(solNoGuess));

        // Can also use convenience version of setGuess().
        ms.setGuess("bounds");
        CHECK(study.solve().unseal().isNumericallyEqual(solNoGuess));

        // Using a random guess should give us a different "solution."
        ms.setGuess(ms.createGuess("random"));
        MocoTrajectory solRandomGuess = study.solve().unseal();
        CHECK(!solRandomGuess.isNumericallyEqual(solNoGuess));

        // Convenience.
        ms.setGuess("random");
        CHECK(!study.solve().unseal().isNumericallyEqual(solNoGuess));

        // Clearing the guess works (this check must come after using a
        // random guess).
        ms.clearGuess();
        CHECK(study.solve().unseal().isNumericallyEqual(solNoGuess));

        // Can call clearGuess() multiple times with no weird issues.
        ms.clearGuess();
        ms.clearGuess();
        CHECK(study.solve().unseal().isNumericallyEqual(solNoGuess));
    }

    // Guess is incompatible with problem.
    {
        MocoTrajectory guess = ms.createGuess();
        // Delete the second state variable name.
        const_cast<std::vector<std::string>&>(guess.getStateNames()).resize(1);
        CHECK_THROWS_AS(ms.setGuess(std::move(guess)), Exception);
    }

    // Unrecognized guess type.
    CHECK_THROWS_AS(ms.createGuess("unrecognized"), Exception);
    CHECK_THROWS_AS(ms.setGuess("unrecognized"), Exception);

    // Setting a guess from a file.
    // ----------------------------
    {
        MocoTrajectory guess = ms.createGuess("bounds");
        // Use weird number to ensure the solver actually loads the file:
        guess.setControl("/actuator", SimTK::Vector(Nc, 13.28));
        const std::string fname = "testMocoInterface_testGuess_file.sto";
        guess.write(fname);
        ms.setGuessFile(fname);

        CHECK(ms.getGuess().isNumericallyEqual(guess));
        CHECK(!study.solve().unseal().isNumericallyEqual(solNoGuess));

        // Using setGuess(MocoTrajectory) overrides the file setting.
        ms.setGuess(ms.createGuess("bounds"));
        CHECK(study.solve().unseal().isNumericallyEqual(solNoGuess));

        ms.setGuessFile(fname);
        CHECK(ms.getGuess().isNumericallyEqual(guess));
        CHECK(!study.solve().unseal().isNumericallyEqual(solNoGuess));

        // Clearing the file causes the default guess type to be used.
        ms.setGuessFile("");
        CHECK(study.solve().unseal().isNumericallyEqual(solNoGuess));

        // TODO mismatched state/control names.

        // Solve from deserialization.
        // TODO
    }

    // Customize a guess.
    // ------------------
    // This is really just a test of the MocoTrajectory class.
    {
        MocoTrajectory guess = ms.createGuess();
        guess.setNumTimes(2);
        CHECK(SimTK::isNaN(guess.getTime()[0]));
        CHECK(SimTK::isNaN(guess.getStatesTrajectory()(0, 0)));
        CHECK(SimTK::isNaN(guess.getControlsTrajectory()(0, 0)));

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
        CHECK_THROWS_AS(guess.setState("none", SimTK::Vector(2)), Exception);
        CHECK_THROWS_AS(guess.setControl("none", SimTK::Vector(2)), Exception);
        SimTK_TEST_MUST_THROW_EXC(guess.getState("none"), Exception);
        SimTK_TEST_MUST_THROW_EXC(guess.getControl("none"), Exception);

        // Incorrect length.
        CHECK_THROWS_AS(
                guess.setState("/slider/position/value", SimTK::Vector(1)),
                Exception);
        CHECK_THROWS_AS(
                guess.setControl("/actuator", SimTK::Vector(3)), Exception);
    }

    // Resampling.
    {
        const int N = 4;
        const int Nc = 2 * N + 1;
        ms.set_num_mesh_intervals(N);
        MocoTrajectory guess0 = ms.createGuess();
        guess0.setControl("/actuator", createVectorLinspace(Nc, 2.8, 7.3));
        CHECK(guess0.getTime().size() == Nc); // midpoint of [0, 10]
        SimTK_TEST_EQ(guess0.getTime()[Nc - 1], N + 1);

        // resampleWithNumTimes
        {
            MocoTrajectory guess = guess0;
            guess.resampleWithNumTimes(10);
            CHECK(guess.getTime().size() == 10);
            CHECK(guess.getTime()[9] == Approx(5));
            CHECK(guess.getStatesTrajectory().nrow() == 10);
            CHECK(guess.getValuesTrajectory().nrow() == 10);
            CHECK(guess.getSpeedsTrajectory().nrow() == 10);
            CHECK(guess.getControlsTrajectory().nrow() == 10);
            SimTK_TEST_EQ(guess.getControl("/actuator"),
                    createVectorLinspace(10, 2.8, 7.3));
        }

        // resampleWithInterval
        {
            MocoTrajectory guess = guess0;
            // We can't achieve exactly the interval the user provides.
            // time_interval = duration/(num_times - 1)
            // actual_num_times = ceil(duration/desired_interval) + 1
            // actual_interval = duration/(actual_num_times - 1)
            auto actualInterval = guess.resampleWithInterval(0.9);
            int expectedNumTimes = (int)ceil(5 / 0.9) + 1;
            SimTK_TEST_EQ(actualInterval, 5 / ((double)expectedNumTimes - 1));
            CHECK(guess.getTime().size() == expectedNumTimes);
            SimTK_TEST_EQ(guess.getTime()[expectedNumTimes - 1], 5);
            CHECK(guess.getStatesTrajectory().nrow() == expectedNumTimes);
            CHECK(guess.getValuesTrajectory().nrow() == expectedNumTimes);
            CHECK(guess.getSpeedsTrajectory().nrow() == expectedNumTimes);
            CHECK(guess.getControlsTrajectory().nrow() == expectedNumTimes);
            SimTK_TEST_EQ(guess.getControl("/actuator"),
                    createVectorLinspace(expectedNumTimes, 2.8, 7.3));
        }

        // resampleWithFrequency
        {
            // We can't achieve exactly the interval the user provides.
            // frequency = num_times/duration
            MocoTrajectory guess = guess0;
            // Here, we also ensure that we can downsample.
            auto actualFrequency = guess.resampleWithFrequency(0.7);
            int expectedNumTimes = (int)ceil(5 * 0.7); // 4
            SimTK_TEST_EQ(actualFrequency, (double)expectedNumTimes / 5);
            CHECK(guess.getTime().size() == expectedNumTimes);
            SimTK_TEST_EQ(guess.getTime()[expectedNumTimes - 1], 5);
            CHECK(guess.getStatesTrajectory().nrow() == expectedNumTimes);
            CHECK(guess.getValuesTrajectory().nrow() == expectedNumTimes);
            CHECK(guess.getSpeedsTrajectory().nrow() == expectedNumTimes);
            CHECK(guess.getControlsTrajectory().nrow() == expectedNumTimes);
            SimTK_TEST_EQ(guess.getControl("/actuator"),
                    createVectorLinspace(expectedNumTimes, 2.8, 7.3));
        }

        // resample
        {
            MocoTrajectory guess = guess0;
            CHECK_THROWS_AS(guess.resample(createVector(
                                    {guess.getInitialTime() - 1e-15, 0, 1})),
                    Exception);
            CHECK_THROWS_AS(guess.resample(createVector(
                                    {0.5, 0.6, guess.getFinalTime() + 1e15})),
                    Exception);
            CHECK_THROWS_AS(
                    guess.resample(createVector({0.5, 0.6, 0.59999999, 0.8})),
                    Exception);

            {
                SimTK::Vector newTime = createVector({0.1, 0.3, 0.8});
                MocoTrajectory guess2 = guess;
                guess2.resample(newTime);
                SimTK_TEST_EQ(newTime, guess2.getTime());
            }
        }
    }

    // Number of points required for splining.
    {
        // 3 and 2 points are okay.
        ms.set_num_mesh_intervals(3);
        MocoTrajectory guess3 = ms.createGuess();
        guess3.resampleWithNumTimes(10);

        ms.set_num_mesh_intervals(2);
        MocoTrajectory guess2 = ms.createGuess();
        guess2.resampleWithNumTimes(10);

        // 1 point is too few.
        MocoTrajectory guess1(guess2);
        guess1.setNumTimes(1);
        CHECK_THROWS_AS(guess1.resampleWithNumTimes(10), Exception);
    }

    // Can't use a guess from explicit dynamics in implicit dynamics mode.
    {
        MocoTrajectory explicitGuess = ms.createGuess();
        ms.set_multibody_dynamics_mode("implicit");
        CHECK_THROWS_WITH(ms.setGuess(explicitGuess),
            Catch::Contains(
                "'multibody_dynamics_mode' set to 'implicit' and coordinate states "
                "exist in the guess, but no coordinate accelerations were "
                "found in the guess. Consider using "
                "MocoTrajectory::generateAccelerationsFromValues() or "
                "MocoTrajectory::generateAccelerationsFromSpeeds() to "
                "construct an appropriate guess."));
        CHECK(explicitGuess.getDerivativeNames().empty());
        explicitGuess.generateAccelerationsFromSpeeds();
        // Only one coordinate in the sliding mass model.
        CHECK(explicitGuess.getDerivativeNames().size() == 1);
    }

    // TODO ordering of states and controls in MocoTrajectory should not
    // matter!

    // TODO getting a guess, editing the problem, asking for another guess,
    // requires calling initSolver<>(). TODO can check the Problem's
    // isObjectUptoDateWithProperties(); simply flipping a flag with
    // updProblem() is not sufficient, b/c a user could make changes way
    // after they get the mutable reference.
}

TEMPLATE_TEST_CASE("Guess time-stepping", "[tropter]",
        MocoTropterSolver /*, MocoCasADiSolver*/) {
    // This problem is just a simulation (there are no costs), and so the
    // forward simulation guess should reduce the number of iterations to
    // converge, and the guess and solution should also match our own
    // forward simulation.
    MocoStudy study;
    study.setName("pendulum");
    study.set_write_solution("false");
    auto& problem = study.updProblem();
    problem.setModel(createPendulumModel());
    const SimTK::Real initialAngle = 0.25 * SimTK::Pi;
    const SimTK::Real initialSpeed = .5;
    // Make the simulation interesting.
    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, initialAngle);
    problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, initialSpeed);
    problem.setControlInfo("/forceset/tau0", 0);
    auto& solver = study.initSolver<TestType>();
    solver.set_num_mesh_intervals(20);
    solver.setGuess("random");
    // With MUMPS: 4 iterations.
    const MocoSolution solutionRandom = study.solve();

    solver.setGuess("time-stepping");
    // With MUMPS: 2 iterations.
    MocoSolution solutionSim = study.solve();

    CHECK(solutionSim.getNumIterations() < solutionRandom.getNumIterations());

    {
        MocoTrajectory guess = solver.createGuess("time-stepping");
        REQUIRE(solutionSim.compareContinuousVariablesRMS(guess) < 1e-2);

        Model modelCopy(study.updProblem().getPhase().getModel());
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
        const auto trajectoryFromManager =
                MocoTrajectory::createFromStatesControlsTables(
                        study.getProblem().createRep(),
                        manager.getStatesTable(),
                        controlsTable);
        SimTK_TEST(solutionSim.compareContinuousVariablesRMS(
                           trajectoryFromManager) < 1e-2);
    }

    // Ensure the forward simulation guess uses the correct time bounds.
    {
        study.updProblem().setTimeBounds({-10, -5}, {6, 15});
        auto& solver = study.initSolver<TestType>();
        MocoTrajectory guess = solver.createGuess("time-stepping");
        SimTK_TEST(guess.getTime()[0] == -5);
        SimTK_TEST(guess.getTime()[guess.getNumTimes() - 1] == 6);
    }
}

TEMPLATE_TEST_CASE("MocoTrajectory", "", MocoCasADiSolver, MocoTropterSolver) {
    // Reading and writing.
    {
        const std::string fname = "testMocoInterface_testMocoTrajectory.sto";
        SimTK::Vector time(3);
        time[0] = 0;
        time[1] = 0.1;
        time[2] = 0.25;
        MocoTrajectory orig(time, {"a", "b"}, {"g", "h", "i", "j"}, {"m"},
                {"o", "p"}, SimTK::Test::randMatrix(3, 2),
                SimTK::Test::randMatrix(3, 4), SimTK::Test::randMatrix(3, 1),
                SimTK::Test::randVector(2).transpose());
        orig.write(fname);

        MocoTrajectory deserialized(fname);
        SimTK_TEST(deserialized.isNumericallyEqual(orig));
    }

    {
        const std::string fname =
                "testMocoInterface_testMocoSolutionSuccess.sto";
        MocoStudy study = createSlidingMassMocoStudy<TestType>();
        auto& solver =
                dynamic_cast<MocoDirectCollocationSolver&>(study.updSolver());

        solver.set_optim_max_iterations(1);
        MocoSolution failedSolution = study.solve();
        failedSolution.unseal();
        failedSolution.write(fname);
        MocoTrajectory deserialized(fname);

        std::ifstream mocoSolutionFile(fname);
        for (std::string line; getline(mocoSolutionFile, line);) {
            if (line.compare("success=false")) {
                break;
            } else if (line.compare("success=true")) {
                SimTK_TEST(false);
            }
        }
    }

    // Test sealing/unsealing.
    {
        // Create a class that gives access to the sealed functions, which
        // are otherwise protected.
        class MocoTrajectoryDerived : public MocoTrajectory {
        public:
            using MocoTrajectory::MocoTrajectory;
            MocoTrajectoryDerived* clone() const override {
                return new MocoTrajectoryDerived(*this);
            }
            void setSealedD(bool sealed) { MocoTrajectory::setSealed(sealed); }
            bool isSealedD() const { return MocoTrajectory::isSealed(); }
        };
        MocoTrajectoryDerived trajectory;
        SimTK_TEST(!trajectory.isSealedD());
        trajectory.setSealedD(true);
        SimTK_TEST(trajectory.isSealedD());
        SimTK_TEST_MUST_THROW_EXC(
                trajectory.getNumTimes(), MocoTrajectoryIsSealed);
        SimTK_TEST_MUST_THROW_EXC(trajectory.getTime(), MocoTrajectoryIsSealed);
        SimTK_TEST_MUST_THROW_EXC(
                trajectory.getStateNames(), MocoTrajectoryIsSealed);
        SimTK_TEST_MUST_THROW_EXC(
                trajectory.getControlNames(), MocoTrajectoryIsSealed);
        SimTK_TEST_MUST_THROW_EXC(
                trajectory.getControlNames(), MocoTrajectoryIsSealed);

        // The clone() function doesn't call ensureSealed(), but the clone
        // should preserve the value of m_sealed.
        std::unique_ptr<MocoTrajectoryDerived> ptr(trajectory.clone());
        SimTK_TEST(ptr->isSealedD());
        SimTK_TEST_MUST_THROW_EXC(
                trajectory.getNumTimes(), MocoTrajectoryIsSealed);
    }

    // getInitialTime(), getFinalTime()
    {
        {
            // With 0 times, these functions throw an exception.
            MocoTrajectory it;
            SimTK_TEST_MUST_THROW_EXC(it.getInitialTime(), Exception);
            SimTK_TEST_MUST_THROW_EXC(it.getFinalTime(), Exception);
        }

        {
            SimTK::Vector time = createVectorLinspace(5, -3.1, 8.9);
            std::vector<std::string> snames{"s0", "s1"};
            std::vector<std::string> cnames{"c0"};
            SimTK::Matrix states = SimTK::Test::randMatrix(5, 2);
            SimTK::Matrix controls = SimTK::Test::randMatrix(5, 1);
            MocoTrajectory it(time, snames, cnames, {}, {}, states, controls,
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
            MocoTrajectory it(time, snames, cnames, {}, {}, states, controls,
                    SimTK::Matrix(), SimTK::RowVector());

            SimTK_TEST_EQ(it.getInitialTime(), 7.2);
            SimTK_TEST_EQ(it.getFinalTime(), 7.2);
        }
    }

    // trimToIndices()
    {
        SimTK::Vector time = createVectorLinspace(5, -3.1, 8.9);
        std::vector<std::string> snames{"s0"};
        std::vector<std::string> cnames{"c0"};
        SimTK::Matrix states = SimTK::Test::randMatrix(5, 1);
        states.set(1, 0, 1.23);
        states.set(3, 0, 4.56);
        SimTK::Matrix controls = SimTK::Test::randMatrix(5, 1);
        controls.set(1, 0, 7.89);
        controls.set(3, 0, 1.01);
        MocoTrajectory it(time, snames, cnames, {}, {}, states, controls,
                SimTK::Matrix(), SimTK::RowVector());

        it.trimToIndices(1, 3);
        SimTK_TEST_EQ(it.getInitialTime(), time[1]);
        SimTK_TEST_EQ(it.getFinalTime(), time[3]);

        SimTK::VectorView_<double> s0 = it.getState("s0");
        SimTK::VectorView_<double> c0 = it.getControl("c0");
        SimTK_TEST_EQ(s0[0], 1.23);
        SimTK_TEST_EQ(s0[it.getNumTimes()-1], 4.56);
        SimTK_TEST_EQ(c0[0], 7.89);
        SimTK_TEST_EQ(c0[it.getNumTimes()-1], 1.01);
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
                MocoTrajectory a(time, snames, cnames, mnames, {}, states,
                        controls, multipliers, SimTK::RowVector());
                MocoTrajectory b(time, snames, cnames, mnames, {},
                        states.elementwiseAddScalar(error),
                        controls.elementwiseAddScalar(error),
                        multipliers.elementwiseAddScalar(error),
                        SimTK::RowVector());
                // If error is constant:
                // sqrt(1/(T*N) * integral_t (sum_i^N (err_{i,t}^2))) = err
                auto rmsBA = b.compareContinuousVariablesRMS(
                        a, {{"states", statesToCompare},
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
                auto rmsAB = a.compareContinuousVariablesRMS(
                        b, {{"states", statesToCompare},
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
        MocoTrajectory a(SimTK::Vector(), {}, {}, {}, pnames, SimTK::Matrix(),
                SimTK::Matrix(), SimTK::Matrix(), parameters);
        MocoTrajectory b(SimTK::Vector(), {}, {}, {}, pnames, SimTK::Matrix(),
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

TEST_CASE("MocoTrajectory isCompatible") {
    MocoProblem problem;
    problem.setModel(createSlidingMassModel());
    problem.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(0, 10));
    problem.setStateInfo("/slider/position/value", MocoBounds(0, 1),
            MocoInitialBounds(0), MocoFinalBounds(1));
    problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);

    MocoProblemRep rep = problem.createRep();

    CHECK(MocoTrajectory({"/slider/position/value", "/slider/position/speed"},
            {"/actuator"}, {}, {}).isCompatible(rep, false, true));
    CHECK(!MocoTrajectory({"/slider/position/value", "/slider/position/speed"},
            {"/actuator"}, {}, {}).isCompatible(rep, true));

    CHECK(MocoTrajectory({"/slider/position/value", "/slider/position/speed"},
            {"/actuator"}, {}, {"/slider/position/accel"}, {}).isCompatible(rep, true));

    CHECK(!MocoTrajectory({}, {}, {}, {}).isCompatible(rep));
    CHECK(!MocoTrajectory({"/slider/position/value", "/slider/position/speed",
                          "nonexistent"},
            {"/actuator"}, {}, {}).isCompatible(rep));
    CHECK(!MocoTrajectory({"/slider/position/value", "/slider/position/speed"},
            {"/actuator"}, {"nonexistent"}, {}).isCompatible(rep));

    CHECK_THROWS_WITH(
            MocoTrajectory({"/slider/position/value",
                             "/slider/position/speed"},
                              {"/actuator"}, {}, {})
                               .isCompatible(rep, true, true),
            Catch::Contains("accel"));
    CHECK_THROWS_WITH(
            MocoTrajectory({}, {}, {}, {}).isCompatible(rep, false, true),
            Catch::Contains("position"));
    CHECK_THROWS_WITH(
            MocoTrajectory({"/slider/position/value", "/slider/position/speed",
                             "nonexistent"},
                    {"/actuator"}, {}, {})
                     .isCompatible(rep, false, true),
            Catch::Contains("nonexistent"));
    CHECK_THROWS_WITH(
            MocoTrajectory({"/slider/position/value", "/slider/position/speed"}, {"/actuator"},
            {"nonexistent"}, {})
                               .isCompatible(rep, false, true),
            Catch::Contains("nonexistent"));
}

TEST_CASE("MocoTrajectory randomize") {
    SimTK::Vector time(3);
    time[0] = 0;
    time[1] = 0.1;
    time[2] = 0.25;
    MocoTrajectory orig(time, {"a", "b"}, {"g", "h", "i", "j"}, {"m"},
            {"o", "p"}, SimTK::Test::randMatrix(3, 2),
            SimTK::Test::randMatrix(3, 4), SimTK::Test::randMatrix(3, 1),
            SimTK::Test::randVector(2).transpose());
    SECTION("randomizeAdd") {
        MocoTrajectory randomized = orig;
        randomized.randomizeAdd(SimTK::Random::Uniform(-0.01, 0.01));
        const auto rms = orig.compareContinuousVariablesRMS(randomized);
        CHECK(0.001 < rms);
        CHECK(rms < 0.01);
    }
    SECTION("randomizeReplace") {
        MocoTrajectory randomized = orig;
        randomized.randomizeReplace(SimTK::Random::Uniform(-0.01, 0.01));
        const auto rms = orig.compareContinuousVariablesRMS(randomized);
        CHECK(rms > 0.01);
    }
}

TEST_CASE("createPeriodicTrajectory") {
    const std::string hip_r = "hip_r/hip_flexion_r/value";
    const std::string hip_l = "hip_l/hip_flexion_l/value";
    std::vector<std::string> sn{
            "pelvis_tx/value",
            "pelvis_list/value",
            hip_r, hip_l
    };
    std::vector<std::string> cn{
            "soleus_r",
            "soleus_l",
    };
    int N = 4;
    const auto time = createVectorLinspace(N, 0, 1);
    SimTK::Matrix st(N, 4);
    SimTK::Matrix ct(N, 2);
    for (int i = 0; i < N; ++i) {
        st(i, 0) = std::sin(time[i]);
        st(i, 1) = std::cos(time[i]);
        st(i, 2) = std::exp(time[i]);
        st(i, 3) = std::log(time[i] + 1.0);

        ct(i, 0) = std::tan(time[i]);
        ct(i, 1) = SimTK::cube(time[i]);
    }

    MocoTrajectory halfTraj(
            time, {{"states", {sn, st}}, {"controls", {cn, ct}}});
    const auto fullTraj = createPeriodicTrajectory(halfTraj);

    // add
    {
        SimTK::Vector secondHalf = halfTraj.getState("pelvis_tx/value")
                                           .block(1, 0, N - 1, 1)
                                           .col(0);
        secondHalf += st(N - 1, 0);
        OpenSim_CHECK_MATRIX(
                fullTraj.getState("pelvis_tx/value").block(N, 0, N - 1, 1),
                secondHalf);
    }

    // negate
    {
        SimTK::Vector secondHalf(N - 1);
        for (int i = 0; i < N - 1; ++i) {
            secondHalf[i] = -std::cos(time[i + 1]) + 2 * std::cos(time[N - 1]);
        }
        OpenSim_CHECK_MATRIX(
                fullTraj.getState("pelvis_list/value").block(N, 0, N - 1, 1),
                secondHalf);
    }

    // symmetry
    OpenSim_CHECK_MATRIX(
            fullTraj.getState(hip_r).block(N, 0, N - 1, 1),
            fullTraj.getState(hip_l).block(1, 0, N - 1, 1));
    OpenSim_CHECK_MATRIX(
            fullTraj.getState(hip_l).block(N, 0, N - 1, 1),
            fullTraj.getState(hip_r).block(1, 0, N - 1, 1));

    OpenSim_CHECK_MATRIX(fullTraj.getControl("soleus_r").block(N, 0, N - 1, 1),
            fullTraj.getControl("soleus_l").block(1, 0, N - 1, 1));
    OpenSim_CHECK_MATRIX(fullTraj.getControl("soleus_l").block(N, 0, N - 1, 1),
            fullTraj.getControl("soleus_r").block(1, 0, N - 1, 1));
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

TEMPLATE_TEST_CASE("Sliding mass", "", MocoCasADiSolver, MocoTropterSolver) {
    MocoStudy study = createSlidingMassMocoStudy<TestType>();
    MocoSolution solution = study.solve();
    int numTimes = 20;
    int numStates = 2;
    int numValues = 1;
    int numSpeeds = 1;
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
    const auto& values = solution.getValuesTrajectory();
    SimTK_TEST(values.nrow() == numTimes);
    SimTK_TEST(values.ncol() == numValues);
    const auto& speeds = solution.getSpeedsTrajectory();
    SimTK_TEST(speeds.nrow() == numTimes);
    SimTK_TEST(speeds.ncol() == numSpeeds);
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

TEMPLATE_TEST_CASE("Solving an empty MocoProblem", "",
        MocoCasADiSolver, MocoTropterSolver) {
    MocoStudy study;
    auto& solver = study.initSolver<TestType>();
    THEN("problem solves without error, solution trajectories are empty.") {
        MocoSolution solution = study.solve();
        const int N = solver.get_num_mesh_intervals();
        const int Nc = 2 * N + 1; // collocation points
        CHECK(solution.getTime().size() == Nc);
        CHECK(solution.getStatesTrajectory().ncol() == 0);
        CHECK(solution.getStatesTrajectory().nrow() == Nc);
        CHECK(solution.getControlsTrajectory().ncol() == 0);
        CHECK(solution.getControlsTrajectory().nrow() == Nc);
        CHECK(solution.getMultipliersTrajectory().ncol() == 0);
        CHECK(solution.getMultipliersTrajectory().nrow() == Nc);
        CHECK(solution.getDerivativesTrajectory().ncol() == 0);
        CHECK(solution.getDerivativesTrajectory().nrow() == Nc);
        CHECK(solution.getValuesTrajectory().ncol() == 0);
        CHECK(solution.getValuesTrajectory().nrow() == Nc);
        CHECK(solution.getSpeedsTrajectory().ncol() == 0);
        CHECK(solution.getSpeedsTrajectory().nrow() == Nc);
        CHECK(solution.getAccelerationsTrajectory().ncol() == 0);
        CHECK(solution.getAccelerationsTrajectory().nrow() == Nc);
        CHECK(solution.getDerivativesWithoutAccelerationsTrajectory()
              .ncol() == 0);
        CHECK(solution.getDerivativesWithoutAccelerationsTrajectory()
              .nrow() == Nc);

    }
}

/// Ensure that using a joint that has an empty quaternion slot does not
/// cause us to misalign states between OpenSim and Tropter/CasADi.
/// Even when not using quaternions, Simbody has a slot in the state vector
/// for the 4th quaternion coordinate.
template <typename SolverType>
void testSkippingOverQuaternionSlots(
        bool constrained, bool constraintDerivs, std::string dynamicsMode) {
    Model model;
    using SimTK::Vec3;
    auto* b1 = new Body("b1", 1, Vec3(0), SimTK::Inertia(1));
    model.addBody(b1);
    auto* j1 = new BallJoint("j1", model.getGround(), Vec3(0, -1, 0), Vec3(0),
            *b1, Vec3(0), Vec3(0));
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

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);
    const double duration = 1.0;
    problem.setTimeBounds(0, duration);
    problem.setStateInfo(
            "/jointset/j1/j1_coord_1/value", {-0.2, 0.2}, -0.2, 0.2);
    const double speed = 0.36;
    problem.setStateInfo("/jointset/j2/j2_coord_0/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j2/j2_coord_0/speed", speed);

    problem.addGoal<MocoControlGoal>();

    auto& solver = study.initSolver<SolverType>();
    const int N = 4;
    solver.set_num_mesh_intervals(N);
    solver.set_multibody_dynamics_mode(dynamicsMode);
    solver.set_transcription_scheme("hermite-simpson");
    solver.set_optim_constraint_tolerance(1e-2);
    solver.set_optim_convergence_tolerance(1e-2);
    if (constrained) {
        solver.set_enforce_constraint_derivatives(constraintDerivs);
    }

    MocoSolution solution = study.solve();

    const auto& valueTraj = solution.getState("/jointset/j2/j2_coord_0/value");
    const double lastValue = valueTraj[valueTraj.size() - 1];
    CHECK(lastValue == Approx(speed * duration));
    for (int i = 0; i < N; ++i) {
        CHECK(solution.getState("/jointset/j2/j2_coord_0/speed").getElt(i, 0) ==
                Approx(speed));
    }
}

// TODO these tests are currently disabled until we support joints where
// qdot != u, since all joints with quaternion slots fall into this category.
//TEST_CASE("Skip over empty quaternion slots; CasADi.", "[casadi]") {
//    auto mode = GENERATE(as<std::string>{}, "explicit", "implicit");
//    testSkippingOverQuaternionSlots<MocoCasADiSolver>(false, false, mode);
//    testSkippingOverQuaternionSlots<MocoCasADiSolver>(true, false, mode);
//    testSkippingOverQuaternionSlots<MocoCasADiSolver>(true, true, mode);
//}
//
//TEST_CASE("Skip over empty quaternion slots; Tropter.", "[tropter]") {
//    testSkippingOverQuaternionSlots<MocoTropterSolver>(
//            false, false, "explicit");
//    testSkippingOverQuaternionSlots<MocoTropterSolver>(true, false, "explicit");
//    testSkippingOverQuaternionSlots<MocoTropterSolver>(true, true, "explicit");
//    testSkippingOverQuaternionSlots<MocoTropterSolver>(
//            false, false, "implicit");
//}

/// Joints where the derivative of the generalized coordinates is not equal to
/// the generalized speeds are not supported yet in Moco.
void testDisallowedJointTypes(const std::string& jointType) {
    Model model;
    using SimTK::Vec3;
    auto* b1 = new Body("b1", 1, Vec3(0), SimTK::Inertia(1));
    model.addBody(b1);
    if (jointType == "BallJoint") {
        auto *j1 = new BallJoint("j1", model.getGround(), *b1);
        model.addJoint(j1);
    } else if (jointType == "FreeJoint") {
        auto *j1 = new FreeJoint("j1", model.getGround(), *b1);
        model.addJoint(j1);
    } else if (jointType == "EllipsoidJoint") {
        auto *j1 = new EllipsoidJoint("j1", model.getGround(), *b1, Vec3(0));
        model.addJoint(j1);
    } else if (jointType == "ScapulothoracicJoint") {
        auto *j1 = new ScapulothoracicJoint("j1", model.getGround(), *b1,
                                            Vec3(0), SimTK::Vec2(0), 0);
        model.addJoint(j1);
    }
    model.finalizeConnections();

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);
    SimTK_TEST_MUST_THROW(study.solve());
}

TEST_CASE("Test disallowed joints") {
    testDisallowedJointTypes("BallJoint");
    testDisallowedJointTypes("FreeJoint");
    testDisallowedJointTypes("EllipsoidJoint");
    testDisallowedJointTypes("ScapulothoracicJoint");
}

TEST_CASE("MocoPhase::bound_activation_from_excitation") {
    MocoStudy study;
    auto& problem = study.updProblem();
    Model& model = problem.updModel();
    model.setName("muscle");
    auto* body = new Body("body", 0.5, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);
    auto* joint = new SliderJoint("joint", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("x");
    model.addComponent(joint);
    auto* musclePtr = new DeGrooteFregly2016Muscle();
    musclePtr->set_ignore_tendon_compliance(true);
    musclePtr->set_fiber_damping(0);
    musclePtr->setName("muscle");
    musclePtr->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    musclePtr->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addComponent(musclePtr);
    model.finalizeConnections();
    SECTION("bound_activation_from_excitation is false") {
        MocoPhase& ph0 = problem.updPhase(0);
        ph0.setBoundActivationFromExcitation(false);
        auto rep = problem.createRep();
        CHECK_THROWS_WITH(rep.getStateInfo("/muscle/activation"),
                Catch::Contains(
                        "No info available for state '/muscle/activation'."));
    }
    SECTION("bound_activation_from_excitation is true") {
        auto rep = problem.createRep();
        CHECK(rep.getStateInfo("/muscle/activation").getBounds().getLower() ==
                0);
        CHECK(rep.getStateInfo("/muscle/activation").getBounds().getUpper() ==
                1);
    }
    SECTION("bound_activation_from_excitation is true; non-default min/max "
            "control") {
        musclePtr->setMinControl(0.35);
        musclePtr->setMaxControl(0.96);
        auto rep = problem.createRep();
        CHECK(rep.getStateInfo("/muscle/activation").getBounds().getLower() ==
                Approx(0.35));
        CHECK(rep.getStateInfo("/muscle/activation").getBounds().getUpper() ==
                Approx(0.96));
    }
    SECTION("Custom excitation bounds") {
        problem.setControlInfo("/muscle", {0.41, 0.63});
        auto rep = problem.createRep();
        CHECK(rep.getStateInfo("/muscle/activation").getBounds().getLower() ==
                Approx(0.41));
        CHECK(rep.getStateInfo("/muscle/activation").getBounds().getUpper() ==
                Approx(0.63));
    }
    SECTION("ignore_activation_dynamics") {
        musclePtr->set_ignore_activation_dynamics(true);
        auto rep = problem.createRep();
        CHECK_THROWS_WITH(rep.getStateInfo("/muscle/activation"),
                Catch::Contains(
                        "No info available for state '/muscle/activation'."));
    }
}

TEST_CASE("updateStateLabels40") {
    auto model = ModelFactory::createPendulum();
    model.initSystem();
    std::vector<std::string> labels = {
            "q0",
            "q0_u",
            "nonexistent",
    };
    updateStateLabels40(model, labels);
    CHECK(labels[0] == "/jointset/j0/q0/value");
    CHECK(labels[1] == "/jointset/j0/q0/speed");
    CHECK(labels[2] == "nonexistent");
}

TEST_CASE("solveBisection()") {

    auto calcResidual = [](const SimTK::Real& x) { return x - 3.78; };
    {
        const auto root = solveBisection(calcResidual, -5, 5, 1e-6);
        SimTK_TEST_EQ_TOL(root, 3.78, 1e-6);
        // Make sure the tolerance has an effect.
        SimTK_TEST_NOTEQ_TOL(root, 3.78, 1e-10);
    }
    {
        const auto root = solveBisection(calcResidual, -5, 5, 1e-10);
        SimTK_TEST_EQ_TOL(root, 3.78, 1e-10);
    }

    // Multiple roots.
    {
        auto parabola = [](const SimTK::Real& x) {
            return SimTK::square(x - 2.5);
        };
        REQUIRE_THROWS_AS(solveBisection(parabola, -5, 5), Exception);
    }
}

TEST_CASE("Objective breakdown", "[casadi]") {
    class MocoConstantGoal : public MocoGoal {
        OpenSim_DECLARE_CONCRETE_OBJECT(MocoConstantGoal, MocoGoal);
    public:
        MocoConstantGoal() {}
        MocoConstantGoal(std::string name, double weight, double constant)
                : MocoGoal(std::move(name), weight), m_constant(constant) {}

    protected:
        void initializeOnModelImpl(const Model&) const override {
            setRequirements(0, 1);
        }
        void calcGoalImpl(
                const GoalInput& input, SimTK::Vector& cost) const override {
            cost[0] = m_constant;
        }
    private:
        double m_constant = 0;
    };

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.addGoal<MocoConstantGoal>("goal_a", 1.5, 5.2);
    problem.addGoal<MocoConstantGoal>("goal_b", 0.01, 7.3);

    MocoSolution solution = study.solve();
    CHECK(solution.getNumObjectiveTerms() == 2);
    CHECK(solution.getObjectiveTermByIndex(0) == Approx(1.5 * 5.2));
    CHECK(solution.getObjectiveTermByIndex(1) == Approx(0.01 * 7.3));
    CHECK(solution.getObjectiveTerm("goal_a") == Approx(1.5 * 5.2));
    CHECK(solution.getObjectiveTerm("goal_b") == Approx(0.01 * 7.3));
}

TEST_CASE("generateAccelerationsFromXXX() does not overwrite existing "
          "non-accleration derivatives.") {
    int N = 20;
    SimTK::Vector time = createVectorLinspace(20, 0.0, 1.0);
    std::vector<std::string> snames{"/jointset/joint/coord/value",
                                    "/jointset/joint/coord/speed",
                                    "/forceset/muscle/normalized_tendon_force"};
    std::vector<std::string> cnames{"/forceset/muscle"};
    std::vector<std::string> dnames{
        "/forceset/muscle/implicitderiv_normalized_tendon_force"};
    SimTK::Matrix states = SimTK::Test::randMatrix(N, 3);
    SimTK::Matrix controls = SimTK::Test::randMatrix(N, 1);
    SimTK::Matrix derivatives = SimTK::Test::randMatrix(N, 1);
    MocoTrajectory traj(time, snames, cnames, {}, dnames, {}, states, controls,
            SimTK::Matrix(), derivatives, SimTK::RowVector());

    std::vector<std::string> derivativeNames{
        "/jointset/joint/coord/accel",
        "/forceset/muscle/implicitderiv_normalized_tendon_force"};
    {
        traj.generateAccelerationsFromValues();
        CHECK(traj.getNumDerivatives() == 2);
        CHECK(traj.getDerivativeNames() == derivativeNames);
        SimTK::Matrix derivativesWithoutAccelerations =
                traj.getDerivativesWithoutAccelerationsTrajectory();
        double error = 0.0;
        for (int irow = 0; irow < derivatives.nrow(); ++irow) {
            error += pow(derivatives(irow, 0) -
                         derivativesWithoutAccelerations(irow, 0), 2);
        }
        SimTK_TEST_EQ(error, 0.0);
    }
    {
        traj.generateAccelerationsFromSpeeds();
        CHECK(traj.getNumDerivatives() == 2);
        CHECK(traj.getDerivativeNames() == derivativeNames);
        SimTK::Matrix derivativesWithoutAccelerations =
                traj.getDerivativesWithoutAccelerationsTrajectory();
        double error = 0.0;
        for (int irow = 0; irow < derivatives.nrow(); ++irow) {
            error += pow(derivatives(irow, 0) -
                         derivativesWithoutAccelerations(irow, 0), 2);
        }
        SimTK_TEST_EQ(error, 0.0);
    }
}

TEST_CASE("Solver isAvailable()") {
#ifdef OPENSIM_WITH_CASADI
    CHECK(MocoCasADiSolver::isAvailable());
#else
    CHECK(!MocoCasADiSolver::isAvailable());
#endif

#ifdef OPENSIM_WITH_TROPTER
    CHECK(MocoTropterSolver::isAvailable());
#else
    CHECK(!MocoTropterSolver::isAvailable());
#endif
}


/*
TEMPLATE_TEST_CASE("Controllers in the model", "",
        MocoCasADiSolver, MocoTropterSolver) {
    MocoStudy study;
    auto& problem = study.updProblem();
    auto model = createSlidingMassModel();
    auto* controller = new PrescribedController();
    controller->addActuator(model->getComponent<Actuator>("actuator"));
    controller->prescribeControlForActuator("actuator", new Constant(0.4));
    model->addController(controller);
    problem.setModel(std::move(model));
    problem.setTimeBounds(0, {0, 10});
    problem.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
    problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
    problem.addGoal<MocoFinalTimeGoal>();

    auto& solver = study.initSolver<TestType>();
    solver.set_num_mesh_points(20);
    MocoSolution solution = study.solve();
    std::cout << "DEBUG " << solution.getControl("/actuator") << std::endl;

}
*/
