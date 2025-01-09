/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxMuscle.cpp                                            *
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

// Some of this code is based on testSingleMuscle,
// testSingleMuscleDeGrooteFregly2016.

#include <Moco/Components/DeGrooteFregly2016Muscle.h>
#include <Moco/osimMoco.h>

#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

using namespace OpenSim;

// TODO move into the actual test case.
void testDeGrooteFregly2016Muscle() {

    Model model;
    auto* musclePtr = new DeGrooteFregly2016Muscle();
    musclePtr->setName("muscle");
    musclePtr->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    musclePtr->addNewPathPoint(
            "insertion", model.updGround(), SimTK::Vec3(1.0, 0, 0));
    model.addComponent(musclePtr);
    auto& muscle = model.getComponent<DeGrooteFregly2016Muscle>("muscle");

    // Property value bounds
    // ---------------------
    {
        DeGrooteFregly2016Muscle musc = muscle;
        musc.set_optimal_force(1.5);
        SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(), Exception);
    }
    {
        DeGrooteFregly2016Muscle musc = muscle;
        musc.set_default_norm_fiber_length(0.1999);
        SimTK_TEST_MUST_THROW_EXC(
                musc.finalizeFromProperties(), SimTK::Exception::ErrorCheck);
    }
    {
        DeGrooteFregly2016Muscle musc = muscle;
        musc.set_default_norm_fiber_length(1.800001);
        SimTK_TEST_MUST_THROW_EXC(
                musc.finalizeFromProperties(), SimTK::Exception::ErrorCheck);
    }
    {
        DeGrooteFregly2016Muscle musc = muscle;
        musc.set_activation_time_constant(0);
        SimTK_TEST_MUST_THROW_EXC(
                musc.finalizeFromProperties(), SimTK::Exception::ErrorCheck);
    }
    {
        DeGrooteFregly2016Muscle musc = muscle;
        musc.set_deactivation_time_constant(0);
        SimTK_TEST_MUST_THROW_EXC(
                musc.finalizeFromProperties(), SimTK::Exception::ErrorCheck);
    }
    {
        DeGrooteFregly2016Muscle musc = muscle;
        musc.set_default_activation(-0.0001);
        SimTK_TEST_MUST_THROW_EXC(
                musc.finalizeFromProperties(), SimTK::Exception::ErrorCheck);
    }
    {
        DeGrooteFregly2016Muscle musc = muscle;
        musc.set_fiber_damping(-0.0001);
        SimTK_TEST_MUST_THROW_EXC(
                musc.finalizeFromProperties(), SimTK::Exception::ErrorCheck);
    }
    {
        DeGrooteFregly2016Muscle musc = muscle;
        musc.set_tendon_strain_at_one_norm_force(0);
        SimTK_TEST_MUST_THROW_EXC(
                musc.finalizeFromProperties(), SimTK::Exception::ErrorCheck);
    }

    // printCurvesToSTOFiles()
    // -----------------------
    muscle.printCurvesToSTOFiles();

    printMessage("%f %f %f %f %f %f\n", muscle.calcTendonForceMultiplier(1),
            muscle.calcPassiveForceMultiplier(1),
            muscle.calcActiveForceLengthMultiplier(1),
            muscle.calcForceVelocityMultiplier(-1),
            muscle.calcForceVelocityMultiplier(0),
            muscle.calcForceVelocityMultiplier(1));

    // Test that the force-velocity curve inverse is correct.
    // ------------------------------------------------------
    const auto normFiberVelocity = createVectorLinspace(100, -1, 1);
    for (int i = 0; i < normFiberVelocity.nrow(); ++i) {
        const SimTK::Real& vMTilde = normFiberVelocity[i];
        SimTK_TEST_EQ(muscle.calcForceVelocityInverseCurve(
                              muscle.calcForceVelocityMultiplier(vMTilde)),
                vMTilde);
    }

    // solveBisection().
    // -----------------
    {
        auto calcResidual = [](const SimTK::Real& x) { return x - 3.78; };
        {
            const auto root =
                    muscle.solveBisection(calcResidual, -5, 5, 1e-6, 1e-12);
            SimTK_TEST_EQ_TOL(root, 3.78, 1e-6);
            // Make sure the x tolerance has an effect.
            SimTK_TEST_NOTEQ_TOL(root, 3.78, 1e-10);
        }
        {
            const auto root =
                    muscle.solveBisection(calcResidual, -5, 5, 1e-10, 1e-12);
            SimTK_TEST_EQ_TOL(root, 3.78, 1e-10);
        }
        // Make sure the y tolerance has an effect.
        {
            const auto root =
                    muscle.solveBisection(calcResidual, -5, 5, 1e-12, 1e-4);
            const auto residual = calcResidual(root);
            SimTK_TEST_EQ_TOL(residual, 0, 1e-4);
            // Make sure the x tolerance has an effect.
            SimTK_TEST_NOTEQ_TOL(residual, 0, 1e-10);
        }
        {
            const auto root =
                    muscle.solveBisection(calcResidual, -5, 5, 1e-12, 1e-10);
            const auto residual = calcResidual(root);
            SimTK_TEST_EQ_TOL(residual, 0, 1e-10);
        }
    }
    {
        auto parabola = [](const SimTK::Real& x) {
            return SimTK::square(x - 2.5);
        };
        SimTK_TEST_MUST_THROW_EXC(
                muscle.solveBisection(parabola, -5, 5), Exception);
    }

    SimTK::State state = model.initSystem();

    // getActivation(), setActivation()
    // --------------------------------
    SimTK_TEST_EQ(muscle.getActivation(state), muscle.get_default_activation());
    muscle.setActivation(state, 0.451);
    SimTK_TEST_EQ(muscle.getActivation(state), 0.451);
    // This model only has the muscle states, so activation is index 0.
    SimTK_TEST_EQ(state.getY()[0], 0.451);
}

Model createHangingMuscleModel(
        bool ignoreActivationDynamics, bool ignoreTendonCompliance) {
    Model model;
    model.setName("isometric_muscle");
    model.set_gravity(SimTK::Vec3(9.81, 0, 0));
    auto* body = new Body("body", 0.5, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("joint", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("height");
    model.addComponent(joint);

    auto* actu = new DeGrooteFregly2016Muscle();
    actu->setName("actuator");
    actu->set_max_isometric_force(30.0);
    actu->set_optimal_fiber_length(0.10);
    actu->set_tendon_slack_length(0.05);
    actu->set_tendon_strain_at_one_norm_force(0.10);
    actu->set_ignore_activation_dynamics(ignoreActivationDynamics);
    actu->set_ignore_tendon_compliance(ignoreTendonCompliance);
    actu->set_max_contraction_velocity(10);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addForce(actu);

    /*
    auto* actu = new Millard2012EquilibriumMuscle();
    // TODO actu->set_fiber_damping(0);
    actu->setName("actuator");
    actu->set_max_isometric_force(30.0);
    actu->set_optimal_fiber_length(0.10);
    actu->set_ignore_tendon_compliance(ignoreTendonCompliance);
    actu->set_tendon_slack_length(0.05);
    actu->set_pennation_angle_at_optimal(0); // TODO 0.1);
    actu->set_max_contraction_velocity(10);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addForce(actu);
    */

    /*
    auto* actu = new ActivationCoordinateActuator();
    actu->setName("actuator");
    actu->setCoordinate(&coord);
    actu->set_activation_time_constant(0.001);
    actu->set_optimal_force(30);
    model.addComponent(actu);

    auto* contr = new PrescribedController();
    contr->setName("controller");
    contr->addActuator(*actu);
    contr->prescribeControlForActuator("actuator", new Constant(1.0));
    model.addComponent(contr);
    */

    body->attachGeometry(new Sphere(0.05));

    return model;
}

template <typename SolverType>
void testHangingMuscleMinimumTime(
        bool ignoreActivationDynamics, bool ignoreTendonCompliance) {

    std::cout << "testHangingMuscleMinimumTime. "
              << "ignoreTendonCompliance: " << ignoreTendonCompliance << "."
              << std::endl;

    SimTK::Real initActivation = 0.01;
    SimTK::Real initHeight = 0.15;
    SimTK::Real finalHeight = 0.14;

    Model model = createHangingMuscleModel(
            ignoreActivationDynamics, ignoreTendonCompliance);

    // Passive forward simulation.
    // ---------------------------
    // auto* controller = new PrescribedController();
    // controller->addActuator(model.getComponent<Actuator>("actuator"));
    // controller->prescribeControlForActuator("actuator", new Constant(0.10));
    // model.addController(controller);
    // model.finalizeFromProperties();
    //
    // TableReporter* rep = new TableReporter();
    // rep->addToReport(model.getComponent("actuator/geometrypath").getOutput("length"));
    // rep->addToReport(model.getComponent("actuator").getOutput("normalized_fiber_length"));
    // rep->addToReport(model.getComponent("actuator").getOutput("normalized_fiber_velocity"));
    // //
    // rep->addToReport(model.getComponent("actuator").getOutput("fiber_velocity"));
    // rep->addToReport(model.getComponent("actuator").getOutput("force_velocity_multiplier"));
    // rep->set_report_time_interval(0.001);
    // model.addComponent(rep);

    SimTK::State state = model.initSystem();
    const auto& actuator = model.getComponent("forceset/actuator");

    const auto* muscle = dynamic_cast<const Muscle*>(&actuator);
    const auto* dgf = dynamic_cast<const DeGrooteFregly2016Muscle*>(&actuator);
    const bool usingDGF = dgf != nullptr;

    if (!ignoreTendonCompliance) {

        model.setStateVariableValue(state, "joint/height/value", initHeight);
        model.realizeVelocity(state);
        muscle->setActivation(state, initActivation);
        model.equilibrateMuscles(state);
        if (usingDGF) {
            std::cout << "Equilibrium norm fiber length: "
                      << model.getStateVariableValue(
                                 state, "forceset/actuator/norm_fiber_length")
                      << std::endl;
        } else {
            std::cout << "Equilibrium fiber length: "
                      << model.getStateVariableValue(
                                 state, "forceset/actuator/fiber_length")
                      << std::endl;
        }
    }
    //
    // Manager manager(model, state);
    // manager.getIntegrator().setAccuracy(1e-10);
    // manager.getIntegrator().setMaximumStepSize(0.001);
    // manager.integrate(2.0);
    // // TODO STOFileAdapter::write(rep->getTable(),
    // "DEBUG_sandboxMuscle_reporter.sto"); visualize(model,
    // manager.getStateStorage());

    // Minimum time trajectory optimization.
    // -------------------------------------
    const auto svn = model.getStateVariableNames();
    for (int i = 0; i < svn.size(); ++i) { std::cout << svn[i] << std::endl; }
    MocoSolution solutionTrajOpt;
    {
        MocoStudy study;
        MocoProblem& problem = study.updProblem();
        problem.setModelCopy(model);
        problem.setTimeBounds(0, {0.05, 1.0});
        // TODO this might have been the culprit when using the Millard muscle:
        // TODO TODO TODO
        problem.setStateInfo(
                "/joint/height/value", {0.10, 0.16}, initHeight, finalHeight);
        problem.setStateInfo("/joint/height/speed", {-10, 10}, 0, 0);
        // TODO initial fiber length?
        // TODO how to enforce initial equilibrium with explicit dynamics?
        if (!ignoreTendonCompliance) {
            if (usingDGF) {
                // We would prefer to use a range of [0.2, 1.8] but then IPOPT
                // tries very small fiber lengths that cause tendon stretch to
                // be HUGE, causing insanely high tendon forces.
                problem.setStateInfo("/forceset/actuator/norm_fiber_length",
                        {0.8, 1.8},
                        model.getStateVariableValue(
                                state, "forceset/actuator/norm_fiber_length"));
            } else {
                problem.setStateInfo("forceset/actuator/fiber_length",
                        {0.0, 0.3},
                        model.getStateVariableValue(
                                state, "forceset/actuator/fiber_length"));
            }
        }
        // OpenSim might not allow activations of 0.
        if (!ignoreActivationDynamics) {
            problem.setStateInfo(
                    "/forceset/actuator/activation", {0.01, 1}, initActivation);
        }
        problem.setControlInfo("/forceset/actuator", {0.01, 1});

        problem.addGoal<MocoFinalTimeGoal>();

        auto& solver = study.initSolver<SolverType>();
        solver.set_num_mesh_intervals(20);
        solver.set_multibody_dynamics_mode("implicit");
        solver.set_optim_convergence_tolerance(1e-4);
        solver.set_optim_constraint_tolerance(1e-3);
        // TODO if compliant tendon, use rigid tendon as initial guess.
        // solver.set_optim_sparsity_detection("initial-guess");
        MocoTrajectory guessForwardSim = solver.createGuess("time-stepping");
        // solver.setGuess(guessForwardSim);
        guessForwardSim.write("sandboxMuscle_guess_forward_sim.sto");
        std::cout << "Guess from forward sim: "
                  << guessForwardSim.getStatesTrajectory() << std::endl;
        // study.visualize(guessForwardSim);

        solutionTrajOpt = moco.solve();
        std::string solutionFilename = "sandboxMuscle_solution";
        if (ignoreTendonCompliance) solutionFilename += "_rigidtendon";
        solutionFilename += ".sto";
        solutionTrajOpt.write(solutionFilename);
        std::cout << "Solution joint/height/value trajectory: "
                  << solutionTrajOpt.getState("/joint/height/value")
                  << std::endl;
        std::cout << "Solution joint/height/speed trajectory: "
                  << solutionTrajOpt.getState("/joint/height/speed")
                  << std::endl;
    }

    // Perform time stepping forward simulation using optimized controls.
    // ------------------------------------------------------------------
    // See if we end up at the correct final state.
    {
        const auto iterateSim =
                simulateTrajectoryWithTimeStepping(solutionTrajOpt, model);
        const double error = iterateSim.compareContinuousVariablesRMS(
                solutionTrajOpt, {{"states", {}}, {"controls", {}}});
        SimTK_TEST(error < 0.05);
    }

    // Track the kinematics from the trajectory optimization.
    // ------------------------------------------------------
    // We will try to recover muscle activity.
    {
        std::cout << "Tracking the trajectory optimization coordinate solution."
                  << std::endl;
        MocoStudy study;
        MocoProblem& problem = study.updProblem();
        problem.setModelCopy(model);
        // Using an equality constraint for the time bounds was essential for
        // recovering the correct excitation.
        const double finalTime =
                solutionTrajOpt.getTime()[solutionTrajOpt.getNumTimes() - 1];
        const double slop = 0; // TODO 1e-4;
        problem.setTimeBounds(0 + slop, finalTime - slop);
        problem.setStateInfo("/joint/height/value",
                {0.10, 0.16}); // , initHeight, finalHeight);
        problem.setStateInfo("/joint/height/speed", {-10, 10}); // , 0, 0);
        if (!ignoreTendonCompliance) {
            if (usingDGF) {
                // We would prefer to use a range of [0.2, 1.8] but then IPOPT
                // tries very small fiber lengths that cause tendon stretch to
                // be HUGE, causing insanely high tendon forces.
                problem.setStateInfo("/forceset/actuator/norm_fiber_length",
                        {0.8, 1.8},
                        model.getStateVariableValue(
                                state, "forceset/actuator/norm_fiber_length"));
            } else {
                problem.setStateInfo("/forceset/actuator/fiber_length",
                        {0.0, 0.3},
                        model.getStateVariableValue(
                                state, "forceset/actuator/fiber_length"));
            }
        }
        // OpenSim might not allow activations of 0.
        if (!ignoreActivationDynamics) {
            problem.setStateInfo(
                    "/forceset/actuator/activation", {0.01, 1}, initActivation);
        }
        problem.setControlInfo("/forceset/actuator", {0.01, 1});

        auto* tracking = problem.addGoal<MocoStateTrackingCost>();

        auto states = solutionTrajOpt.exportToStatesStorage().exportToTable();
        TimeSeriesTable ref(states.getIndependentColumn());
        ref.appendColumn("/joint/height/value",
                states.getDependentColumn("/joint/height/value"));
        // Tracking speed has a huge effect on getting a good solution for the
        // control signal.
        ref.appendColumn("/joint/height/speed",
                states.getDependentColumn("/joint/height/speed"));
        // Tracking joint/height/speed slightly increases the
        // iterations to converge, and tracking activation cuts the iterations
        // in half.
        // TODO try tracking all states, for fun.
        tracking->setReference(ref);
        tracking->setAllowUnusedReferences(true);

        auto& solver = moco.initSolver<SolverType>();
        solver.set_num_mesh_intervals(20);
        solver.set_multibody_dynamics_mode("implicit");
        // solver.set_optim_convergence_tolerance(1e-3);
        // solver.set_optim_constraint_tolerance(1e-3);
        // solver.set_optim_sparsity_detection("initial-guess");
        // solver.set_optim_hessian_approximation("exact");
        // solver.set_optim_finite_difference_scheme("forward");
        // solver.setGuess("time-stepping");
        // Don't need to use the TrajOpt solution as the initial guess; kinda
        // neat. Although, using TrajOpt for the guess improves convergence.
        // TODO solver.setGuess(solutionTrajOpt);

        MocoSolution solutionTrack = study.solve();
        std::string solutionFilename = "sandboxMuscle_track_solution";
        if (ignoreTendonCompliance) solutionFilename += "_rigidtendon";
        solutionFilename += ".sto";
        solutionTrack.write(solutionFilename);
        double error =
                solutionTrack.compareContinuousVariablesRMS(solutionTrajOpt);
        std::cout << "RMS error for states and controls: " << error
                  << std::endl;
    }

    // TODO perform the tracking with INDYGO.

    // TODO support constraining initial fiber lengths to their equilibrium
    // lengths in Tropter!!!!!!!!!!!!!! (in explicit mode).
}

int main() {

    testDeGrooteFregly2016Muscle();

    testHangingMuscleMinimumTime<MocoCasADiSolver>(true, true);
    // testHangingMuscleMinimumTime<MocoCasADiSolver>(true, false);
    testHangingMuscleMinimumTime<MocoCasADiSolver>(false, true);
    // testHangingMuscleMinimumTime<MocoCasADiSolver>(false, false);

    return EXIT_SUCCESS;
}
