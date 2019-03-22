/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testDeGrooteFregly2016Muscle.cpp                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include <Moco/osimMoco.h>

#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/LogManager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

#define CATCH_CONFIG_MAIN
#include "Testing.h"

using namespace OpenSim;

TEST_CASE("DeGrooteFregly2016Muscle basics") {

    Model model;
    auto* musclePtr = new DeGrooteFregly2016Muscle();
    musclePtr->setName("muscle");
    musclePtr->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    musclePtr->addNewPathPoint(
            "insertion", model.updGround(), SimTK::Vec3(1.0, 0, 0));
    model.addComponent(musclePtr);
    auto& muscle = model.getComponent<DeGrooteFregly2016Muscle>("muscle");

    SECTION("Property value bounds") {

        // Property value bounds
        // ---------------------
        {
            DeGrooteFregly2016Muscle musc = muscle;
            musc.set_optimal_force(1.5);
            SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(), Exception);
        }
        {
            DeGrooteFregly2016Muscle musc = muscle;
            musc.set_default_normalized_fiber_length(0.1999);
            SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        {
            DeGrooteFregly2016Muscle musc = muscle;
            musc.set_default_normalized_fiber_length(1.800001);
            SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        {
            DeGrooteFregly2016Muscle musc = muscle;
            musc.set_activation_time_constant(0);
            SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        {
            DeGrooteFregly2016Muscle musc = muscle;
            musc.set_deactivation_time_constant(0);
            SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        {
            DeGrooteFregly2016Muscle musc = muscle;
            musc.set_default_activation(-0.0001);
            SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        {
            DeGrooteFregly2016Muscle musc = muscle;
            musc.set_active_force_width_scale(0.99999999);
            SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        {
            DeGrooteFregly2016Muscle musc = muscle;
            musc.set_fiber_damping(-0.0001);
            SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        {
            DeGrooteFregly2016Muscle musc = muscle;
            musc.set_tendon_strain_at_one_norm_force(0);
            SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
    }

    SECTION("printCurvesToSTOFiles") {

        // printCurvesToSTOFiles()
        // -----------------------
        muscle.printCurvesToSTOFiles();

        printMessage("%f %f %f %f %f %f\n", muscle.calcTendonForceMultiplier(1),
                muscle.calcPassiveForceMultiplier(1),
                muscle.calcActiveForceLengthMultiplier(1),
                muscle.calcForceVelocityMultiplier(-1),
                muscle.calcForceVelocityMultiplier(0),
                muscle.calcForceVelocityMultiplier(1));
    }

    SECTION("Force-velocity curve inverse") {

        // Test that the force-velocity curve inverse is correct.
        // ------------------------------------------------------
        const auto normFiberVelocity = createVectorLinspace(100, -1, 1);
        for (int i = 0; i < normFiberVelocity.nrow(); ++i) {
            const SimTK::Real& vMTilde = normFiberVelocity[i];
            CHECK(muscle.calcForceVelocityInverseCurve(
                          muscle.calcForceVelocityMultiplier(vMTilde)) ==
                    Approx(vMTilde));
        }
    }

    SECTION("solveBisection()") {

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
                const auto root = muscle.solveBisection(
                        calcResidual, -5, 5, 1e-10, 1e-12);
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
                const auto root = muscle.solveBisection(
                        calcResidual, -5, 5, 1e-12, 1e-10);
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
        SimTK_TEST_EQ(
                muscle.getActivation(state), muscle.get_default_activation());
        muscle.setActivation(state, 0.451);
        SimTK_TEST_EQ(muscle.getActivation(state), 0.451);
        // This model only has the muscle states, so activation is index 0.
        SimTK_TEST_EQ(state.getY()[0], 0.451);
    }
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

    body->attachGeometry(new Sphere(0.05));

    return model;
}

template <typename SolverType>
void testHangingMuscleMinimumTime(
        bool ignoreActivationDynamics, bool ignoreTendonCompliance) {

    CAPTURE(ignoreActivationDynamics);
    CAPTURE(ignoreTendonCompliance);

    SimTK::Real initActivation = 0.01;
    SimTK::Real initHeight = 0.15;
    SimTK::Real finalHeight = 0.14;

    Model model = createHangingMuscleModel(
            ignoreActivationDynamics, ignoreTendonCompliance);

    SimTK::State state = model.initSystem();
    const auto& actuator = model.getComponent("forceset/actuator");

    const auto* muscle = dynamic_cast<const Muscle*>(&actuator);

    if (!ignoreTendonCompliance) {
        model.setStateVariableValue(state, "joint/height/value", initHeight);
        model.realizeVelocity(state);
        muscle->setActivation(state, initActivation);
        model.equilibrateMuscles(state);
        std::cout << "Equilibrium norm fiber length: "
                  << model.getStateVariableValue(
                             state, "forceset/actuator/norm_fiber_length")
                  << std::endl;
    }

    // Minimum time trajectory optimization.
    // -------------------------------------
    const auto svn = model.getStateVariableNames();
    MocoSolution solutionTrajOpt;
    {
        MocoTool moco;
        MocoProblem& problem = moco.updProblem();
        problem.setModelCopy(model);
        problem.setTimeBounds(0, {0.05, 1.0});
        problem.setStateInfo(
                "/joint/height/value", {0.10, 0.16}, initHeight, finalHeight);
        problem.setStateInfo("/joint/height/speed", {-10, 10}, 0, 0);
        // TODO initial fiber length?
        // TODO how to enforce initial equilibrium with explicit dynamics?
        if (!ignoreTendonCompliance) {
            // We would prefer to use a range of [0.2, 1.8] but then IPOPT
            // tries very small fiber lengths that cause tendon stretch to
            // be HUGE, causing insanely high tendon forces.
            problem.setStateInfo("/forceset/actuator/norm_fiber_length",
                    {0.8, 1.8},
                    model.getStateVariableValue(
                            state, "forceset/actuator/norm_fiber_length"));
        }
        // OpenSim might not allow activations of 0.
        if (!ignoreActivationDynamics) {
            problem.setStateInfo(
                    "/forceset/actuator/activation", {0.01, 1}, initActivation);
        }
        problem.setControlInfo("/forceset/actuator", {0.01, 1});

        problem.addCost<MocoFinalTimeCost>();

        auto& solver = moco.initSolver<SolverType>();
        solver.set_num_mesh_points(20);
        solver.set_dynamics_mode("implicit");
        solver.set_optim_convergence_tolerance(1e-4);
        solver.set_optim_constraint_tolerance(1e-3);

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
                simulateIterateWithTimeStepping(solutionTrajOpt, model);
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
        MocoTool moco;
        MocoProblem& problem = moco.updProblem();
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
            // We would prefer to use a range of [0.2, 1.8] but then IPOPT
            // tries very small fiber lengths that cause tendon stretch to
            // be HUGE, causing insanely high tendon forces.
            problem.setStateInfo("/forceset/actuator/norm_fiber_length",
                    {0.8, 1.8},
                    model.getStateVariableValue(
                            state, "forceset/actuator/norm_fiber_length"));
        }
        // OpenSim might not allow activations of 0.
        if (!ignoreActivationDynamics) {
            problem.setStateInfo(
                    "/forceset/actuator/activation", {0.01, 1}, initActivation);
        }
        problem.setControlInfo("/forceset/actuator", {0.01, 1});

        auto* tracking = problem.addCost<MocoStateTrackingCost>();

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
        tracking->setReference(ref);
        tracking->setAllowUnusedReferences(true);

        auto& solver = moco.initSolver<SolverType>();
        solver.set_num_mesh_points(20);
        solver.set_dynamics_mode("implicit");

        MocoSolution solutionTrack = moco.solve();
        std::string solutionFilename = "sandboxMuscle_track_solution";
        if (ignoreTendonCompliance) solutionFilename += "_rigidtendon";
        solutionFilename += ".sto";
        solutionTrack.write(solutionFilename);
        double error =
                solutionTrack.compareContinuousVariablesRMS(solutionTrajOpt);
        CHECK(error < 0.01);
    }
    // TODO: Support constraining initial fiber lengths to their equilibrium
    // lengths (in explicit mode).
}

TEST_CASE("Hanging muscle minimum time") {
    testHangingMuscleMinimumTime<MocoCasADiSolver>(true, true);
    testHangingMuscleMinimumTime<MocoCasADiSolver>(false, true);
    // TODO: Handle tendon compliance.
    // testHangingMuscleMinimumTime<MocoCasADiSolver>(true, false);
    // testHangingMuscleMinimumTime<MocoCasADiSolver>(false, false);
}
