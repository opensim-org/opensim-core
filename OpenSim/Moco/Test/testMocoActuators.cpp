/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoActuators.cpp                                        *
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

#include <OpenSim/Actuators/ActivationCoordinateActuator.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/CMCTool.h>
#include "OpenSim/Tools/CMC_TaskSet.h"

#define CATCH_CONFIG_MAIN
#include "Testing.h"
#include "OpenSim/Tools/CMC_Joint.h"

using namespace OpenSim;

Model createHangingMuscleModel(double optimalFiberLength,
        double tendonSlackLength, bool ignoreActivationDynamics,
        bool ignoreTendonCompliance, bool isTendonDynamicsExplicit) {
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
    actu->setName("muscle");
    actu->set_max_isometric_force(100.0);
    actu->set_optimal_fiber_length(optimalFiberLength);
    actu->set_tendon_slack_length(tendonSlackLength);
    actu->set_tendon_strain_at_one_norm_force(0.10);
    actu->set_ignore_activation_dynamics(ignoreActivationDynamics);
    actu->set_ignore_tendon_compliance(ignoreTendonCompliance);
    actu->set_fiber_damping(0.01);
    if (!isTendonDynamicsExplicit) {
        actu->set_tendon_compliance_dynamics_mode("implicit");
    }
    actu->set_max_contraction_velocity(10);
    actu->set_pennation_angle_at_optimal(0);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addForce(actu);

    body->attachGeometry(new Sphere(0.05));

    CMC_TaskSet tasks;
    CMC_Joint task;
    task.setName(coord.getName());
    task.setCoordinateName(coord.getName());
    task.setKP(100, 1, 1);
    task.setKV(20, 1, 1);
    task.setActive(true, false, false);
    tasks.cloneAndAppend(task);
    tasks.print("hanging_muscle_cmc_tasks.xml");

    return model;
}

TEMPLATE_TEST_CASE(
        "Hanging muscle minimum time", "[casadi]", MocoCasADiSolver) {
    auto ignoreActivationDynamics = GENERATE(true, false);
    auto ignoreTendonCompliance = GENERATE(true, false);
    auto isTendonDynamicsExplicit = GENERATE(true, false);

    CAPTURE(ignoreActivationDynamics);
    CAPTURE(ignoreTendonCompliance);
    CAPTURE(isTendonDynamicsExplicit);

    const double optimalFiberLength = 0.1;
    const double tendonSlackLength = 0.05;
    SimTK::Real initHeight = 0.165;
    SimTK::Real finalHeight = 0.155;
    MocoBounds heightBounds(0.14, 0.17);
    MocoBounds speedBounds(-10, 10);
    MocoBounds actuBounds(0.02, 1);

    Model model = createHangingMuscleModel(optimalFiberLength,
            tendonSlackLength, ignoreActivationDynamics, ignoreTendonCompliance,
            isTendonDynamicsExplicit);

    SimTK::State state = model.initSystem();

    // Minimum time trajectory optimization.
    // -------------------------------------
    const auto svn = model.getStateVariableNames();
    MocoSolution solutionTrajOpt;
    std::string solutionFilename;
    {
        MocoStudy study;
        MocoProblem& problem = study.updProblem();
        problem.setModelAsCopy(model);
        problem.setTimeBounds(0, 0.5);
        problem.setStateInfo(
                "/joint/height/value", heightBounds, initHeight, finalHeight);
        problem.setStateInfo("/joint/height/speed", speedBounds, 0, 0);
        problem.setControlInfo("/forceset/muscle", actuBounds);

        auto* initial_equilibrium =
                problem.addGoal<MocoInitialForceEquilibriumDGFGoal>();
        initial_equilibrium->setName("initial_equilibrium");

        problem.addGoal<MocoControlGoal>("effort");

        auto& solver = study.initSolver<TestType>();
        solver.set_num_mesh_intervals(30);
        solver.set_multibody_dynamics_mode("explicit");
        solver.set_optim_convergence_tolerance(1e-4);
        solver.set_optim_constraint_tolerance(1e-4);
        solver.set_minimize_implicit_auxiliary_derivatives(true);

        solutionTrajOpt = study.solve();
        solutionFilename = "testDeGrooteFregly2016Muscle_solution";
        if (!ignoreActivationDynamics) solutionFilename += "_actdyn";
        if (ignoreTendonCompliance) solutionFilename += "_rigidtendon";
        if (isTendonDynamicsExplicit) solutionFilename += "_exptendyn";

        std::vector<std::string> outputPaths;
        outputPaths.emplace_back(".*tendon_force.*");
        outputPaths.emplace_back(".*fiber_force_along_tendon.*");
        outputPaths.emplace_back(".*length.*");
        outputPaths.emplace_back(".*velocity.*");
        auto table = study.analyze(solutionTrajOpt, outputPaths);
        STOFileAdapter::write(table, solutionFilename + "_outputs.sto");

        solutionFilename += ".sto";
        solutionTrajOpt.write(solutionFilename);
    }

    // Perform time stepping forward simulation using optimized controls.
    // ------------------------------------------------------------------
    // See if we end up at the correct final state.
    {
        // We need to temporarily switch the muscle in the model to explicit
        // tendon compliance dynamics mode to perform time stepping.
        auto* mutableDGFMuscle = dynamic_cast<DeGrooteFregly2016Muscle*>(
                &model.updComponent("forceset/muscle"));
        if (!ignoreTendonCompliance && !isTendonDynamicsExplicit) {
            mutableDGFMuscle->set_tendon_compliance_dynamics_mode("explicit");
        }
        const auto trajSim =
                simulateTrajectoryWithTimeStepping(solutionTrajOpt, model);
        std::string trajFilename = "testDeGrooteFregly2016Muscle_timestepping";
        if (!ignoreActivationDynamics) trajFilename += "_actdyn";
        if (ignoreTendonCompliance) trajFilename += "_rigidtendon";
        if (isTendonDynamicsExplicit) trajFilename += "_exptendyn";
        trajFilename += ".sto";
        trajSim.write(trajFilename);

        const double error = trajSim.compareContinuousVariablesRMS(
                solutionTrajOpt, {{"states", {}}, {"controls", {}}});
        CHECK(error < 0.01);
        if (!ignoreTendonCompliance && !isTendonDynamicsExplicit) {
            mutableDGFMuscle->set_tendon_compliance_dynamics_mode("implicit");
        }
    }

    // Solve problem again using CMC.
    // ------------------------------
    // See if we get the correct muscle activity.
    {
        Model modelCMC =
                createHangingMuscleModel(optimalFiberLength, tendonSlackLength,
                        ignoreActivationDynamics, ignoreTendonCompliance, true);

        // Need a reserve for CMC to solve.
        auto* actu = new CoordinateActuator();
        actu->setName("actuator");
        actu->setOptimalForce(0.1);
        actu->setMinControl(-100);
        actu->setMaxControl(100);
        actu->setCoordinate(&modelCMC.getCoordinateSet().get(0));
        modelCMC.addForce(actu);
        modelCMC.finalizeConnections();

        // Need to set the initial state of the model so that the
        // initial equilibrium solve converges.
        modelCMC.updCoordinateSet().get("height").setDefaultValue(
                solutionTrajOpt.getState("/joint/height/value")[0]);
        modelCMC.updCoordinateSet().get("height").setDefaultSpeedValue(
                solutionTrajOpt.getState("/joint/height/speed")[0]);
        auto* mutableDGFMuscleCMC = dynamic_cast<DeGrooteFregly2016Muscle*>(
                &modelCMC.updComponent("forceset/muscle"));
        if (!ignoreActivationDynamics) {
            mutableDGFMuscleCMC->set_default_activation(
                    solutionTrajOpt.getState("/forceset/muscle/activation")[0]);
        }
        if (!ignoreTendonCompliance) {
            mutableDGFMuscleCMC->set_default_normalized_tendon_force(
                    solutionTrajOpt.getState(
                            "/forceset/muscle/normalized_tendon_force")[0]);
        }

        // Run CMC
        // -------
        CMCTool cmc;
        std::string cmcFilename = "testDeGrooteFregly2016Muscle_cmc";
        cmc.setResultsDir(cmcFilename);
        if (!ignoreActivationDynamics) cmcFilename += "_actdyn";
        if (ignoreTendonCompliance) cmcFilename += "_rigidtendon";
        if (isTendonDynamicsExplicit) cmcFilename += "_exptendyn";
        cmc.setName(cmcFilename);
        cmc.setModel(modelCMC);
        cmc.setInitialTime(0);
        const double finalTime =
                solutionTrajOpt.getTime()[solutionTrajOpt.getNumTimes() - 1];
        cmc.setFinalTime(finalTime);
        cmc.setDesiredKinematicsFileName(solutionFilename);
        cmc.setTaskSetFileName("hanging_muscle_cmc_tasks.xml");
        cmc.setSolveForEquilibrium(false);
        cmc.setTimeWindow(0.01);
        cmc.setUseFastTarget(true);
        cmc.run();

        // Create a MocoTrajectory from the CMC solution.
        TimeSeriesTable cmcStates;
        std::vector<std::string> stateColumnLabels;
        cmcStates = TimeSeriesTable("testDeGrooteFregly2016Muscle_cmc/" +
                                    cmcFilename + "_states.sto");
        stateColumnLabels = cmcStates.getColumnLabels();

        TimeSeriesTable cmcControls("testDeGrooteFregly2016Muscle_cmc/" +
                                    cmcFilename + "_controls.sto");
        const auto& stdTime = cmcControls.getIndependentColumn();
        SimTK::Vector time((int)stdTime.size());
        for (int i = 0; i < time.size(); i++) { time[i] = stdTime[i]; }
        std::vector<std::string> controlNames{"/forceset/muscle"};
        MocoTrajectory cmcTraj(time, stateColumnLabels, controlNames, {}, {},
                cmcStates.getMatrix(),
                cmcControls.getMatrixBlock(0, 0, cmcControls.getNumRows(), 1),
                SimTK::Matrix(), SimTK::RowVector());
        cmcTraj.write(
                "testDeGrooteFregly2016Muscle_cmc/" + cmcFilename + ".sto");

        // Compare CMC solution to the Moco solution.
        std::vector<std::string> states;
        if (!ignoreActivationDynamics) {
            states.push_back("/forceset/muscle/activation");
        }
        if (!ignoreTendonCompliance) {
            states.push_back("/forceset/muscle/normalized_tendon_force");
        }
        std::vector<std::string> controls{"/forceset/muscle"};
        const double error = cmcTraj.compareContinuousVariablesRMS(
                solutionTrajOpt, {{"states", states}, {"controls", controls}});
        CHECK(error < 0.05);
    }

    // Track the kinematics from the trajectory optimization.
    // ------------------------------------------------------
    // We will try to recover muscle activity.
    if (!isTendonDynamicsExplicit) {
        std::cout << "Tracking the trajectory optimization coordinate solution."
                  << std::endl;
        MocoStudy study;
        MocoProblem& problem = study.updProblem();
        problem.setModelAsCopy(model);
        // Using an equality constraint for the time bounds was essential for
        // recovering the correct excitation.
        const double finalTime =
                solutionTrajOpt.getTime()[solutionTrajOpt.getNumTimes() - 1];
        problem.setTimeBounds(0, finalTime);
        problem.setStateInfo(
                "/joint/height/value", heightBounds, initHeight, finalHeight);
        problem.setStateInfo("/joint/height/speed", speedBounds, 0, 0);
        problem.setControlInfo("/forceset/muscle", actuBounds);

        auto* initial_equilibrium =
                problem.addGoal<MocoInitialForceEquilibriumDGFGoal>();
        initial_equilibrium->setName("initial_equilibrium");

        auto* tracking = problem.addGoal<MocoStateTrackingGoal>("tracking");

        auto states = solutionTrajOpt.exportToStatesTable();
        TimeSeriesTable ref(states.getIndependentColumn());
        ref.addTableMetaData("inDegrees", std::string("no"));
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

        auto& solver = study.initSolver<TestType>();
        solver.set_num_mesh_intervals(30);
        solver.set_minimize_implicit_auxiliary_derivatives(true);

        MocoSolution solutionTrack = study.solve();
        std::string solutionFilename =
                "testDeGrooteFregly2016Muscle_track_solution";
        if (!ignoreActivationDynamics) solutionFilename += "_actdyn";
        if (ignoreTendonCompliance) solutionFilename += "_rigidtendon";
        if (isTendonDynamicsExplicit) solutionFilename += "_exptendyn";
        solutionFilename += ".sto";
        solutionTrack.write(solutionFilename);
        double error =
                solutionTrack.compareContinuousVariablesRMS(solutionTrajOpt);
        CHECK(error < 0.01);
    }
}

TEST_CASE("ActivationCoordinateActuator") {
    // Create a problem with ACA and ensure the activation bounds are
    // set as expected.
    auto model = ModelFactory::createSlidingPointMass();
    auto* actu = new ActivationCoordinateActuator();
    actu->setName("aca");
    actu->setCoordinate(&model.updCoordinateSet().get("position"));
    actu->setMinControl(-0.31);
    actu->setMaxControl(0.78);
    model.addForce(actu);
    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);
    auto rep = problem.createRep();
    CHECK(rep.getStateInfo("/forceset/aca/activation").getBounds().getLower() ==
            Approx(-0.31));
    CHECK(rep.getStateInfo("/forceset/aca/activation").getBounds().getUpper() ==
            Approx(0.78));
}
