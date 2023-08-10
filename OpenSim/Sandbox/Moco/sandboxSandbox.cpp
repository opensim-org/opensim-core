/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
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

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <OpenSim/Moco/osimMoco.h>
#include "OpenSim/Actuators/CoordinateActuator.h"
#include "OpenSim/Common/LinearFunction.h"

using namespace OpenSim;

/// Create a torque-actuated double pendulum model. Each subtest will add to the
/// model the relevant constraint(s).
std::unique_ptr<Model> createDoublePendulumModel() {
    auto model = make_unique<Model>();
    model->setName("double_pendulum");

    using SimTK::Inertia;
    using SimTK::Vec3;

    // Create two links, each with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    auto* b0 = new OpenSim::Body("b0", 1, Vec3(0), Inertia(1));
    model->addBody(b0);
    auto* b1 = new OpenSim::Body("b1", 1, Vec3(0), Inertia(1));
    model->addBody(b1);

    // Add station representing the model end-effector.
    auto* endeff = new Station(*b1, Vec3(0));
    endeff->setName("endeff");
    model->addComponent(endeff);

    // Connect the bodies with pin joints. Assume each body is 1 m long.
    auto* j0 = new PinJoint("j0", model->getGround(), Vec3(0), Vec3(0), *b0,
            Vec3(-1, 0, 0), Vec3(0));
    auto& q0 = j0->updCoordinate();
    q0.setName("q0");
    q0.setDefaultValue(0);
    auto* j1 = new PinJoint(
            "j1", *b0, Vec3(0), Vec3(0), *b1, Vec3(-1, 0, 0), Vec3(0));
    auto& q1 = j1->updCoordinate();
    q1.setName("q1");
    q1.setDefaultValue(SimTK::Pi);
    model->addJoint(j0);
    model->addJoint(j1);

    // Add coordinate actuators.
    auto* tau0 = new CoordinateActuator();
    tau0->setCoordinate(&j0->updCoordinate());
    tau0->setName("tau0");
    tau0->setOptimalForce(1);
    model->addComponent(tau0);
    auto* tau1 = new CoordinateActuator();
    tau1->setCoordinate(&j1->updCoordinate());
    tau1->setName("tau1");
    tau1->setOptimalForce(1);
    model->addComponent(tau1);

    // Add display geometry.
    Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
    SimTK::Transform transform(SimTK::Vec3(-0.5, 0, 0));
    auto* b0Center = new PhysicalOffsetFrame("b0_center", *b0, transform);
    b0->addComponent(b0Center);
    b0Center->attachGeometry(bodyGeometry.clone());
    auto* b1Center = new PhysicalOffsetFrame("b1_center", *b1, transform);
    b1->addComponent(b1Center);
    b1Center->attachGeometry(bodyGeometry.clone());

    return model;
}

/// Run a forward simulation using controls from an OCP solution and compare the
/// state trajectories.
MocoTrajectory runForwardSimulation(
        Model model, const MocoSolution& solution, const double& tol) {

    // Get actuator names.
    model.initSystem();
    OpenSim::Array<std::string> actuNames;
    const auto modelPath = model.getAbsolutePath();
    for (const auto& actu : model.getComponentList<Actuator>()) {
        actuNames.append(actu.getAbsolutePathString());
    }

    // Add prescribed controllers to actuators in the model, where the control
    // functions are splined versions of the actuator controls from the OCP
    // solution.
    const SimTK::Vector& time = solution.getTime();
    auto* controller = new PrescribedController();
    controller->setName("prescribed_controller");
    for (int i = 0; i < actuNames.size(); ++i) {
        const auto control = solution.getControl(actuNames[i]);
        auto* controlFunction =
                new GCVSpline(5, time.nrow(), &time[0], &control[0]);
        const auto& actu = model.getComponent<Actuator>(actuNames[i]);
        controller->addActuator(actu);
        controller->prescribeControlForActuator(
                actu.getName(), controlFunction);
    }
    model.addController(controller);

    // Add states reporter to the model.
    auto* statesRep = new StatesTrajectoryReporter();
    statesRep->setName("states_reporter");
    statesRep->set_report_time_interval(0.001);
    model.addComponent(statesRep);

    // Add a TableReporter to collect the controls.
    auto* controlsRep = new TableReporter();
    for (int i = 0; i < actuNames.size(); ++i) {
        controlsRep->addToReport(
                model.getComponent(actuNames[i]).getOutput("actuation"),
                actuNames[i]);
    }
    model.addComponent(controlsRep);

    // Simulate!
    SimTK::State state = model.initSystem();
    state.setTime(time[0]);
    Manager manager(model);
    manager.getIntegrator().setAccuracy(1e-9);
    manager.initialize(state);
    state = manager.integrate(time[time.size() - 1]);

    // Export results from states reporter to a TimeSeries Table
    TimeSeriesTable states;
    states = statesRep->getStates().exportToTable(model);

    TimeSeriesTable controls;
    controls = controlsRep->getTable();

    // Create a MocoTrajectory to facilitate states trajectory comparison (with
    // dummy data for the multipliers, which we'll ignore).
    const auto& statesTimes = states.getIndependentColumn();
    SimTK::Vector timeVec((int)statesTimes.size(), statesTimes.data(), true);
    auto forwardSolution = MocoTrajectory(timeVec, states.getColumnLabels(),
            controls.getColumnLabels(), states.getColumnLabels(), {},
            states.getMatrix(), controls.getMatrix(), states.getMatrix(),
            SimTK::RowVector(0));

    // Compare controls between foward simulation and OCP solution. These
    // should match very closely, since the foward simulation controls are
    // created from splines of the OCP solution controls
    SimTK_TEST_EQ_TOL(solution.compareContinuousVariablesRMS(
                              forwardSolution, {{"controls", {}}}),
            0, 1e-9);

    // Compare states trajectory between forward simulation and OCP solution.
    // The states trajectory may not match as well as the controls.
    SimTK_TEST_EQ_TOL(solution.compareContinuousVariablesRMS(
                              forwardSolution, {{"states", {}}}),
            0, tol);

    return forwardSolution;
}

/// Solve an optimal control problem where a double pendulum must reach a
/// specified final configuration while subject to a constraint that couples
/// its two coordinates together via a linear relationship and minimizing
/// control effort. TODO
void testDoublePendulumCoordinateCouplerProjection(
        MocoSolution& solution, std::string dynamics_mode) {
    MocoStudy study;
    study.setName("double_pendulum_coordinate_coupler_projection");

    // Create double pendulum model and add the coordinate coupler constraint.
    auto model = createDoublePendulumModel();
    const Coordinate& q0 = model->getCoordinateSet().get("q0");
    const Coordinate& q1 = model->getCoordinateSet().get("q1");
    CoordinateCouplerConstraint* constraint = new CoordinateCouplerConstraint();
    Array<std::string> indepCoordNames;
    indepCoordNames.append("q0");
    constraint->setIndependentCoordinateNames(indepCoordNames);
    constraint->setDependentCoordinateName("q1");
    // Represented by the following equation,
    //      q1 = m*q0 + b
    // this linear function couples the two model coordinates such that given
    // the boundary conditions for q0 from testDoublePendulumPointOnLine, the
    // same boundary conditions for q1 should be achieved without imposing
    // bounds for this coordinate.
    const SimTK::Real m = -2;
    const SimTK::Real b = SimTK::Pi;
    LinearFunction linFunc(m, b);
    // Avoid CoordinateCoupler::setFunction(const Function&); it has a leak.
    constraint->setFunction(&linFunc);
    model->addConstraint(constraint);
    model->finalizeConnections();

    MocoProblem& mp = study.updProblem();
    mp.setModelAsCopy(*model);
    mp.setTimeBounds(0, 1);
    // Boundary conditions are only enforced for the first coordinate, so we can
    // test that the second coordinate is properly coupled.
    mp.setStateInfo("/jointset/j0/q0/value", {-5, 5}, 0, SimTK::Pi / 2);
    mp.setStateInfo("/jointset/j0/q0/speed", {-10, 10}, 0, 0);
    mp.setStateInfo("/jointset/j1/q1/value", {-10, 10});
    mp.setStateInfo("/jointset/j1/q1/speed", {-5, 5}, 0, 0);
    mp.setControlInfo("/tau0", {-50, 50});
    mp.setControlInfo("/tau1", {-50, 50});
    mp.addGoal<MocoControlGoal>();

    auto& ms = study.initSolver<MocoCasADiSolver>();
    ms.set_num_mesh_intervals(20);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    ms.set_transcription_scheme("trapezoidal");
    ms.set_kinematic_constraint_method("projection");
    ms.set_multibody_dynamics_mode(dynamics_mode);
    ms.set_parallel(0);
    ms.setGuess("bounds");

    solution = study.solve();
    solution.write("testConstraints_testDoublePendulumCoordinateCoupler.sto");
    //study.visualize(solution);

    model->initSystem();
    StatesTrajectory states = solution.exportToStatesTrajectory(mp);
    for (int i = 0; i < (int)states.getSize(); ++i) {
        const auto& s = states.get(i);
        model->realizePosition(s);

        // The coordinates should be coupled according to the linear function
        // described above.
        SimTK_TEST_EQ_TOL(q1.getValue(s), m * q0.getValue(s) + b, 1e-2);
    }

    // Run a forward simulation using the solution controls in prescribed
    // controllers for the model actuators and see if we get the correct states
    // trajectory back.
    runForwardSimulation(*model, solution, 1e-1);
}

int main() {
    MocoSolution couplerSol;
    testDoublePendulumCoordinateCouplerProjection(couplerSol, "explicit");

    return EXIT_SUCCESS;
}
