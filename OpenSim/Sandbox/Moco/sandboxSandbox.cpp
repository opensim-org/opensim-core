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

#include "OpenSim/Actuators/CoordinateActuator.h"
#include "OpenSim/Common/LinearFunction.h"

#include <OpenSim/Moco/osimMoco.h>

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

/// Solve an optimal control problem where a double pendulum must reach a
/// specified final configuration while subject to a constraint that couples
/// its two coordinates together via a linear relationship and minimizing
/// control effort.
void testDoublePendulumCoordinateCoupler(MocoSolution& couplerSolution,
        bool enforce_constraint_derivatives, std::string dynamics_mode,
        std::string kinematic_constraint_method = "Posa2016",
        std::string transcription_scheme = "hermite-simpson",
        int num_mesh_intervals = 20) {
    MocoStudy study;
    study.setName("double_pendulum_coordinate_coupler");

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
    auto* effort = mp.addGoal<MocoControlGoal>();

    auto& ms = study.initSolver<MocoCasADiSolver>();
    ms.set_num_mesh_intervals(num_mesh_intervals);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    ms.set_transcription_scheme(transcription_scheme);
    ms.set_enforce_constraint_derivatives(enforce_constraint_derivatives);
    ms.set_kinematic_constraint_method(kinematic_constraint_method);
    if (!enforce_constraint_derivatives) {
        ms.set_minimize_lagrange_multipliers(true);
        ms.set_lagrange_multiplier_weight(10);
    }
    ms.set_multibody_dynamics_mode(dynamics_mode);
    ms.setGuess("bounds");

    couplerSolution = study.solve();
    couplerSolution.write("sandbox_testDoublePendulumCoordinateCoupler.sto");
}

/// Solve an optimal control problem where a double pendulum must follow a
/// prescribed motion based on the previous test case (see
/// testDoublePendulumCoordinateCoupler).
void testDoublePendulumPrescribedMotion(MocoSolution& couplerSolution,
        bool enforce_constraint_derivatives, std::string dynamics_mode,
        std::string kinematic_constraint_method = "Posa2016",
        std::string transcription_scheme = "hermite-simpson",
        int num_mesh_intervals = 20) {
    MocoStudy study;
    study.setName("double_pendulum_prescribed_motion");
    MocoProblem& mp = study.updProblem();

    // Create double pendulum model.
    auto model = createDoublePendulumModel();
    // Create a spline set for the model states from the previous solution. We
    // need to call initSystem() and set the model here in order to convert the
    // solution from the previous problem to a StatesTrajectory.
    model->initSystem();
    mp.setModelAsCopy(*model);

    TimeSeriesTable statesTrajCoupler =
            couplerSolution.exportToStatesTrajectory(mp).exportToTable(*model);
    GCVSplineSet statesSpline(statesTrajCoupler);

    // Apply the prescribed motion constraints.
    Coordinate& q0 = model->updJointSet().get("j0").updCoordinate();
    q0.setPrescribedFunction(statesSpline.get("/jointset/j0/q0/value"));
    q0.setDefaultIsPrescribed(true);
    Coordinate& q1 = model->updJointSet().get("j1").updCoordinate();
    q1.setPrescribedFunction(statesSpline.get("/jointset/j1/q1/value"));
    q1.setDefaultIsPrescribed(true);
    // Set the model again after implementing the constraints.
    mp.setModelAsCopy(*model);

    mp.setTimeBounds(0, 1);
    // No bounds here, since the problem is already highly constrained by the
    // prescribed motion constraints on the coordinates.
    mp.setStateInfo("/jointset/j0/q0/value", {-10, 10});
    mp.setStateInfo("/jointset/j0/q0/speed", {-50, 50});
    mp.setStateInfo("/jointset/j1/q1/value", {-10, 10});
    mp.setStateInfo("/jointset/j1/q1/speed", {-50, 50});
    mp.setControlInfo("/tau0", {-25, 25});
    mp.setControlInfo("/tau1", {-25, 25});

    auto* effort = mp.addGoal<MocoControlGoal>();

    auto& ms = study.initSolver<MocoCasADiSolver>();
    ms.set_num_mesh_intervals(num_mesh_intervals);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-2);
    ms.set_transcription_scheme(transcription_scheme);
    ms.set_enforce_constraint_derivatives(enforce_constraint_derivatives);
    ms.set_kinematic_constraint_method(kinematic_constraint_method);
    if (!enforce_constraint_derivatives) {
        ms.set_minimize_lagrange_multipliers(true);
        ms.set_lagrange_multiplier_weight(10);
    }
    ms.set_multibody_dynamics_mode(dynamics_mode);

    // Set guess based on coupler solution trajectory.
    MocoTrajectory guess(ms.createGuess("bounds"));
    guess.setStatesTrajectory(statesTrajCoupler);
    ms.setGuess(guess);

    MocoSolution solution = study.solve().unseal();
    solution.write("sandbox_testDoublePendulumPrescribedMotion.sto");
    //study.visualize(solution);
}

int main() {

    MocoSolution couplerSol;
//    testDoublePendulumCoordinateCoupler(
//            couplerSol, true, "explicit");
//    testDoublePendulumPrescribedMotion(
//            couplerSol, true, "explicit");

    testDoublePendulumCoordinateCoupler(couplerSol, true, "explicit", "Bordalba2023", "legendre-gauss-radau-3",
            20);

    return EXIT_SUCCESS;
}
