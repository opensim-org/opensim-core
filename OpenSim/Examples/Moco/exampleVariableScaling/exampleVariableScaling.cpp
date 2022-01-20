/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleVariableScaling.cpp                                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2022 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

/// This example ... TODO

#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Moco/osimMoco.h>

using namespace OpenSim;

std::unique_ptr<Model> createDoublePendulumModel() {
    // This function is similar to ModelFactory::createNLinkPendulum().

    auto model = make_unique<Model>();
    model->setName("double_pendulum");

    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create two links, each with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    double scale = 1e-10;
    auto* b0 = new OpenSim::Body("b0", 1.0/scale, Vec3(0), Inertia(1.0/scale));
    model->addBody(b0);
    auto* b1 = new OpenSim::Body("b1", scale, Vec3(0), Inertia(scale));
    model->addBody(b1);

    // Add markers to body origin locations
    auto* m0 = new Marker("m0", *b0, Vec3(0));
    auto* m1 = new Marker("m1", *b1, Vec3(0));
    model->addMarker(m0);
    model->addMarker(m1);

    // Connect the bodies with pin joints. Assume each body is 1 m long.
    auto* j0 = new PinJoint("j0", model->getGround(), Vec3(0), Vec3(0),
            *b0, Vec3(-1, 0, 0), Vec3(0));
    auto& q0 = j0->updCoordinate();
    q0.setName("q0");
    auto* j1 = new PinJoint("j1",
            *b0, Vec3(0), Vec3(0), *b1, Vec3(-1, 0, 0), Vec3(0));
    auto& q1 = j1->updCoordinate();
    q1.setName("q1");
    model->addJoint(j0);
    model->addJoint(j1);

    auto* tau0 = new CoordinateActuator();
    tau0->setCoordinate(&j0->updCoordinate());
    tau0->setName("tau0");
    tau0->setOptimalForce(1);
    model->addForce(tau0);

    auto* tau1 = new CoordinateActuator();
    tau1->setCoordinate(&j1->updCoordinate());
    tau1->setName("tau1");
    tau1->setOptimalForce(1);
    model->addForce(tau1);

    model->finalizeConnections();

    return model;
}

int main() {

    MocoStudy study;
    study.setName("double_pendulum_tracking");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = study.updProblem();

    // Model (dynamics).
    // -----------------
    problem.setModel(createDoublePendulumModel());

    // Bounds.
    // -------
    problem.setTimeBounds(0, 10.0);
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0, 10.0);
    problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0);
    problem.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0, 10.0);
    problem.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0);
    double scale = 1e-10;
    problem.setControlInfo("/forceset/tau0", {-100/scale, 100/scale});
    problem.setControlInfo("/forceset/tau1", {-100*scale, 100*scale});

    // Cost.
    // -----
    auto* effort = problem.addGoal<MocoControlGoal>();

    // Configure the solver.
    // =====================
    auto& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(100);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_scale_variables_using_bounds(true);

    study.print("double_pendulum_tracking.omoco");

    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve();
    solution.write("exampleVariableScaling_solution.sto");

    return EXIT_SUCCESS;
}
