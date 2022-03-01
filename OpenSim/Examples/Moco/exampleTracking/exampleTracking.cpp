/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleTracking.cpp                                          *
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

/// This example solves a basic coordinate tracking problem using a double
/// pendulum.

#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Moco/osimMoco.h>

using namespace OpenSim;

/// This model is torque-actuated.
std::unique_ptr<Model> createDoublePendulumModel() {
    // This function is similar to ModelFactory::createNLinkPendulum().

    auto model = make_unique<Model>();
    model->setName("double_pendulum");

    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create two links, each with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    auto* b0 = new OpenSim::Body("b0", 1, Vec3(0), Inertia(1));
    model->addBody(b0);
    auto* b1 = new OpenSim::Body("b1", 1, Vec3(0), Inertia(1));
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
    q0.setRangeMin(-10);
    q0.setRangeMax(10);
    q0.setName("q0");
    auto* j1 = new PinJoint("j1",
            *b0, Vec3(0), Vec3(0), *b1, Vec3(-1, 0, 0), Vec3(0));
    auto& q1 = j1->updCoordinate();
    q1.setRangeMin(-10);
    q1.setRangeMax(10);
    q1.setName("q1");
    model->addJoint(j0);
    model->addJoint(j1);

    auto* tau0 = new CoordinateActuator();
    tau0->setCoordinate(&j0->updCoordinate());
    tau0->setName("tau0");
    tau0->setOptimalForce(1);
    tau0->setMinControl(-40);
    tau0->setMaxControl(40);
    model->addForce(tau0);

    auto* tau1 = new CoordinateActuator();
    tau1->setCoordinate(&j1->updCoordinate());
    tau1->setName("tau1");
    tau1->setOptimalForce(1);
    tau1->setMinControl(-40);
    tau1->setMaxControl(40);
    model->addForce(tau1);

    // Add display geometry.
    SimTK::Transform transform(SimTK::Vec3(-0.5, 0, 0));
    auto* b0Center = new PhysicalOffsetFrame("b0_center", *b0, transform);
    b0->addComponent(b0Center);
    b0Center->attachGeometry(new Ellipsoid(0.5, 0.1, 0.1));
    auto* b1Center = new PhysicalOffsetFrame("b1_center", *b1, transform);
    b1->addComponent(b1Center);
    b1Center->attachGeometry(new Ellipsoid(0.5, 0.1, 0.1));

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
    double finalTime = 1.0;
    problem.setTimeBounds(0, finalTime);
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10});
    problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50});
    problem.setStateInfo("/jointset/j1/q1/value", {-10, 10});
    problem.setStateInfo("/jointset/j1/q1/speed", {-50, 50});
    problem.setControlInfo("/forceset/tau0", {-40, 40});
    problem.setControlInfo("/forceset/tau1", {-40, 40});

    // Cost.
    // -----
    auto* tracking = problem.addGoal<MocoStateTrackingGoal>("tracking");
    TimeSeriesTable ref;
    ref.addTableMetaData<std::string>("inDegrees", "no");
    ref.setColumnLabels({"/jointset/j0/q0/value", "/jointset/j1/q1/value"});
    // We supply a reference whose time range is a superset of the problem's
    // time bounds: Moco performs finite differences internally, which may
    // require sampling outside the problem's time bounds.
    for (double time = -0.05; time < finalTime + 0.05; time += 0.01) {
        ref.appendRow(time, {
                0.5 * SimTK::Pi * time,
                0.25 * SimTK::Pi * time
        });
    }

    tracking->setReference(ref);

    // A low-weighted cost term minimizing controls helps with convergence.
    problem.addGoal<MocoControlGoal>("effort", 0.001);

    // Configure the solver.
    // =====================
    auto& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");

    study.print("double_pendulum_tracking.omoco");

    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve();
    solution.write("exampleTracking_solution.sto");

    study.visualize(solution);

    return EXIT_SUCCESS;
}
