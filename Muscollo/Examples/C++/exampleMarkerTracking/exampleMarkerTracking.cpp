/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: exampleMarkerTracking.cpp                                *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

/// This example solves a basic marker tracking problem using a double
/// pendulum.

#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <Muscollo/osimMuscollo.h>

using namespace OpenSim;

/// This model is torque-actuated.
Model createDoublePendulumModel() {
    Model model;
    model.setName("double_pendulum");

    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create two links, each with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    auto* b0 = new OpenSim::Body("b0", 1, Vec3(0), Inertia(1));
    model.addBody(b0);
    auto* b1 = new OpenSim::Body("b1", 1, Vec3(0), Inertia(1));
    model.addBody(b1);

    // Add markers to body origin locations
    auto* m0 = new Marker("m0", *b0, Vec3(0));
    auto* m1 = new Marker("m1", *b1, Vec3(0));
    model.addMarker(m0);
    model.addMarker(m1);

    // Connect the bodies with pin joints. Assume each body is 1 m long.
    auto* j0 = new PinJoint("j0", model.getGround(), Vec3(0), Vec3(0),
        *b0, Vec3(-1, 0, 0), Vec3(0));
    auto& q0 = j0->updCoordinate();
    q0.setName("q0");
    auto* j1 = new PinJoint("j1",
        *b0, Vec3(0), Vec3(0), *b1, Vec3(-1, 0, 0), Vec3(0));
    auto& q1 = j1->updCoordinate();
    q1.setName("q1");
    model.addJoint(j0);
    model.addJoint(j1);

    auto* tau0 = new CoordinateActuator();
    tau0->setCoordinate(&j0->updCoordinate());
    tau0->setName("tau0");
    tau0->setOptimalForce(1);
    model.addComponent(tau0);

    auto* tau1 = new CoordinateActuator();
    tau1->setCoordinate(&j1->updCoordinate());
    tau1->setName("tau1");
    tau1->setOptimalForce(1);
    model.addComponent(tau1);

    // Add display geometry.
    Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
    SimTK::Transform transform(SimTK::Vec3(-0.5, 0, 0));
    auto* b0Center = new PhysicalOffsetFrame("b0_center", "b0", transform);
    b0->addComponent(b0Center);
    b0Center->attachGeometry(bodyGeometry.clone());
    auto* b1Center = new PhysicalOffsetFrame("b1_center", "b1", transform);
    b1->addComponent(b1Center);
    b1Center->attachGeometry(bodyGeometry.clone());

    return model;
}

int main() {

    MucoTool muco;
    muco.setName("double_pendulum_marker_tracking");

    // Define the optimal control problem.
    // ===================================
    MucoProblem& problem = muco.updProblem();

    // Model (dynamics).
    // -----------------
    problem.setModel(createDoublePendulumModel());

    // Bounds.
    // -------
    double finalTime = 1.0;
    problem.setTimeBounds(0, finalTime);
    problem.setStateInfo("j0/q0/value", { -10, 10 });
    problem.setStateInfo("j0/q0/speed", { -50, 50 });
    problem.setStateInfo("j1/q1/value", { -10, 10 });
    problem.setStateInfo("j1/q1/speed", { -50, 50 });
    problem.setControlInfo("tau0", { -40, 40 });
    problem.setControlInfo("tau1", { -40, 40 });

    // Cost.
    // -----
    // Create marker trajectories based on exampleTracking.cpp joint angles.
    TimeSeriesTableVec3 markerTrajectories;
    markerTrajectories.setColumnLabels({"m0", "m1"});
    for (double time = 0; time < finalTime; time += 0.01) {

        SimTK::Real q0 = (time / 1.0) * 0.5 * SimTK::Pi;
        SimTK::Real q1 = (time / 1.0) * 0.25 * SimTK::Pi;
        SimTK::Vec3 m0(cos(q0), sin(q0), 0);
        SimTK::Vec3 m1 = m0 + SimTK::Vec3(cos(q0 + q1), sin(q0 + q1), 0);

        markerTrajectories.appendRow(time, {m0, m1});
    }

    // Assign a weight to each marker.
    Set<MarkerWeight> markerWeights;
    markerWeights.cloneAndAppend({"m0", 100});
    markerWeights.cloneAndAppend({"m1", 10});

    // Create the MarkersReference to be passed to the cost.
    //MarkersReference ref(markerTrajectories);
    MarkersReference ref(markerTrajectories, &markerWeights);

    // Create cost, set reference, and attach to problem.
    MucoMarkerTrackingCost markerTracking;
    markerTracking.setMarkersReference(ref);
    problem.addCost(markerTracking);

    // Configure the solver.
    // =====================
    MucoTropterSolver& solver = muco.initSolver();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_hessian_approximation("exact");

    // Solve the problem.
    // ==================
    MucoSolution solution = muco.solve();
    solution.write("exampleMarkerTracking_solution.sto");

    muco.visualize(solution);

    return EXIT_SUCCESS;
}