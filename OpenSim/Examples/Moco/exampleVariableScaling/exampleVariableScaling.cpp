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

/// This example 
/// Inspired by: https://web.casadi.org/blog/nlp-scaling/.

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

using namespace OpenSim;

std::unique_ptr<Model> createRocketModel() {
    auto model = make_unique<Model>();
    model->setName("sliding_mass");
    model->set_gravity(SimTK::Vec3(9.81, 0, 0));
    auto* body = new Body("body", 100000.0, SimTK::Vec3(0), SimTK::Inertia(0));
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
    model->addComponent(actu);

    body->attachGeometry(new Sphere(0.05));

    model->finalizeConnections();

    return model;
}

void solveRocketProblem(bool scaleVariables) {

    MocoStudy study;
    study.setName("rocket_problem");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = study.updProblem();

    // Model (dynamics).
    // -----------------
    problem.setModel(createRocketModel());

    // Bounds.
    // -------
    // The rocket must reach a height of 100000 meters in 100 seconds
    // starting from rest.
    problem.setTimeBounds(0, 100);
    problem.setStateInfo("/slider/position/value", {0, 1e5}, 0, 1e5);
    problem.setStateInfo("/slider/position/speed", {-1e4, 1e4}, 0);
    problem.setControlInfo("/actuator", {0, 1e9});

    // Cost.
    // -----
    problem.addGoal<MocoControlGoal>();

    // Configure the solver.
    // =====================
    MocoCasADiSolver& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(100);
    solver.set_scale_variables_using_bounds(scaleVariables);

    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve();
    solution.write("exampleVariableScaling_solution.sto");
}

int main() {

    // solves in ~150 iterations
    solveRocketProblem(false);

    // solves in ~10 iterations
    solveRocketProblem(true);


    return EXIT_SUCCESS;
}
