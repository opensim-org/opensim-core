/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleSlidingMass.cpp                                       *
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

/// Translate a point mass in one dimension in minimum time. This is a very
/// simple example that shows only the basics of Moco.
///
/// @verbatim
/// minimize   t_f
/// subject to xdot = v
///            vdot = F/m
///            x(0)   = 0
///            x(t_f) = 1
///            v(0)   = 0
///            v(t_f) = 0
/// w.r.t.     x   in [-5, 5]    position of mass
///            v   in [-50, 50]  speed of mass
///            F   in [-50, 50]  force applied to the mass
///            t_f in [0, 5]     final time
/// constants  m       mass
/// @endverbatim

#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Moco/osimMoco.h>

using namespace OpenSim;

std::unique_ptr<Model> createSlidingMassModel() {
    auto model = make_unique<Model>();
    model->setName("sliding_mass");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 2.0, SimTK::Vec3(0), SimTK::Inertia(0));
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

int main() {

    MocoStudy study;
    study.setName("sliding_mass");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = study.updProblem();

    // Model (dynamics).
    // -----------------
    problem.setModel(createSlidingMassModel());

    // Bounds.
    // -------
    // Initial time must be 0, final time can be within [0, 5].
    problem.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(0, 5));

    // Position must be within [-5, 5] throughout the motion.
    // Initial position must be 0, final position must be 1.
    problem.setStateInfo("/slider/position/value", MocoBounds(-5, 5),
                         MocoInitialBounds(0), MocoFinalBounds(1));
    // Speed must be within [-50, 50] throughout the motion.
    // Initial and final speed must be 0. Use compact syntax.
    problem.setStateInfo("/slider/position/speed", {-50, 50}, 0, 0);

    // Applied force must be between -50 and 50.
    problem.setControlInfo("/actuator", MocoBounds(-50, 50));

    // Cost.
    // -----
    problem.addGoal<MocoFinalTimeGoal>();

    // Configure the solver.
    // =====================
    MocoCasADiSolver& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(50);

    // Now that we've finished setting up the tool, print it to a file.
    study.print("sliding_mass.omoco");

    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve();

    //solution.write("sliding_mass_solution.sto");

    // Visualize.
    // ==========
    study.visualize(solution);

    return EXIT_SUCCESS;
}
