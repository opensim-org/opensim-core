/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxContact.cpp                                       *
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

#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <Muscollo/osimMuscollo.h>
#include <OpenSim/Simulation/Manager/Manager.h>

using namespace OpenSim;
using SimTK::Vec3;

class CustomContactForce : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(CustomContactForce, Force);
    void computeForce(const SimTK::State& s,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& generalizedForces) const override {
        applyGeneralizedForce(s, getModel().getCoordinateSet().get(0),
                -getModel().getTotalMass(s) * getModel().getGravity()[1],
                generalizedForces);
    }
};

Model createModel() {
    Model model;
    model.setName("point_mass");
    auto* body = new Body("body", 2.0, Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider",
            model.getGround(), Vec3(0), SimTK::Vec3(0, 0, 0.5 * SimTK::Pi),
            *body, Vec3(0), Vec3(0));
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("y");
    model.addComponent(joint);

    auto* force = new CustomContactForce();
    model.addComponent(force);

    return model;
}

int main() {

    Model model = createModel();
    auto state = model.initSystem();

    Manager manager(model);
    manager.integrate(state, 1.0);
    visualize(model, manager.getStateStorage());
    /*
    MucoTool muco;
    muco.setName("sliding_mass");

    // Define the optimal control problem.
    // ===================================
    MucoProblem& mp = muco.updProblem();

    // Model (dynamics).
    // -----------------
    mp.setModel(createSlidingMassModel());

    // Bounds.
    // -------
    // Initial time must be 0, final time can be within [0, 5].
    mp.setTimeBounds(MucoInitialBounds(0), MucoFinalBounds(0, 5));

    // Initial position must be 0, final position must be 1.
    mp.setStateInfo("slider/y/value", MucoBounds(-5, 5),
            MucoInitialBounds(0), MucoFinalBounds(1));
    // Initial and final speed must be 0. Use compact syntax.
    mp.setStateInfo("slider/y/speed", {-50, 50}, 0, 0);

    // Applied force must be between -50 and 50.
    mp.setControlInfo("actuator", MucoBounds(-50, 50));

    // Cost.
    // -----
    MucoFinalTimeCost ftCost;
    mp.addCost(ftCost);

    // Configure the solver.
    // =====================
    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(50);

    // TODO interface for setting these options:
    // TODO ms.setOption("optim.hessian-approximation", "limited-memory");
    // TODO ms.set_optimizer_algorithm("ipopt");


    // Now that we've finished setting up the tool, print it to a file.
    muco.print("sliding_mass.omuco");

    // Solve the problem.
    // ==================
    MucoSolution solution = muco.solve();

    //solution.write("sliding_mass_solution.sto");

    // Visualize.
    // ==========
    muco.visualize(solution);
    */

    return EXIT_SUCCESS;
}
