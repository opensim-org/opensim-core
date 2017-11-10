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
public:
    OpenSim_DECLARE_SOCKET(station, Station, "TODO");
    void computeForce(const SimTK::State& s,
            SimTK::Vector_<SimTK::SpatialVec>& /*bodyForces*/,
            SimTK::Vector& generalizedForces) const override {
        const auto& pt = getConnectee<Station>("station");
        const auto& pos = pt.getLocationInGround(s);
        const auto& vel = pt.getVelocityInGround(s);
        const SimTK::Real y = pos[1];
        const SimTK::Real vy = vel[1];
        SimTK::Real force = 0;
        const SimTK::Real depth = 0 - y;
        const SimTK::Real depthRate = 0 - vy;
        const SimTK::Real a = 5e7; // N/m^3
        const SimTK::Real b = 1.0; // s/m
        if (depth > 0) {
            //force = 1000 * depth;
            force = fmax(0, a * pow(depth, 3) * (1 + b * depthRate));
        }
        const SimTK::Real voidStiffness = 1.0; // N/m
        force += voidStiffness * depth;
        applyGeneralizedForce(s, getModel().getCoordinateSet().get(0),
                force, generalizedForces);
    }
};

Model createModel() {
    Model model;
    model.setName("point_mass");
    auto* body = new Body("body", 50.0, Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider",
            model.getGround(), Vec3(0), SimTK::Vec3(0, 0, 0.5 * SimTK::Pi),
            *body, Vec3(0), Vec3(0));
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("y");
    model.addComponent(joint);

    auto* station = new Station();
    station->setName("contact_point");
    station->connectSocket_parent_frame(*body);
    model.addComponent(station);

    auto* force = new CustomContactForce();
    force->setName("contact");
    model.addComponent(force);
    force->connectSocket_station(*station);

    return model;
}

int main() {

    const SimTK::Real y0 = 0.5;
    const SimTK::Real finalTime = 1.0;
    Model model = createModel();
    auto state = model.initSystem();
    model.setStateVariableValue(state, "/slider/y/value", y0);

    Manager manager(model);
    manager.integrate(state, finalTime);
    visualize(model, manager.getStateStorage());

    // TODO use the simulation as an initial guess!!!


    MucoTool muco;
    muco.setName("sliding_mass");

    // Define the optimal control problem.
    // ===================================
    MucoProblem& mp = muco.updProblem();

    // Model (dynamics).
    // -----------------
    mp.setModel(model);

    // Bounds.
    // -------
    // Initial time must be 0, final time can be within [0, 5].
    mp.setTimeBounds(0, finalTime);

    // Initial position must be 0, final position must be 1.
    mp.setStateInfo("slider/y/value", MucoBounds(-5, 5), y0);
    // Initial and final speed must be 0. Use compact syntax.
    mp.setStateInfo("slider/y/speed", {-50, 50}, 0);

    // Configure the solver.
    // =====================
    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(100);

    // TODO interface for setting these options:
    // TODO ms.setOption("optim.hessian-approximation", "limited-memory");
    // TODO ms.set_optimizer_algorithm("ipopt");


    // Now that we've finished setting up the tool, print it to a file.
    //muco.print("contact.omuco");

    // Solve the problem.
    // ==================
    MucoSolution solution = muco.solve();

    //solution.write("sliding_mass_solution.sto");

    // TODO copmare the forward and direct collocation integration.

    // Visualize.
    // ==========
    muco.visualize(solution);

    return EXIT_SUCCESS;
}
